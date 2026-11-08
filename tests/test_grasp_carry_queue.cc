#include "GraspCarryQueue.hh"

#include <cassert>
#include <cstdint>
#include <iostream>
#include <string>
#include <utility>

namespace
{

struct Command
{
  std::string op;
  std::string model;
  std::string joint;
  std::uint64_t requestId{0};
  int value{0};
};

bool CoalesceMatch(const Command &queued, const Command &incoming)
{
  const bool stream = incoming.op == "set" ||
                      (incoming.op == "place" && incoming.requestId == 0);
  return stream && queued.op == incoming.op &&
         queued.model == incoming.model && queued.requestId == 0;
}

bool DuplicateMatch(const Command &queued, const Command &incoming)
{
  return incoming.requestId != 0 &&
         queued.op == incoming.op &&
         queued.model == incoming.model &&
         queued.joint == incoming.joint &&
         queued.requestId == incoming.requestId;
}

using Queue = vla::BoundedCommandQueue<Command>;

vla::QueueInsertResult Push(Queue &queue, Command command)
{
  return queue.Push(std::move(command), CoalesceMatch, DuplicateMatch);
}

void TestInFlightRequestRetriesAreSuppressedUntilApplyCompletes()
{
  vla::InFlightRequestRegistry inFlight;
  const std::string key = "joint|cup|water_joint|41";
  assert(inFlight.Begin(key));
  assert(inFlight.Contains(key));
  // A retry arriving while Apply() is executing is rejected as a duplicate.
  assert(!inFlight.Begin(key));
  assert(inFlight.Contains(key));
  inFlight.Finish(key);
  assert(!inFlight.Contains(key));
  // The key can be registered again after the execution window closes.
  assert(inFlight.Begin(key));
  inFlight.Finish(key);
}

void TestCompletedAckCacheReplaysAndEvictsOldest()
{
  vla::CompletedRequestAckCache cache(2);
  std::string ack;
  assert(!cache.Find("request-1", ack));
  cache.Remember("request-1", "ok joint cup water_joint 1");
  assert(cache.Find("request-1", ack));
  assert(ack == "ok joint cup water_joint 1");
  cache.Remember("request-2", "ok release cup 2");
  cache.Remember("request-3", "fail grab kettle 3 no-model");
  assert(!cache.Find("request-1", ack));
  assert(cache.Find("request-2", ack) && ack == "ok release cup 2");
  assert(cache.Find("request-3", ack) && ack == "fail grab kettle 3 no-model");
  // Updating an existing key must not consume another cache slot.
  cache.Remember("request-2", "ok release cup 2");
  assert(cache.Find("request-2", ack));
}

void TestFifoAndCapacity()
{
  Queue queue(128);
  for (int i = 0; i < 128; ++i)
    assert(Push(queue, {"grab", "model", "", static_cast<std::uint64_t>(i + 1), i})
           == vla::QueueInsertResult::Enqueued);

  assert(queue.Size() == 128);
  assert(Push(queue, {"grab", "overflow", "", 999, 999})
         == vla::QueueInsertResult::Full);

  for (int i = 0; i < 128; ++i)
  {
    Command out;
    assert(queue.Pop(out));
    assert(out.value == i);
  }
  assert(queue.Empty());
  Command out;
  assert(!queue.Pop(out));
}

void TestAdjacentSetCoalescingPreservesQueuePosition()
{
  Queue queue(8);
  assert(Push(queue, {"grab", "kettle", "", 1, 10}) ==
         vla::QueueInsertResult::Enqueued);
  assert(Push(queue, {"set", "cup", "", 0, 20}) ==
         vla::QueueInsertResult::Enqueued);
  assert(Push(queue, {"set", "cup", "", 0, 40}) ==
         vla::QueueInsertResult::Coalesced);
  assert(queue.Size() == 2);

  Command out;
  assert(queue.Pop(out) && out.op == "grab" && out.value == 10);
  assert(queue.Pop(out) && out.op == "set" && out.value == 40);
}

void TestSetDoesNotCoalesceAcrossTransactionBarrier()
{
  Queue queue(8);
  assert(Push(queue, {"set", "cup", "", 0, 10}) ==
         vla::QueueInsertResult::Enqueued);
  assert(Push(queue, {"joint", "cup", "water_joint", 2, 20}) ==
         vla::QueueInsertResult::Enqueued);
  assert(Push(queue, {"set", "cup", "", 0, 30}) ==
         vla::QueueInsertResult::Enqueued);
  assert(queue.Size() == 3);

  Command out;
  assert(queue.Pop(out) && out.op == "set" && out.value == 10);
  assert(queue.Pop(out) && out.op == "joint" && out.value == 20);
  assert(queue.Pop(out) && out.op == "set" && out.value == 30);
}

void TestTransactionDeduplicationUsesFullIdentity()
{
  Queue queue(8);
  assert(Push(queue, {"joint", "cup", "water_joint", 41, 1}) ==
         vla::QueueInsertResult::Enqueued);
  assert(Push(queue, {"joint", "cup", "water_joint", 41, 2}) ==
         vla::QueueInsertResult::Duplicate);
  assert(Push(queue, {"joint", "cup", "jet_joint", 41, 3}) ==
         vla::QueueInsertResult::Enqueued);
  assert(Push(queue, {"joint", "kettle", "water_joint", 41, 4}) ==
         vla::QueueInsertResult::Enqueued);
  assert(Push(queue, {"joint", "cup", "water_joint", 42, 5}) ==
         vla::QueueInsertResult::Enqueued);
  assert(queue.Size() == 4);
}

void TestCoalescingAndDeduplicationStillWorkAtCapacity()
{
  Queue queue(2);
  assert(Push(queue, {"grab", "kettle", "", 9, 2}) ==
         vla::QueueInsertResult::Enqueued);
  assert(Push(queue, {"set", "cup", "", 0, 1}) ==
         vla::QueueInsertResult::Enqueued);
  assert(Push(queue, {"set", "cup", "", 0, 3}) ==
         vla::QueueInsertResult::Coalesced);
  assert(Push(queue, {"grab", "kettle", "", 9, 4}) ==
         vla::QueueInsertResult::Duplicate);
  assert(Push(queue, {"release", "cup", "", 10, 5}) ==
         vla::QueueInsertResult::Full);

  Command out;
  assert(queue.Pop(out) && out.op == "grab" && out.value == 2);
  assert(queue.Pop(out) && out.op == "set" && out.value == 3);
}

void TestStreamsAreScopedByModelAndOperation()
{
  Queue queue(8);
  assert(Push(queue, {"set", "cup", "", 0, 1}) ==
         vla::QueueInsertResult::Enqueued);
  assert(Push(queue, {"set", "kettle", "", 0, 2}) ==
         vla::QueueInsertResult::Enqueued);
  assert(Push(queue, {"place", "cup", "", 0, 3}) ==
         vla::QueueInsertResult::Enqueued);
  assert(Push(queue, {"set", "cup", "", 0, 4}) ==
         vla::QueueInsertResult::Enqueued);
  assert(queue.Size() == 4);
}

}  // namespace

int main()
{
  TestInFlightRequestRetriesAreSuppressedUntilApplyCompletes();
  TestCompletedAckCacheReplaysAndEvictsOldest();
  TestFifoAndCapacity();
  TestAdjacentSetCoalescingPreservesQueuePosition();
  TestSetDoesNotCoalesceAcrossTransactionBarrier();
  TestTransactionDeduplicationUsesFullIdentity();
  TestCoalescingAndDeduplicationStillWorkAtCapacity();
  TestStreamsAreScopedByModelAndOperation();
  std::cout << "GraspCarry queue tests: 6/6 passed\n";
  return 0;
}
