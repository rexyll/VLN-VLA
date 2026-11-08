#include "GraspCarryQueue.hh"

#include <cassert>
#include <string>
#include <utility>
#include <vector>

namespace
{
struct Command
{
  std::string op;
  std::string model;
  unsigned requestId{0};
};

bool Coalesce(const Command &queued, const Command &incoming)
{
  return incoming.op == "set" && queued.op == "set" &&
         queued.model == incoming.model && incoming.requestId == 0 &&
         queued.requestId == 0;
}

bool Duplicate(const Command &queued, const Command &incoming)
{
  return incoming.requestId != 0 && queued.op == incoming.op &&
         queued.model == incoming.model &&
         queued.requestId == incoming.requestId;
}

void TestFifoDoesNotCoalesceAcrossTransaction()
{
  vla::BoundedCommandQueue<Command> queue(4);
  assert(queue.Push(Command{"set", "cup", 0}, Coalesce, Duplicate) ==
         vla::QueueInsertResult::Enqueued);
  assert(queue.Push(Command{"joint", "cup", 7}, Coalesce, Duplicate) ==
         vla::QueueInsertResult::Enqueued);
  assert(queue.Push(Command{"set", "cup", 0}, Coalesce, Duplicate) ==
         vla::QueueInsertResult::Enqueued);

  Command out;
  assert(queue.Pop(out) && out.op == "set");
  assert(queue.Pop(out) && out.op == "joint" && out.requestId == 7);
  assert(queue.Pop(out) && out.op == "set");
  assert(queue.Empty());
}

void TestTailCoalescingAndTransactionDeduplication()
{
  vla::BoundedCommandQueue<Command> queue(3);
  assert(queue.Push(Command{"set", "cup", 0}, Coalesce, Duplicate) ==
         vla::QueueInsertResult::Enqueued);
  assert(queue.Push(Command{"set", "cup", 0}, Coalesce, Duplicate) ==
         vla::QueueInsertResult::Coalesced);
  assert(queue.Size() == 1);
  assert(queue.Push(Command{"grab", "cup", 42}, Coalesce, Duplicate) ==
         vla::QueueInsertResult::Enqueued);
  assert(queue.Push(Command{"grab", "cup", 42}, Coalesce, Duplicate) ==
         vla::QueueInsertResult::Duplicate);
  assert(queue.Size() == 2);
}

void TestCapacityAndAckReplayCache()
{
  vla::BoundedCommandQueue<Command> queue(1);
  assert(queue.Push(Command{"release", "all", 9}, Coalesce, Duplicate) ==
         vla::QueueInsertResult::Enqueued);
  assert(queue.Push(Command{"grab", "cup", 10}, Coalesce, Duplicate) ==
         vla::QueueInsertResult::Full);

  vla::CompletedRequestAckCache cache(2);
  std::string ack;
  cache.Remember("grab|cup||1", "ok grab cup 1");
  assert(cache.Find("grab|cup||1", ack) && ack == "ok grab cup 1");
  cache.Remember("release|all||2", "ok release all 2");
  cache.Remember("joint|cup|water_joint|3", "ok joint cup water_joint 3");
  assert(!cache.Find("grab|cup||1", ack));
  assert(cache.Find("release|all||2", ack));
}

void TestInflightRegistryLifecycle()
{
  vla::InFlightRequestRegistry registry;
  assert(registry.Begin("grab|cup||11"));
  assert(!registry.Begin("grab|cup||11"));
  assert(registry.Contains("grab|cup||11"));
  registry.Finish("grab|cup||11");
  assert(!registry.Contains("grab|cup||11"));
  assert(registry.Begin("grab|cup||11"));
}
}  // namespace

int main()
{
  TestFifoDoesNotCoalesceAcrossTransaction();
  TestTailCoalescingAndTransactionDeduplication();
  TestCapacityAndAckReplayCache();
  TestInflightRegistryLifecycle();
  return 0;
}
