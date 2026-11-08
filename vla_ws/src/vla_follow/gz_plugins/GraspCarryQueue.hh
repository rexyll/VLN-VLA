#pragma once

#include <cstddef>
#include <deque>
#include <utility>

namespace vla
{

enum class QueueInsertResult
{
  Enqueued,
  Coalesced,
  Duplicate,
  Full
};

// Thread safety is deliberately owned by the caller so the same queue can be
// tested independently and protected together with plugin state by one mutex.
template <typename Command>
class BoundedCommandQueue
{
public:
  explicit BoundedCommandQueue(std::size_t capacity = 128)
      : capacity_(capacity)
  {
  }

  template <typename CoalesceMatch, typename DuplicateMatch>
  QueueInsertResult Push(Command &&command,
                         CoalesceMatch coalesceMatch,
                         DuplicateMatch duplicateMatch)
  {
    // Only coalesce with the queue tail. Looking past another command can move
    // a later set/place update ahead of an intervening transaction (joint,
    // grab, or release) for the same model and violate FIFO semantics.
    if (!pending_.empty() && coalesceMatch(pending_.back(), command))
    {
      pending_.back() = std::forward<Command>(command);
      return QueueInsertResult::Coalesced;
    }

    for (const auto &queued : pending_)
    {
      if (duplicateMatch(queued, command))
        return QueueInsertResult::Duplicate;
    }

    if (pending_.size() >= capacity_)
      return QueueInsertResult::Full;

    pending_.emplace_back(std::forward<Command>(command));
    return QueueInsertResult::Enqueued;
  }

  bool Pop(Command &out)
  {
    if (pending_.empty())
      return false;
    out = std::move(pending_.front());
    pending_.pop_front();
    return true;
  }

  std::size_t Size() const
  {
    return pending_.size();
  }

  bool Empty() const
  {
    return pending_.empty();
  }

private:
  std::size_t capacity_;
  std::deque<Command> pending_;
};

}  // namespace vla
