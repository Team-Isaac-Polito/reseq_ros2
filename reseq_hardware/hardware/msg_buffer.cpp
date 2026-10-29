#include "reseq_hardware/msg_buffer.hpp"
#include <utility>  // for pair

namespace reseq_hardware
{
void MessageBuffer::push_msg(const CanID & id, const uint8_t * data, size_t size)
{
  // Clamp to the ParsedMessage data buffer size (64 bytes, CAN FD max)
  if (size > sizeof(ParsedMessage{}.data)) {
    size = sizeof(ParsedMessage{}.data);
  }
  ParsedMessage pm = {id, {}, size, std::chrono::steady_clock::now()};
  std::memcpy(pm.data, data, size);

  std::lock_guard<std::mutex> lock(mutex_);
  queue_[pm.id] = pm;
}

std::vector<ParsedMessage> MessageBuffer::get_all()
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<ParsedMessage> messages;
  for (auto & pair : queue_) {
    messages.push_back(pair.second);
  }
  // Messages are kept in the map, not cleared
  return messages;
}

void MessageBuffer::clear()
{
  std::lock_guard<std::mutex> lock(mutex_);
  queue_.clear();
}

}  // namespace reseq_hardware
