#pragma once

#include <stdint.h>                   // for uint8_t
#include <chrono>                     // for steady_clock::time_point, stead...
#include <cstring>                    // for size_t
#include <map>                        // for map
#include <mutex>                      // for mutex
#include <vector>                     // for vector
#include "reseq_hardware/canbus.hpp"  // for CanID

namespace reseq_hardware
{

/**
 * @brief Represents a parsed CAN message with metadata.
 */
struct ParsedMessage
{
  CanID id; ///< CAN identifier of the message.
  uint8_t data[8]; ///< Data payload of the message (up to 8 bytes).
  size_t size; ///< Size of the data payload.
  std::chrono::steady_clock::time_point timestamp; ///< Timestamp when the message was received.
};

/**
 * @brief Thread-safe buffer for storing and retrieving CAN messages.
 *
 * This class allows pushing new messages into the buffer and retrieving all stored messages.
 * Internally, it uses a map to associate each CAN ID with its latest message.
 */
class MessageBuffer
{
public:
  /**
   * @brief Pushes a new message into the buffer.
   * @param id The CAN identifier of the message.
   * @param data Pointer to the message data.
   * @param size Size of the message data in bytes.
   */
  void push_msg(const CanID & id, const uint8_t * data, size_t size);

  /**
   * @brief Retrieves all messages currently stored in the buffer.
   * @return A vector containing all parsed messages.
   */
  std::vector<ParsedMessage> get_all();

  /**
   * @brief Clears all messages from the buffer.
   */
  void clear();

private:
  std::map<CanID, ParsedMessage> queue_; ///< Map from CAN ID to the latest parsed message.
  std::mutex mutex_; ///< Mutex for thread-safe access to the buffer.
};

}  // namespace reseq_hardware
