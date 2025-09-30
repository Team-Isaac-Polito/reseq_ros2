#pragma once

#include <stddef.h>    // for size_t
#include <stdint.h>    // for uint8_t
#include <atomic>      // for atomic
#include <chrono>      // for milliseconds
#include <functional>  // for function
#include <string>      // for allocator, string
#include <thread>      // for thread
#include <tuple>       // for tie, operator<, tuple

namespace reseq_hardware
{

/**
 * @brief Represents a CAN bus identifier composed of a module ID and a message ID.
 */
struct CanID
{
  uint8_t mod_id;  ///< Module identifier.
  uint8_t msg_id;  ///< Message identifier.

  bool operator<(const CanID & other) const
  {
    return std::tie(mod_id, msg_id) < std::tie(other.mod_id, other.msg_id);
  }
};

/**
 * @brief Provides an interface for CAN bus communication.
 *
 * This class allows sending and receiving messages over a CAN bus interface.
 * It supports registering a callback for incoming messages, waiting for specific messages,
 * and running a background thread to handle message reception.
 */
class CanBus
{
public:
  /**
   * @brief Constructs a CanBus object and opens the specified CAN interface.
   * @param ifn The CAN interface name (default: "can0").
   */
  explicit CanBus(const std::string & ifn = "can0");

  /**
   * @brief Sends a message over the CAN bus.
   * @param id The CAN identifier.
   * @param data Pointer to the data buffer to send.
   * @param size Size of the data buffer in bytes.
   * @return True if the message was sent successfully.
   */
  bool send(const CanID & id, const void * data, size_t size);

  /**
   * @brief Registers a callback to be invoked when a message is received.
   * @param cb The callback function, taking CanID, data pointer, and data size.
   */
  void register_callback(std::function<void(const CanID &, const uint8_t *, size_t)> cb);

  /**
   * @brief Waits for a message with the specified CAN ID.
   * @param id The CAN identifier to wait for.
   * @param timeout Maximum time to wait for the message.
   * @return True if the message was received within the timeout.
   */
  bool wait_for_message(const CanID & id, std::chrono::milliseconds timeout);

  /**
   * @brief Starts the background thread for receiving CAN messages.
   */
  void start();

  /**
   * @brief Stops the background receiving thread.
   */
  void stop();

  /**
   * @brief Checks if the CAN interface is open.
   * @return True if the socket is valid.
   */
  operator bool() const
  {
    return sock_ >= 0;
  }

  /**
   * @brief Destructor. Closes the CAN interface and stops the receiving thread.
   */
  ~CanBus();

private:
  /**
   * @brief The main loop for receiving CAN messages in a background thread.
   */
  void receive_loop();

  int sock_ = -1;                                                       ///< Socket descriptor for the CAN interface.
  std::chrono::milliseconds timeout_{500};                              ///< Default timeout for operations.
  std::atomic<bool> running_ = false;                                   ///< Indicates if the receive loop is running.
  std::thread rx_thread_;                                               ///< Thread for receiving CAN messages.
  std::function<void(const CanID &, const uint8_t *, size_t)> callback_;  ///< Registered callback for received messages.
};

}  // namespace reseq_hardware
