#include "reseq_hardware/canbus.hpp"

#include <linux/can.h>     // for can_frame, sockaddr_can, can_frame::(anony...
#include <net/if.h>        // for ifreq, IFNAMSIZ, ifr_ifindex, ifr_name
#include <sys/ioctl.h>     // for ioctl, SIOCGIFINDEX
#include <sys/socket.h>    // for bind, setsockopt, socket, ssize_t, AF_CAN
#include <sys/time.h>      // for timeval
#include <unistd.h>        // for close, read, write
#include <cstring>         // for memcpy, strncpy, size_t
#include <utility>         // for move

namespace reseq_hardware
{

CanBus::CanBus(const std::string & ifn)
{
  // Open CAN socket
  sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (sock_ < 0) {
    return;
  }

  // Locate the interface
  struct ifreq ifr = {};
  std::strncpy(ifr.ifr_name, ifn.c_str(), IFNAMSIZ - 1);
  if (ioctl(sock_, SIOCGIFINDEX, &ifr) < 0) {
    close(sock_);
    sock_ = -1;
    return;
  }

  // Bind the socket to the CAN interface
  struct sockaddr_can addr = {};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(sock_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    close(sock_);
    sock_ = -1;
    return;
  }

  // Set socket receive timeout
  struct timeval tv;
  tv.tv_sec = timeout_.count() / 1000;
  tv.tv_usec = (timeout_.count() % 1000) * 1000;
  setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof(tv));
}

bool CanBus::send(const CanID & id, const void * data, size_t size)
{
  if (sock_ < 0 || size > 8) {
    return false;
  }

  // Compose arbitration ID: [msg_id:8 bits][mod_id:8 bits][unused:16 bits]
  uint32_t arb_id = (id.msg_id << 16) | (id.mod_id << 8);

  struct can_frame frame = {};
  frame.can_id = arb_id | CAN_EFF_FLAG;
  frame.can_dlc = static_cast<__u8>(size);
  std::memcpy(frame.data, data, size);

  ssize_t nbytes = write(sock_, &frame, sizeof(frame));
  return nbytes == sizeof(frame);
}

bool CanBus::wait_for_message(const CanID & id, std::chrono::milliseconds timeout)
{
  if (sock_ < 0) {
    return false;
  }

  using namespace std::chrono;
  auto start_t = steady_clock::now();

  struct can_frame frame = {};

  while (true) {
    ssize_t nbytes = read(sock_, &frame, sizeof(frame));
    if (nbytes == sizeof(frame)) {
      // Extract mod_id and msg_id from received CAN ID
      uint8_t r_mod_id = frame.can_id & 0xFF;
      uint8_t r_msg_id = (frame.can_id >> 16) & 0xFF;
      
      if (r_mod_id == id.mod_id && r_msg_id == id.msg_id) {
        return true;
      }
    }

    if (steady_clock::now() - start_t > timeout) {
      return false;
    }
  }
  return false;
}

void CanBus::start()
{
  if (running_ || !*this) {
    return;
  }
  running_ = true;
  // Start background thread for receiving CAN messages
  rx_thread_ = std::thread(&CanBus::receive_loop, this);
}

void CanBus::stop()
{
  if (!running_) {
    return;
  }
  running_ = false;
  if (rx_thread_.joinable()) {
    rx_thread_.join();
  }
}

void CanBus::register_callback(std::function<void(const CanID &, const uint8_t *, size_t)> cb)
{
  callback_ = std::move(cb);
}

void CanBus::receive_loop()
{
  struct can_frame frame = {};
  while (running_) {
    ssize_t nbytes = read(sock_, &frame, sizeof(frame));
    if (nbytes == sizeof(frame) && callback_) {
      // Extract mod_id and msg_id from received CAN ID
      uint8_t r_mod_id = frame.can_id & 0xFF;
      uint8_t r_msg_id = (frame.can_id >> 16) & 0xFF;
      callback_({r_mod_id, r_msg_id}, frame.data, frame.can_dlc);
    }
  }
}

CanBus::~CanBus()
{
  stop();
  if (sock_ >= 0) {
    close(sock_);
  }
}
}  // namespace reseq_hardware
