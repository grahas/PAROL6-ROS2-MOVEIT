#include "parol6_hardware_interface/bridge_client.hpp"

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstring>
#include <thread>

namespace parol6_hardware_interface
{

BridgeClient::~BridgeClient() { close(); }

void BridgeClient::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool BridgeClient::connect(const std::string & host, int port, double timeout_sec)
{
  close();

  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_sec);

  addrinfo hints{};
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_STREAM;

  addrinfo * res = nullptr;
  const int gai_err = getaddrinfo(host.c_str(), std::to_string(port).c_str(), &hints, &res);
  if (gai_err != 0 || res == nullptr) {
    last_error_ = std::string("getaddrinfo failed: ") + gai_strerror(gai_err);
    return false;
  }

  bool connected = false;
  do {
    const int fd = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (fd < 0) {
      last_error_ = std::string("socket() failed: ") + std::strerror(errno);
      break;
    }

    if (::connect(fd, res->ai_addr, res->ai_addrlen) == 0) {
      int one = 1;
      setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
      fd_ = fd;
      connected = true;
      break;
    }
    last_error_ = std::string("connect() failed: ") + std::strerror(errno);
    ::close(fd);

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  } while (std::chrono::steady_clock::now() < deadline);

  freeaddrinfo(res);
  return connected;
}

namespace
{
bool send_all(int fd, const void * buf, size_t len)
{
  const char * p = static_cast<const char *>(buf);
  size_t sent = 0;
  while (sent < len) {
    const ssize_t n = ::send(fd, p + sent, len - sent, 0);
    if (n <= 0) {
      return false;
    }
    sent += static_cast<size_t>(n);
  }
  return true;
}

bool recv_all(int fd, void * buf, size_t len)
{
  char * p = static_cast<char *>(buf);
  size_t received = 0;
  while (received < len) {
    const ssize_t n = ::recv(fd, p + received, len - received, 0);
    if (n <= 0) {
      return false;
    }
    received += static_cast<size_t>(n);
  }
  return true;
}
}  // namespace

bool BridgeClient::exchange(
  uint8_t op, const std::array<double, 6> & target_pos_rad, BridgeResponse & out)
{
  if (fd_ < 0) {
    last_error_ = "not connected";
    return false;
  }

  BridgeRequest req{};
  req.op = op;
  for (size_t i = 0; i < 6; ++i) {
    req.target_pos_rad[i] = target_pos_rad[i];
  }

  if (!send_all(fd_, &req, sizeof(req)) || !recv_all(fd_, &out, sizeof(out))) {
    last_error_ = std::string("socket I/O failed: ") + std::strerror(errno);
    close();
    return false;
  }

  return true;
}

}  // namespace parol6_hardware_interface
