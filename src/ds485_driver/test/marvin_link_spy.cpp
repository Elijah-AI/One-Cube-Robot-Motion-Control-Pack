#include "ds485_driver/marvin_rs485_port.h"
#include <stdexcept>
bool MarvinLink::connected_ = false;
bool MarvinLink::owner_ = false;
bool MarvinLink::Connect(const std::string &, std::string *) {
  throw std::runtime_error("offline test must not connect SDK");
}
void MarvinLink::Attach() {
  connected_ = true;
  owner_ = false;
}
void MarvinLink::Release() {
  if (owner_)
    throw std::runtime_error("test unexpectedly owns SDK");
  connected_ = false;
}
bool MarvinLink::IsConnected() {
  return connected_;
}
MarvinRs485Port::MarvinRs485Port(char arm, int channel)
    : arm_(arm), channel_(channel), sent_packets_(0), received_packets_(0), dropped_packets_(0) {}
bool MarvinRs485Port::Send(const uint8_t *, int) {
  throw std::runtime_error("unexpected real transport");
}
bool MarvinRs485Port::SendBefore(const uint8_t* data, int size, double deadline) {
  return NowMs() < deadline && Send(data, size);
}
int MarvinRs485Port::Receive(uint8_t *, int, int) {
  return 0;
}
void MarvinRs485Port::Clear() {}
void MarvinRs485Port::SleepMs(int) {}
std::string MarvinRs485Port::Name() const {
  return "SDK-spy";
}
std::string MarvinRs485Port::LastError() const {
  return "offline";
}
