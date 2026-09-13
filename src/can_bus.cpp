#include "eae/can_bus.hpp"

namespace eae {

namespace {

// Pushes onto a queue, discarding the oldest entry once the limit is reached.
// A real controller has a finite transmit buffer and will drop frames under
// sustained overload; modelling that here means the behaviour is visible in
// testing rather than appearing for the first time on hardware.
void push_bounded(std::deque<CanFrame>& queue,
                  const CanFrame& frame,
                  std::size_t limit,
                  std::size_t& dropped) {
    if (queue.size() >= limit) {
        queue.pop_front();
        ++dropped;
    }
    queue.push_back(frame);
}

}  // namespace

void SimulatedCanBus::send(const CanFrame& frame) {
    push_bounded(tx_, frame, kQueueLimit, dropped_);
}

void SimulatedCanBus::inject(const CanFrame& frame) {
    push_bounded(rx_, frame, kQueueLimit, dropped_);
}

std::optional<CanFrame> SimulatedCanBus::receive() {
    if (rx_.empty()) {
        return std::nullopt;
    }
    const CanFrame frame = rx_.front();
    rx_.pop_front();
    return frame;
}

std::optional<CanFrame> SimulatedCanBus::pop_transmitted() {
    if (tx_.empty()) {
        return std::nullopt;
    }
    const CanFrame frame = tx_.front();
    tx_.pop_front();
    return frame;
}

}  // namespace eae
