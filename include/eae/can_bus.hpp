#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>

namespace eae {

/**
 * A single CAN 2.0B data frame.
 *
 * The PV450 display specifies two CAN 2.0B ports, which support both 11-bit
 * standard and 29-bit extended identifiers. This system has only two nodes
 * on the bus, the PLC and the display, so there is no need to conform to a
 * higher-level standard such as J1939. An application-specific 11-bit set is
 * sufficient and simpler to reason about.
 */
struct CanFrame {
    uint32_t id = 0;             // 11-bit identifier.
    uint8_t data[8] = {};        // Payload bytes.
    uint8_t dlc = 0;             // Number of valid payload bytes, 0 to 8.
};

/**
 * Interface to a CAN bus.
 *
 * The control code depends on this rather than on a concrete bus, so the same
 * logic can run against the simulation used here or against a real driver
 * later without being rewritten.
 */
class ICanBus {
public:
    virtual ~ICanBus() = default;

    /** Transmit a frame onto the bus. */
    virtual void send(const CanFrame& frame) = 0;

    /** Take the next received frame, if one is waiting. */
    virtual std::optional<CanFrame> receive() = 0;
};

/**
 * In-memory CAN bus simulation.
 *
 * Transmitted frames are kept so a test or a log can inspect what the
 * controller put on the wire. Frames from the other node are introduced with
 * inject(), which keeps the two directions clearly separated rather than
 * looping transmissions straight back.
 */
class SimulatedCanBus : public ICanBus {
public:
    /** Queue depth, after which the oldest frame is dropped. */
    static constexpr std::size_t kQueueLimit = 64;

    void send(const CanFrame& frame) override;
    std::optional<CanFrame> receive() override;

    /** Simulate a frame arriving from another node on the bus. */
    void inject(const CanFrame& frame);

    /** Take the oldest transmitted frame, for inspection. */
    std::optional<CanFrame> pop_transmitted();

    std::size_t transmitted_count() const { return tx_.size(); }
    std::size_t pending_count() const { return rx_.size(); }

    /** Frames discarded because a queue was full. */
    std::size_t dropped_count() const { return dropped_; }

private:
    std::deque<CanFrame> tx_;
    std::deque<CanFrame> rx_;
    std::size_t dropped_ = 0;
};

}  // namespace eae
