// RC frame intake: source arbitration and per-second link statistics.
// The rover follows one source; another takes over only after the current
// one has been silent for failsafe_ms. Loss comes from seq gaps, latency from
// the sender's t_us against the rover's own clock (both krono-synced).
#pragma once

#include <cstdint>
#include <vector>

#include "protocol.hpp"

namespace rover_sample
{

class rc_link
{
   public:
    // now_us: monotonic; now_rt_us: CLOCK_REALTIME low 32 bits. False when
    // the frame is from a source the rover is not following.
    bool accept(const rc_frame& rc, int64_t now_us, uint32_t now_rt_us, uint32_t failsafe_ms);
    // Closes the statistics window once a second.
    void tick(int64_t now_us);

    bool ever() const
    {
        return have_;
    }
    const rc_frame& last() const
    {
        return last_;
    }
    int64_t last_us() const
    {
        return last_us_;
    }
    uint8_t source() const
    {
        return have_ ? last_.src : 0;
    }
    // Last full window.
    uint16_t loss_permille() const
    {
        return loss_;
    }
    uint16_t latency_p50_ms() const
    {
        return lat_p50_;
    }
    uint32_t frames() const
    {
        return frames_;
    }
    uint32_t rejected() const
    {
        return rejected_;
    }

   private:
    bool have_ = false;
    rc_frame last_;
    int64_t last_us_ = 0;
    // window
    int64_t win_start_us_ = 0;
    uint32_t win_rx_ = 0;
    uint32_t win_span_ = 0;
    std::vector<int32_t> win_lat_;
    uint16_t loss_ = 0;
    uint16_t lat_p50_ = 0;
    uint32_t frames_ = 0;
    uint32_t rejected_ = 0;
};

}  // namespace rover_sample
