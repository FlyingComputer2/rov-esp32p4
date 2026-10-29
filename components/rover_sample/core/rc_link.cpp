#include "rc_link.hpp"

#include <algorithm>

namespace rover_sample
{
namespace
{
constexpr size_t k_lat_samples = 64;
}

bool rc_link::accept(const rc_frame& rc, int64_t now_us, uint32_t now_rt_us, uint32_t failsafe_ms)
{
    if (have_ && rc.src != last_.src && now_us - last_us_ < int64_t(failsafe_ms) * 1000)
    {
        ++rejected_;
        return false;
    }
    if (have_ && rc.src == last_.src)
    {
        const uint16_t d = static_cast<uint16_t>(rc.seq - last_.seq);
        if (d == 0 || d > 0x8000)
        {
            ++rejected_;  // duplicate or reordered
            return false;
        }
        win_span_ += d;
    }
    else
    {
        win_span_ += 1;
    }
    ++win_rx_;
    ++frames_;
    if (win_lat_.size() < k_lat_samples)
    {
        win_lat_.push_back(static_cast<int32_t>(now_rt_us - rc.t_us));
    }
    have_ = true;
    last_ = rc;
    last_us_ = now_us;
    return true;
}

void rc_link::tick(int64_t now_us)
{
    if (now_us - win_start_us_ < 1000000)
    {
        return;
    }
    win_start_us_ = now_us;
    loss_ = win_span_ ? static_cast<uint16_t>(1000 - 1000ull * win_rx_ / win_span_) : 0;
    if (win_span_ == 0 && have_)
    {
        loss_ = 1000;  // nothing arrived in a whole second
    }
    if (!win_lat_.empty())
    {
        auto mid = win_lat_.begin() + win_lat_.size() / 2;
        std::nth_element(win_lat_.begin(), mid, win_lat_.end());
        lat_p50_ = static_cast<uint16_t>(std::min<int32_t>(std::max<int32_t>(*mid / 1000, 0), 65535));
    }
    win_rx_ = win_span_ = 0;
    win_lat_.clear();
}

}  // namespace rover_sample
