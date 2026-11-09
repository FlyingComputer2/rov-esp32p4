#pragma once
// rov-esp32p4's bfc config: every bfc user in the firmware (winject-l3,
// vstreamer, the SDIO proxy, the applets) compiles with it.
#include <functional>
#include <memory>

#define BFC_CONFIG_ID "rov-esp32p4"
// ESP-IDF has poll() but no epoll.
#define BFC_CONFIG_REACTOR_POLL 1

namespace bfc::config
{
// Reactor, timer and queue callbacks.
template <typename Signature>
using function = std::function<Signature>;

// Containers, payload buffers and queue slots. std::allocator for now (the
// IDF heap policy, SPIRAM_MALLOC_ALWAYSINTERNAL, decides internal vs PSRAM).
template <typename T>
using allocator = std::allocator<T>;
} // namespace bfc::config
