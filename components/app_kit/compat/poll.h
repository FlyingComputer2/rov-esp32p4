// bfc's poll_reactor includes <poll.h>; ESP-IDF only ships <sys/poll.h>
// (winject-l3 carries the same shim).
#pragma once
#include <sys/poll.h>
