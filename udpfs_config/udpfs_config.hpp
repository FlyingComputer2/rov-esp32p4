// This firmware's udpfs configuration: wire <-> newlib errno and open flags,
// paired by macro name.
#pragma once

#include "udpfs/posix_map.hpp"

#define UDPFS_CONFIG_ID "rov-esp32p4"

namespace udpfs::config
{
using namespace udpfs::posix_map;
}  // namespace udpfs::config
