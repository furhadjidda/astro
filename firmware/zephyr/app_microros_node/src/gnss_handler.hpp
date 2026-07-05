#pragma once

#include "sensor_state.hpp"

/* Device-access macros — used in both gnss_handler.cpp and node_main.cpp */
#if DT_NODE_HAS_STATUS(DT_ALIAS(gnss), okay)
#define mtk3333_gnss DEVICE_DT_GET(DT_ALIAS(gnss))
extern GnssState mtk3333_state;
#endif

#if DT_NODE_HAS_STATUS(DT_ALIAS(ubloxgnss), okay)
#define ublox_gnss DEVICE_DT_GET(DT_ALIAS(ubloxgnss))
extern GnssState ublox_state;
#endif
