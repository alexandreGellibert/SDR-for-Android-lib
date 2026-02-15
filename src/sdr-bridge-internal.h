#ifndef SDR_PROTRACK_SDR_BRIDGE_INTERNAL_H
#define SDR_PROTRACK_SDR_BRIDGE_INTERNAL_H

#include <atomic>
#include "jni.h"

namespace sdr_bridge_internal {

    extern std::atomic<bool> isCenterFrequencyChanged;
    extern JavaVM *gJavaVM;
}   // namespace sdr_bridge_internal

#endif //SDR_PROTRACK_SDR_BRIDGE_INTERNAL_H
