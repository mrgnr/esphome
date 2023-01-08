#include "as7341.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace dps310 {

static const char *const TAG = "as7341";

void AS7341Component::setup() {
    ESP_LOGCONFIG(TAG, "Setting up AS7341...");
}

} // namespace as7341
} // namespace esphome
