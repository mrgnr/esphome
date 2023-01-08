#include "as7341.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace dps310 {

static const char *const TAG = "as7341";

void AS7341Component::setup() {
    ESP_LOGCONFIG(TAG, "Setting up AS7341...");
}

void AS7341Component::update() {
    ESP_LOGCONFIG(TAG, "Update AS7341...");
    publish_state(321.0);
}

} // namespace as7341
} // namespace esphome
