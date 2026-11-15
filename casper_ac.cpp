#include "casper_ac.h"

#include "esphome/components/climate/climate.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/helpers.h"
#include "esphome/core/version.h"
#include "esphome/core/log.h"

#ifndef USE_ARDUINO
using String = std::string;
#define F(string_literal) (string_literal)
#endif

namespace esphome {
namespace casper_ac {

static const char *TAG = "casper_ac";

// ============================================================
// CONSTANTS
// ============================================================
class Constants {
 public:
  static constexpr const char *AC_FIRMWARE_VERSION = "1.0.0";
  static constexpr const char *MUTE = "mute";
  static constexpr const char *TURBO = "turbo";
  static constexpr const char *CLEAN = "Clean";
  static constexpr const char *HEALTH = "Health";
  static constexpr const char *ANTIFUNGUS = "Antifungus";
  static constexpr const float AC_MIN_TEMPERATURE = 16.0;
  static constexpr const float AC_MAX_TEMPERATURE = 32.0;
  static constexpr const float AC_TEMPERATURE_STEP = 0.5;
  static constexpr const uint8_t AC_MIN_INVERTER_POWER_LIMIT = 0;
  static constexpr const uint8_t AC_MAX_INVERTER_POWER_LIMIT = 100;
  static constexpr const uint32_t AC_STATES_REQUEST_INTERVAL = 7000;
  static constexpr const uint32_t AC_PACKET_TIMEOUT_MAX = 1000;
  static constexpr const uint32_t AC_PACKET_TIMEOUT_MIN = 100;
};

// ============================================================
// PACKET STRUCTURES (AUX/CASPER PROTOCOL)
// ============================================================
#define AC_HEADER_SIZE 8
#define AC_BUFFER_SIZE 35

#define AC_PTYPE_PING 0x01
#define AC_PTYPE_CMD 0x06
#define AC_PTYPE_INFO 0x07
#define AC_PTYPE_INIT 0x09
#define AC_PTYPE_UNKN 0x0b

#define AC_CMD_SET_PARAMS 0x01
#define AC_CMD_STATUS_SMALL 0x11
#define AC_CMD_STATUS_BIG 0x21
#define AC_CMD_STATUS_PERIODIC 0x2C

#define AC_PACKET_START_BYTE 0xBB
#define AC_PACKET_ANSWER 0x80

struct packet_header_t {
  uint8_t start_byte = AC_PACKET_START_BYTE;
  uint8_t _unknown1;
  uint8_t packet_type;
  uint8_t wifi;
  uint8_t ping_answer_01;
  uint8_t _unknown2;
  uint8_t body_length;
  uint8_t _unknown3;
};

union packet_crc_t {
  uint16_t crc16;
  uint8_t crc[2];
};

struct packet_t {
  uint32_t msec;
  packet_header_t *header;
  packet_crc_t *crc;
  uint8_t *body;
  uint8_t bytesLoaded;
  uint8_t data[AC_BUFFER_SIZE];
};

// ============================================================
// COMMAND ENUMS (subset, extend as needed)
// ============================================================
#define AC_POWER_MASK 0b00100000
enum ac_power : uint8_t { AC_POWER_OFF = 0x00, AC_POWER_ON = 0x20, AC_POWER_UNTOUCHED = 0xFF };

#define AC_MODE_MASK 0b11100000
enum ac_mode : uint8_t {
  AC_MODE_AUTO = 0x00,
  AC_MODE_COOL = 0x20,
  AC_MODE_DRY = 0x40,
  AC_MODE_HEAT = 0x80,
  AC_MODE_FAN = 0xC0,
  AC_MODE_UNTOUCHED = 0xFF
};

#define AC_FANSPEED_MASK 0b11100000
enum ac_fanspeed : uint8_t {
  AC_FANSPEED_HIGH = 0x20,
  AC_FANSPEED_MEDIUM = 0x40,
  AC_FANSPEED_LOW = 0x60,
  AC_FANSPEED_AUTO = 0xA0,
  AC_FANSPEED_UNTOUCHED = 0xFF
};

#define AC_LOUVERV_MASK 0b00000111
enum ac_louver_V : uint8_t {
  AC_LOUVERV_SWING_UPDOWN = 0x00,
  AC_LOUVERV_SWING_TOP = 0x01,
  AC_LOUVERV_SWING_MIDDLE_ABOVE = 0x02,
  AC_LOUVERV_SWING_MIDDLE = 0x03,
  AC_LOUVERV_SWING_MIDDLE_BELOW = 0x04,
  AC_LOUVERV_SWING_BOTTOM = 0x05,
  AC_LOUVERV_OFF = 0x07,
  AC_LOUVERV_UNTOUCHED = 0xFF
};

enum ac_vlouver_frontend : uint8_t {
  AC_VLOUVER_FRONTEND_SWING = 0x00,
  AC_VLOUVER_FRONTEND_STOP = 0x01,
  AC_VLOUVER_FRONTEND_TOP = 0x02,
  AC_VLOUVER_FRONTEND_MIDDLE_ABOVE = 0x03,
  AC_VLOUVER_FRONTEND_MIDDLE = 0x04,
  AC_VLOUVER_FRONTEND_MIDDLE_BELOW = 0x05,
  AC_VLOUVER_FRONTEND_BOTTOM = 0x06,
};

// ============================================================
// CORE IMPLEMENTATION
// ============================================================
void CasperAC::initAC(esphome::uart::UARTComponent *parent) {
  _ac_serial = parent;
  _hw_initialized = (_ac_serial != nullptr);
  _has_connection = false;
  _packet_timeout = Constants::AC_PACKET_TIMEOUT_MIN;
  _dataMillis = millis();
  _startupSequenceComplete = false;
}

esphome::climate::ClimateTraits CasperAC::traits() {
  auto traits = esphome::climate::ClimateTraits();
  traits.set_supports_current_temperature(true);
  traits.set_visual_min_temperature(Constants::AC_MIN_TEMPERATURE);
  traits.set_visual_max_temperature(Constants::AC_MAX_TEMPERATURE);
  traits.set_visual_temperature_step(Constants::AC_TEMPERATURE_STEP);
  traits.set_supported_modes(this->_supported_modes);
  traits.set_supported_swing_modes(this->_supported_swing_modes);
  traits.set_supported_presets(this->_supported_presets);
  traits.set_supported_custom_presets(this->_supported_custom_presets);
  traits.set_supported_custom_fan_modes(this->_supported_custom_fan_modes);
  return traits;
}

void CasperAC::dump_config() {
  ESP_LOGCONFIG(TAG, "Casper AC UART:");
  ESP_LOGCONFIG(TAG, "  Firmware version: %s", Constants::AC_FIRMWARE_VERSION);
  ESP_LOGCONFIG(TAG, "  Period: %u ms", this->get_period());
  ESP_LOGCONFIG(TAG, "  Show action: %s", TRUEFALSE(this->get_show_action()));
  ESP_LOGCONFIG(TAG, "  Display inverted: %s", TRUEFALSE(this->get_display_inverted()));
  ESP_LOGCONFIG(TAG, "  Packet timeout: %u ms", this->get_packet_timeout());
}

void CasperAC::setup() {
  _traits.add_feature_flags(esphome::climate::CLIMATE_SUPPORTS_CURRENT_TEMPERATURE);
  _traits.add_supported_mode(esphome::climate::CLIMATE_MODE_OFF);
  _traits.add_supported_mode(esphome::climate::CLIMATE_MODE_FAN_ONLY);
  _traits.add_supported_fan_mode(esphome::climate::CLIMATE_FAN_AUTO);
  _traits.add_supported_fan_mode(esphome::climate::CLIMATE_FAN_LOW);
  _traits.add_supported_fan_mode(esphome::climate::CLIMATE_FAN_MEDIUM);
  _traits.add_supported_fan_mode(esphome::climate::CLIMATE_FAN_HIGH);
  _traits.add_supported_swing_mode(esphome::climate::CLIMATE_SWING_OFF);
  _traits.add_supported_preset(esphome::climate::CLIMATE_PRESET_NONE);
  if (this->_show_action) {
    _traits.add_feature_flags(esphome::climate::CLIMATE_SUPPORTS_ACTION);
  }
}

void CasperAC::loop() {
  if (!get_hw_initialized()) return;

  if ((millis() - _dataMillis) > _update_period) {
    _dataMillis = millis();
    // Poll status from AC here if needed
  }
}

void CasperAC::control(const esphome::climate::ClimateCall &call) {
  // TODO: Implement command sending to AC via UART
  if (call.get_mode().has_value()) {
    ESP_LOGI(TAG, "Mode change requested");
  }
  if (call.get_target_temperature().has_value()) {
    ESP_LOGI(TAG, "Temperature change requested: %.1f", *call.get_target_temperature());
  }
  this->publish_all_states();
}

void CasperAC::publish_all_states() {
  this->publish_state();
}

// ============================================================
// ACTIONS
// ============================================================
bool CasperAC::displayOffSequence() {
  ESP_LOGI(TAG, "Display OFF");
  return true;
}
bool CasperAC::displayOnSequence() {
  ESP_LOGI(TAG, "Display ON");
  return true;
}
bool CasperAC::setVLouverSwingSequence() { ESP_LOGI(TAG, "VLouver Swing"); return true; }
bool CasperAC::setVLouverStopSequence() { ESP_LOGI(TAG, "VLouver Stop"); return true; }
bool CasperAC::setVLouverTopSequence() { ESP_LOGI(TAG, "VLouver Top"); return true; }
bool CasperAC::setVLouverMiddleAboveSequence() { ESP_LOGI(TAG, "VLouver Middle Above"); return true; }
bool CasperAC::setVLouverMiddleSequence() { ESP_LOGI(TAG, "VLouver Middle"); return true; }
bool CasperAC::setVLouverMiddleBelowSequence() { ESP_LOGI(TAG, "VLouver Middle Below"); return true; }
bool CasperAC::setVLouverBottomSequence() { ESP_LOGI(TAG, "VLouver Bottom"); return true; }

bool CasperAC::setVLouverFrontendSequence(ac_vlouver_frontend vLouver) {
  ESP_LOGI(TAG, "VLouver Frontend: %u", static_cast<uint8_t>(vLouver));
  return true;
}

bool CasperAC::setVLouverSequence(ac_louver_V vLouver) {
  ESP_LOGI(TAG, "VLouver: 0x%02X", static_cast<uint8_t>(vLouver));
  return true;
}

bool CasperAC::powerLimitationOffSequence() {
  ESP_LOGI(TAG, "Power Limit OFF");
  return true;
}
bool CasperAC::powerLimitationOnSequence() {
  ESP_LOGI(TAG, "Power Limit ON");
  return true;
}
bool CasperAC::powerLimitationOnSequence(uint8_t power_limit) {
  ESP_LOGI(TAG, "Power Limit ON: %u", power_limit);
  return true;
}
bool CasperAC::powerLimitationSetSequence(uint8_t power_limit, bool set_on) {
  ESP_LOGI(TAG, "Power Limit Set: %u (on=%d)", power_limit, set_on);
  return true;
}

bool CasperAC::sendTestPacket(const std::vector<uint8_t> &data) {
  ESP_LOGI(TAG, "Test Packet: %zu bytes", data.size());
  return true;
}

void CasperAC::set_packet_timeout(uint32_t ms) {
  if (ms < Constants::AC_PACKET_TIMEOUT_MIN) ms = Constants::AC_PACKET_TIMEOUT_MIN;
  if (ms > Constants::AC_PACKET_TIMEOUT_MAX) ms = Constants::AC_PACKET_TIMEOUT_MIN;
  this->_packet_timeout = ms;
}

// ============================================================
// ESPHome external component registration
// ============================================================
// Note: For ESPHome external_components, you must add the platform registration
// code here that matches your ESPHome version. The YAML `platform: casper_ac`
// requires ESPHome to know how to construct CasperAC from config.

}  // namespace casper_ac
}  // namespace esphome
