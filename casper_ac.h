#pragma once

#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "esphome/components/uart/uart.h"

namespace esphome {
namespace casper_ac {

// Forward declaration
class CasperAC;

// **************************************** DISPLAY ACTIONS ****************************************
template <typename... Ts>
class AirConDisplayOffAction : public Action<Ts...> {
 public:
  explicit AirConDisplayOffAction(CasperAC *ac) : ac_(ac) {}
  void play(const Ts &...x) override { this->ac_->displayOffSequence(); }
 protected:
  CasperAC *ac_;
};

template <typename... Ts>
class AirConDisplayOnAction : public Action<Ts...> {
 public:
  explicit AirConDisplayOnAction(CasperAC *ac) : ac_(ac) {}
  void play(const Ts &...x) override { this->ac_->displayOnSequence(); }
 protected:
  CasperAC *ac_;
};

// **************************************** VERTICAL LOUVER ACTIONS ****************************************
template <typename... Ts>
class AirConVLouverSwingAction : public Action<Ts...> {
 public:
  explicit AirConVLouverSwingAction(CasperAC *ac) : ac_(ac) {}
  void play(const Ts &...x) override { this->ac_->setVLouverSwingSequence(); }
 protected:
  CasperAC *ac_;
};

template <typename... Ts>
class AirConVLouverStopAction : public Action<Ts...> {
 public:
  explicit AirConVLouverStopAction(CasperAC *ac) : ac_(ac) {}
  void play(const Ts &...x) override { this->ac_->setVLouverStopSequence(); }
 protected:
  CasperAC *ac_;
};

template <typename... Ts>
class AirConVLouverTopAction : public Action<Ts...> {
 public:
  explicit AirConVLouverTopAction(CasperAC *ac) : ac_(ac) {}
  void play(const Ts &...x) override { this->ac_->setVLouverTopSequence(); }
 protected:
  CasperAC *ac_;
};

template <typename... Ts>
class AirConVLouverMiddleAboveAction : public Action<Ts...> {
 public:
  explicit AirConVLouverMiddleAboveAction(CasperAC *ac) : ac_(ac) {}
  void play(const Ts &...x) override { this->ac_->setVLouverMiddleAboveSequence(); }
 protected:
  CasperAC *ac_;
};

template <typename... Ts>
class AirConVLouverMiddleAction : public Action<Ts...> {
 public:
  explicit AirConVLouverMiddleAction(CasperAC *ac) : ac_(ac) {}
  void play(const Ts &...x) override { this->ac_->setVLouverMiddleSequence(); }
 protected:
  CasperAC *ac_;
};

template <typename... Ts>
class AirConVLouverMiddleBelowAction : public Action<Ts...> {
 public:
  explicit AirConVLouverMiddleBelowAction(CasperAC *ac) : ac_(ac) {}
  void play(const Ts &...x) override { this->ac_->setVLouverMiddleBelowSequence(); }
 protected:
  CasperAC *ac_;
};

template <typename... Ts>
class AirConVLouverBottomAction : public Action<Ts...> {
 public:
  explicit AirConVLouverBottomAction(CasperAC *ac) : ac_(ac) {}
  void play(const Ts &...x) override { this->ac_->setVLouverBottomSequence(); }
 protected:
  CasperAC *ac_;
};

template <typename... Ts>
class AirConVLouverSetAction : public Action<Ts...> {
 public:
  TEMPLATABLE_VALUE(uint8_t, value);
  explicit AirConVLouverSetAction(CasperAC *ac) : ac_(ac) {}
  void play(const Ts &...x) override {
    uint8_t vlpos = this->value_.value(x...);
    this->ac_->setVLouverFrontendSequence(static_cast<ac_vlouver_frontend>(vlpos));
  }
 protected:
  CasperAC *ac_;
};

// **************************************** SEND TEST PACKET ACTION ****************************************
template <typename... Ts>
class AirConSendTestPacketAction : public Action<Ts...> {
 public:
  explicit AirConSendTestPacketAction(CasperAC *ac) : ac_(ac) {}
  void set_data_template(std::function<std::vector<uint8_t>(Ts...)> func) {
    this->data_func_ = func;
    this->static_ = false;
  }
  void set_data_static(const std::vector<uint8_t> &data) {
    this->data_static_ = data;
    this->static_ = true;
  }
  void play(const Ts &...x) override {
    if (this->static_) {
      this->ac_->sendTestPacket(this->data_static_);
    } else {
      auto val = this->data_func_(x...);
      this->ac_->sendTestPacket(val);
    }
  }
 protected:
  CasperAC *ac_;
  bool static_{false};
  std::function<std::vector<uint8_t>(Ts...)> data_func_{};
  std::vector<uint8_t> data_static_{};
};

// **************************************** POWER LIMITATION ACTIONS ****************************************
template <typename... Ts>
class AirConPowerLimitationOffAction : public Action<Ts...> {
 public:
  explicit AirConPowerLimitationOffAction(CasperAC *ac) : ac_(ac) {}
  void play(const Ts &...x) override { this->ac_->powerLimitationOffSequence(); }
 protected:
  CasperAC *ac_;
};

template <typename... Ts>
class AirConPowerLimitationOnAction : public Action<Ts...> {
 public:
  TEMPLATABLE_VALUE(uint8_t, value);
  explicit AirConPowerLimitationOnAction(CasperAC *ac) : ac_(ac) {}
  void play(const Ts &...x) override {
    this->pwr_lim_ = this->value_.value(x...);
    this->ac_->powerLimitationOnSequence(this->pwr_lim_);
  }
 protected:
  CasperAC *ac_;
  uint8_t pwr_lim_;
};

// **************************************** MAIN CLASS ****************************************
class CasperAC : public esphome::Component, public esphome::climate::Climate {
 public:
  void initAC(esphome::uart::UARTComponent *parent = nullptr);
  float get_setup_priority() const override { return esphome::setup_priority::DATA; }

  // Sensor setters
  void set_indoor_temperature_sensor(esphome::sensor::Sensor *s) { sensor_indoor_temperature_ = s; }
  void set_outdoor_temperature_sensor(esphome::sensor::Sensor *s) { sensor_outdoor_temperature_ = s; }
  void set_inbound_temperature_sensor(esphome::sensor::Sensor *s) { sensor_inbound_temperature_ = s; }
  void set_outbound_temperature_sensor(esphome::sensor::Sensor *s) { sensor_outbound_temperature_ = s; }
  void set_compressor_temperature_sensor(esphome::sensor::Sensor *s) { sensor_compressor_temperature_ = s; }
  void set_vlouver_state_sensor(esphome::sensor::Sensor *s) { sensor_vlouver_state_ = s; }
  void set_defrost_state(esphome::binary_sensor::BinarySensor *s) { sensor_defrost_ = s; }
  void set_display_sensor(esphome::binary_sensor::BinarySensor *s) { sensor_display_ = s; }
  void set_inverter_power_sensor(esphome::sensor::Sensor *s) { sensor_inverter_power_ = s; }
  void set_preset_reporter_sensor(esphome::text_sensor::TextSensor *s) { sensor_preset_reporter_ = s; }
  void set_inverter_power_limit_value_sensor(esphome::sensor::Sensor *s) { sensor_inverter_power_limit_value_ = s; }
  void set_inverter_power_limit_state_sensor(esphome::binary_sensor::BinarySensor *s) { sensor_inverter_power_limit_state_ = s; }

  bool get_hw_initialized() { return _hw_initialized; }
  bool get_has_connection() { return _has_connection; }

  // Actions exposed to YAML
  bool displayOffSequence();
  bool displayOnSequence();
  bool setVLouverSwingSequence();
  bool setVLouverStopSequence();
  bool setVLouverTopSequence();
  bool setVLouverMiddleAboveSequence();
  bool setVLouverMiddleSequence();
  bool setVLouverMiddleBelowSequence();
  bool setVLouverBottomSequence();
  bool setVLouverFrontendSequence(ac_vlouver_frontend vLouver);
  bool setVLouverSequence(ac_louver_V vLouver);
  bool powerLimitationOffSequence();
  bool powerLimitationOnSequence();
  bool powerLimitationOnSequence(uint8_t power_limit);
  bool powerLimitationSetSequence(uint8_t power_limit, bool set_on = false);
  bool sendTestPacket(const std::vector<uint8_t> &data);

  // Configuration setters
  void set_period(uint32_t ms) { _update_period = ms; }
  uint32_t get_period() { return _update_period; }
  void set_show_action(bool show_action) { _show_action = show_action; }
  bool get_show_action() { return _show_action; }
  void set_display_inverted(bool display_inverted) { _display_inverted = display_inverted; }
  bool get_display_inverted() { return _display_inverted; }
  void set_packet_timeout(uint32_t ms);
  uint32_t get_packet_timeout() { return _packet_timeout; }
  void set_optimistic(bool optimistic) { _optimistic = optimistic; }
  bool get_optimistic() { return _optimistic; }

  void set_supported_modes(esphome::climate::ClimateModeMask modes) { _supported_modes = modes; }
  void set_supported_swing_modes(esphome::climate::ClimateSwingModeMask modes) { _supported_swing_modes = modes; }
  void set_supported_presets(esphome::climate::ClimatePresetMask presets) { _supported_presets = presets; }
  void set_custom_presets(std::initializer_list<const char *> presets) { _supported_custom_presets = presets; }
  void set_custom_fan_modes(std::initializer_list<const char *> modes) { _supported_custom_fan_modes = modes; }

  void dump_config();

 protected:
  // --- Implementation details moved to .cpp ---
  esphome::climate::ClimateTraits traits() override;
  void control(const esphome::climate::ClimateCall &call) override;
  void setup() override;
  void loop() override;
  void publish_all_states();

  esphome::uart::UARTComponent *_ac_serial = nullptr;
  bool _hw_initialized = false;
  bool _has_connection = false;
  bool _show_action = true;
  bool _display_inverted = false;
  bool _optimistic = true;
  uint32_t _packet_timeout = 150;
  uint32_t _dataMillis = 0;
  uint32_t _update_period = 7000;
  bool _is_inverter = false;
  bool _startupSequenceComplete = false;

  esphome::climate::ClimateModeMask _supported_modes{};
  esphome::climate::ClimateSwingModeMask _supported_swing_modes{};
  esphome::climate::ClimatePresetMask _supported_presets{};
  std::vector<const char *> _supported_custom_fan_modes{};
  std::vector<const char *> _supported_custom_presets{};
  esphome::climate::ClimateTraits _traits;

  // Sensors
  esphome::sensor::Sensor *sensor_indoor_temperature_ = nullptr;
  esphome::sensor::Sensor *sensor_outdoor_temperature_ = nullptr;
  esphome::sensor::Sensor *sensor_inbound_temperature_ = nullptr;
  esphome::sensor::Sensor *sensor_outbound_temperature_ = nullptr;
  esphome::sensor::Sensor *sensor_compressor_temperature_ = nullptr;
  esphome::sensor::Sensor *sensor_inverter_power_ = nullptr;
  esphome::sensor::Sensor *sensor_vlouver_state_ = nullptr;
  esphome::sensor::Sensor *sensor_inverter_power_limit_value_ = nullptr;
  esphome::binary_sensor::BinarySensor *sensor_display_ = nullptr;
  esphome::binary_sensor::BinarySensor *sensor_defrost_ = nullptr;
  esphome::binary_sensor::BinarySensor *sensor_inverter_power_limit_state_ = nullptr;
  esphome::text_sensor::TextSensor *sensor_preset_reporter_ = nullptr;
};

}  // namespace casper_ac
}  // namespace esphome
