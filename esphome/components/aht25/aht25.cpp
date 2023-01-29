// Implementation based on:
//  - AHT25: https://github.com/Thinary/AHT25
//  - Official Datasheet (cn):
//  http://www.aosong.com/userfiles/files/media/aht10%E8%A7%84%E6%A0%BC%E4%B9%A6v1_1%EF%BC%8820191015%EF%BC%89.pdf
//  - Unofficial Translated Datasheet (en):
//  https://wiki.liutyi.info/download/attachments/30507639/Aosong_AHT25_en_draft_0c.pdf
//
// When configured for humidity, the log 'Components should block for at most 20-30ms in loop().' will be generated in
// verbose mode. This is due to technical specs of the sensor and can not be avoided.
//
// According to the datasheet, the component is supposed to respond in more than 75ms. In fact, it can answer almost
// immediately for temperature. But for humidity, it takes >90ms to get a valid data. From experience, we have best
// results making successive requests; the current implementation makes 3 attempts with a delay of 30ms each time.

#include "aht25.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace aht25 {

static const char *const TAG = "aht25";
static const uint8_t AHT25_STATUS_CMD[] = {0x71};

static const uint8_t AHT25_MEASURE_CMD[]   = {0xAC, 0x33, 0x00};

static const uint8_t AHT25_NORMAL_CMD[]    = {0xA8, 0x00, 0x00};
static const uint8_t AHT25_RESET_CMD       = 0xBA;

static const uint8_t AHT25_DEFAULT_DELAY = 5;    // ms, for calibration and temperature measurement
static const uint8_t AHT25_SECONDARY_DELAY = 10;    // ms, for calibration
static const uint8_t AHT25_HUMIDITY_DELAY = 30;  // ms
static const uint8_t AHT25_ATTEMPTS = 3;         // safety margin, normally 3 attempts are enough: 3*30=90ms

uint8_t AHT25Component::reset_REG(uint8_t addr) {

  uint8_t TMP_DATATOSEND[] = {0x00, 0x00, 0x00, 0x00};
  uint8_t READ_DATA[] = {0x00, 0x00, 0x00};

  //build the new data to send
  TMP_DATATOSEND = {0x71, addr, 0x00, 0x00};

  if (!this->write_bytes(0, TMP_DATATOSEND, sizeof(TMP_DATATOSEND))) {
    ESP_LOGE(TAG, "Communication with AHT25 failed! - RR1");
    this->mark_failed();
    return(0);
  }

  delay(AHT25_DEFAULT_DELAY);

  if (!this->write_bytes(0, AHT25_STATUS_CMD, sizeof(AHT25_STATUS_CMD))) {
    ESP_LOGE(TAG, "Communication with AHT25 failed! - RR2");
    this->mark_failed();
    return(0);
  }

  delay(AHT25_DEFAULT_DELAY);

  if (this->read(READ_DATA, 3) != i2c::ERROR_OK) {
    ESP_LOGE(TAG, "Communication with AHT25 failed! - RR3");
    this->mark_failed();
    return(0);
  }

  delay(AHT25_SECONDARY_DELAY);

  //build the new data to send
  TMP_DATATOSEND = {0x71, (0xB0|addr), READ_DATA[1], READ_DATA[2]};

  if (!this->write_bytes(0, TMP_DATATOSEND, sizeof(TMP_DATATOSEND))) {
    ESP_LOGE(TAG, "Communication with AHT25 failed! - RR4");
    this->mark_failed();
    return(0);
  }

  //if we got here we must be good
  return(1);
}

void AHT25Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up AHT25...");

  if (!this->write_bytes(0, AHT25_STATUS_CMD, sizeof(AHT25_STATUS_CMD))) {
    ESP_LOGE(TAG, "Communication with AHT25 failed! - S1");
    this->mark_failed();
    return;
  }
  uint8_t data = 0;
  if (this->write(&data, 1) != i2c::ERROR_OK) {
    ESP_LOGD(TAG, "Communication with AHT25 failed! - S2");
    this->mark_failed();
    return;
  }
  delay(AHT25_DEFAULT_DELAY);
  if (this->read(&data, 1) != i2c::ERROR_OK) {
    ESP_LOGD(TAG, "Communication with AHT25 failed! - S3");
    this->mark_failed();
    return;
  }
  if (this->read(&data, 1) != i2c::ERROR_OK) {
    ESP_LOGD(TAG, "Communication with AHT25 failed! - S4");
    this->mark_failed();
    return;
  }
  if ((data & 0x18) != 0x18) {  // Bit[6:5] = 0b00, NORMAL mode and Bit[3] = 0b1, CALIBRATED
    ESP_LOGE(TAG, "AHT25 status check failed!, Re-initialising");

    if(! reset_REG(0x1B) ){
      ESP_LOGD(TAG, "Communication with AHT25 failed! - S5");
      this->mark_failed();
      return;
    }

    if(! reset_REG(0x1C) ){
      ESP_LOGD(TAG, "Communication with AHT25 failed! - S6");
      this->mark_failed();
      return;
    }

    if(! reset_REG(0x1E) ){
      ESP_LOGD(TAG, "Communication with AHT25 failed! - S7");
      this->mark_failed();
      return;
    }
  }

  ESP_LOGV(TAG, "AHT25 calibrated");
}

void AHT25Component::update() {
  if (!this->write_bytes(0, AHT25_MEASURE_CMD, sizeof(AHT25_MEASURE_CMD))) {
    ESP_LOGE(TAG, "Communication with AHT25 failed!");
    this->status_set_warning();
    return;
  }
  uint8_t data[6];
  uint8_t delay_ms = AHT25_DEFAULT_DELAY;
  if (this->humidity_sensor_ != nullptr)
    delay_ms = AHT25_HUMIDITY_DELAY;
  bool success = false;
  for (int i = 0; i < AHT25_ATTEMPTS; ++i) {
    ESP_LOGVV(TAG, "Attempt %d at %6u", i, millis());
    delay(delay_ms);
    if (this->read(data, 6) != i2c::ERROR_OK) {
      ESP_LOGD(TAG, "Communication with AHT25 failed, waiting...");
      continue;
    }

    if ((data[0] & 0x80) == 0x80) {  // Bit[7] = 0b1, device is busy
      ESP_LOGD(TAG, "AHT25 is busy, waiting...");
    } else if (data[1] == 0x0 && data[2] == 0x0 && (data[3] >> 4) == 0x0) {
      // Unrealistic humidity (0x0)
      if (this->humidity_sensor_ == nullptr) {
        ESP_LOGVV(TAG, "ATH10 Unrealistic humidity (0x0), but humidity is not required");
        break;
      } else {
        ESP_LOGD(TAG, "ATH10 Unrealistic humidity (0x0), retrying...");
        if (!this->write_bytes(0, AHT25_MEASURE_CMD, sizeof(AHT25_MEASURE_CMD))) {
          ESP_LOGE(TAG, "Communication with AHT25 failed!");
          this->status_set_warning();
          return;
        }
      }
    } else {
      // data is valid, we can break the loop
      ESP_LOGVV(TAG, "Answer at %6u", millis());
      success = true;
      break;
    }
  }
  if (!success || (data[0] & 0x80) == 0x80) {
    ESP_LOGE(TAG, "Measurements reading timed-out!");
    this->status_set_warning();
    return;
  }

  uint32_t raw_temperature = ((data[3] & 0x0F) << 16) | (data[4] << 8) | data[5];
  uint32_t raw_humidity = ((data[1] << 16) | (data[2] << 8) | data[3]) >> 4;

  LOG_SENSOR("  ", "GOT RAW TEMP VAL = ", raw_temperature)

  float temperature = ((200.0f * (float) raw_temperature) / 1048576.0f) - 50.0f;
  float humidity;
  if (raw_humidity == 0) {  // unrealistic value
    humidity = NAN;
  } else {
    humidity = (float) raw_humidity * 100.0f / 1048576.0f;
  }

  if (this->temperature_sensor_ != nullptr) {
    this->temperature_sensor_->publish_state(temperature);
  }
  if (this->humidity_sensor_ != nullptr) {
    if (std::isnan(humidity)) {
      ESP_LOGW(TAG, "Invalid humidity! Sensor reported 0%% Hum");
    }
    this->humidity_sensor_->publish_state(humidity);
  }
  this->status_clear_warning();
}

float AHT25Component::get_setup_priority() const { return setup_priority::DATA; }

void AHT25Component::dump_config() {
  ESP_LOGCONFIG(TAG, "AHT25:");
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with AHT25 failed!");
  }
  LOG_SENSOR("  ", "Temperature", this->temperature_sensor_);
  LOG_SENSOR("  ", "Humidity", this->humidity_sensor_);
}

}  // namespace aht25
}  // namespace esphome
