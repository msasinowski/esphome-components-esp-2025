#include "wmbus.h"
#include "version.h"
#include "meters.h"
#include "address.h"
#include "esphome/core/application.h"
#include "esphome/core/helpers.h"

#include <algorithm>

namespace esphome {
namespace wmbus {

static const char *const TAG = "wmbus";

void InfoComponent::setup() { return; }

void WMBusComponent::setup() {
  this->high_freq_.start();
  if (this->led_pin_ != nullptr) {
    this->led_pin_->setup();
    this->led_pin_->digital_write(false);
    this->led_on_ = false;
  }
  // Initialize CC1101 radio chip
  if (!rf_mbus_.init(this->spi_conf_.mosi->get_pin(), this->spi_conf_.miso->get_pin(),
                     this->spi_conf_.clk->get_pin(),  this->spi_conf_.cs->get_pin(),
                     this->spi_conf_.gdo0->get_pin(), this->spi_conf_.gdo2->get_pin(),
                     this->frequency_, this->sync_mode_)) {
    this->mark_failed();
    ESP_LOGE(TAG, "CC1101 initialization failed!");
    return;
  }
  ESP_LOGI(TAG, "CC1101 initialized successfully at %.3f MHz", this->frequency_);
}

void WMBusComponent::loop() {
  this->led_handler();
  if (rf_mbus_.task()) {
    WMbusFrame mbus_data = rf_mbus_.get_frame();
    std::string telegram = format_hex_pretty(mbus_data.frame);
    telegram.erase(std::remove(telegram.begin(), telegram.end(), '.'), telegram.end());

    this->frame_timestamp_ = this->time_->timestamp_now();
    
    // Log raw frame immediately after reception
    ESP_LOGD(TAG, "Raw frame received (%c): %s", mbus_data.mode, telegram.c_str());

    send_to_clients(mbus_data);

    Telegram t;
    if (t.parseHeader(mbus_data.frame) && t.addresses.empty()) {
      ESP_LOGW(TAG, "Header parsed but address list is empty!");
    } else {
      uint32_t meter_id = (uint32_t)strtoul(t.addresses[0].id.c_str(), nullptr, 16);
      bool meter_in_config = (this->wmbus_listeners_.count(meter_id) == 1);

      if (this->log_all_ || meter_in_config) {
        auto detected_drv_info = pickMeterDriver(&t);
        std::string used_driver = (detected_drv_info.name().str().empty() ? "" : detected_drv_info.name().str().c_str());
        auto used_drv_info = detected_drv_info;

        if (meter_in_config) {
          auto *sensor = this->wmbus_listeners_[meter_id];
          ESP_LOGI(TAG, "Matched configured meter: ID 0x%08X", meter_id);

          if (!(sensor->type).empty()) {
            auto *used_drv_info_ptr = lookupDriver(sensor->type);
            if (used_drv_info_ptr != nullptr) {
              used_driver = sensor->type;
              used_drv_info = *used_drv_info_ptr;
            }
          }

          if (!used_driver.empty()) {
            this->led_blink();
            
            // Force zero-key logic if YAML key is empty or "0"
            std::string key_to_use = sensor->myKey;
            if (key_to_use.empty() || key_to_use == "0") {
                key_to_use = "00000000000000000000000000000000";
                ESP_LOGW(TAG, "Forcing use of ZERO-KEY (all zeros) for meter 0x%08X", meter_id);
            }

            MeterInfo mi;
            mi.parse("ESPHome", used_driver, t.addresses[0].id + ",", key_to_use);
            auto meter = createMeter(&mi);

            std::vector<Address> addresses;
            bool id_match;
            AboutTelegram about{"ESPHome wM-Bus", mbus_data.rssi, FrameType::WMBUS, this->frame_timestamp_};

            ESP_LOGD(TAG, "Attempting decryption/decoding with driver: %s", used_driver.c_str());
            meter->handleTelegram(about, mbus_data.frame, false, &addresses, &id_match, &t);

            if (id_match) {
              ESP_LOGI(TAG, "Decoding successful for meter 0x%08X", meter_id);
              
              for (auto const &field : sensor->fields) {
                std::string field_name = field.first.first;
                if (field_name == "rssi") {
                  field.second->publish_state(mbus_data.rssi);
                } else {
                  Unit field_unit = toUnit(field.second->get_unit_of_measurement());
                  double value = meter->getNumericValue(field_name, field_unit);
                  
                  if (!std::isnan(value)) {
                    ESP_LOGI(TAG, "Field '%s': Value = %.3f", field_name.c_str(), value);
                    field.second->publish_state(value);
                  } else {
                    ESP_LOGW(TAG, "Field '%s': Result is NaN! Check if field exists in %s driver and if AES key is correct.", 
                             field_name.c_str(), used_driver.c_str());
                  }
                }
              }
            } else {
              ESP_LOGE(TAG, "ERROR: Decryption/ID match failed for 0x%08X. Incorrect AES key?", meter_id);
            }
          }
        } else if (this->log_all_) {
           ESP_LOGD(TAG, "Found unconfigured meter: ID 0x%08X, Driver: %s", meter_id, used_driver.c_str());
        }
      }
    }
  }
}

void WMBusComponent::send_to_clients(WMbusFrame &mbus_data) {
  // Forward frames to network clients (TCP/UDP)
  for (auto &client : this->clients_) {
    if (client.transport == TRANSPORT_TCP) {
      if (this->tcp_client_.connect(client.ip.str().c_str(), client.port)) {
        this->tcp_client_.write((const uint8_t *) mbus_data.frame.data(), mbus_data.frame.size());
        this->tcp_client_.stop();
      }
    } else if (client.transport == TRANSPORT_UDP) {
      this->udp_client_.beginPacket(client.ip.str().c_str(), client.port);
      this->udp_client_.write((const uint8_t *) mbus_data.frame.data(), mbus_data.frame.size());
      this->udp_client_.endPacket();
    }
  }
}

void WMBusComponent::led_blink() {
  if (this->led_pin_ != nullptr) {
    this->led_on_ = true;
    this->led_pin_->digital_write(true);
    this->last_blink_ = millis();
  }
}

void WMBusComponent::led_handler() {
  if (this->led_on_) {
    uint32_t now = millis();
    if (now - this->last_blink_ > 100) {
      this->led_on_ = false;
      this->led_pin_->digital_write(false);
    }
  }
}

}  // namespace wmbus
}  // namespace esphome
