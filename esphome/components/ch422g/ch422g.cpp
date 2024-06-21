#include "ch422g.h"
#include "esphome/core/log.h"
#include <ESP_IOExpander_Library.h>
namespace esphome {
namespace ch422g {

// for 16 bit expanders, these addresses will be doubled.
const uint8_t INPUT_REG = 0x26;
const uint8_t OUTPUT_REG = 0x38;
const uint8_t INVERT_REG = 2;
const uint8_t CONFIG_REG = 0;

static const char *const TAG = "ch422g";
ESP_IOExpander_CH422G *expander;

void Ch422gComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up ch422g...");
  // this->reg_width_ = (this->pin_count_ + 7) / 8;
  expander = new ESP_IOExpander_CH422G(-1, this->address_);
  expander->init();
  // Test to see if device exists
  if (!this->read_inputs_()) {
    ESP_LOGE(TAG, "ch422g not detected at 0x%02X", this->address_);
    this->mark_failed();
    return;
  }

  // No polarity inversion
  // this->write_register_(INVERT_REG, 0);
  // All inputs at initialization
  //  this->config_mask_ = 0;
  // Invert mask as the part sees a 1 as an input
  // this->write_register_(CONFIG_REG, ~this->config_mask_);
  // All outputs low
  // this->output_mask_ = 0;
  // this->write_register_(OUTPUT_REG, this->output_mask_);
  // Read the inputs
  //  this->read_inputs_();
  ESP_LOGD(TAG, "Initialization complete. Warning: %d, Error: %d", this->status_has_warning(),
           this->status_has_error());
}

void Ch422gComponent::loop() {
  // The read_inputs_() method will cache the input values from the chip.
  // this->read_inputs_();
  // expander->multiDigitalRead();
  // Clear all the previously read flags.
  // this->was_previously_read_ = 0x00;
}

void Ch422gComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "ch422g:");
  ESP_LOGCONFIG(TAG, "  I/O Pins: %d", this->pin_count_);
  LOG_I2C_DEVICE(this)
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with ch422g failed!");
  }
}

bool Ch422gComponent::digital_read(uint8_t pin) {
  // Note: We want to try and avoid doing any I2C bus read transactions here
  // to conserve I2C bus bandwidth. So what we do is check to see if we
  // have seen a read during the time esphome is running this loop. If we have,
  // we do an I2C bus transaction to get the latest value. If we haven't
  // we return a cached value which was read at the time loop() was called.
  // if (this->was_previously_read_ & (1 << pin))
  //  this->read_inputs_();  // Force a read of a new value
  // Indicate we saw a read request for this pin in case a
  // read happens later in the same loop.
  // this->was_previously_read_ |= (1 << pin);
  //  return this->input_mask_ & (1 << pin);
  u_int8_t read[4];
  this->read_register(INPUT_REG, read, 4, true);
  return read[0] & (1 << pin * 8);
  // return expander->digitalRead(pin);
}

void Ch422gComponent::digital_write(uint8_t pin, bool value) {
  // if (value) {
  //   expander->digitalWrite(pin, HIGH);
  //   ESP_LOGD(TAG, "Setting pin %d to HIGH", pin);
  // } else {
  //   expander->digitalWrite(pin, LOW);
  // }
  uint8_t write[4];
  this->read_register(OUTPUT_REG, write, 4, true);
  if (value) {
    ESP_LOGD(TAG, "Setting pin %d to HIGH %d", pin, write[0] | (1 << pin * 8);
    this->write_register_(OUTPUT_REG, write[0] | (1 << pin * 8));
  } else {
    this->write_register_(OUTPUT_REG, write[0] & ~(1 << pin * 8));
  }

  // if (value) {
  //   this->output_mask_ |= (1 << pin);
  // } else {
  //   this->output_mask_ &= ~(1 << pin);
  // }
  // this->write_register_(OUTPUT_REG, this->output_mask_);
}

void Ch422gComponent::pin_mode(uint8_t pin, gpio::Flags flags) {
  expander->pinMode(pin, flags);

  if (flags == gpio::FLAG_INPUT) {
    // Clear mode mask bit
    expander->pinMode(pin, INPUT);
  } else if (flags == gpio::FLAG_OUTPUT) {
    // Set mode mask bit
    expander->pinMode(pin, OUTPUT);
  }

  // if (flags == gpio::FLAG_INPUT) {
  //   // Clear mode mask bit
  //   this->config_mask_ &= ~(1 << pin);
  // } else if (flags == gpio::FLAG_OUTPUT) {
  //   // Set mode mask bit
  //   this->config_mask_ |= 1 << pin;
  // }
  // this->write_register_(CONFIG_REG, ~this->config_mask_);
}

bool Ch422gComponent::read_inputs_() {
  uint8_t inputs[2];

  if (this->is_failed()) {
    ESP_LOGD(TAG, "Device marked failed");
    return false;
  }

  if ((this->last_error_ = this->read_register(INPUT_REG * this->reg_width_, inputs, this->reg_width_, true)) !=
      esphome::i2c::ERROR_OK) {
    this->status_set_warning();
    ESP_LOGE(TAG, "read_register_(): I2C I/O error: %d", (int) this->last_error_);
    return false;
  }
  this->status_clear_warning();
  this->input_mask_ = inputs[0];
  if (this->reg_width_ == 2) {
    this->input_mask_ |= inputs[1] << 8;
  }
  return true;
}

bool Ch422gComponent::write_register_(uint8_t reg, uint16_t value) {
  uint8_t outputs[2];
  outputs[0] = (uint8_t) value;
  outputs[1] = (uint8_t) (value >> 8);
  if ((this->last_error_ = this->write_register(reg * this->reg_width_, outputs, this->reg_width_, true)) !=
      esphome::i2c::ERROR_OK) {
    this->status_set_warning();
    ESP_LOGE(TAG, "write_register_(): I2C I/O error: %d", (int) this->last_error_);
    return false;
  }

  this->status_clear_warning();
  return true;
}

float Ch422gComponent::get_setup_priority() const { return setup_priority::IO; }

// Run our loop() method very early in the loop, so that we cache read values before
// before other components call our digital_read() method.
float Ch422gComponent::get_loop_priority() const { return 9.0f; }  // Just after WIFI

void ch422gGPIOPin::setup() { pin_mode(flags_); }
void ch422gGPIOPin::pin_mode(gpio::Flags flags) { this->parent_->pin_mode(this->pin_, flags); }
bool ch422gGPIOPin::digital_read() { return this->parent_->digital_read(this->pin_) != this->inverted_; }
void ch422gGPIOPin::digital_write(bool value) { this->parent_->digital_write(this->pin_, value != this->inverted_); }
std::string ch422gGPIOPin::dump_summary() const {
  char buffer[32];
  snprintf(buffer, sizeof(buffer), "%u via ch422g", pin_);
  return buffer;
}

}  // namespace ch422g
}  // namespace esphome
