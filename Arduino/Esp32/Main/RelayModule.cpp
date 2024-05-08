#include "esp32-hal-gpio.h"
#include "RelayModule.h"

RelayModule::RelayModule(uint8_t pin, bool isHighMode, bool isNO) {
  _pin = pin;
  _isHighMode = isHighMode;
  _isNO = isNO; // NO = Normally Open, NC = Normally Close
  _isOpen = true;

  pinMode(_pin, OUTPUT);
  digitalWrite(_pin, (_isHighMode ? LOW : HIGH)); // Open the line for initialize
}

void RelayModule::On() {
  _isOpen = (_isNO ? false : true); // if Normally Open, Close the line. or if Normally Close, Open the line.
  digitalWrite(_pin, (_isHighMode ? HIGH : LOW));
}

void RelayModule::Off() {
  _isOpen = (_isNO ? true : false); // if Normally Open, Open the line. or if Normally Close, Close the line.
  digitalWrite(_pin, (_isHighMode ? LOW : HIGH));
}

bool RelayModule::ReadRealState() {
  return digitalRead(_pin);
}

bool RelayModule::GetState() {
  return _isNO ? !_isOpen : _isOpen; // if Normally Open and line was Opened, this state is Off
}

void RelayModule::SetState(bool on) {
  if (on) this->On();
  else this->Off();
}

void RelayModule::Switch() {
  if (this->GetState()) this->Off(); // if This state is On, turn off
  else this->On();
}
