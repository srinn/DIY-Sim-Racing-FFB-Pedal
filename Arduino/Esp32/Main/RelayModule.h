#pragma once

class RelayModule {
  private:
    uint8_t _pin;
    bool _isHighMode;
    bool _isNO; // NO = Normally Open
    bool _isOpen;

  public:
    RelayModule(uint8_t pin, bool isHighMode = true, bool isNO = false); // NO = Normally Open
    void On();
    void Off();
    bool GetState();
    bool ReadRealState();
    void SetState(bool on);
    void Switch();
};