#include "DiyActivePedal_types.h"
#include "Arduino.h"

#include "PedalGeometry.h"
#include "StepperWithLimits.h"

#include <EEPROM.h>

static const float ABS_SCALING = 50;

void DAP_config_st::initialiseDefaults() {
  payLoadHeader_.payloadType = DAP_PAYLOAD_TYPE_CONFIG;
  payLoadHeader_.version = DAP_VERSION_CONFIG;

  payLoadPedalConfig_.pedalStartPosition = 35;
  payLoadPedalConfig_.pedalEndPosition = 80;

  payLoadPedalConfig_.maxForce = 90;
  payLoadPedalConfig_.preloadForce = 1;

  payLoadPedalConfig_.relativeForce_p000 = 0;
  payLoadPedalConfig_.relativeForce_p020 = 20;
  payLoadPedalConfig_.relativeForce_p040 = 40;
  payLoadPedalConfig_.relativeForce_p060 = 60;
  payLoadPedalConfig_.relativeForce_p080 = 80;
  payLoadPedalConfig_.relativeForce_p100 = 100;

  payLoadPedalConfig_.dampingPress = 0;
  payLoadPedalConfig_.dampingPull = 0;

  payLoadPedalConfig_.absFrequency = 15;
  payLoadPedalConfig_.absAmplitude = 30.0f;

  payLoadPedalConfig_.lengthPedal_AC = 150;
  payLoadPedalConfig_.horPos_AB = 215;
  payLoadPedalConfig_.verPos_AB = 80;
  payLoadPedalConfig_.lengthPedal_CB = 200;
}


void DAP_config_st::initialiseDefaults_Accelerator() {
  payLoadHeader_.payloadType = DAP_PAYLOAD_TYPE_CONFIG;
  payLoadHeader_.version = DAP_VERSION_CONFIG;


  payLoadPedalConfig_.pedalStartPosition = 35;
  payLoadPedalConfig_.pedalEndPosition = 80;

  payLoadPedalConfig_.maxForce = 20;
  payLoadPedalConfig_.preloadForce = 3;

  payLoadPedalConfig_.relativeForce_p000 = 0;
  payLoadPedalConfig_.relativeForce_p020 = 20;
  payLoadPedalConfig_.relativeForce_p040 = 40;
  payLoadPedalConfig_.relativeForce_p060 = 60;
  payLoadPedalConfig_.relativeForce_p080 = 80;
  payLoadPedalConfig_.relativeForce_p100 = 100;

  payLoadPedalConfig_.dampingPress = 0;
  payLoadPedalConfig_.dampingPull = 0;

  payLoadPedalConfig_.absFrequency = 60;
  payLoadPedalConfig_.absAmplitude = 0.0f;

  payLoadPedalConfig_.lengthPedal_AC = 150;
  payLoadPedalConfig_.horPos_AB = 215;
  payLoadPedalConfig_.verPos_AB = 80;
  payLoadPedalConfig_.lengthPedal_CB = 200;
}



void DAP_config_st::storeConfigToEprom(DAP_config_st& config_st)
{
  EEPROM.put(0, config_st); 
  EEPROM.commit();
  Serial.println("Successfully stored config in EPROM!");
}

void DAP_config_st::loadConfigFromEprom(DAP_config_st& config_st)
{
  DAP_config_st local_config_st;

  EEPROM.get(0, local_config_st);
  EEPROM.commit();

  // check if version matches revision, in case, update the default config
  if (local_config_st.payLoadHeader_.version == DAP_VERSION_CONFIG)
  {
    config_st = local_config_st;
    Serial.println("Successfully loaded config from EPROM!");
  }
  else
  {
    Serial.println("Couldn't load config from EPROM due to version mismatch!");
  }

}





void DAP_calculationVariables_st::updateFromConfig(DAP_config_st& config_st) {
  startPosRel = ((float)config_st.payLoadPedalConfig_.pedalStartPosition) / 100.0f;
  endPosRel = ((float)config_st.payLoadPedalConfig_.pedalEndPosition) / 100.0f;

  absFrequency = 2 * PI * ((float)config_st.payLoadPedalConfig_.absFrequency);
  absAmplitude = ((float)config_st.payLoadPedalConfig_.absAmplitude)/ TRAVEL_PER_ROTATION_IN_MM * STEPS_PER_MOTOR_REVOLUTION / ABS_SCALING; // in mm

  dampingPress = ((float)config_st.payLoadPedalConfig_.dampingPress) / 400.0f;

  // update force variables
  Force_Min = ((float)config_st.payLoadPedalConfig_.preloadForce) / 10.0f;
  Force_Max = ((float)config_st.payLoadPedalConfig_.maxForce) / 10.0f;
  Force_Range = Force_Max - Force_Min;
}

void DAP_calculationVariables_st::updateEndstops(long newMinEndstop, long newMaxEndstop) {
  stepperPosMinEndstop = newMinEndstop;
  stepperPosMaxEndstop = newMaxEndstop;
  stepperPosEndstopRange = stepperPosMaxEndstop - stepperPosMinEndstop;

  stepperPosMin = stepperPosEndstopRange * startPosRel;
  stepperPosMax = stepperPosEndstopRange * endPosRel;
  stepperPosRange = stepperPosMax - stepperPosMin;
}

void DAP_calculationVariables_st::updateStiffness() {
  springStiffnesss = Force_Range / stepperPosRange;
  springStiffnesssInv = 1.0 / springStiffnesss;
}