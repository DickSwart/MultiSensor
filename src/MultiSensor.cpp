#include "MultiSensor.h"

DHT dht(DHT_PIN, DHT22);
volatile uint8_t evt = NO_SENSOR_EVT;

///////////////////////////////////////////////////////////////////////////
//  ISRs
///////////////////////////////////////////////////////////////////////////
void ICACHE_RAM_ATTR motionSensorISR(void)
{
  evt = MOTION_SENSOR_EVT;
}

///////////////////////////////////////////////////////////////////////////
//  Constructor, init(), handleEvt() & loop()
///////////////////////////////////////////////////////////////////////////
MultiSensor::MultiSensor(void) {}

void MultiSensor::init(void)
{
  pinMode(MOTION_SENSOR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MOTION_SENSOR_PIN), motionSensorISR, CHANGE);
  this->_motionState = digitalRead(MOTION_SENSOR_PIN);

  pinMode(LDR_PIN, INPUT);
  this->_ldrValue = analogRead(LDR_PIN);

  dht.begin();
  delay(2000);
  this->_readDHTTemperature();
  this->_readDHTHumidity();
  this->_readDHTRealFeel();

}

void MultiSensor::handleEvt(void)
{
  switch (evt)
  {
  case NO_SENSOR_EVT:
    break;
  case MOTION_SENSOR_EVT:
    if (digitalRead(MOTION_SENSOR_PIN) != this->_motionState)
    {
      this->_motionState = !this->_motionState;
      this->_callback(MOTION_SENSOR_EVT);
    }
    evt = NO_SENSOR_EVT;
    break;
  case LDR_SENSOR_EVT:
    this->_callback(LDR_SENSOR_EVT);
    evt = NO_SENSOR_EVT;
    break;
  case DHT_TEMPERATURE_SENSOR_EVT:
    this->_callback(DHT_TEMPERATURE_SENSOR_EVT);
    evt = NO_SENSOR_EVT;
    break;
  case DHT_HUMIDITY_SENSOR_EVT:
    this->_callback(DHT_HUMIDITY_SENSOR_EVT);
    evt = NO_SENSOR_EVT;
    break;
  case DHT_REALFEEL_SENSOR_EVT:
    this->_callback(DHT_REALFEEL_SENSOR_EVT);
    evt = NO_SENSOR_EVT;
    break;
  }
}

void MultiSensor::loop(void)
{
  this->handleEvt();

  static unsigned long lastLdrSensorMeasure = 0;
  if (lastLdrSensorMeasure + LDR_MEASURE_INTERVAL <= millis())
  {
    lastLdrSensorMeasure = millis();
    uint16_t currentLdrValue = analogRead(LDR_PIN);
    if (checkBoundSensor(currentLdrValue, this->_ldrValue, LDR_OFFSET_VALUE))
    {
      this->_ldrValue = currentLdrValue;
      evt = LDR_SENSOR_EVT;
      return;
    }
  }

  static unsigned long lastDHTTemperatureSensorMeasure = 0;
  if (lastDHTTemperatureSensorMeasure + DHT_MEASURE_INTERVAL <= millis())
  {
    lastDHTTemperatureSensorMeasure = millis();
    float currentDHTTemperature = this->_readDHTTemperature();
    if (checkBoundSensor(currentDHTTemperature, this->_DHTTemperature, DHT_TEMPERATURE_OFFSET_VALUE))
    {
      this->_DHTTemperature = currentDHTTemperature;
      evt = DHT_TEMPERATURE_SENSOR_EVT;
      return;
    }
  }

  static unsigned long lastDHTHumiditySensorMeasure = 0;
  if (lastDHTHumiditySensorMeasure + DHT_MEASURE_INTERVAL <= millis())
  {
    lastDHTHumiditySensorMeasure = millis();
    float currentDHTHumidity = this->_readDHTHumidity();
    if (checkBoundSensor(currentDHTHumidity, this->_DHTHumidity, DHT_HUMIDITY_OFFSET_VALUE))
    {
      this->_DHTHumidity = currentDHTHumidity;
      evt = DHT_HUMIDITY_SENSOR_EVT;
      return;
    }
  }

  static unsigned long lastDHTRealFeelSensorMeasure = 0;
  if (lastDHTRealFeelSensorMeasure + DHT_MEASURE_INTERVAL <= millis())
  {
    lastDHTRealFeelSensorMeasure = millis();
    float currentDHTRealFeel = this->_readDHTRealFeel();
    if (checkBoundSensor(currentDHTRealFeel, this->_DHTRealFeel, DHT_TEMPERATURE_OFFSET_VALUE))
    {
      this->_DHTRealFeel = currentDHTRealFeel;
      evt = DHT_REALFEEL_SENSOR_EVT;
      return;
    }
  }
}

///////////////////////////////////////////////////////////////////////////
//  setCallback()
///////////////////////////////////////////////////////////////////////////
void MultiSensor::setCallback(void (*callback)(uint8_t))
{
  this->_callback = callback;
}

///////////////////////////////////////////////////////////////////////////
//  Getters
///////////////////////////////////////////////////////////////////////////

bool MultiSensor::getMotionState(void)
{
  return this->_motionState;
}

uint16_t MultiSensor::getLux(void)
{
  // http://forum.arduino.cc/index.php?topic=37555.0
  // https://forum.arduino.cc/index.php?topic=185158.0
  float volts = this->_ldrValue * REFERENCE_VOLTAGE / ADC_PRECISION;
  float amps = volts / LDR_RESISTOR_VALUE;
  float lux = amps * 1000000 * 2.0;
  return uint16_t(lux);
}

float MultiSensor::_readDHTTemperature(void)
{
  float temperature = dht.readTemperature();

  if (isnan(temperature))
  {
    return this->_DHTTemperature;
  }
  return temperature;
}

float MultiSensor::_readDHTHumidity(void)
{
  float humidity = dht.readHumidity();

  if (isnan(humidity))
  {
    return this->_DHTHumidity;
  }

  return humidity;
}

float MultiSensor::_readDHTRealFeel(void)
{
  // Compute heat index in Celsius (isFahreheit = false)
  float realFeel = dht.computeHeatIndex(this->_readDHTTemperature(), this->_readDHTHumidity(), false);
  // Check if we have a heatIndex value
  if (isnan(realFeel))
  {
    return this->_DHTRealFeel;
  }

  return realFeel;
}

float MultiSensor::getDHTTemperature(void)
{
  return this->_DHTTemperature;
}

float MultiSensor::getDHTHumidity(void)
{
  return this->_DHTHumidity;
}
float MultiSensor::getDHTRealFeel(void)
{
  return this->_DHTRealFeel;
}

///////////////////////////////////////////////////////////////////////////
//  Helpers
///////////////////////////////////////////////////////////////////////////
bool MultiSensor::checkBoundSensor(float newValue, float prevValue, float maxDiff)
{
  return isnan(prevValue) || newValue <= prevValue - maxDiff || newValue >= prevValue + maxDiff;
}