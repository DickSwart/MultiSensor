#ifndef MultiSensor_h
#define MultiSensor_h

#include "DHT.h"
#include "user_config.h"

#define NO_SENSOR_EVT                 0
#define MOTION_SENSOR_EVT             1
#define LDR_SENSOR_EVT                2
#define DHT_TEMPERATURE_SENSOR_EVT    3
#define DHT_HUMIDITY_SENSOR_EVT       4
#define DHT_REALFEEL_SENSOR_EVT       5

class MultiSensor {
  public:
    MultiSensor(void);
    void init(void);
    void loop(void);
    void setCallback(void (*callback)(uint8_t));
    bool getMotionState(void);
    uint16_t getLux(void);
    float getDHTTemperature(void);
    float getDHTHumidity(void);
    float getDHTRealFeel(void);

  private:
    void (*_callback)(uint8_t);
    void handleEvt(void);
    bool checkBoundSensor(float newValue, float prevValue, float maxDiff) ;
    bool _motionState = false;
    uint16_t _ldrValue = NAN;
    float _readDHTTemperature(void);
    float _readDHTHumidity(void);
    float _readDHTRealFeel(void);
    float _DHTRealFeel = NAN;
    float _DHTTemperature = NAN;
    float _DHTHumidity = NAN;
};

#endif
