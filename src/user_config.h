#ifndef _USER_CONFIG_H_
#define _USER_CONFIG_H_

///////////////////////////////////////////////////////////////////////////
//   WIFI
///////////////////////////////////////////////////////////////////////////
#define WIFI_SSID "wifi_ssid"
#define WIFI_PASSWORD "wifi_password"
#define WIFI_QUALITY_OFFSET_VALUE 2
#define WIFI_QUALITY_INTERVAL 50000 // [ms]
#define WIFI_QUALITY_SENSOR_NAME "wifi"

///////////////////////////////////////////////////////////////////////////
//   MQTT
///////////////////////////////////////////////////////////////////////////
#define MQTT_SERVER "xxx.xxx.xxx.xxx"
#define MQTT_SERVER_PORT 1883
#define MQTT_USERNAME "mqtt_user_name"
#define MQTT_PASSWORD "mqtt_password"

#define MQTT_AVAILABILITY_TOPIC_TEMPLATE "%s/status" // MQTT availability: online/offline
#define MQTT_SENSOR_TOPIC_TEMPLATE "%s/sensor/%s"
#define MQTT_LIGHT_STATE_TOPIC_TEMPLATE "%s/light/state"
#define MQTT_LIGHT_COMMAND_TOPIC_TEMPLATE "%s/light/set"

#define MQTT_PAYLOAD_ON "ON"
#define MQTT_PAYLOAD_OFF "OFF"

///////////////////////////////////////////////////////////////////////////
//   LIGHT SENSOR
///////////////////////////////////////////////////////////////////////////
#define LDR_SENSOR_NAME "lux"
#define LDR_OFFSET_VALUE 25
#define LDR_MEASURE_INTERVAL 1000  // [ms]
#define REFERENCE_VOLTAGE 3.3      // [v]
#define ADC_PRECISION 1024.0       // 10 bits
#define LDR_RESISTOR_VALUE 10000.0 // [Ohms]
#define LDR_PIN A0

///////////////////////////////////////////////////////////////////////////
//   MOTION SENSOR
///////////////////////////////////////////////////////////////////////////
#define MOTION_SENSOR_NAME "motion"
#define MOTION_SENSOR_PIN D5

///////////////////////////////////////////////////////////////////////////
//   DHT SENSOR PINS
//   - Temperature and humidity sensor (DHT22)
///////////////////////////////////////////////////////////////////////////
#define DHT_TEMPERATURE_SENSOR_NAME "temperature"
#define DHT_HUMIDITY_SENSOR_NAME "humidity"
#define DHT_REALFEEL_SENSOR_NAME "realfeel"
#define DHT_TEMPERATURE_OFFSET_VALUE 0.2 // [°C]
#define DHT_HUMIDITY_OFFSET_VALUE 0.5    // [%]
#define DHT_MEASURE_INTERVAL 30000       // [ms]
#define DHT_PIN D7

///////////////////////////////////////////////////////////////////////////
//   LED STRIP PINS
//   - In case of BRIGHTNESS: only WHITE is used
//   - In case of RGB(W): red, green, blue(, white) is used
//   - All values need to be present, if they are not needed, set to -1,
//     it will be ignored.
///////////////////////////////////////////////////////////////////////////
#define RED_PIN D1   // For RGB(W)
#define GREEN_PIN D2 // For RGB(W)
#define BLUE_PIN D3  // For RGB(W)
#define WHITE_PIN D4 // For BRIGHTNESS and RGBW

///////////////////////////////////////////////////////////////////////////
//   Over-the-Air update (OTA)
///////////////////////////////////////////////////////////////////////////
#define OTA_PORT 8266  // port 8266 by default

#endif  // _USER_CONFIG_H_