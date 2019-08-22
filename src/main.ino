#pragma GCC diagnostic ignored "-Wwrite-strings"
#include <ArduinoJson.h> // https://github.com/bblanchon/ArduinoJson
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ESP8266WiFi.h>  // https://github.com/esp8266/Arduino
#include <PubSubClient.h> // https://github.com/knolleary/pubsubclient
#include <ArduinoOTA.h> //https://github.com/esp8266/Arduino/tree/master/libraries/ArduinoOTA
#include "MultiSensor.h"
#include "Light.h"

#include "user_config.h" // Fixed user configurable options
#ifdef USE_CONFIG_OVERRIDE
#include "user_config_override.h" // Configuration overrides for my_user_config.h
#endif

/* -------------------------------------------------
 *  WiFi
 * ------------------------------------------------- */
// variables declaration
int previousWiFiSignalStrength = -1;
unsigned long previousMillis = 0;
int reqConnect = 0;
int isConnected = 0;
const long interval = 500;
const long reqConnectNum = 15; // number of intervals to wait for connection
WiFiEventHandler mConnectHandler;
WiFiEventHandler mDisConnectHandler;
WiFiEventHandler mGotIpHandler;

// function declaration
void setupWiFi(void);
void connectWiFi(void);
void onConnected(const WiFiEventStationModeConnected &event);
void onDisconnect(const WiFiEventStationModeDisconnected &event);
void onGotIP(const WiFiEventStationModeGotIP &event);
void loopWiFiSensor(void);
int getWiFiSignalStrength(void);

// Initialize the Ethernet client object
WiFiClient wifiClient;

/* -------------------------------------------------
 *  MQTT
 * ------------------------------------------------- */
// function declaration
void setupMQTT();
void connectToMQTT();
void subscribeToMQTT(char *p_topic);
void publishToMQTT(char *p_topic, char *p_payload);
void handleMQTTMessage(char *topic, byte *payload, unsigned int length);

// variables declaration
volatile unsigned long lastMQTTConnection = 0;
char MQTT_CLIENT_ID[7] = {0};
char MQTT_PAYLOAD[8] = {0};
char MQTT_AVAILABILITY_TOPIC[sizeof(MQTT_CLIENT_ID) + sizeof(MQTT_AVAILABILITY_TOPIC_TEMPLATE) - 2] = {0};
char MQTT_WIFI_QUALITY_TOPIC[sizeof(MQTT_CLIENT_ID) + sizeof(MQTT_SENSOR_TOPIC_TEMPLATE) + sizeof(WIFI_QUALITY_SENSOR_NAME) - 4] = {0};
char MQTT_LIGHT_STATE_TOPIC[sizeof(MQTT_CLIENT_ID) + sizeof(MQTT_LIGHT_STATE_TOPIC_TEMPLATE) - 2] = {0};
char MQTT_LIGHT_COMMAND_TOPIC[sizeof(MQTT_CLIENT_ID) + sizeof(MQTT_LIGHT_COMMAND_TOPIC_TEMPLATE) - 2] = {0};
char MQTT_MOTION_SENSOR_TOPIC[sizeof(MQTT_CLIENT_ID) + sizeof(MQTT_SENSOR_TOPIC_TEMPLATE) + sizeof(MOTION_SENSOR_NAME) - 4] = {0};
char MQTT_LDR_SENSOR_TOPIC[sizeof(MQTT_CLIENT_ID) + sizeof(MQTT_SENSOR_TOPIC_TEMPLATE) + sizeof(LDR_SENSOR_NAME) - 4] = {0};
char MQTT_DHT_TEMPERATURE_SENSOR_TOPIC[sizeof(MQTT_CLIENT_ID) + sizeof(MQTT_SENSOR_TOPIC_TEMPLATE) + sizeof(DHT_TEMPERATURE_SENSOR_NAME) - 4] = {0};
char MQTT_DHT_HUMIDITY_SENSOR_TOPIC[sizeof(MQTT_CLIENT_ID) + sizeof(MQTT_SENSOR_TOPIC_TEMPLATE) + sizeof(DHT_HUMIDITY_SENSOR_NAME) - 4] = {0};
char MQTT_DHT_REALFEEL_SENSOR_TOPIC[sizeof(MQTT_CLIENT_ID) + sizeof(MQTT_SENSOR_TOPIC_TEMPLATE) + sizeof(DHT_REALFEEL_SENSOR_NAME) - 4] = {0};

// Initialize the mqtt client object
PubSubClient mqttClient(wifiClient);

/* -------------------------------------------------
 *  MultiSensor
 * ------------------------------------------------- */
// function declaration
void onMultiSensorEvent();

float tempTemperature;
float tempHumidity;
float tempRealFeel;

MultiSensor ms;

/* -------------------------------------------------
 *  Light
 * ------------------------------------------------- */
// function declaration
void handleLightCMD();

volatile uint8_t cmd = CMD_NOT_DEFINED;
Light bulb;

///////////////////////////////////////////////////////////////////////////
//   Main Setup & loop
///////////////////////////////////////////////////////////////////////////
void setup()
{
  Serial.begin(115200);

  // WIFI
  setupWiFi();

  // MQTT
  setupMQTT();

  // Over the air
  ArduinoOTA.begin();

  // Initialise light
  bulb.init();
  cmd = CMD_STATE_CHANGED;

  // initialise multi-sensor
  ms.init();
  ms.setCallback(onMultiSensorEvent);
}

void loop()
{
  // WIFI
  connectWiFi();

  // Code will only run if connected to WiFi
  if (isConnected == 2)
  {
    // MQTT
    if (!mqttClient.connected())
    {
      connectToMQTT();
    }
    mqttClient.loop();

    // Over the air
    ArduinoOTA.handle();

    // Check WiFi signal
    loopWiFiSensor();

    // handle multisensor
    ms.loop();

    // handle light
    bulb.loop();
    handleLightCMD();
  }
}

/* -------------------------------------------------
 *  Light
 * ------------------------------------------------- */
void handleLightCMD()
{
  switch (cmd)
  {
  case CMD_NOT_DEFINED:
    break;
  case CMD_STATE_CHANGED:
    cmd = CMD_NOT_DEFINED;

    DynamicJsonDocument doc(1024);

    doc["brightness"] = bulb.getBrightness();
    doc["color_temp"] = bulb.getColorTemperature();

    JsonObject color = doc.createNestedObject("color");
    color["r"] = bulb.getColor().red;
    color["g"] = bulb.getColor().green;
    color["b"] = bulb.getColor().blue;
    doc["state"] = bulb.getState() ? MQTT_PAYLOAD_ON : MQTT_PAYLOAD_OFF;
    doc["white_value"] = bulb.getColor().white;

    String output;
    serializeJson(doc, output);

    char *buffer = new char[output.length() + 1];
    strcpy(buffer, output.c_str());

    publishToMQTT(MQTT_LIGHT_STATE_TOPIC, buffer);

    // Cleanup
    delete[] buffer;
    break;
  }
}

/* -------------------------------------------------
 *  MultiSensor
 * ------------------------------------------------- */
void onMultiSensorEvent(uint8_t p_evt)
{
  Serial.print(F("INFO: onMultiSensorEvent(): evt: "));
  Serial.println(p_evt);

  switch (p_evt)
  {
  case MOTION_SENSOR_EVT:
    if (ms.getMotionState())
    {
      publishToMQTT(MQTT_MOTION_SENSOR_TOPIC, MQTT_PAYLOAD_ON);
    }
    else
    {
      publishToMQTT(MQTT_MOTION_SENSOR_TOPIC, MQTT_PAYLOAD_OFF);
    }
    break;
  case LDR_SENSOR_EVT:
    itoa(ms.getLux(), MQTT_PAYLOAD, 10);
    publishToMQTT(MQTT_LDR_SENSOR_TOPIC, MQTT_PAYLOAD);
    break;

  case DHT_TEMPERATURE_SENSOR_EVT:
    tempTemperature = ms.getDHTTemperature();
    if (!isnan(tempTemperature))
    {
      dtostrf(tempTemperature, 2, 2, MQTT_PAYLOAD);
      publishToMQTT(MQTT_DHT_TEMPERATURE_SENSOR_TOPIC, MQTT_PAYLOAD);
    }
    break;
  case DHT_HUMIDITY_SENSOR_EVT:
    tempHumidity = ms.getDHTHumidity();
    if (!isnan(tempHumidity))
    {
      dtostrf(tempHumidity, 2, 2, MQTT_PAYLOAD);
      publishToMQTT(MQTT_DHT_HUMIDITY_SENSOR_TOPIC, MQTT_PAYLOAD);
    }
    break;
  case DHT_REALFEEL_SENSOR_EVT:
    tempRealFeel = ms.getDHTRealFeel();
    if (!isnan(tempRealFeel))
    {
      dtostrf(tempRealFeel, 2, 2, MQTT_PAYLOAD);
      publishToMQTT(MQTT_DHT_REALFEEL_SENSOR_TOPIC, MQTT_PAYLOAD);
    }
    break;
  }
}

///////////////////////////////////////////////////////////////////////////
//   WiFi
///////////////////////////////////////////////////////////////////////////

/*
 * Function called to setup WiFi module
 */
void setupWiFi(void)
{
  WiFi.disconnect();
  WiFi.persistent(false);
  mConnectHandler = WiFi.onStationModeConnected(onConnected);
  mDisConnectHandler = WiFi.onStationModeDisconnected(onDisconnect);
  mGotIpHandler = WiFi.onStationModeGotIP(onGotIP);
}

/*
 * Function called to connect to WiFi
 */
void connectWiFi(void)
{
  if (WiFi.status() != WL_CONNECTED && reqConnect > reqConnectNum && isConnected < 2)
  {
    reqConnect = 0;
    isConnected = 0;
    WiFi.disconnect();

    Serial.println();
    Serial.print("[WIFI]: Attempting to connect to WPA SSID: ");
    Serial.println(WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.println("[WIFI]: Connecting...");
  }

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;
    reqConnect++;
  }
}

/*
 * Function called to handle WiFi events
 */
void onConnected(const WiFiEventStationModeConnected &event)
{
  char macAdddress[20];
  sprintf(macAdddress, "%02X:%02X:%02X:%02X:%02X:%02X", event.bssid[5], event.bssid[4], event.bssid[3], event.bssid[2], event.bssid[1], event.bssid[0]);
  Serial.print(F("[WIFI]: You're connected to the AP. (MAC - "));
  Serial.print(macAdddress);
  Serial.println(")");
  isConnected = 1;
}

void onDisconnect(const WiFiEventStationModeDisconnected &event)
{
  Serial.println("[WIFI]: Disconnected");
  Serial.print("[WIFI]: Reason: ");
  Serial.println(event.reason);
  isConnected = 0;
}

void onGotIP(const WiFiEventStationModeGotIP &event)
{
  Serial.print("[WIFI]: IP Address : ");
  Serial.println(event.ip);
  Serial.print("[WIFI]: Subnet     : ");
  Serial.println(event.mask);
  Serial.print("[WIFI]: Gateway    : ");
  Serial.println(event.gw);

  isConnected = 2;
}

/*
 * Function to check WiFi signal strength
 */
void loopWiFiSensor(void)
{
  static unsigned long lastWiFiQualityMeasure = 0;
  if (lastWiFiQualityMeasure + WIFI_QUALITY_INTERVAL <= millis() || previousWiFiSignalStrength == -1)
  {
    lastWiFiQualityMeasure = millis();
    int currentWiFiSignalStrength = getWiFiSignalStrength();
    if (isnan(previousWiFiSignalStrength) || currentWiFiSignalStrength <= previousWiFiSignalStrength - WIFI_QUALITY_OFFSET_VALUE || currentWiFiSignalStrength >= previousWiFiSignalStrength + WIFI_QUALITY_OFFSET_VALUE)
    {
      previousWiFiSignalStrength = currentWiFiSignalStrength;
      dtostrf(currentWiFiSignalStrength, 2, 2, MQTT_PAYLOAD);
      publishToMQTT(MQTT_WIFI_QUALITY_TOPIC, MQTT_PAYLOAD);
    }
  }
}

/*
 * Helper function to get the current WiFi signal strength
 */
int getWiFiSignalStrength(void)
{
  if (WiFi.status() != WL_CONNECTED)
    return -1;
  int dBm = WiFi.RSSI();
  if (dBm <= -100)
    return 0;
  if (dBm >= -50)
    return 100;
  return 2 * (dBm + 100);
}

/*
 * Function called to setup MQTT topics
 */
void setupMQTT()
{
  sprintf(MQTT_CLIENT_ID, "%06X", ESP.getChipId());
  sprintf(MQTT_AVAILABILITY_TOPIC, MQTT_AVAILABILITY_TOPIC_TEMPLATE, MQTT_CLIENT_ID);

  Serial.print(F("[MQTT]: Availability topic: "));
  Serial.println(MQTT_AVAILABILITY_TOPIC);

  sprintf(MQTT_WIFI_QUALITY_TOPIC, MQTT_SENSOR_TOPIC_TEMPLATE, MQTT_CLIENT_ID, WIFI_QUALITY_SENSOR_NAME);
  Serial.print(F("[MQTT]: WiFi Quality topic: "));
  Serial.println(MQTT_WIFI_QUALITY_TOPIC);

  sprintf(MQTT_LIGHT_STATE_TOPIC, MQTT_LIGHT_STATE_TOPIC_TEMPLATE, MQTT_CLIENT_ID);
  Serial.print(F("[MQTT]: Light state topic: "));
  Serial.println(MQTT_LIGHT_STATE_TOPIC);

  sprintf(MQTT_LIGHT_COMMAND_TOPIC, MQTT_LIGHT_COMMAND_TOPIC_TEMPLATE, MQTT_CLIENT_ID);
  Serial.print(F("[MQTT]: Light command topic: "));
  Serial.println(MQTT_LIGHT_COMMAND_TOPIC);

  sprintf(MQTT_MOTION_SENSOR_TOPIC, MQTT_SENSOR_TOPIC_TEMPLATE, MQTT_CLIENT_ID, MOTION_SENSOR_NAME);
  Serial.print(F("[MQTT]: Motion sensor topic: "));
  Serial.println(MQTT_MOTION_SENSOR_TOPIC);

  sprintf(MQTT_LDR_SENSOR_TOPIC, MQTT_SENSOR_TOPIC_TEMPLATE, MQTT_CLIENT_ID, LDR_SENSOR_NAME);
  Serial.print(F("[MQTT]: luminance sensor topic: "));
  Serial.println(MQTT_LDR_SENSOR_TOPIC);

  sprintf(MQTT_DHT_TEMPERATURE_SENSOR_TOPIC, MQTT_SENSOR_TOPIC_TEMPLATE, MQTT_CLIENT_ID, DHT_TEMPERATURE_SENSOR_NAME);
  Serial.print(F("[MQTT]: DHT temperature sensor topic: "));
  Serial.println(MQTT_DHT_TEMPERATURE_SENSOR_TOPIC);
  sprintf(MQTT_DHT_HUMIDITY_SENSOR_TOPIC, MQTT_SENSOR_TOPIC_TEMPLATE, MQTT_CLIENT_ID, DHT_HUMIDITY_SENSOR_NAME);
  Serial.print(F("[MQTT]: DHT humidity sensor topic: "));
  Serial.println(MQTT_DHT_HUMIDITY_SENSOR_TOPIC);
  sprintf(MQTT_DHT_REALFEEL_SENSOR_TOPIC, MQTT_SENSOR_TOPIC_TEMPLATE, MQTT_CLIENT_ID, DHT_REALFEEL_SENSOR_NAME);
  Serial.print(F("[MQTT]: DHT real feel sensor topic: "));
  Serial.println(MQTT_DHT_REALFEEL_SENSOR_TOPIC);

  mqttClient.setServer(MQTT_SERVER, MQTT_SERVER_PORT);
  mqttClient.setCallback(handleMQTTMessage);
}

/*
  Function called to connect/reconnect to the MQTT broker
*/
void connectToMQTT()
{
  int retries = 0;

  // Loop until we're connected / reconnected
  while (!mqttClient.connected())
  {
    if (retries < 15)
    {
      if (mqttClient.connect(MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD, MQTT_AVAILABILITY_TOPIC, 0, 1, "offline"))
      {
        Serial.println(F("[MQTT]: The client is successfully connected to the MQTT broker"));
        publishToMQTT(MQTT_AVAILABILITY_TOPIC, "online");

        // subscribe to light command topic
        subscribeToMQTT(MQTT_LIGHT_COMMAND_TOPIC);
        handleLightCMD();

        // publish motion state
        if (ms.getMotionState())
        {
          publishToMQTT(MQTT_MOTION_SENSOR_TOPIC, MQTT_PAYLOAD_ON);
        }
        else
        {
          publishToMQTT(MQTT_MOTION_SENSOR_TOPIC, MQTT_PAYLOAD_OFF);
        }

        // publish luminance
        itoa(ms.getLux(), MQTT_PAYLOAD, 10);
        publishToMQTT(MQTT_LDR_SENSOR_TOPIC, MQTT_PAYLOAD);

        // publish temperature
        tempTemperature = ms.getDHTTemperature();
        if (!isnan(tempTemperature))
        {
          dtostrf(tempTemperature, 2, 2, MQTT_PAYLOAD);
          publishToMQTT(MQTT_DHT_TEMPERATURE_SENSOR_TOPIC, MQTT_PAYLOAD);
        }

        // publish humidity
        tempHumidity = ms.getDHTHumidity();
        if (!isnan(tempHumidity))
        {
          dtostrf(tempHumidity, 2, 2, MQTT_PAYLOAD);
          publishToMQTT(MQTT_DHT_HUMIDITY_SENSOR_TOPIC, MQTT_PAYLOAD);
        }

        // publish real feel
        tempRealFeel = ms.getDHTRealFeel();
        if (!isnan(tempRealFeel))
        {
          dtostrf(tempRealFeel, 2, 2, MQTT_PAYLOAD);
          publishToMQTT(MQTT_DHT_REALFEEL_SENSOR_TOPIC, MQTT_PAYLOAD);
        }
      }
      else
      {
        Serial.println(F("[MQTT]: ERROR - The connection to the MQTT broker failed"));
        Serial.print(F("[MQTT]: MQTT username: "));
        Serial.println(MQTT_USERNAME);
        Serial.print(F("[MQTT]: MQTT password: "));
        Serial.println(MQTT_PASSWORD);
        Serial.print(F("[MQTT]: MQTT broker: "));
        Serial.println(MQTT_SERVER);
        retries++;
        // Wait 5 seconds before retrying
        delay(5000);
      }
    }
    if (retries > 14)
    {
      ESP.restart();
    }
  }
}

/*
  Function called to subscribe to a MQTT topic
*/
void subscribeToMQTT(char *p_topic)
{
  if (mqttClient.subscribe(p_topic))
  {
    Serial.print(F("[MQTT]: subscribeToMQTT - Sending the MQTT subscribe succeeded for topic: "));
    Serial.println(p_topic);
  }
  else
  {
    Serial.print(F("[MQTT]: subscribeToMQTT - ERROR, Sending the MQTT subscribe failed for topic: "));
    Serial.println(p_topic);
  }
}

/*
  Function called to publish to a MQTT topic with the given payload
*/
void publishToMQTT(char *p_topic, char *p_payload)
{
  if (mqttClient.publish(p_topic, p_payload, true))
  {
    Serial.print(F("[MQTT]: publishToMQTT - MQTT message published successfully, topic: "));
    Serial.print(p_topic);
    Serial.print(F(", payload: "));
    Serial.println(p_payload);
  }
  else
  {
    Serial.println(F("[MQTT]: publishToMQTT - ERROR, MQTT message not published, either connection lost, or message too large. Topic: "));
    Serial.print(p_topic);
    Serial.print(F(" , payload: "));
    Serial.println(p_payload);
  }
}

void handleMQTTMessage(char *p_topic, byte *p_payload, unsigned int p_length)
{
  // concatenates the payload into a string
  String payload;
  for (uint8_t i = 0; i < p_length; i++)
  {
    payload.concat((char)p_payload[i]);
  }

  Serial.print("[MQTT]: handleMQTTMessage - Message arrived, topic: ");
  Serial.print(p_topic);
  Serial.print(", payload: ");
  Serial.println(payload);

  if (String(MQTT_LIGHT_COMMAND_TOPIC).equals(p_topic))
  {
    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, p_payload);

    if (error)
    {
      Serial.print(F("ERROR: deserializeJson() failed with code "));
      Serial.println(error.c_str());
      return;
    }

    if (doc.containsKey("state"))
    {
      if (strcmp(doc["state"], MQTT_PAYLOAD_ON) == 0)
      {
        if (bulb.setState(true))
        {
          Serial.print(F("INFO: State changed to: "));
          Serial.println(bulb.getState());
          cmd = CMD_STATE_CHANGED;
        }
      }
      else if (strcmp(doc["state"], MQTT_PAYLOAD_OFF) == 0)
      {
        if (bulb.setState(false))
        {
          Serial.print(F("INFO: State changed to: "));
          Serial.println(bulb.getState());
          cmd = CMD_STATE_CHANGED;
        }
      }
    }

    if (doc.containsKey("color"))
    {
      JsonObject color = doc["color"];
      uint8_t r = color["r"];
      uint8_t g = color["g"];
      uint8_t b = color["b"];

      if (bulb.setColor(r, g, b))
      {
        Serial.print(F("INFO: Color changed to: "));
        Serial.print(bulb.getColor().red);
        Serial.print(F(", "));
        Serial.print(bulb.getColor().green);
        Serial.print(F(", "));
        Serial.println(bulb.getColor().blue);
        cmd = CMD_STATE_CHANGED;
      }
    }

    if (doc.containsKey("brightness"))
    {
      int brightness = doc["brightness"];
      if (bulb.setBrightness(brightness))
      {
        Serial.print(F("INFO: Brightness changed to: "));
        Serial.println(bulb.getBrightness());
        cmd = CMD_STATE_CHANGED;
      }
    }

    if (doc.containsKey("white_value"))
    {
      int white_value = doc["white_value"];

      if (bulb.setWhite(white_value))
      {
        Serial.print(F("INFO: White changed to: "));
        Serial.println(bulb.getColor().white);
        cmd = CMD_STATE_CHANGED;
      }
    }

    if (doc.containsKey("color_temp"))
    {
      int color_temp = doc["color_temp"];

      if (bulb.setColorTemperature(color_temp))
      {
        Serial.print(F("INFO: Color temperature changed to: "));
        Serial.println(bulb.getColorTemperature());
        cmd = CMD_STATE_CHANGED;
      }
    }
  }
}
