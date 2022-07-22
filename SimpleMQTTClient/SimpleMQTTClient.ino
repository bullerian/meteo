#include <ESP8266WiFi.h>
#include "PubSubClient.h"

#include <BME280I2C.h>
#include <Wire.h>
#include <BH1750.h>
#include <EEPROM.h>


#define SERIAL_BAUD 115200
#define SDA_PIN 13
#define SCL_PIN 12
#define LUX_WAIT_DELAY_MS 100
#define MQTT_LOOP_NUM 10
#define MQTT_LOOP_DELAY_MS 100
#define DEBUG

static const long SLEEP_MULT = 60e6;
static const int DEFAULT_SLEEP_MIN = 1;
static const int EEPROM_ADDR = 0;
#define SLEEP_MIN(m) (m * SLEEP_MULT)

typedef struct {
  long sleeptime;
} config_t;
config_t config;

// ###################  Sensors related 
BH1750 lightMeter;

BME280I2C::Settings settings(
   BME280::OSR_X1,
   BME280::OSR_X1,
   BME280::OSR_X1,
   BME280::Mode_Forced,
   BME280::StandbyTime_1000ms,
   BME280::Filter_Off,
   BME280::SpiEnable_False,
   BME280I2C::I2CAddr_0x76
);
BME280I2C bme(settings);

typedef struct {
  float lux, temp, hum, pres;
} environ_data_t;
// ########################################

// ############################# Connection related
const char* WLAN_SSID = "MASLO";
const char* WLAN_PASSWD = "Moroz1vo";
const char* MQTT_BROKER_IP = "192.168.1.106";
const int MQTT_PORT = 1883;
const char* MQTT_USERNAME = "MeteoProbeOut";
const char* MQTT_PASSWD = "nzl98m9";
const char* MQTT_CLIENT_ID = "meteo_probe1";
const char* MQTT_TEMP_TOPIC =  "home/sensors/out/temperature";
const char* MQTT_PRESS_TOPIC = "home/sensors/out/pressure";
const char* MQTT_HUM_TOPIC =   "home/sensors/out/humidity";
const char* MQTT_LUX_TOPIC =   "home/sensors/out/lux";
const char* MQTT_CFG_TOPIC =   "home/sensors/out/cfg/period";
WiFiClient espClient;
PubSubClient mqtt_client(espClient);
// ###################################

void setup()
{
  environ_data_t environ_data;
  delay(50);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  modulesSetup();

  get_thp_data(&environ_data);
  get_lux_data(&environ_data);

  waitForWifiConnection();

//  while(1) {
//    Serial.println(WiFi.RSSI());
//    delay(200);
//  }

  mqtt_client.setServer(MQTT_BROKER_IP, MQTT_PORT);
  mqtt_client.setCallback(callback);
  if (!mqtt_client.connected()) {
    reconnect();
  }

  sendMQTTmessage(&environ_data);

  for(int i=0; i < MQTT_LOOP_NUM; i++)
  {
   mqtt_client.loop();
   delay(MQTT_LOOP_DELAY_MS);
  }

  WiFi.disconnect(true);
  delay(1);
  Serial.println("Going to sleep");
  digitalWrite(LED_BUILTIN, HIGH);
  ESP.deepSleep(SLEEP_MIN(5), WAKE_RF_DISABLED );
}

void modulesSetup(){
#ifdef DEBUG
  Serial.begin(SERIAL_BAUD);
#endif
  Serial.println("Woken up!");
  WiFi.mode( WIFI_OFF );
  WiFi.forceSleepBegin();
  delay(1);

  Wire.begin(SDA_PIN, SCL_PIN);
  while(!bme.begin())
  {
    delay(1000);
  }
  settings.tempOSR = BME280::OSR_X4;
  bme.setSettings(settings);
  lightMeter.begin(BH1750::ONE_TIME_HIGH_RES_MODE);

  EEPROM.begin(sizeof(config));
  EEPROM.get(EEPROM_ADDR, config);
  if (!config.sleeptime) config.sleeptime = DEFAULT_SLEEP_MIN;
}

void sendMQTTmessage(environ_data_t *environ_data)
{
#ifdef DEBUG
  Serial.printf("Message: temp %.02f, pres %.02f, hum %.02f, lux %.02f\n", environ_data->temp, environ_data->pres, environ_data->hum, environ_data->lux);
#endif
  mqtt_client.publish(MQTT_TEMP_TOPIC, String(environ_data->temp, 2).c_str(), false);
  mqtt_client.publish(MQTT_PRESS_TOPIC, String(environ_data->pres, 2).c_str(), false);
  mqtt_client.publish(MQTT_HUM_TOPIC, String(environ_data->hum, 2).c_str(), false);
  mqtt_client.publish(MQTT_LUX_TOPIC, String(environ_data->lux, 2).c_str(), false);
  /* Close MQTT client cleanly */
  mqtt_client.disconnect();
}

void reconnect() {
  while (!mqtt_client.connected())
  {
    if (mqtt_client.connect(MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWD))
    {
      Serial.print("Connected");
      mqtt_client.subscribe(MQTT_CFG_TOPIC);
    } else {
      Serial.print("failed, rc= ");
      Serial.print(mqtt_client.state());
      delay(1000);
    }
  }
}

void waitForWifiConnection(){
  WiFi.forceSleepWake();
  delay(1);
  WiFi.mode( WIFI_STA );
  WiFi.begin(WLAN_SSID, WLAN_PASSWD);
    //------now wait for connection
  int retries = 0;
  int wifiStatus = WiFi.status();
  while ( wifiStatus != WL_CONNECTED ) {
    retries++;
    if ( retries == 100 ) {
      // Quick connect is not working, reset WiFi and try regular connection
      WiFi.disconnect();
      delay( 10 );
      WiFi.forceSleepBegin();
      delay( 10 );
      WiFi.forceSleepWake();
      delay( 10 );
      WiFi.begin( WLAN_SSID, WLAN_PASSWD );
    }
    if ( retries == 600 ) {
      // Giving up after 30 seconds and going back to sleep
      WiFi.disconnect( true );
      delay( 1 );
      WiFi.mode( WIFI_OFF );
      Serial.println("Going to sleep!");
      ESP.deepSleep( SLEEP_MULT * 1, WAKE_RF_DISABLED );
      return; // Not expecting this to be called, the previous call will never return.
    }
    delay( 50 );
    wifiStatus = WiFi.status();
  }
  Serial.println("WIFI connected!");
}

void get_thp_data(environ_data_t *environ_data)
{
   BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
   BME280::PresUnit presUnit(BME280::PresUnit_hPa);
   bme.read(environ_data->pres, environ_data->temp, environ_data->hum, tempUnit, presUnit);
}

void get_lux_data(environ_data_t *environ_data)
{
  while (!lightMeter.measurementReady(true)) {
    delay(LUX_WAIT_DELAY_MS);
  }
  environ_data->lux = lightMeter.readLightLevel();
  lightMeter.configure(BH1750::ONE_TIME_HIGH_RES_MODE);
}

void callback(char* topic, byte* payload, unsigned int length) {
 #ifdef DEBUG
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);
  Serial.print("Message:");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
#endif
  if (!length) return;
  EEPROM.begin(sizeof(config));
  config.sleeptime = payload[0];
  EEPROM.put(EEPROM_ADDR, config);
  EEPROM.commit();
  EEPROM.end();
}

void loop()
{
}
