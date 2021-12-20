#include <ESP8266WiFi.h>
#include "PubSubClient.h"

#include <BME280I2C.h>
#include <Wire.h>
#include <BH1750.h>


#define SERIAL_BAUD 115200
#define SDA_PIN D2
#define SCL_PIN D1
#define LUX_WAIT_DELAY_MS 100
#define MQTT_LOOP_NUM 10
#define MQTT_LOOP_DELAY_MS 100

long SLEEPTIME = 1 * 5e6; // 15 min

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
const char* MQTT_USERNAME = "MeteoProbeOut";
const char* MQTT_PASSWD = "nzl98m9";
const char* MQTT_CLIENT_ID = "meteo_probe1";
const char* MQTT_TEMP_TOPIC = "home/sensors/out/temperature";
const char* MQTT_PRESS_TOPIC = "home/sensors/out/pressure";
const char* MQTT_HUM_TOPIC = "home/sensors/out/humidity";
const char* MQTT_LUX_TOPIC = "home/sensors/out/lux";
const char* MQTT_CFG_TOPIC = "home/sensors/out/cfg/reload";
WiFiClient espClient;
PubSubClient mqtt_client(espClient);
// ###################################

void setup()
{
  environ_data_t environ_data;

  modulesSetup();

  get_thp_data(&environ_data);
  get_lux_data(&environ_data);

  waitForWifiConnection();

  mqtt_client.setServer(MQTT_BROKER_IP, 1883);
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
  ESP.deepSleep( SLEEPTIME, WAKE_RF_DISABLED );
}

void modulesSetup(){
  Serial.begin(SERIAL_BAUD);
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
}

void sendMQTTmessage(environ_data_t *environ_data)
{
  Serial.printf("Message: temp %f, pres %f, hum %f, lux %f\n", environ_data->temp, environ_data->pres, environ_data->hum, environ_data->lux);
  mqtt_client.publish(MQTT_TEMP_TOPIC, String(environ_data->temp, 4).c_str(), false);
  mqtt_client.publish(MQTT_PRESS_TOPIC, String(environ_data->pres, 4).c_str(), false);
  mqtt_client.publish(MQTT_HUM_TOPIC, String(environ_data->hum, 4).c_str(), false);
  mqtt_client.publish(MQTT_LUX_TOPIC, String(environ_data->lux, 4).c_str(), false);
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
      ESP.deepSleep( SLEEPTIME, WAKE_RF_DISABLED );
      return; // Not expecting this to be called, the previous call will never return.
    }
    delay( 50 );
    wifiStatus = WiFi.status();
  }
}

void get_thp_data(environ_data_t *environ_data)
{
   BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
   BME280::PresUnit presUnit(BME280::PresUnit_Pa);
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
 
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);
 
  Serial.print("Message:");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
 
}

void loop()
{
}
