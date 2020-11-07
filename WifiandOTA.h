#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
extern "C" {
  #include "user_interface.h"
}

//#define debug                   // Comment for Switch behavior
//-------------- String declairation--------------------------
const char* ssid                = "LakeViewWiFi";
const char* password            = "P@ssLakeView";
const char* mqtt_server         = "192.168.2.12";
const char* mqtt_uname          = "onkar20";
const char* mqtt_pass           = "onkar20";
const char* mqtt_device_name    = "ESP8266WindowBlind1";
const char* ota_device_name     = "Family_Room_Window_Blind";
const char* ota_password        = "onkar20";
const char* mqtt_topic_command  = "home/familyRoom/windowBlind1/command";
const char* mqtt_topic_state    = "home/familyRoom/windowBlind1/state";

//---------------------------------- setup_wifi function ---------------------------------------
void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  #ifdef debug
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);
  #endif
  //for Light sleep
  WiFi.mode(WIFI_STA);
  wifi_set_sleep_type(LIGHT_SLEEP_T); 
  
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    #ifdef debug
      Serial.print(".");
    #endif
  }

  #ifdef debug
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
  #endif
}

//--------------------------------setup_OTA function --------------------------------------------
void setup_OTA() {
  // Port defaults to 8266
  ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname(ota_device_name);

  // No authentication by default
  ArduinoOTA.setPassword((const char *)ota_password);

  #ifdef debug
    ArduinoOTA.onStart([]() {
      Serial.println("Start");
    });
    ArduinoOTA.onEnd([]() {
      Serial.println("\nEnd");
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });
  #endif
  ArduinoOTA.begin();
}
