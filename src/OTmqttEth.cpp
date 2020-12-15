/* Board Mega 2560    */

#include <SPI.h>
#include <Ethernet2.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#include <OpenTherm.h>

#include "User.h"

#define THERMOSTAT_IN 2
#define THERMOSTAT_OUT 4
#define BOILER_IN 3
#define BOILER_OUT 5
#define PORTAIL 23
#define ONE_WIRE_BUS 25

const int inPin = BOILER_IN;
const int outPin = BOILER_OUT;

uint8_t mac[6] = {0x00,0x01,0x02,0x03,0x04,0x05};

OpenTherm ot(inPin, outPin);
EthernetClient EthClient;
PubSubClient client(EthClient);
char buf[10];

float spBoiler = 0, 
      last_spBoiler = 0,
      spECS = 0,
      last_spECS = 0;  
bool  stBoiler = false,
      stECS = true;
unsigned long ts = 0, new_ts = 0; //timestamp
unsigned long response;

bool enableCentralHeating = false;
bool isBoilerEnabled = true;
bool Last_isBoilerEnabled = false;
bool enableHotWater = true;
bool isDhwEnabled = true;
bool Last_isDhwEnabled = false;
bool enableCooling = false;
bool isGazOn = false;
bool Last_isGazOn = true;
float boilertemp;
float Last_boilertemp = 0;
float ECStemp;
float Last_ECStemp = 0;
float exttemp;
float Last_exttemp = 0;
float modulation = 0;
float Last_modulation = -1;
OpenThermResponseStatus responseStatus;

void handleInterrupt() {
  ot.handleInterrupt();
}

void publish_all() {
  if (boilertemp != Last_boilertemp) {
    String(boilertemp).toCharArray(buf, 10);
    client.publish("TH/boiler/temp", buf, true);
    Last_boilertemp = boilertemp;
  }
  if (ECStemp != Last_ECStemp) {
    String(ECStemp).toCharArray(buf, 10);
    client.publish("TH/ECS/temp", buf, true);
    Last_ECStemp = ECStemp;
  }
  if (exttemp != Last_exttemp) {
    String(exttemp).toCharArray(buf, 10);
    client.publish("TH/ext/temp", buf, true);
    Last_exttemp = exttemp;
  }
  if (modulation != Last_modulation) {
    String(modulation).toCharArray(buf, 10);
    client.publish("TH/boiler/modulation", buf, true);
    Last_modulation = modulation;
  }
  if (isBoilerEnabled != Last_isBoilerEnabled) {
    String(isBoilerEnabled ? "ON" : "OFF").toCharArray(buf, 10);
    client.publish("TH/boiler/state", buf, true);
    Last_isBoilerEnabled = isBoilerEnabled;
  }
  if (isDhwEnabled != Last_isDhwEnabled) {
    String(isDhwEnabled ? "ON" : "OFF").toCharArray(buf, 10);
    client.publish("TH/ECS/state", buf, true);
    Last_isDhwEnabled = isDhwEnabled;
  }
  if (Last_isGazOn != isGazOn) {
    String(isGazOn ? "ON" : "OFF").toCharArray(buf, 10);
    client.publish("TH/flame/state", buf, true);
    Last_isGazOn = isGazOn;
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  String str = String();
  for (unsigned int i = 0; i < length; i++) {
    str += (char)payload[i];
  }
  if (strcmp(topic, "TH/consigne/boiler/temp") == 0) {
    Serial.println("spBoiler=" + str);
    spBoiler = str.toFloat();
  }
  if (strcmp(topic, "TH/consigne/ECS/temp") == 0) {
    Serial.println("spECS=" + str);
    spECS = str.toFloat();
  }
  if (strcmp(topic, "TH/consigne/boiler/state") == 0) {
    Serial.println("stBoiler=" + str);
    stBoiler = str.toInt();
  }
  if (strcmp(topic, "TH/consigne/ECS/state") == 0) {
    Serial.println("stECS=" + str);
    stECS = str.toInt();
  }
    if (strstr(topic, "out") != NULL) {

    DynamicJsonBuffer  jsonBuffer(200);
    JsonObject& root = jsonBuffer.parseObject(str);
    if (!root.success()) {
      Serial.println("parseObject() failed");
      return;
    }
    int idx = root["idx"];
    if (idx == 118) {
      Serial.println("portail !");
      digitalWrite(PORTAIL, LOW);
      delay(500);
      digitalWrite(PORTAIL, HIGH);
    }
  }
}

void reconnect() {
//  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("OTmqtt", mqtt_user, mqtt_password)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      publish_all();
      // ... and resubscribe
      client.subscribe("TH/consigne");
      client.subscribe("DZ/out");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
//      delay(5000);
    }
//  }
}

void setup(void) {
  Serial.begin(115200);
  Ethernet.begin(mac);
  Serial.print("My IP : ");
  Serial.println(Ethernet.localIP());
  ts = millis();

  //Init OpenTherm Controller
  ot.begin(handleInterrupt);

  //Init MQTT Client
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  pinMode(PORTAIL, OUTPUT);
  digitalWrite(PORTAIL, HIGH);

}

float getDHWTemperature() {
        unsigned long response = ot.sendRequest(ot.buildRequest(OpenThermMessageType::READ_DATA, OpenThermMessageID::Tdhw, 0));
        return ot.isValidResponse(response) ? ot.getFloat(response) : 0;
}

float getOutsideTemperature() {
        unsigned long response = ot.sendRequest(ot.buildRequest(OpenThermMessageType::READ_DATA, OpenThermMessageID::Toutside, 0));
        return ot.isValidResponse(response) ? ot.getFloat(response) : 0;
}

float getModulationLevel() {
        unsigned long response = ot.sendRequest(ot.buildRequest(OpenThermMessageType::READ_DATA, OpenThermMessageID::RelModLevel, 0));
        return ot.isValidResponse(response) ? ot.getFloat(response) : 0;
}

bool setECSTemperature(float temperature) {
  unsigned int data = ot.temperatureToData(temperature);
	unsigned long response = ot.sendRequest(ot.buildRequest(OpenThermMessageType::WRITE_DATA, OpenThermMessageID::TdhwSet, data));
	return ot.isValidResponse(response);
}

void loop(void) {
  new_ts = millis();
  if (new_ts - ts > 1000) {
    
    response = ot.setBoilerStatus(stBoiler, stECS, enableCooling);
    responseStatus = ot.getLastResponseStatus();
    if (responseStatus == OpenThermResponseStatus::SUCCESS) {
      isBoilerEnabled = ot.isCentralHeatingActive(response);
      isDhwEnabled = ot.isHotWaterActive(response);
      isGazOn = ot.isFlameOn(response);
      boilertemp = ot.getBoilerTemperature();
      ECStemp = getDHWTemperature();
      exttemp = getOutsideTemperature();
      modulation = getModulationLevel();
      if (last_spBoiler != spBoiler ) {
        ot.setBoilerTemperature(spBoiler);
        last_spBoiler = spBoiler;
      }
      if (last_spECS != spECS ) {
        setECSTemperature(spECS);
        last_spECS = spECS;
      }
      
      publish_all();
      
    } else {
      Serial.println("Error: Invalid boiler response " + String(response, HEX));
    }
    ts = new_ts;
  }

  //MQTT Loop
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}
