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

float sp = 0, //set point
      mt1 = 0,      //remote measured temperature 1
      mt1_last = 0, //prior remote temperature 1
      mt2 = 0,      //remote measured temperature 2
      mt2_last = 0, //prior remote temperature 2
      mt3 = 0,      //remote measured temperature 3
      mt3_last = 0, //prior remote temperature 3
      mt4 = 0,      //remote measured temperature 4
      mt4_last = 0, //prior remote temperature 4
      pv = 0,       //current temperature
      pv_last = 0,  //prior temperature
      ierr = 0,     //integral error
      dt = 0,       //time between measurements
      op = 0,       //PID controller output
      last_op = 0;
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
float powerP = 0;
float powerA = 0;
OpenThermResponseStatus responseStatus;

void handleInterrupt() {
  ot.handleInterrupt();
}

float pid(float sp, float pv, float pv_last, float& ierr, float dt, float ext) {
  float Kc = 8.0; // K / %Heater
  float tauI = 50.0; // sec
  float tauD = 1.0;  // sec
  // PID coefficients
  float KP = Kc;
  float KI = Kc / tauI;
  float KD = Kc * tauD;
  // upper and lower bounds on heater level
  float froid = 20 - ext;
  float ophi = (froid * 2.5) + 20.0;
  float minop = 70;
  ophi = min(minop, ophi);
  float oplo = 9;
  // calculate the error
  float error = sp - pv;
  // calculate the integral error
  ierr = ierr + KI * error * dt;
  // calculate the measurement derivative
  float dpv = (pv - pv_last) / dt;
  // calculate the PID output
  float P = KP * error; //proportional contribution
  float I = ierr; //integral contribution
  float D = -KD * dpv; //derivative contribution
  float op = P + I + D;
  // implement anti-reset windup
  if ((op < oplo) || (op > ophi)) {
    I = I - KI * error * dt;
    // clip output
    op = max(oplo, min(ophi, op));
  }
  ierr = I;
  Serial.println("sp=" + String(sp) + " pv=" + String(pv) + " dt=" + String(dt) + " op=" + String(op) + " ierr " + String(ierr) + " P=" + String(P) + " I=" + String(I) + " D=" + String(D));
  return op;
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
  if (strcmp(topic, "NA/salon/therm/SpTemp") == 0) {
    Serial.println("sp=" + str);
    sp = str.toFloat();
  }
  if (strcmp(topic, "OH/SalonTemperature/state") == 0) {
    Serial.println("mt1=" + str);
    mt1 = str.toFloat();
  }
  if (strcmp(topic, "NA/salon/therm/MeasTemp") == 0) {
    Serial.println("mt2=" + str);
    mt2 = str.toFloat();
  }
  if (strcmp(topic, "NA/chambreP/therm/MeasTemp") == 0) {
    Serial.println("mt3=" + str);
    mt3 = str.toFloat();
  }
  if (strcmp(topic, "NA/chambreA/therm/MeasTemp") == 0) {
    Serial.println("mt4=" + str);
    mt4 = str.toFloat();
  }
  if (strcmp(topic, "NA/chambreP/therm/HeatP") == 0) {
    Serial.println("powerP=" + str);
    powerP = str.toFloat();
  }
  if (strcmp(topic, "NA/chambreA/therm/HeatP") == 0) {
    Serial.println("powerA=" + str);
    powerA = str.toFloat();
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
      client.subscribe("OH/SalonTemperature/#");
      client.subscribe("NA/#");
      client.subscribe("domoticz/out");
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
        unsigned long response = ot.sendRequest(ot.buildRequest(OpenThermMessageType::READ_DATA, OpenThermMessageID::Tdhw, 0));
        return ot.isValidResponse(response) ? ot.getFloat(response) : 0;
}

float getModulationLevel() {
        unsigned long response = ot.sendRequest(ot.buildRequest(OpenThermMessageType::READ_DATA, OpenThermMessageID::RelModLevel, 0));
        return ot.isValidResponse(response) ? ot.getFloat(response) : 0;
}

void loop(void) {
  new_ts = millis();
  if (new_ts - ts > 1000) {
   
    //    mt1 = sensors.CByIndex(0);
    if (mt1 <= 0.0) {
      mt1 = mt2;
    }
    //    Serial.println("mt1="+String(mt1) + " mt2=" + String(mt2) + " mt3=" + String(mt3) + " mt4=" + String(mt4) + " powerP " + String(powerP) + " powerA=" + String(powerA));
    pv = (mt1 + mt2 + mt3 + mt4) / 4;
    if ((powerP > 0) || (powerA > 0)) {
      pv = sp - (powerP / 70) - (powerA / 70);
    }

    dt = (new_ts - ts) / 1000.0;
    enableHotWater = true;
 
    op = pid(sp, pv, pv_last, ierr, dt, exttemp);
    enableCentralHeating = (op > 10);
    pv_last = pv;
    
    response = ot.setBoilerStatus(enableCentralHeating, enableHotWater, enableCooling);
    responseStatus = ot.getLastResponseStatus();
    if (responseStatus == OpenThermResponseStatus::SUCCESS) {
      isBoilerEnabled = ot.isCentralHeatingActive(response);
      isDhwEnabled = ot.isHotWaterActive(response);
      isGazOn = ot.isFlameOn(response);
      boilertemp = ot.getBoilerTemperature();
      ECStemp = getDHWTemperature();
      exttemp = getOutsideTemperature();
      modulation = getModulationLevel();
      if (last_op != op ) {
        ot.setBoilerTemperature(op);
      }
      publish_all();
      last_op = op;
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
