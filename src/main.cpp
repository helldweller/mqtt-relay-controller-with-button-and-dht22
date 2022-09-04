#include <ESP8266WiFi.h>
#include <PubSubClient.h> // MQTT_MAX_PACKET_SIZE 512
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ArduinoJson.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Thread.h>

// https://smdx.ru/blog/manuals/wemos-d1-mini-pinout  
#define ONE_WIRE_BUS D1
#define RELAY_1 D5
#define RELAY_2 D6
#define DHTPIN D2
#define DHTTYPE DHT22 // DHT 22 (AM2302)
#define BUTTON_1 D0
// LED_BUILTIN D4 2
// DHT on D0 not works

#define ON true
#define OFF false
#define DEFAULT_ON true
#define DEFAULT_OFF false
#define INVERTED true
#define RELAY_2_TIMER 3600*6 //sec (6h)
#define RELAY_2_UPPER_HUMIDITY 85 // %
#define RELAY_2_LOWER_HUMIDITY 70 // %

float temperature;
int countSensors;

const char* ssid = "mywifi";
const char* pswd = "mywifipassword";
const char* mqtt_server = "mqtt.server.dev";
const char* mqtt_topic = "ihome";
bool wifiIsOk = false;
bool mqttIsOk = false;

class Relay
{
private:
  int  pin;
  bool state;
  long timer;
  long timerStarted;
  bool inverted = false;
  void _turn(bool state) {
    this->state = state;
    // true == HIGH, +V (OFF); false == LOW, -V (ON)
    if (inverted) {
      digitalWrite(pin, state);
    }
    else {
      digitalWrite(pin, !state);
    }
    delay(50);
  }
public:
  Relay(int pin, bool state)
  {
    this->pin = pin;
    this->state = state;
    timer = 0;
    timerStarted = 0;
    pinMode(pin, OUTPUT);
    turn(state);
  }
  Relay(int pin, bool state, bool inverted)
  {
    this->pin = pin;
    this->state = state;
    this->inverted = inverted;
    timer = 0;
    timerStarted = 0;
    pinMode(pin, OUTPUT);
    turn(state);
  }
  bool isEnabled()
  {
    return state;
  }
  void turn(bool state)
  {
    _turn(state);
  }
  void turn(bool state, int timer) // seconds
  {
    this->timer = timer * 1000;
    this->timerStarted = millis();
    _turn(state);
  }
  void check()
  {
    long now = millis();
    if (timer > 0 && now - timerStarted > timer ) {
      timer = 0;
      timerStarted = 0;
      _turn(OFF);
    }
  }
  int getTimer()
  {
    long now = millis();
    if (now - timerStarted < timer ) {
      return ( timer - ( now - timerStarted )) / 1000;
    }
    return 0;
  }
};

class Button
{
private:
  int pin;
  bool state;
  bool laststate;
  bool clickmode;

public:
  Button(int pin)
  {
    this->pin = pin;
    pinMode(pin, INPUT);
  }
  bool getState()
  {
    state = digitalRead(pin);
    return state;
  }
  bool isClicked()
  {
    laststate = getState();
    delay(10);
    if (laststate != getState()) {
      return true;
    }
    return false;
  }
};

Relay Relay_1(RELAY_1, DEFAULT_OFF, INVERTED); // outdor lamp
Relay Relay_2(RELAY_2, DEFAULT_OFF, INVERTED); // vent motor
Relay BuiltinLed(LED_BUILTIN, DEFAULT_ON); // led
Button Button_1(BUTTON_1);
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DHT_Unified dht(DHTPIN, DHTTYPE);
Thread RelayControlThread = Thread(); // Thread 1, relay control by sensor
Thread MqttThread = Thread(); // Thread 2, send data to mqtt
WiFiClient espClient;
PubSubClient mqttClient(espClient);

String macToStr(const uint8_t* mac) {
  String result;
  for (int i = 0; i < 6; ++i) {
    result += String(mac[i], 16);
    if (i < 5)
      result += ':';
  }
  return result;
}

String composeClientID() {
  uint8_t mac[6];
  WiFi.macAddress(mac);
  String clientId;
  clientId += "esp-";
  clientId += macToStr(mac);
  //clientId += "-";
  //clientId += String(micros() & 0xff, 16);
  return clientId;
}

bool checkMqtt() {
  if (!mqttClient.connected()) {
    Serial.print("MQTT: Trying connect to ");
    Serial.println(mqtt_server);
    String clientId = composeClientID() ;
    // clientId += "-";
    // clientId += String(micros() & 0xff, 16); // to randomise. sort of
    if (mqttClient.connect(clientId.c_str())) {
      //mqttClient.publish(mqtt_topic, ("connected " + composeClientID()).c_str() , true );
      String subscription;
      subscription += mqtt_topic;
      subscription += "/";
      subscription += composeClientID() ;
      subscription += "/in";
      mqttClient.subscribe(subscription.c_str() );
      if (!mqttIsOk) {
        Serial.print("MQTT: Subscribed to ");
        Serial.println(subscription);
        mqttIsOk = true;
      }
      return true;
    }
    else {
      Serial.print("Failed. rc=");
      Serial.print(mqttClient.state());
      Serial.print(" wifi=");
      Serial.println(WiFi.status());
      mqttIsOk = false;
      return false;
    }
  }
  else if (!mqttIsOk)
  {
    mqttIsOk = true;
    Serial.print("MQTT: Connected");
  }
  return true;
}

bool checkWiFi() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.print("Wi-Fi: Trying connect to ");
    Serial.println(ssid);
    wifiIsOk = false;
    return false;
  }
  else if (!wifiIsOk)
  {
    wifiIsOk = true;
    Serial.print("Wi-Fi: Connected, IP address: ");
    Serial.println(WiFi.localIP());
  }
  return true;
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (unsigned int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  StaticJsonDocument<256> doc;
  deserializeJson(doc, payload, length);

  if (doc.containsKey("relay_1")) {
    Relay_1.turn(doc["relay_1"]);
  }
  if (doc.containsKey("relay_2")) {
    Relay_2.turn(doc["relay_2"]);
  }
  if (doc.containsKey("builtin_led")) {
    BuiltinLed.turn(doc["builtin_led"]);
  }
}

void loopRelayControl() {
  sensors_event_t event;
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println("Error reading humidity!");
  }
  else {
    if (event.relative_humidity > RELAY_2_UPPER_HUMIDITY) {
      Relay_2.turn(ON, RELAY_2_TIMER);
    }
    else if (event.relative_humidity < RELAY_2_LOWER_HUMIDITY) {
      Relay_2.turn(OFF);
    }
  }

  // check timer ending
  Relay_1.check();
  Relay_2.check();
}

void loopMqttThread() {
  if (!checkWiFi()) {
    return;
  }
  if (!checkMqtt()) {
    return;
  }

  sensors.requestTemperatures();
  temperature = sensors.getTempCByIndex(0);
  StaticJsonDocument<512> doc; // for corect error if to larger than 256
  JsonObject jsonRoot = doc.to<JsonObject>();
  //jsonRoot["board"] = "d1_mini";
  //jsonRoot["place"] = "banya";

  // Controls
  JsonArray JsonArrayControls = jsonRoot.createNestedArray("controls");
  JsonObject jsonControl1 = JsonArrayControls.createNestedObject();
  jsonControl1["key"] = "relay_1";
  jsonControl1["pin"] = RELAY_1;
  jsonControl1["mode"] = Relay_1.isEnabled();
  jsonControl1["timer"] = Relay_1.getTimer();
  JsonObject jsonControl2 = JsonArrayControls.createNestedObject();
  jsonControl2["key"] = "relay_2";
  jsonControl2["pin"] = RELAY_2;
  jsonControl2["mode"] = Relay_2.isEnabled();
  jsonControl2["timer"] = Relay_2.getTimer();

  // Sensors
  JsonArray JsonArraySensors = jsonRoot.createNestedArray("sensors");
  // JsonObject jsonSensor1 = JsonArraySensors.createNestedObject();
  // jsonSensor1["temp"] = temperature;
  // jsonSensor1["pin"] = ONE_WIRE_BUS;
  // jsonSensor1["addr"] = "0";
  // jsonSensor1["type"] = "ds18b20";

  JsonObject jsonSensor2 = JsonArraySensors.createNestedObject();
  jsonSensor2["pin"] = DHTPIN;
  jsonSensor2["type"] = DHTTYPE;
  JsonArray errorList2 = jsonSensor2.createNestedArray("errors");

  sensors_event_t event;
  // DHT temp
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println("Error reading temperature!");
    jsonSensor2["temp"] = 0;
    errorList2.add("Error reading dht22 temp!");
  }
  else {
    jsonSensor2["temp"] = event.temperature;
  }

  // DHT humidity
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println("Error reading humidity!");
    jsonSensor2["humidity"] = 0;
    errorList2.add("Error reading dht22 humidity!");
  }
  else {
    jsonSensor2["humidity"] = event.relative_humidity;
  }

  String payload;
  serializeJson(jsonRoot, payload);

  String pubTopic;
  pubTopic += mqtt_topic ;
  pubTopic += "/";
  pubTopic += composeClientID();
  pubTopic += "/out";
  Serial.print("Publish topic: ");
  Serial.println(pubTopic);
  Serial.print("Publish message: ");
  Serial.println(payload);
  if (payload.length() < 256) {
    mqttClient.publish( (char*) pubTopic.c_str() , (char*) payload.c_str(), true );
  }
  else {
    Serial.print("Error: Message to large! ");
    Serial.println(payload.length());
    // try change MQTT_MAX_PACKET_SIZE and StaticJsonDocument<up to 1024> doc
  }

}

void setup() {
  Serial.begin(115200);
  sensors.begin();
  sensors.setResolution(12);
  dht.begin();

  WiFi.begin(ssid, pswd);
  mqttClient.setServer(mqtt_server, 1883);
  mqttClient.setCallback(mqttCallback);

  RelayControlThread.onRun(loopRelayControl); // method
  RelayControlThread.setInterval(2000); // ms
  MqttThread.onRun(loopMqttThread); // method
  MqttThread.setInterval(20000); // ms

  Relay_1.turn(ON, 5); // test on start for 5 sec
  Relay_2.turn(ON, 5);
}

void loop() {

  // outdor light (relay1 by button1 click)
  if(Button_1.isClicked()) {
    Relay_1.turn(!Relay_1.isEnabled());
  }

  // Threads
  if (RelayControlThread.shouldRun()) RelayControlThread.run();
  if (MqttThread.shouldRun()) MqttThread.run();

  // mqtt events, callback
  mqttClient.loop();

}
