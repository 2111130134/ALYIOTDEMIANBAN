#include <TFT_eSPI.h>
#include <ArduinoMqttClient.h>
#include <ESP8266WiFi.h>
#include <DFRobot_DHT11.h>
#include <ArduinoJson.h>

DFRobot_DHT11 DHT;
#define DHT11_PIN 5  //D1

TFT_eSPI tft = TFT_eSPI();

char ssid[] = "30134";    // your network SSID (name)
char pass[] = "123456789";    // your network password (use for WPA, or use as key for WEP)

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

const char broker[]    = "a14mKira0LS.iot-as-mqtt.cn-shanghai.aliyuncs.com";
int        port        = 1883;
const char inTopic[]   = "/sys/a14mKira0LS/esp8266/thing/service/property/set";
const char outTopic[]  = "/sys/a14mKira0LS/esp8266/thing/event/property/post";


const char skipTopic[] = "/sys/a14mKira0LS/esp8266/thing/event/property/post_reply";

const long interval = 5000;
unsigned long previousMillis = 0;

int count = 0;

String inputString ="";

void setup() {
  
  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  pinMode(2, OUTPUT);
  tft.init();
  tft.begin();
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(2);
  tft.setRotation(1);

  // attempt to connect to WiFi network:
  Serial.print("Attempting to connect to WPA SSID: ");
  Serial.println(ssid);
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(2000);
  }

  Serial.println("You're connected to the network");
  Serial.println();

  mqttClient.setId("a14mKira0LS.esp8266|securemode=2,signmethod=hmacsha256,timestamp=1698975135029|");

  mqttClient.setUsernamePassword("esp8266&a14mKira0LS", "6fbeacece261a4cf928cdbfa61b694bd2544837af4e426d4d05dce1db37eba44");

  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(broker);

  if (!mqttClient.connect(broker, port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());

    while (1);
  }

  Serial.println("You're connected to the MQTT broker!");
  Serial.println();

  mqttClient.onMessage(onMqttMessage);

  Serial.print("Subscribing to topic: ");
  Serial.println(inTopic);
  Serial.println();

  int subscribeQos = 1;

  mqttClient.subscribe(inTopic, subscribeQos);

  Serial.print("Waiting for messages on topic: ");
  Serial.println(inTopic);
  Serial.println();
}

void loop() {
  mqttClient.poll();

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    String payload;
    DHT.read(DHT11_PIN);
    int lux = analogRead(A0);

    DynamicJsonDocument json_msg(512);
    DynamicJsonDocument json_data(512);
    

    json_data["temp"] = DHT.temperature;
    json_data["humi"] = DHT.humidity;
    json_data["mlux"] = analogRead(A0);

    json_msg["params"] = json_data;
    json_msg["version"] = "1.0.0";

    serializeJson(json_msg, payload);

    Serial.print("Sending message to topic: ");
    Serial.println(outTopic);
    Serial.println(payload);

    bool retained = false;
    int qos = 1;
    bool dup = false;

    mqttClient.beginMessage(outTopic, payload.length(), retained, qos, dup);
    mqttClient.print(payload);
    mqttClient.endMessage();

    Serial.println();
          {
        tft.fillScreen(TFT_BLACK);
        tft.setCursor(0, 0);
        tft.print("temp: ");
        tft.print(DHT.temperature);
        tft.print("C");

        tft.setCursor(0, 30);
        tft.print("humi: ");
        tft.print(DHT.humidity);
        tft.print("%");
        
        tft.setCursor(0, 60);
        tft.print("mlux: ");
        tft.print(lux);
        tft.print("Lux");

        delay(200);
      }
    
    count++;

    
  }
  
}


void onMqttMessage(int messageSize) {
  String topic = mqttClient.messageTopic();

  if (topic == skipTopic) {
    // skip printing this message
    return;
  }

  Serial.print("Received a message with topic '");
  Serial.print(topic);
  Serial.print("', duplicate = ");
  Serial.print(mqttClient.messageDup() ? "true" : "false");
  Serial.print(", QoS = ");
  Serial.print(mqttClient.messageQoS());
  Serial.print(", retained = ");
  Serial.print(mqttClient.messageRetain() ? "true" : "false");
  Serial.print("', length ");
  Serial.print(messageSize);
  Serial.println(" bytes:");

  while (mqttClient.available()) {
    char inChar = (char)mqttClient.read();
    inputString += inChar;
    if(inputString.length() == messageSize) {
      DynamicJsonDocument json_msg(1024);
      DynamicJsonDocument json_items(1024);
      DynamicJsonDocument json_value(1024);

      deserializeJson(json_msg, inputString);

      String items = json_msg["items"];

      deserializeJson(json_items, items);
      String led = json_items["led"];

      deserializeJson(json_value, led);
      bool value = json_value["value"];

      if(value == 0) {
        Serial.println("off");
        digitalWrite(2, HIGH);
      } else {
        Serial.println("on");
        digitalWrite(2, LOW);
      }
      inputString = "";
    }
  }
  
  Serial.println();
}