#include <OneWire.h>
#include <DallasTemperature.h>
#include <afstandssensor.h>
#include <SPI.h>
#include <MFRC522.h>
#include <WiFi.h>  // Library for WiFi functionality
#include <PubSubClient.h>

#define RELAY_PIN_FAN 14  // Pin waarop IN1 van het 4-relaismodule is aangesloten voor de fan
#define MOISTURE_SENSOR_PIN 33
const int ldrPin = 34;
// Data wire is connected to GPIO 4
#define ONE_WIRE_BUS 27
#define RST_PIN 21  // Configurable, see typical pin layout above
#define SS_PIN 5    // Configurable, see typical pin layout above

const int buttonPin = 25;  // pin waar de knop is aangesloten
const int pompPin = 26;
bool pompState = true;  // variabele om de status van de LED bij te houden

const char* ssid = "SasHome_5GHz";
const char* password = "Quint4Willem5";

const char* mqtt_server = "192.168.0.178";
const int mqtt_port = 1883;
const char* MQTT_USER = "username";
const char* MQTT_PASSWORD = "test";
const char* MQTT_CLIENT_ID = "MQTTclient";
//Waarden
const char* MQTT_TOPICtemp = "home/serre/temperature";
const char* MQTT_TOPICvocht = "home/serre/vocht";
const char* MQTT_TOPICafstand = "home/serre/afstand";

const char* MQTT_REGEX = "home/([^/]+)/([^/]+)";


MFRC522 mfrc522(SS_PIN, RST_PIN);  // Create MFRC522 instance
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);
AfstandsSensor afstandssensor(4, 2);  // Initialiseer de afstandssensor met triggerPin op 4 en echoPin op 2.

// Pass our oneWire reference to Dallas Temperature sensor
DallasTemperature sensors(&oneWire);

unsigned long previousMillisFan = 0;     // Variabele om de tijd van de vorige relaisactivering van de fan bij te houden
unsigned long previousMillisSensor = 0;  // Variabele om de tijd van de vorige relaisactivering van de vochtigheidssensor bij te houden
const long fanInterval = 10000;          // Interval van 10 seconden (in milliseconden) voor de fan
const long fanDuration = 10000;          // Duur van de relaisactivering van de fan (in milliseconden)
bool fanState = false;                   // Hui

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Verbinding maken met WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi verbonden");
  Serial.println("IP adres: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  while (!mqttClient.connected()) {
    Serial.print("Verbinding maken met MQTT-broker...");
    if (mqttClient.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD)) {
      Serial.println(" verbonden");

    } else {
      Serial.print(" mislukt, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" opnieuw proberen in 5 seconden");
      delay(5000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  unsigned long currentTime = millis();
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  Serial.print("Message: ");
  String bericht;
  for (int i = 0; i < length; i++) {
    bericht += ((char)payload[i]);
  }
  Serial.println(bericht);
}


void setup() {
  // Start serial communication for debugging purposes
  Serial.begin(115200);
  pinMode(RELAY_PIN_FAN, OUTPUT);
  pinMode(ONE_WIRE_BUS, INPUT);
  pinMode(MOISTURE_SENSOR_PIN, INPUT);
  pinMode(pompPin, OUTPUT);  // zet de pin van de LED als output
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(22, OUTPUT);  // Zet pin 22 als uitgang (voor de buzzer)
  pinMode(34, INPUT);
  sensors.begin();
  setup_wifi();
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(callback);
  reconnect();
  while (!Serial)
    ;                  // Do nothing if no serial port is opened (added for Arduinos based on ATMEGA32U4)
  SPI.begin();         // Init SPI bus
  mfrc522.PCD_Init();  // Init MFRC522
  Serial.println(F("Scan PICC to see UID, SAK, type, and data blocks..."));
}

void pomp() {
  if (digitalRead(buttonPin) == LOW) {
    // Toggle de status van de LED
    pompState = !pompState;
    delay(250);
  }
  if (pompState) {
    digitalWrite(pompPin, HIGH);
  } else {
    // Zet de LED aan of uit afhankelijk van de pompState
    digitalWrite(pompPin, LOW);
    // Wacht een korte tijd om debounce te simuleren
  }
  Serial.println("pomp");
}

void loop() {
  mqttClient.loop();
  pomp();
  temperatuur();
  ventilator();
  bodemvocht();
  waterbak();
  light();
  RFID();
}

void temperatuur() {
  sensors.requestTemperatures();
  float temperatureC = sensors.getTempCByIndex(0);
  // Print the temperature to the Serial Monitor
  Serial.print("Temperature: ");
  Serial.print(temperatureC);
  Serial.println(" °C");
  String temps = String(temperatureC);
  mqttClient.publish(MQTT_TOPICtemp, temps.c_str());
  Serial.println(temps);
}

void ventilator() {
  unsigned long currentMillis = millis();  // Huidige tijd in milliseconden
  // Controleer of het tijd is om de fan in te schakelen
  if (currentMillis - previousMillisFan >= fanInterval) {
    previousMillisFan = currentMillis;  // Update de tijd van de laatste activering van de fan
    // Wissel de staat van het relais voor de fan
    fanState = !fanState;
    // Schakel het relais voor de fan in of uit op basis van de huidige staat
    digitalWrite(RELAY_PIN_FAN, fanState ? HIGH : LOW);
  }
}

void bodemvocht() {
  // Lees de vochtigheidswaarde van de sensor op IN2
  float moistureValue = map(analogRead(MOISTURE_SENSOR_PIN), 0, 4096, 100, 0);

  // Stuur de vochtigheidswaarde naar de seriële monitor
  Serial.print("Vochtigheid: ");
  Serial.println(moistureValue);
  String vochts = String(moistureValue);
  mqttClient.publish(MQTT_TOPICvocht, vochts.c_str());
}

void waterbak() {
  int afstand = afstandssensor.afstandCM();
  Serial.print("waterreservoir: ");
  Serial.println(afstand);
  String afstands = String(afstand);
  mqttClient.publish(MQTT_TOPICafstand, afstands.c_str());


  if (afstand < 10) {  // Controleer of de afstand kleiner is dan 10 cm
    tone(22, 1000);    // Laat de buzzer een toon van 1000 Hz afspelen
  } else {
    noTone(22);  // Schakel de buzzer uit
  }
}

void light() {
  int ldrValue = map(analogRead(ldrPin), 0, 4096, 100, 0);

  // Print de gemeten waarde naar de seriële monitor
  Serial.print("LDR Value: ");
  Serial.println(ldrValue);
}

void RFID() {
  static String jeton = "";
  if (!mfrc522.PICC_IsNewCardPresent()) {
    return;
  }

  // Select one of the cards
  if (!mfrc522.PICC_ReadCardSerial()) {
    return;
  }
  Serial.print("UID tag :");
  String content = "";
  for (byte i = 0; i < mfrc522.uid.size; i++) {
    Serial.print(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " ");
    Serial.print(mfrc522.uid.uidByte[i], HEX);
    content.concat(String(mfrc522.uid.uidByte[i] < 0x10 ? "0" : ""));
    content.concat(String(mfrc522.uid.uidByte[i], HEX));
  }
  Serial.println();
  Serial.print("Message : ");
  content.toUpperCase();
  Serial.println(content.substring(1));

  jeton = content.substring(1);
  if (jeton == "374F913") {
    Serial.println("jeton");
  }
}
