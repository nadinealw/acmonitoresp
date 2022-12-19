#include "DHTesp.h"
#include <FirebaseESP32.h>
#include <Arduino.h>
#include <IRremoteESP8266.h>
#include <IRsend.h>

/* FIREBASE */
/* Configuration */
#define FIREBASE_HOST "ENTER_HOST" // Firebase host URL
#define FIREBASE_AUTH "ENTER_AUTH" // Firebase authentication code
#define WIFI_SSID "ENTER_SSID" // change to WiFi name
#define WIFI_PASSWORD "ENTER_PASS" // change to WiFi password
/* Data Objects */
FirebaseData firebaseData;
FirebaseJson json;

/* PINOUTS */
// Digital
#define PIR1 5
#define PIR2 18
#define PIR3 19
#define buzzerPin 22
// Analog
//#define hallSensor 33
#define waterlevelSensor 27
#define dhtSensor 33
// Other
#define transmitterPin 4 // D4 for ESP32
#define THRESHOLD 3200 // water level threshold

/* GLOBAL VAR */
float prevhum = 0;
float prevtemp = 0;
int prevwater = 0;
int prevhall = 0;
int pirsum = 0;
int occupied = 0;

unsigned long pirtimestart;
int waterlevel = 0;

const uint16_t IRPIN = 4; // GPIO4

/* AC codes
* MODE - COOL, SWING(V) - OFF, FAN - 4 (MEDIUM)
* RANGING 16 - 27 DEGREES
*/
const uint16_t accode[] = {0xEC24101003374216, 0xFC24101003384216, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 0 = off, 1 = on
                           0x4416101004424216, 0x5417101004424216, 0x6418101004424216, 0x7419101004424216, 0xF420101004424216, // 16 to 20
                           0x421101004424216, 0x1422101004424216, 0x2423101004424216, 0x4424101004434216, 0x5425101004434216, // 21 to 25, fix 21 later
                           0x6426101004434216, 0x7427101004434216}; // 26 and 27
// ON - accode[1], OFF - accode[0]
// temperature 26 - accode[26]

/* INITIALIZATION */
DHTesp dht;
TempAndHumidity newValues;
IRsend irsend(IRPIN);

void waterbuzzer(void * parameter){ // Multitask for Water Level Sensor
  for(;;){
    if(waterlevel > THRESHOLD){ 
    digitalWrite(buzzerPin, HIGH);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    digitalWrite(buzzerPin, LOW);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
  }
  vTaskDelete(NULL);
}

void setup() {
  Serial.begin(115200);
  pirtimestart = millis();
  
  // PINMODES
  pinMode(PIR1, INPUT);
  pinMode(PIR2, INPUT);
  pinMode(PIR3, INPUT);

  pinMode(buzzerPin, OUTPUT);
  pinMode(transmitterPin, OUTPUT);

  // Multitasking
  xTaskCreate(
      waterbuzzer,
      "Water Level Buzzer",
      1000,
      NULL,
      1,
      NULL
    );
  
  // DHT
  dht.setup(dhtSensor, DHTesp::DHT11);
  
  // IR DAIKIN
  irsend.begin();

  // WIFI
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  // FIREBASE
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.reconnectWiFi(true);
  //Set database read timeout to 1 minute (max 15 minutes)
  Firebase.setReadTimeout(firebaseData, 1000 * 60);
  //tiny, small, medium, large and unlimited.
  //Size and its write timeout e.g. tiny (1s), small (10s), medium (30s) and large (60s).
  Firebase.setwriteSizeLimit(firebaseData, "tiny");

  /*
  This option allows get and delete functions (PUT and DELETE HTTP requests) works for device connected behind the
  Firewall that allows only GET and POST requests.
  
  Firebase.enableClassicRequest(firebaseData, true);
  */

  //String path = "/data";
  
  Serial.println("------------------------------------");
  Serial.println("Connected...");
}

void loop() {
  // Reading sensors
  int PIR1val = digitalRead(PIR1);
  int PIR2val = digitalRead(PIR2);
  int PIR3val = digitalRead(PIR3);

  waterlevel = analogRead(waterlevelSensor);

//  int waterlevel = analogRead(waterlevelSensor);
  newValues = dht.getTempAndHumidity();
  float t = newValues.temperature;
  float h = newValues.humidity;
  

  if (millis() - pirtimestart >= 180000){ // after 3 minutes
    if (pirsum >= 10 && occupied == 0){
      Serial.println("There are people in the room.");
      occupied = 1;
      irsend.sendDaikin64(accode[1], 64, 0); // turn on
      json.set("/occu", occupied);
      Firebase.updateNode(firebaseData, "/sensor", json);
    }
    else if (pirsum < 10 && occupied == 1){
      Serial.println("There is no one in the room.");
      occupied = 0;
      irsend.sendDaikin64(accode[0], 64, 0); // turn off
      json.set("/occu", occupied);
      Firebase.updateNode(firebaseData, "/sensor", json);
    }
    pirsum = 0;
    pirtimestart = millis(); // set start time to current time
  }

  if (t != prevtemp){
    json.set("/temp", t);
    Firebase.updateNode(firebaseData, "/sensor", json);
  }
  if (h != prevhum){
    json.set("/hum", h);
    Firebase.updateNode(firebaseData, "/sensor", json);
  }
  if (waterlevel != prevwater){
    json.set("/water", waterlevel);
    Firebase.updateNode(firebaseData, "/sensor", json);
  }

  // AC Control Algorithm
  if (t > 25){
    int tref = int(round(t))
    irsend.sendDaikin64(accode[tref - 3], 64, 0);
  }

  prevtemp = t;
  prevhum = h;
  prevwater = waterlevel;

  pirsum += PIR1val + PIR2val + PIR3val;

  Serial.print("PIR Sensor Values: ");
  Serial.print(PIR1val);
  Serial.print(PIR2val);
  Serial.println(PIR3val);

  Serial.print("Water level: ");
  Serial.println(waterlevel);

  Serial.print("Temp: ");
  Serial.print(t);
  Serial.println("C");

  Serial.print("Hum: ");
  Serial.print(h);
  Serial.println("%");
  
  delay(2000);
}
