// 2.0
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ArduinoJson.hpp>
//-------------DSM501A--------------
#include<string.h>
#define PM1PIN 15//DSM501A input D6 on ESP8266
#define PM25PIN 2
byte buff[2];
unsigned long durationPM1;
unsigned long durationPM25;
unsigned long starttime;
unsigned long endtime;
unsigned long sampletime_ms = 30000;
unsigned long lowpulseoccupancyPM1 = 0;
unsigned long lowpulseoccupancyPM25 = 0;
int i=0;
float conPM1;
float conPM25;
//-----------DSM501A--------------
//---------mq 135--------
const int MQ135_PIN = 36;    // Analog input pin for the MQ135 sensor
const float V_REF = 5.0;     // Reference voltage
const int RL = 10000;        // Load resistance in ohms (typically 10kΩ)
const float R0 = 20000.0;    // Sensor resistance in clean air (20kΩ, adjust as needed)

// Function to calculate ppm from Rs/R0 ratio (example for CO2)
float getPPM(float ratio) {
    // Assuming the ratio-ppm relationship from the MQ135 datasheet (for CO2)
    // You might need to adjust this based on the specific gas and your sensor calibration
    return 116.6020682 * pow(ratio, -2.769034857);
}
//---------mq 135--------

//----------dht---------
#include "DHT.h"

#define DHTPIN 33    // Digital pin connected to the DHT sensor
// Feather HUZZAH ESP8266 note: use pins 3, 4, 5, 12, 13 or 14 --
// Pin 15 can work but DHT must be disconnected during program upload.

// Uncomment whatever type you're using!
//#define DHTTYPE DHT11   // DHT 11
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

// Connect pin 1 (on the left) of the sensor to +5V
// NOTE: If using a board with 3.3V logic like an Arduino Due connect pin 1
// to 3.3V instead of 5V!
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

// Initialize DHT sensor.
// Note that older versions of this library took an optional third parameter to
// tweak the timings for faster processors.  This parameter is no longer needed
// as the current DHT reading algorithm adjusts itself to work on faster procs.
DHT dht(DHTPIN, DHTTYPE);
//----------dht---------

//const char* ssid = "Pr. Tabarak";
const char* ssid = "Yousif S10+";
const char* password = "1290qwop";
const char* mqtt_server = "192.168.65.181";
const int mqtt_port = 1883;

WiFiClient espClient;
PubSubClient client(espClient);

unsigned long lastMsg = 0;
int h;
int t;
void setup() {
  delay(100);
  Serial.begin(9600);
  while (!Serial);
//--------DSM501A-------
Serial.println("Starting please wait 30s");
  pinMode(PM1PIN,INPUT);
  pinMode(PM25PIN,INPUT);
  pinMode(36,INPUT);
  starttime = millis(); 
//---------DSM501A------------

//----------mq 135---------------

//---------dht------------
Serial.println(F("DHTxx test!"));

  dht.begin();
//---------------dht----------------

  

  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // Set server and port
  client.setServer(mqtt_server, mqtt_port);
}
//------------DSM501A-------------
float calculateConcentration(long lowpulseInMicroSeconds, long durationinSeconds){
  
  float ratio = (lowpulseInMicroSeconds/1000000.0)/30.0*100.0; //Calculate the ratio
  float concentration = 0.001915 * pow(ratio,2) + 0.09522 * ratio - 0.04884;//Calculate the mg/m3
  Serial.print("lowpulseoccupancy:");
  Serial.print(lowpulseInMicroSeconds);
  Serial.print("    ratio:");
  Serial.print(ratio);
  Serial.print("    Concentration:");
  Serial.println(concentration);
  return concentration;
}
//---------------DSM501A-------------
void loop() {

//---------------DSM501A----------
  durationPM1 = pulseIn(PM1PIN, LOW);
  durationPM25 = pulseIn(PM25PIN, LOW);
  
  lowpulseoccupancyPM1 += durationPM1;
  lowpulseoccupancyPM25 += durationPM25;
  
  endtime = millis();
  if ((endtime-starttime) > sampletime_ms) //Only after 30s has passed we calcualte the ratio
  {
    /*
    ratio1 = (lowpulseoccupancy/1000000.0)/30.0*100.0; //Calculate the ratio
    Serial.print("ratio1: ");
    Serial.println(ratio1);
    
    concentration = 0.001915 * pow(ratio1,2) + 0.09522 * ratio1 - 0.04884;//Calculate the mg/m3
    */
    conPM1 = calculateConcentration(lowpulseoccupancyPM1,30);
    conPM25 = calculateConcentration(lowpulseoccupancyPM25,30);
//    Serial.print("PM1 ");
//    Serial.print(conPM1);
//    Serial.print("  PM25 ");
//    Serial.println(conPM25);
    lowpulseoccupancyPM1 = 0;
    lowpulseoccupancyPM25 = 0;
    starttime = millis();
  } 
//-----------DSM501A-------------

//----------mq 135-------
   int analogValue = analogRead(MQ135_PIN);  // Read the analog value from the sensor
    float Vout = (analogValue / 1023.0) * V_REF;  // Convert analog value to voltage

    // Calculate Rs (sensor resistance)
    float Rs = RL * (V_REF / Vout - 1.0);

    // Calculate Rs/R0 ratio
    float ratio = Rs / R0;

    // Calculate ppm (for CO2 in this example)
    float ppm = getPPM(ratio);

    // Print the results
    Serial.print("Analog Value: ");
    Serial.print(analogValue);
    Serial.print(", Voltage: ");
    Serial.print(Vout);
    Serial.print("V, Rs: ");
    Serial.print(Rs);
    Serial.print(" ohms, Ratio: ");
    Serial.print(ratio);
    Serial.print(", PPM: ");
    Serial.println(ppm);

    delay(1000);  // Wait for a second before taking the next reading
//-------------mq 135------------
  
//----------dht-----------
 delay(2000);

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)

  // Check if any reads failed and exit early (to try again).
  int f = 10;
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }

  

  Serial.print(F("Humidity: "));
  Serial.print(h);
  Serial.print(F("%  Temperature: "));
  Serial.print(t);
  Serial.print(F("°C "));
 
//----------dht-----------

  if (!client.connected()) {
    reconnect();
  }

  StaticJsonDocument<80> doc;
  char output[160];

  long now = millis();
  if (now - lastMsg > 1000) {
    lastMsg = now;
    
    //dht
    
    //-------------
    String tempstr = "{\"humidity\":" + String(h) + ",\"temperature\":" + String(t) + ",\"CO2\":" + String(ppm + 335) + ",\"particle_level\":" + String(conPM25) + "}";

strncpy(output, tempstr.c_str(), sizeof(output) - 1);
  
  // Ensure the char array is null-terminated
  output[sizeof(output) - 1] = '\0';
    //--------------
    Serial.println(output);
    delay(200);
    client.publish("aswar", output);
  }

  client.loop();
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      // Subscribe to a topic if needed
      // client.subscribe("aswarme");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      delay(5000);
    }
  }
}
