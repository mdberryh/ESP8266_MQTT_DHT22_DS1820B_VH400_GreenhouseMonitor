/*
 Sketch which publishes temperature data from a DS1820 sensor to a MQTT topic.
 This sketch goes in deep sleep mode once the temperature has been sent to the MQTT
 topic and wakes up periodically (configure SLEEP_DELAY_IN_SECONDS accordingly).
 Hookup guide:
 - connect D0 pin to RST pin in order to enable the ESP8266 to wake up periodically
 - DS18B20:
     + connect VCC (3.3V) to the appropriate DS18B20 pin (VDD)
     + connect GND to the appopriate DS18B20 pin (GND)
     + connect D4 to the DS18B20 data pin (DQ)
     + connect a 4.7K resistor between DQ and VCC.
*/

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

// Uncomment the type of sensor in use:
//#define DHTTYPE           DHT11     // DHT 11 
#define DHTTYPE           DHT22     // DHT 22 (AM2302)
//#define DHTTYPE           DHT21     // DHT 21 (AM2301)

#define DHTHighTunnel            12  
#define DHTGreenHouse            14  
//#include <Streaming.h>

#define SLEEP_DELAY_IN_SECONDS            600   //10 minutes
#define ONE_WIRE_BUS_NorthFace            5      // DS18B20 pin GPIO5
#define ONE_WIRE_BUS_CircuitTemp          4      // DS18B20 pin GPIO4
//#define ONE_WIRE_BUS_DirtSouth            12      // DS18B20 pin GPIO12
//#define ONE_WIRE_BUS_DirtNorth            14      // DS18B20 pin GPIO14
#define ONE_WIRE_BUS_BedTemp                 13      // DS18B20 pin GPIO13

//DS18B20BedTemp

// See guide for details on sensor wiring and usage:
//   https://learn.adafruit.com/dht/overview

DHT_Unified dht_tunnel(DHTHighTunnel, DHTTYPE);

DHT_Unified dht_greenhouse(DHTGreenHouse, DHTTYPE);

uint32_t delayMS;

const char* ssid = "";//
const char* password = "";//

const char* mqtt_server = "192.168.1.44";
const char* mqtt_username = "";
const char* mqtt_password = "";
const char* mqtt_topic = "/dev/esp_gh01M/Out/tempC";

WiFiClient espClient;
PubSubClient client(espClient);

OneWire oneWireCircuitTemp(ONE_WIRE_BUS_CircuitTemp);
OneWire oneWireBedTemp(ONE_WIRE_BUS_BedTemp);
//OneWire oneWire6(15);

DallasTemperature DS18B20CircuitTemp(&oneWireCircuitTemp); 


DallasTemperature DS18B20BedTemp(&oneWireBedTemp); 

void SetDHTSettings(){
  
}

void setup() {
  // setup serial port
  Serial.begin(74880);
  
  
  // setup WiFi
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  // setup OneWire bus
  DS18B20CircuitTemp.begin();
  DS18B20BedTemp.begin();
  dht_tunnel.begin();
  dht_greenhouse.begin();

  sensor_t sensor;
  dht_tunnel.temperature().getSensor(&sensor);
  
  // Print humidity sensor details.
  dht_tunnel.humidity().getSensor(&sensor);
  
  // Set delay between sensor readings based on sensor details.
  delayMS = sensor.min_delay / 1000;

  //get second DHT...we should wait for the longest one...

  dht_greenhouse.temperature().getSensor(&sensor);
  
  // Print humidity sensor details.
  dht_greenhouse.humidity().getSensor(&sensor);

  if(sensor.min_delay/1000 > delayMS){
    delayMS = sensor.min_delay / 1000;
  }
   
}

float readAdc(){
  //the input at 3.4v = .80v by the adc. that is division facter of 3/.95= 3.15
  //sensor is actually 3V not 3.3V
  float factor = 3.158;
  float voltage= (analogRead(A0)) * (1.0 / 1023.0); //max of 1 V on this adc.
  voltage = voltage * factor;
  return voltage;
}

String vhMoisture(){
  float adc1 = readAdc();
  delay(1000);
  float adc2=readAdc();

  //y=mx+b
  float m = ((VWCcalc(adc2) - VWCcalc(adc1))/ (adc2-adc1));
  float b = m*adc2 -VWCcalc(adc2);

  float x = (adc2+b) / m; 

  char temperatureString[6]; 
  dtostrf(x, 2, 2, temperatureString);

  return String(temperatureString);
  
}

float VWCcalc(float voltage){
  float vwc=0.0;
  if(voltage < 1.1){
      vwc=10*voltage - 1;
  } else if(voltage < 1.3){
      vwc=25*voltage -17.5;
  } else if(voltage < 1.82){
      vwc=48.08*voltage -47.5;
  } else if(voltage < 2.2){
      vwc=26.32*voltage -7.89;
  }else if(voltage < 3.0){
      vwc=62.5*voltage -87.5;
  }
  return vwc;  
}

void setup_wifi() {
  
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  //WiFi.persistent(false);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP8266Client", mqtt_username, mqtt_password)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

float getTemperature(DallasTemperature *DS18B20) {
  Serial.print("Requesting DS18B20 temperature...\n");
 // Serial.print("Devices: " + String(DS18B20NorthFace.getDeviceCount()) +"\n");

  float temp;
  //do {
    DS18B20->requestTemperatures(); 
    
    temp = DS18B20->getTempCByIndex(0);
 //   delay(100);
 // } while (temp == 85.0 || temp == (-127.0));
  return temp;
}

String GetDHTSensorTemp(){

  
  char temperatureString[6];
  float temperature = 0;
  
// Get temperature event and print its value.
  sensors_event_t event;  
  String sensorText ="";
  dht_tunnel.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println("Error reading temperature!");
    sensorText += "\"PTmp\":\"N/A\",";
  }
  else {
    temperature = event.temperature;
    dtostrf(temperature, 2, 2, temperatureString);
    
    sensorText += "\"PTmp\":" + String(temperatureString) +",";
    Serial.print("Temp: ");
    Serial.print(event.temperature);
    Serial.println(" *C");
  }
  
  // Get humidity event and print its value.
  dht_tunnel.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println("Error reading humidity!");
    sensorText += "\"PHm\":\"N/A\",";
  }
  else {
    temperature = event.relative_humidity;
    dtostrf(temperature, 2, 2, temperatureString);
    sensorText += "\"PHm\":" + String(temperatureString)+",";
    Serial.print("\"PHm\":" + String(temperatureString)+"%\n");
  }

  //////////GET THE SECOND DHT///////////////////
  
  dht_greenhouse.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println("Error reading temperature!");
    sensorText += "\"GTmp\": \"N/A\",";
  }
  else {
    temperature = event.temperature;
    dtostrf(temperature, 2, 2, temperatureString);
    sensorText += "\"GTmp\":" + String(temperatureString) +",";
    Serial.print("Temp: ");
    Serial.print(event.temperature);
    Serial.println(" *C");
  }
  
  // Get humidity event and print its value.
  dht_greenhouse.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println("Error reading GreenHouse humidity!");
    sensorText += "\"GHm\":\"N/A\",";
  }
  else {
    
    temperature = event.relative_humidity;
    dtostrf(temperature, 2, 2, temperatureString);
    sensorText += "\"GHm\":" + String(temperatureString)+",";
    Serial.print("\"GHm\":" + String(temperatureString)+"%\n");
  }
  return sensorText;
}

String CreateMQTTSensorMessage(){
  
  delay(1000);
  delay(delayMS);
  
  char temperatureString[6]; 
  String message="";

  message +="{";
  message +="\"dev\":\"e8266_gh01M\",";
  
  float temperature = getTemperature(&DS18B20CircuitTemp);
  // convert temperature to a string with two digits before the comma and 2 digits for precision
  dtostrf(temperature, 2, 2, temperatureString);
  message +="\"CrcTmp\":"+String(temperatureString)+",";

  temperature = getTemperature(&DS18B20BedTemp);
  // convert temperature to a string with two digits before the comma and 2 digits for precision
  dtostrf(temperature, 2, 2, temperatureString);
  message +="\"BTmp\":"+String(temperatureString)+",";
  
  message += GetDHTSensorTemp();
  message += "\"BMst\":" +String(vhMoisture()) +"}\n";

  return message;
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  String message=CreateMQTTSensorMessage();
  
  // send temperature to the MQTT topic
  char charBuf[128];
  message.toCharArray(charBuf, message.length()) ;

  client.publish(mqtt_topic,charBuf);
  
  // send temperature to the serial console
  Serial.print("Sending temperature: \n" + String(charBuf)+"\n");

  Serial.print( "Closing MQTT connection...\n");
  client.disconnect();

 Serial.print("Closing WiFi connection...\n");
  WiFi.disconnect();

  delay(100);

  Serial.print("Entering deep sleep mode for "+String(SLEEP_DELAY_IN_SECONDS)+" seconds...");
  ESP.deepSleep(SLEEP_DELAY_IN_SECONDS * 1000000, WAKE_RF_DEFAULT);
  //ESP.deepSleep(10 * 1000, WAKE_NO_RFCAL);
  delay(500);   // wait for deep sleep to happen
}
