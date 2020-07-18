
#include <WiFi.h>
#include <PubSubClient.h>

#include "DHT.h"
const char* ssid = "fusim6";
const char* password = "fusiglobaltechno";
const char* mqtt_server = "159.89.192.117";//"octopus.fusi.id";//"192.168.100.13";
#define MQTTuser        "MJwdWHGXpvVSeLl6lIa1"//"CNKunrnlhCXwd5IxvkCL"//"JlYkrcDJjlWaBZO2mWxI" 
#define MQTTpasw  ""
const char* pubTopic = "v1/devices/me/telemetry";
WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[128];
int value = 0;

#define SOIL_PIN (36)

#define pinPWM_A 5
#define pinDIR_A 18

#define DHTPIN 4
#define DHTTYPE DHT22

DHT dht(DHTPIN, DHTTYPE);
/************************Hardware Related Macros************************************/
#define         MG_PIN                       (35)     //define which analog input channel you are going to use
#define         BOOL_PIN                     (2)
#define         DC_GAIN                      (8.5)   //define the DC gain of amplifier
#define         LDR_PIN                      (36)

/***********************Software Related Macros************************************/
#define         READ_SAMPLE_INTERVAL         (50)    //define how many samples you are going to take in normal operation
#define         READ_SAMPLE_TIMES            (5)     //define the time interval(in milisecond) between each samples in
                                                     //normal operation

/**********************Application Related Macros**********************************/
//These two values differ from sensor to sensor. user should derermine this value.
#define         ZERO_POINT_VOLTAGE           (0.341) //define the output of the sensor in volts when the concentration of CO2 is 400PPM
#define         REACTION_VOLTGAE             (0.030) //define the voltage drop of the sensor when move the sensor from air into 1000ppm CO2

/*****************************Globals***********************************************/
float           CO2Curve[3]  =  {2.602,ZERO_POINT_VOLTAGE,(REACTION_VOLTGAE/(2.602-3))};
                                                     //two points are taken from the curve.
                                                     //with these two points, a line is formed which is
                                                     //"approximately equivalent" to the original curve.
                                                     //data format:{ x, y, slope}; point1: (lg400, 0.324), point2: (lg4000, 0.280)
                                                     //slope = ( reaction voltage ) / (log400 –log1000)

void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

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

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1') {
//    digitalWrite(BUILTIN_LED, LOW);   // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
    // it is active low on the ESP-01)
  } else {
//    digitalWrite(BUILTIN_LED, HIGH);  // Turn the LED off by making the voltage HIGH
  }

}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str(),MQTTuser,MQTTpasw)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}


void setup()
{
  
    Serial.begin(115200);                              //UART setup, baudrate = 9600bps
    dht.begin();
    ode(pinDIR_A, OUTPUT);pinMode(pinPWM_A, OUTPUT);   // define output PWM to change speed motor
    pinMode(pinDIR_A, OUTPUT);
    setup_wifi();
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);    
    Serial.print("MG-811 Demostration\n");
}

void loop()
{
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  digitalWrite(pinPWM_A, 100);
  
  int soil;
  soil = analogRead(SOIL_PIN);
  if (soil > 700){
  digitalWrite(pinPWM_A, 1);
  }else if (soil <700 && soil >350){
  digitalWrite(pinPWM_A, 0);
  }else{
  digitalWrite(pinPWM_A, 0);
  }
  
    int percentage;
    float volts;

    volts = MGRead(MG_PIN);
    Serial.print( "SEN0159:" );
    Serial.print(volts);
    Serial.print( "V           " );

    percentage = MGGetPercentage(volts,CO2Curve);
    Serial.print("CO2:");
    if (percentage == -1) {
        Serial.print( "<400" );
        percentage = 399;
    } else {
        Serial.print(percentage);
    }
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    unsigned int ldr = analogRead(LDR_PIN);
    float l = 100-(ldr*100/4096);
    if (ldr < 1000){
      digitalWrite(pinPWM_B, HIGH);
      }else (ldr >1000 && ldr <1500){
        digitalWrite(pinPWM_B, LOW);
        }
    if (isnan(h) || isnan(t) ) {
      Serial.println(F("Failed to read from DHT sensor!"));
      return;
    }
    Serial.print("\n");
    snprintf (msg, 128, "{\"lat\":\"-6.899794\",\"lon\":\"107.653618\",\"temp\":\"%.2f\",\"hum\":\"%.2f\",\"lux\":\"%.2f\",\"co2\":\"%d\",\"mois\":\"%2f\"}", t, h,l,percentage,soil);
    Serial.print("Publish message: ");
    Serial.println(msg);
    client.publish(pubTopic, msg);
    
    delay(500);
}

float MGRead(int mg_pin)
{
    int i;
    float v=0;

    for (i=0;i<READ_SAMPLE_TIMES;i++) {
        v += analogRead(mg_pin);
        delay(READ_SAMPLE_INTERVAL);
    }
    Serial.println(v);
    v = (v/READ_SAMPLE_TIMES) *3.3/4096 ;
    return v;
}
int  MGGetPercentage(float volts, float *pcurve)
{
  Serial.println(volts);
  Serial.println(volts/DC_GAIN);
   if ((volts/DC_GAIN )>=ZERO_POINT_VOLTAGE) {
      return -1;
   } else {
      return pow(10, ((volts/DC_GAIN)-pcurve[1])/pcurve[2]+pcurve[0]);
   }
}
