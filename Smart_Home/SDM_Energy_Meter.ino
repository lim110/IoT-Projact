
/*  WEMOS D1 Mini
                     ______________________________
                    |   L T L T L T L T L T L T    |
                    |                              |
                 RST|                             1|TX HSer
                  A0|                             3|RX HSer
                  D0|16                           5|D1
                  D5|14                           4|D2
                  D6|12                    10kPUP_0|D3
RX SSer/HSer swap D7|13                LED_10kPUP_2|D4
TX SSer/HSer swap D8|15                            |GND
                 3V3|__                            |5V
                       |                           |
                       |___________________________|
*/

//REMEMBER! uncomment #define USE_HARDWARESERIAL 
//in SDM_Config_User.h file if you want to use hardware uart

#include <WiFi.h>
#include <PubSubClient.h>
#include <SDM.h>                                                                //import SDM library


#if defined ( USE_HARDWARESERIAL )                                              //for HWSERIAL

#if defined ( ESP8266 )                                                         //for ESP8266
#include <ESP8266WiFi.h>
SDM sdm(Serial1, 2400, 4, SERIAL_8N1);                                  //config SDM
#elif defined ( ESP32 )                                                         //for ESP32
#include <WiFi.h>
SDM sdm(Serial1, 2400, 4, SERIAL_8N1, SDM_RX_PIN, SDM_TX_PIN);          //config SDM
#else                                                                           //for AVR
SDM sdm(Serial1, 4800, 4);                                              //config SDM on Serial1 (if available!)
#endif

#else                                                                           //for SWSERIAL

#include <SoftwareSerial.h>                                                     //import SoftwareSerial library
#if defined ( ESP8266 ) || defined ( ESP32 )                                    //for ESP
SoftwareSerial swSerSDM;                                                        //config SoftwareSerial
SDM sdm(swSerSDM, 2400, 4, SWSERIAL_8N1, SDM_RX_PIN, SDM_TX_PIN);       //config SDM
#else                                                                           //for AVR
SoftwareSerial swSerSDM(SDM_RX_PIN, SDM_TX_PIN);                                //config SoftwareSerial
SDM sdm(swSerSDM, 2400, 4);                                             //config SDM
#endif

#endif

const char* ssid = "fusim6";
const char* password = "fusiglobaltechno";
const char* mqtt_server = "159.89.192.117";
const char* pubTopic = "v1/devices/me/telemetry";
const char* subTopic = "v1/devices/me/rpc/request/+";

#define MQTTuser        "vJ8xqFZXOWuW2TQPlTfG"
#define MQTTpasw  ""
WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[128];
const int Relay = 19;
void setup_wifi() {
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
   long timerStart = millis();
   long timerEnd;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
//     timeout function security
//     timerEnd = millis();
//     if((timerEnd - timerStart) > 60000){
//       buzzerOn();
//       delay(180000);
//     }
    yield();
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

  if ((char)payload[31] == '1'){
    digitalWrite(Relay, LOW);}
    else{
      digitalWrite(Relay, HIGH);
      }
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  char checkPayload = (char)payload[0];
  // Switch on the LED if an 1 was received as first character
}
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    if (client.connect(clientId.c_str(),MQTTuser,MQTTpasw)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.subscribe(subTopic);
      
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
    yield();
  }
}

void setup() {
  Serial.begin(115200);                                                         //initialize serial
  sdm.begin();                                                                  //initialize SDM communication
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  
  pinMode(Relay, OUTPUT);
  digitalWrite(Relay, HIGH);
}

void loop() {
   if (!client.connected()) {
     reconnect();
   }
   client.loop();
  
  char bufout[10];
  sprintf(bufout, "%c[1;0H", 27);
  Serial.print(bufout);

  Serial.print("Voltage:   ");
  float volt = sdm.readVal(SDM220T_VOLTAGE);
  Serial.print(volt, 2);                                //display voltage
  Serial.println("V");

  delay(50);

  Serial.print("Current:   ");
  float current = sdm.readVal(SDM220T_CURRENT);
  Serial.print(current, 2);                                //display current  
  Serial.println("A");

  delay(50);

  Serial.print("Power:     ");
  float power = sdm.readVal(SDM220T_POWER);
  Serial.print(power, 2);                                  //display power
  Serial.println("W");

  delay(50);

  Serial.print("Frequency: ");
  float freq = sdm.readVal(SDM220T_FREQUENCY);
  Serial.print(freq, 2);                              //display frequency
  Serial.println("Hz");
  snprintf (msg, 128, "{\"lat\":\"-6.899794\",\"lon\":\"107.653618\",\"volt\":\"%.2f\",\"curr\":\"%.2f\",\"pow\":\"%.2f\",\"freq\":\"%.2f\"}",volt , current,power,freq);
  Serial.print("Publish message: ");
  Serial.println(msg);
  client.publish(pubTopic, msg);
  
  delay(1000);                                                                  //wait a while before next loop
}
