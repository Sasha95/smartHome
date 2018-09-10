#include <WiFi.h> //esp32
#include <PubSubClient.h>
#include "DHT.h"
#include <IRrecv.h>

#define lamp1 33  //33    //32
#define lamp2 32  //25    //33
#define lamp1_button 34   //35    //34
#define lamp2_button 35    //32    //35

#define fire 26
#define trigPin 14
#define echoPin 27
#define IKLamp 2
#define lum 36
#define IKDist 23
#define analogGas 39

#define relays_topic "ESP/Lampa"
#define relays_topic1 "ESP/Lampa1"
#define data_topic "ESP/data"
#define IK_topic "ESP/IK"
#define DHTPIN 15    
#define DHTTYPE DHT21  
uint16_t RECV_PIN = 22;

IRrecv irrecv(RECV_PIN);

decode_results results;
DHT dht(DHTPIN, DHTTYPE);


long duration;
int distance;
boolean lastbtnStat1 = false;
boolean lastbtnStat2 = false;
boolean btnPress1 = false;
boolean btnPress2 = false;
boolean r_state1 = false;
boolean r_state2 = false;
long last_mls = millis();
long lastMsg = 0;

const char* ssid = "ping";
const char* password = "103mag103";
const char* mqtt_server = "192.168.0.100";

WiFiClient espClient;
PubSubClient client(espClient);

void setup_wifi() {
  delay(10);
  Serial.println();
  
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  int i = 0;
  while (i < 20 && WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    i++;
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  int i = 0;   
  while (i<3 && !client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("arduinoClient_temperature_sensor")) {
      Serial.println("connected");
      client.subscribe(relays_topic);
      client.subscribe(relays_topic1);
 
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
  i++;
}
 
void setup(){
  Serial.begin(115200);
  setup_wifi(); 
  client.setServer(mqtt_server, 1883);
  
  pinMode(lamp1, OUTPUT);
  pinMode(lamp2, OUTPUT);
  pinMode(lamp1_button, INPUT);
  pinMode(lamp2_button, INPUT);
  pinMode(fire, INPUT);
  pinMode(trigPin, OUTPUT); 
  pinMode(echoPin, INPUT); 
  pinMode(IKLamp, INPUT); 
  pinMode(IKDist, INPUT); 

  dht.begin();
  irrecv.enableIRIn(); 
  
  lastbtnStat1 = digitalRead(lamp1_button);
  lastbtnStat2 = digitalRead(lamp2_button);
  
  client.setCallback(callback);
}

void callback(char* topic, byte* payload, unsigned int length) {
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  if (String(topic) == relays_topic)
  {
    if ((char)payload[0] == '1') {
      r_state1 = true;
      digitalWrite(lamp1, r_state1);   
    } else {      
      r_state1 = false;
      digitalWrite(lamp1, r_state1); 
    }
  }
  else if (String(topic) == relays_topic1)
  {
    if((char)payload[0] == '1'){
      r_state2 = true;
      digitalWrite(lamp2, r_state2); 
  }
  else {
    r_state2 = false;
    digitalWrite(lamp2, r_state2); 
    }
  }
}

void buttonF(){
  btnPress1 = digitalRead(lamp1_button);
  btnPress2 = digitalRead(lamp2_button);
  
  if (btnPress1 != lastbtnStat1){
    delay(30); // защита от дребезга    
    btnPress1 = digitalRead(lamp1_button);
     if (btnPress1 != lastbtnStat1){
      r_state1 = !r_state1;
       
    digitalWrite(lamp1, r_state1);      
    // публикуем изменение состояния реле на брокер
    client.publish(relays_topic, String(r_state1).c_str(), true);    
    lastbtnStat1 = btnPress1;
     }
  }  
  
    if (btnPress2 != lastbtnStat2){
    delay(30); // защита от дребезга    
    btnPress2 = digitalRead(lamp2_button);
     if (btnPress2 != lastbtnStat2){
      r_state2 = !r_state2;
     
    digitalWrite(lamp2, r_state2);    
    // публикуем изменение состояния реле на брокер
    client.publish(relays_topic1, String(r_state2).c_str(), true); 
    lastbtnStat2 = btnPress2;
     }            
  }
}

String humidity(){
  String data =""; 
  float h = dht.readHumidity();
  float t = dht.readTemperature();
   if (isnan(t) || isnan(h)) 
   {
     Serial.println("Failed to read from DHT");
   } 
   else
   {
     String temperature = String(t);
     String humidity = String(h);   
     data += "\"temperature\":"; data += temperature; data += ",";
     data += "\"humidity\":"; data += humidity;
  }

  return data;
}

String Distance(){

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  
  // Calculating the distance
  distance = duration*0.034/2;

  return "\"distance\":" + String(distance);
}

String Fire(){
  return "\"fire\":" + String(digitalRead(fire));  
}


String Gas(){
  return "\"gas\":" + String(analogRead(analogGas));  
}

String ikLamp(){
  return "\"IKLamp\":" + String(digitalRead(IKLamp));
}

String getLum(){
  return "\"lum\":" + String(analogRead(lum));
}

String ikDist(){
  return "\"IKDist\":" + String(digitalRead(IKDist));
}

void loop(){
  client.loop();
  if (millis() - last_mls > 12000) {
      last_mls = millis();
      reconnect();
    }
  long now = millis();
  if (now - lastMsg > 3000) {    
     lastMsg = now;

  String payload = "{";
  payload+=humidity();
  payload+=",";
  payload+=Distance();
  payload+=",";
  payload+=Fire();
  payload+=",";
  payload+=Gas();
  payload+=",";
  payload+=ikLamp();
  payload+=",";
  payload+=getLum();
  payload+=",";
  payload+=ikDist();   
  payload += "}";
  char attributes[1000];
  payload.toCharArray( attributes, 1000 );
  client.publish( data_topic, attributes );   
  }
  buttonF();

  if (irrecv.decode(&results)) {
    Serial.println((uint32_t) (results.value & 0xFFFFFFFF), DEC);
    client.publish(IK_topic, String((uint32_t) (results.value & 0xFFFFFFFF), DEC).c_str(), true);   
    irrecv.resume();  // Receive the next value
  }
}
