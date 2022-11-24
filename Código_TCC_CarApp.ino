// --- Bibliotecas ---  
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// --- Declaração de variaveis---
const int buzzer = 18;
const int PIR = 19;
int resposta;
int PirStatusAgora  = LOW;
int PirStatusAnterior  = LOW;
Adafruit_MPU6050 mpu;


// --- login e senha do wifi ---
const char* ssid = "RABBIT__2G";
const char* password = "Sueli1964";

// ---  criação de objeto ---
WiFiClient esp32S;
PubSubClient client(esp32S);

// --- ip do brooker ---
const char* brooker_esp = "192.168.0.29";
const char* nome = "**";
const char* senha = "**";

// --- Setup ---
void setup() {
  Serial.begin(115200);
  client.setServer(brooker_esp, 1883);
  pinMode(buzzer, OUTPUT); 
  ledcAttachPin(buzzer, 0);
  ledcSetup(0, 2000, 8);
  client.setCallback(callback);
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(20);
  mpu.setInterruptPinLatch(true); 
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);
}

// --- Loop ---
void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    conexao_wifi();
  }
  if (!client.connected()) {
    reconectar();
  }
  publicar_dados();
  client.loop();
  delay(2000);
}

// --- Conectar no wifi ---
void conexao_wifi() {

  delay(10);
  Serial.println(); // retirar
  Serial.print("Connecting to ");
  Serial.println(ssid); 

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println(""); //retirar
  Serial.println("WiFi connected"); 
  Serial.println("IP address: "); 
  Serial.println(WiFi.localIP());
}

// --- função para receber payloads --- 
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  String msg_temp;
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println(msg_temp);
  Serial.println((char)payload[0]);
  Serial.println((char)payload[1]); 
  if (String(topic) == "esp32Sensor/alarme"){
    StaticJsonDocument<16> doc;

    DeserializationError error = deserializeJson(doc, payload);

    if (error) {
      Serial.print("deserializeJson() failed: ");
      Serial.println(error.c_str());
    }

    bool alarme = doc["sensoral"];
    if (alarme == true){
    }
    else {
    }
    verifica_alarme(alarme);
  }
}

// --- Função que toca o alarme caso seja pedido ---
void verifica_alarme(bool msg_temp) {
  if (msg_temp == true){
    ledcWrite(0, 255);
  }
  else if (msg_temp == false){
    ledcWrite(0, 0);
  }
}

// --- reconectar no servidor mqtt ---
void reconectar() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "esp32S";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str(),nome,senha)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("esp32Sensor/status", "sim",true);
      client.subscribe("esp32Sensor/alarme");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

// --- Função para publicar dados no MQTT ---
void publicar_dados(){
  char output[128];
  StaticJsonDocument<32> doc;
  doc["sensorpre"] = sensor_presenca();
  serializeJson(doc, output);
  client.publish("esp32Sensor/S/presenca", output, true);
  doc.clear();
  doc["sensormov"] = sensor_movimento();
  serializeJson(doc, output);
  client.publish("esp32Sensor/S/movimento", output, true);
  doc.clear();  
}

// --- Função que retorna True caso o acelerômetro detecte algo ---
bool sensor_movimento(){
  if(mpu.getMotionInterruptStatus()) {
    /* Get new sensor events with the readings */
    return true;
  }
  else{
    return false;
  }
}

// --- Função que retorna True caso o sensor de presença detecte algo ---
bool sensor_presenca(){
  char output[128];
  PirStatusAnterior = PirStatusAgora;
  PirStatusAgora = digitalRead(PIR);
  if (PirStatusAgora == HIGH && PirStatusAnterior == LOW){
    Serial.println("Motion detected");
    return true;
  }
  else if (PirStatusAgora == HIGH && PirStatusAnterior == HIGH){
    return true;
  }
  else if (PirStatusAgora == LOW && PirStatusAnterior == HIGH){
    Serial.println("Motion Stopped");
    return false;
  }
}
