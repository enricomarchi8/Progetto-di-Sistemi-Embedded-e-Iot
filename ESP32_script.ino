#include "DHT.h"
#include <WiFi.h>
extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/timers.h"
}
#include <AsyncMqttClient.h>

// Credenziali WiFi
#define WIFI_SSID "rpi4"
#define WIFI_PASSWORD "raspberrypi4"

// Parametri MQTT Broker
#define MQTT_HOST IPAddress(192, 168, 169, 100)
#define MQTT_PORT 1883

// Topic MQTT sensore 1
#define MQTT_PUB_TEMP "s1/temp"   // Temperatura
#define MQTT_PUB_HUM  "s1/hum"    // Umidità
#define MQTT_PUB_HI "s1/hi"       // Temperatura percepita

// Parametri sensore DHT
#define DHTPIN 33  
#define DHTTYPE DHT11 

DHT dht(DHTPIN, DHTTYPE);          // Inizializzazione dell'oggetto per il sensore DHT 

float temp;   // Variabile per la temperatura
float hum;    // Variabile per l'umidità
float hi;     // Variabile per la temperatura percepita

AsyncMqttClient mqttClient;        // Creazione MQTT client
TimerHandle_t mqttReconnectTimer;  // Creazione timer di riconnessine MQTT
TimerHandle_t wifiReconnectTimer;  // Creazione timer di riconnessione WIFI

// Variabili per la gestione del timer MQTT
unsigned long previousMillis = 0;   
const long interval = 1000;        

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

/*
Funzione che verifica se è connesso al WIFI e in quel caso prova a connettersi all'MQTT broker 
oppure in caso contrario avvia il timer per gestire la riconnessione
*/
void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %dn", event);
  switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      xTimerStop(mqttReconnectTimer, 0); 
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttPublish(uint16_t packetId) {
  Serial.print("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void setup() {
  // Inizializzazione seriale
  Serial.begin(115200);
  Serial.println();

  dht.begin();

  // Inizializzazione timer MQTT e WIFI
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  // Dichiarazione delle callback per gli eventi WIFI e MQTT
  WiFi.onEvent(WiFiEvent);
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  connectToWifi();
}

void loop() {
  // Lettura da sensore (Temperatura, Umidità, Temperatura Percepita) ogni intervallo di tempo definito da interval
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    hum = dht.readHumidity();         // Lettura valore umidità dal sensore dht
    temp = dht.readTemperature();     // Lettura valore temperatura dal sensore dht
    if (isnan(temp) || isnan(hum)) {  // In caso non riceva un numero (lettura fallita)
      Serial.println(F("Failed to read from DHT sensor!"));
      return;
    }
    hi = dht.computeHeatIndex(temp, hum, false);  // Calcolo temperatura percepita tramite libreria dht
    
    // Pubblica un messaggio MQTT sul topic relativo alla temperature
    uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_TEMP, 1, true, String(temp).c_str());                            
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i \t", MQTT_PUB_TEMP, packetIdPub1);
    Serial.printf("Message: %.2f n", temp);

    // Pubblica un messaggio MQTT sul topic relativo all'umidità
    uint16_t packetIdPub2 = mqttClient.publish(MQTT_PUB_HUM, 1, true, String(hum).c_str());                            
    Serial.printf("Publishing on topic %s at QoS 1, packetId %i \t", MQTT_PUB_HUM, packetIdPub2);
    Serial.printf("Message: %.2f n", hum);

    // Pubblica un messaggio MQTT sul topic relativo alla temperatura percepita
    uint16_t packetIdPub3 = mqttClient.publish(MQTT_PUB_HI, 1, true, String(hi).c_str());                            
    Serial.printf("Publishing on topic %s at QoS 1, packetId %i \t", MQTT_PUB_HI, packetIdPub3);
    Serial.printf("Message: %.2f n", hi);
  }
}
