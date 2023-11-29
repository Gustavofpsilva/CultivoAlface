#include <Wire.h>
#include <Adafruit_SHT31.h>
#include <PubSubClient.h>
#include <WiFi.h>

// Defina suas credenciais Wi-Fi
const char *ssid = "SEU_SSID";
const char *password = "SUA_SENHA";

// Defina as informações do servidor MQTT
const char *mqtt_server = "BROKER_MQTT";
const int mqtt_port = 1883;
const char *mqtt_user = "SEU_USUARIO_MQTT";
const char *mqtt_password = "SUA_SENHA_MQTT";
const char *mqtt_client_id = "ID_CLIENTE";

// Pinos de sensores
const int phSensorPin = A0;
const int tdsSensorPin = A1;
const int co2SensorPin = A2;
const int relayPin = 13;  // Pino do relé

// Objeto para o sensor SHT20
Adafruit_SHT31 sht = Adafruit_SHT31();

// Objeto para o cliente MQTT
WiFiClient espClient;
PubSubClient client(espClient);

unsigned long previousMillis = 0;
const long interval = 12 * 60 * 60 * 1000;  // 12 horas em milissegundos

void setup_wifi() {
  delay(10);
  // Conecte-se à rede Wi-Fi
  Serial.println();
  Serial.print("Conectando à rede ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi conectado");
  Serial.println("Endereço IP: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  // Loop até que possamos reconectar
  while (!client.connected()) {
    Serial.print("Tentando reconectar ao MQTT...");

    if (client.connect(mqtt_client_id, mqtt_user, mqtt_password)) {
      Serial.println("Conectado");
    } else {
      Serial.print("Falhou, rc=");
      Serial.print(client.state());
      Serial.println("Tentando novamente em 5 segundos");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);

  // Inicialize o sensor SHT20
  if (!sht.begin(0x44)) {
    Serial.println("Não foi possível encontrar o sensor SHT20, verifique a conexão!");
    while (1);
  }

  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, LOW);  // Inicialmente, desligue o relé

  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }

  unsigned long currentMillis = millis();

  // Leitura de dados dos sensores a cada 10 segundos
  if (currentMillis - previousMillis >= 10000) {
    previousMillis = currentMillis;

    // Leitura de dados dos sensores
    float temperature = sht.readTemperature();
    float humidity = sht.readHumidity();
    int phValue = analogRead(phSensorPin);
    int tdsValue = analogRead(tdsSensorPin);
    int co2Value = analogRead(co2SensorPin);

    // Envio dos dados para o servidor MQTT
    char tempString[8];
    char humString[8];
    char phString[8];
    char tdsString[8];
    char co2String[8];

    dtostrf(temperature, 2, 2, tempString);
    dtostrf(humidity, 2, 2, humString);
    dtostrf(phValue, 2, 0, phString);
    dtostrf(tdsValue, 2, 0, tdsString);
    dtostrf(co2Value, 2, 0, co2String);

    client.publish("cultivo/temperatura", tempString);
    client.publish("cultivo/umidade", humString);
    client.publish("cultivo/ph", phString);
    client.publish("cultivo/tds", tdsString);
    client.publish("cultivo/co2", co2String);
  }

  // Controle do relé - liga por 12 horas, desliga por 12 horas
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Inverte o estado do relé
    if (digitalRead(relayPin) == LOW) {
      digitalWrite(relayPin, HIGH);  // Liga o relé
      client.publish("cultivo/acao", "Relé ligado");
    } else {
      digitalWrite(relayPin, LOW);   // Desliga o relé
      client.publish("cultivo/acao", "Relé desligado");
    }
  }

  delay(100);  // Pequeno atraso para estabilidade
  client.loop();
}
