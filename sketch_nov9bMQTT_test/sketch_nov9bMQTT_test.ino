#include <WiFi.h> //library to connect ESP to Wi-Fi
#include <PubSubClient.h> //library for communication with broker using MQTT

// WiFi
const char *ssid = "xxxxx"; // Enter Wi-Fi name
const char *password = "xxxxx";  // Enter Wi-Fi password

// MQTT Broker - enter details
const char *mqtt_broker = "xxxxx"; //address (IP or domain)
const char *topic = "xxxxx"; //topic to pubish / subscribe to
const char *mqtt_username = "xxxxx"; 
const char *mqtt_password = "xxxxx";
const int mqtt_port = 1883; //broker port (1883 for not secure on default)

WiFiClient espClient; //Wi-Fi client used for communication
PubSubClient client(espClient); //MQtt client using Wi-Fi client

void setup() {
    // Set software serial baud to 115200;
    Serial.begin(115200);
    // Connecting to a Wi-Fi network
    WiFi.begin(ssid, password);
    //wait until connected to Wi-Fi
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.println("Connecting to WiFi..");
    }
    Serial.println("Connected to the Wi-Fi network");
    //connecting to a mqtt broker
    client.setServer(mqtt_broker, mqtt_port);
    client.setCallback(callback); //callback function to handle incoming messages
    while (!client.connected()) {
        String client_id = "esp32-client-";
        client_id += String(WiFi.macAddress()); //create unique client ID
        Serial.printf("The client %s connects to the public MQTT broker\n", client_id.c_str());
        if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
            Serial.println("Public EMQX MQTT broker connected");
        } else {
            Serial.print("failed with state ");
            Serial.print(client.state()); //print state if it fails
            delay(2000); //try again after 2 seconds
        }
    }
    // Publish and subscribe
    client.publish(topic, "Hi, I'm ESP32 ^^");
    client.subscribe(topic);
}

//called whenever message is received on subscribed topic
void callback(char *topic, byte *payload, unsigned int length) {
    Serial.print("Message arrived in topic: ");
    Serial.println(topic);
    Serial.print("Message:");
    for (int i = 0; i < length; i++) {
        Serial.print((char) payload[i]);
    }
    Serial.println();
    Serial.println("-----------------------");
}

void loop() {
    client.loop(); //maintain connection with MQTT broker, process incoming messages
}

