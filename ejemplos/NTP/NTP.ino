/*
 *Programa conexión WiFi y conexión a servidor NTP para obtener la hora con la tarjeta de IoHT  
 *Conexión a servidor NTP 
 *Descripción: En este programa usaremos la tarjeta de IoHT, conectaremos a una red WiFi y un servidor NTP
 *Con ello obtendremos la hora mundial
 *
 *Programa realizado por Angel Isidro - 29 enero 2021 - Versión 1 
 */

#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>


const char *ssid     = "Cuarto_De_Juego";
const char *password = "A15004127";

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

void setup(){
  Serial.begin(115200);
  //Inicializamos WiFi
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  timeClient.begin();
}

void loop() {
  timeClient.update(); // Hace update en la hora 
  Serial.println(timeClient.getFormattedTime()); // Imprime en el monitor serial  la hora 
  delay(1000);
}
