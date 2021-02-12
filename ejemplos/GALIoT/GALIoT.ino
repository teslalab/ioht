/*
 *Programa conexión WiFi y envio de datos a la plataforma GALIoT con la tarjeta de IoHT  
 *Conexión y envio de información 
 *Descripción: En este programa usaremos la tarjeta de IoHT, conectaremos a la plataforma GALIoT
 *con ello podremos visualizar información de nuestras variables ambientales en los dashboard de IoHT 
 *cada 30 minutos se publicaran nuestro datos en el dashboard de GALIoT
 *Programa realizado por Angel Isidro - 12 febrero 2021 - Versión 1.2 
 */
// Libreria para usar las propiedades de conexión WiFi
  #include <WiFi.h>
  #include <PubSubClient.h> //Libreria para publicación y recepción de datos.
  #include <Wire.h> //Conexión de dispositivos I2C
  #include "bsec.h" 
  #include <NTPClient.h>
  #include <WiFiUdp.h>
  
// Variables para conexion NTP
  WiFiUDP ntpUDP;
  NTPClient timeClient(ntpUDP);
// Libreria para usar las propiedades de la pantalla OLED
  #include <Adafruit_FeatherOLED.h> 
  Adafruit_FeatherOLED oled = Adafruit_FeatherOLED();  

//Credenciales para poder conectarnos a la red-WiFi sustituya dentro de las comillas
  const char* ssid = "SSID"; // Nombre del SSID
  const char* password = "contraseña";   // Contraseña
//Modificar al nombre que se asigne en el dashboard.
  #define TEAM_NAME "xxxx/yyyy/00x" //  proyecto/usuario/no.estacion 
  /*
  ioht/ugal/001
  ioht/oscar/001
  ioht/isidro/001
  ioht/gabriel/001
  */


// Credenciales para GALioT
  #define USERNAME "aquality"
  #define PASSWORD "$Air333"
  
//Nombre del Servidor MQTT
  const char* mqtt_server = "galiot.galileo.edu";

//Declaración de funciones de ayuda
  void checkIaqSensorStatus(void);
//void errLeds(void);
// Creamos un objeto con clase BSEC Software
  Bsec iaqSensor;
  String output, output2;  

// data variables
int lastReadingTime,lastReadingTime2 = 0;
double temp,ds18, hume, pres = 0;
double aqi , sAQI , AQIa = 0;
double CO2e, VOCe, gas, rssi = 0; 

char msg[50];
char msg1[50];
char cnt[50];
char msg_r[50];
char topic_name[250];

// network variables
WiFiClient espClient;
PubSubClient mqtt_client(espClient);

    
void setup() {
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
  //Fin de la inicialización WiFi
  //Setup para MQTT
  setupMQTT();
  timeClient.begin();
//Setup para el BME680
   Wire.begin(21,22);
   iaqSensor.begin(0x77, Wire);
   output = "\nBSEC library version " + String(iaqSensor.version.major) + "." + String(iaqSensor.version.minor) + "." + String(iaqSensor.version.major_bugfix) + "." + String(iaqSensor.version.minor_bugfix);
   Serial.println(output);
   checkIaqSensorStatus();
   bsec_virtual_sensor_t sensorList[10] = {
    BSEC_OUTPUT_RAW_TEMPERATURE,
    BSEC_OUTPUT_RAW_PRESSURE,
    BSEC_OUTPUT_RAW_HUMIDITY,
    BSEC_OUTPUT_RAW_GAS,
    BSEC_OUTPUT_IAQ,
    BSEC_OUTPUT_STATIC_IAQ,
    BSEC_OUTPUT_CO2_EQUIVALENT,
    BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
  };

  iaqSensor.updateSubscription(sensorList, 10, BSEC_SAMPLE_RATE_LP);
  checkIaqSensorStatus();

  // Print the header
  output = "Timestamp [ms], raw temperature [°C], pressure [hPa], raw relative humidity [%], gas [Ohm], IAQ, IAQ accuracy, temperature [°C], relative humidity [%], Static IAQ, CO2 equivalent, breath VOC equivalent";
  Serial.println(output);
  delay(3000);
  oled.init();
  oled.clearDisplay();
}

void loop() {
  if (!mqtt_client.connected()) {
    reconnect();
  }
  mqtt_client.loop();
  
  timeClient.update(); // Hace update en la hora 
  //Serial.println(timeClient.getFormattedTime()); // Imprime en el monitor serial  la hora

  if(((timeClient.getMinutes() == 00) && (timeClient.getSeconds() == 00)) || ((timeClient.getMinutes() == 30) && (timeClient.getSeconds() == 00))) {
    publicarDatos();
  }

  
  if(((timeClient.getMinutes() > 14) && (timeClient.getMinutes() < 31)) || ((timeClient.getMinutes() > 44) )) {
  preHeatSensor();
  }

  if(timeClient.getSeconds() == 15){
    String str69 = "Estacion en linea";
    str69.toCharArray(msg, 50);
    mqtt_client.publish(getTopic("Online"), msg); 
  }

  impresion();

  delay(1000);
}

// Helper function definitions
void checkIaqSensorStatus(void)
{
  if (iaqSensor.status != BSEC_OK) {
    if (iaqSensor.status < BSEC_OK) {
      output = "BSEC error code : " + String(iaqSensor.status);
      Serial.println(output);
      for (;;)
        errLeds(); /* Halt in case of failure */
    } else {
      output = "BSEC warning code : " + String(iaqSensor.status);
      Serial.println(output);
    }
  }

  if (iaqSensor.bme680Status != BME680_OK) {
    if (iaqSensor.bme680Status < BME680_OK) {
      output = "BME680 error code : " + String(iaqSensor.bme680Status);
      Serial.println(output);
      for (;;)
        errLeds(); /* Halt in case of failure */
    } else {
      output = "BME680 warning code : " + String(iaqSensor.bme680Status);
      Serial.println(output);
    }
  }
}

void errLeds(void)
{
 pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
 delay(100);
  digitalWrite(2, LOW);
  delay(100);
}

void wifiInfo(){
  //oled.clearDisplay();
  oled.setCursor(0,9);
  oled.print("SSID: ");
  oled.println(ssid);
  oled.println("Direccion IP: ");
  oled.println(WiFi.localIP());
  oled.display();
  oled.setTextSize(1);
}

void publish(char* topic, char* payload) {
  Serial.println(topic_name);
  mqtt_client.publish(topic_name, payload);
}

char* getTopic(char* topic) {
  sprintf(topic_name, "/%s/%s", TEAM_NAME, topic);
  return topic_name;
}

void reconnect() {
  // Loop until we're reconnected
  while (!mqtt_client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    Serial.println("connected");

    if (mqtt_client.connect(TEAM_NAME)) {
      
      //mqtt_client.subscribe(getTopic("rgb"));
    }else {
      Serial.print("failed, rc=");
      Serial.print(mqtt_client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    msg_r[i] = (char)payload[i];
  }
  msg_r[length] = 0;
  Serial.print("'");
  Serial.print(msg_r);
  Serial.println("'");
}

void setupMQTT() {
  delay(10);
  // Inciamos la conexion WiFi con la Red que colocamos
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin( ssid , password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  mqtt_client.setServer(mqtt_server, 1883);
  mqtt_client.setCallback(callback);
}

void publicarDatos(){

    //output = String(time_trigger);
    
    //************ Posteamos la temperatura ************
    temp = iaqSensor.temperature;
    Serial.println("Temperatura : " + String(temp));
    String str(temp);
    str.toCharArray(msg, 50);
    mqtt_client.publish(getTopic("temp"), msg);  
    
    //************ Posteamos la humedad ************
    hume = iaqSensor.humidity;
    Serial.println("Humedad : " + String(hume));
    String str2(hume);
    str2.toCharArray(msg, 50);
    mqtt_client.publish(getTopic("hume"), msg);  
    
    //************ Posteamos la Presion Atmosferica ************
    pres = iaqSensor.pressure;
    Serial.println("Presion Atmosferica : " + String(pres));
    String str3(pres);
    str3.toCharArray(msg, 50);
    mqtt_client.publish(getTopic("pres"), msg);  
    
    //************ Posteamos el Index Air Quality ************
    aqi = iaqSensor.iaq;
    Serial.println("Index Air Quality : " + String(aqi));
    String str4(aqi);
    str4.toCharArray(msg, 50);
    mqtt_client.publish(getTopic("aqi"), msg);  
    
    //************ Posteamos el Static Index Air Quality ************
    sAQI = iaqSensor.staticIaq;
    Serial.println("Static Index Air Quality : " + String(sAQI));
    String str5(sAQI);
    str5.toCharArray(msg, 50);
    mqtt_client.publish(getTopic("sAQI"), msg);  
    
    //************ Posteamos el Index Air Quality Accurary ************
    AQIa = iaqSensor.iaqAccuracy;
    Serial.println("Index Air Quality Accuracy : " + String(AQIa));
    String str6(AQIa);
    str6.toCharArray(msg, 50);
    mqtt_client.publish(getTopic("AQIa"), msg);
    
    //************ Posteamos el Gas Resistence ************
    gas = (iaqSensor.gasResistance)/1000;
    Serial.println("Gas Resistance kOhms: " + String(gas));
    String str7(gas);
    str7.toCharArray(msg, 50);
    mqtt_client.publish(getTopic("gas"), msg);  
    
    //************ Posteamos el CO2 Equivalente ************
    CO2e = iaqSensor.co2Equivalent;
    Serial.println("CO2 Equivalente : " + String(CO2e));
    String str8(CO2e);
    str8.toCharArray(msg, 50);
    mqtt_client.publish(getTopic("CO2e"), msg); 
    
    //************ Posteamos el VOC Equivalente ************
    VOCe = iaqSensor.breathVocEquivalent;
     Serial.println("VOC Equivalente : " + String(VOCe));
    String str9(VOCe);
    str9.toCharArray(msg, 50);
    mqtt_client.publish(getTopic("VOCe"), msg);  
    
    //************ Posteamos la intensidad de señal ************
    rssi = WiFi.RSSI();
    Serial.println("Intensidad de Señal : " + String(rssi));
    String str10(rssi);
    str10.toCharArray(msg, 50);
    mqtt_client.publish(getTopic("rssi"), msg); 

}

void preHeatSensor(){
  unsigned long time_trigger = millis();
  if (iaqSensor.run()) { // If new data is available
    output2 = String(time_trigger);
    
    output2 = String(time_trigger);
    output2 += ", " + String(iaqSensor.rawTemperature);
    output2 += ", " + String(iaqSensor.pressure);
    output2 += ", " + String(iaqSensor.rawHumidity);
    output2 += ", " + String(iaqSensor.gasResistance);
    output2 += ", " + String(iaqSensor.iaq);
    output2 += ", " + String(iaqSensor.iaqAccuracy);
    output2 += ", " + String(iaqSensor.temperature);
    output2 += ", " + String(iaqSensor.humidity);
    output2 += ", " + String(iaqSensor.staticIaq);
    output2 += ", " + String(iaqSensor.co2Equivalent);
    output2 += ", " + String(iaqSensor.breathVocEquivalent);
    Serial.println(output2);
   
    
  } else {
    checkIaqSensorStatus();
  }
}

void impresion(){
  oled.setCursor(0,9);
  oled.clearDisplay();
  oled.setTextSize(1);

  oled.print("Temperatura: ");
  oled.println(iaqSensor.temperature);
  
  oled.print("Humedad: ");
  oled.println(iaqSensor.humidity);

  oled.print("sAQI: ");
  oled.println(iaqSensor.staticIaq);
}