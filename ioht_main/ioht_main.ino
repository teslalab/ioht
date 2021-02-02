/*
 *Programa para el funcionamiento de la tarjeta de IoHT  
 *visualización de datos, envío de datos a GALIoT y funcionamiento de pantalla OLEd y Botones touch 
 *Programa realizado por Angel Isidro y Gabriel Monzón - 02 febrero 2021 - Versión 1 
 */

#include <EasyBuzzer.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_FeatherOLED.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_NeoPixel.h>
#include "Adafruit_BME680.h"
#include "bsec.h"
#include <WiFi.h>
#include <PubSubClient.h> //Libreria para publicación y recepción de datos.
#include <NTPClient.h>
#include <WiFiUdp.h>
/*
Importante configuarar las siguientes variables, para el correcto funcionamiento en el dashboar de GALIoT
también para logra una conexión a Nuestra Red WiF
*/
//A continuación debe reemplazar el nombre y numero de estación que le fue asignado
#define TEAM_NAME "ioht/isidro/003" //  proyecto/nombre/estacion 
// Variables para la conexión a Red WiFi
const char* ssid = "Tigo-9635"; // Debe reemplazar por el nombre de su RED WiFi
const char* password = "2NJ555301438"; // Debe reemplazar por la contraseña de su RED WiFi

/*
A Partir de aca no tocar nada en el código
*/
// Credenciales para GALioT
#define USERNAME "aquality"
#define PASSWORD "$Air333"
const char* mqtt_server = "galiot.galileo.edu";

#define SEALEVELPRESSURE_HPA (1013.25)

#define buzzer 27

#define pinRelay 35

Adafruit_FeatherOLED oled = Adafruit_FeatherOLED();

Adafruit_BME680 bme; // I2C
//Adafruit_BME680 bme(BME_CS); // hardware SPI
//Adafruit_BME680 bme(BME_CS, BME_MOSI, BME_MISO,  BME_SCK);

#define PIXEL_PIN   14  // Digital IO pin connected to the NeoPixels.
#define PIXEL_COUNT 6  // Number of NeoPixels

//values for alarms
int TEMP_MAX = 33;
int TEMP_MIN = 15;

int HUME_MAX = 85;
int HUME_MIN = 30;

int SAQI_MAX = -999;
int SAQI_MIN = -999;

int WIFI_WARNING = -999;

// CONFIGURACION DE GALIOT Y MQTT 
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
// FIN CONFIGURACION DE GALIOT Y MQTT

//double temp, hume, pres, sAQI = 0;

const int touch_treshold = 10;

#include "bsec.h" // Libreria BSEC de Bosch para BME680

//************** Funciones para el Sensor BME680 **************
//Declaración de funciones de ayuda
void checkIaqSensorStatus(void);
// Creamos un objeto con clase BSEC Software
Bsec iaqSensor;
String output, output2;

int pin_boton_arriba = 4;
int pin_boton_derecha = 15;  
int pin_boton_abajo = 33;
int pin_boton_izquierda = 2;
int pin_boton_cancel = 12;
int pin_boton_enter = 13;

// Declare our NeoPixel strip object:
Adafruit_NeoPixel neopixelLEDs(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

/**
 * Made with Marlin Bitmap Converter
 * https://marlinfw.org/tools/u8glib/converter.html
 *
 * This bitmap from the file 'TL.bmp'
 */
const unsigned char TL[] PROGMEM = {
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0xFF,0xFF,0xFF,0xFF,0xFC,0x03,0xFF,0xFC,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x01,0xFF,0xFF,0xFF,0xF7,0xFC,0x07,0xFF,0xFC,0x00,0x00,0x00,0x00,0xF0,0x00,0x00,
  0x03,0xFF,0xFF,0xFF,0xEF,0xFC,0x07,0xFF,0xFC,0x00,0x00,0x00,0x00,0xF0,0x00,0x00,
  0x07,0xFF,0xFF,0xFF,0xCF,0xF8,0x07,0xFF,0xF8,0x00,0x00,0x00,0x00,0xF0,0x00,0x00,
  0x0F,0xFF,0xFF,0xFF,0x8F,0xF8,0x00,0x1E,0x00,0x00,0x00,0x00,0x00,0xF0,0x00,0x00,
  0x1F,0xFF,0xFF,0xFF,0x9F,0xF8,0x00,0x1E,0x00,0x00,0x00,0x00,0x00,0xF0,0x00,0x00,
  0x3F,0xFF,0xFF,0xFF,0x1F,0xF0,0x00,0x1E,0x01,0xFF,0x00,0x7F,0x80,0xF0,0x07,0xF8,
  0x3F,0xFF,0xFF,0xFE,0x1F,0xF0,0x00,0x1E,0x03,0xFF,0xC1,0xFF,0xE0,0xF0,0x1F,0xFC,
  0x7F,0xFF,0xFF,0xFC,0x3F,0xF0,0x00,0x1E,0x07,0xFF,0xE3,0xFF,0xF0,0xF0,0x3F,0xFE,
  0x00,0x03,0xFE,0x00,0x3F,0xE0,0x00,0x1E,0x07,0xFF,0xE3,0xFF,0xF0,0xF0,0x3F,0xFE,
  0x00,0x03,0xFE,0x00,0x3F,0xE0,0x00,0x1E,0x0F,0x81,0xE3,0xC0,0xF0,0xF0,0x3F,0xFF,
  0x00,0x07,0xFE,0x00,0x7F,0xE0,0x00,0x1E,0x0F,0x81,0xE3,0xC0,0x00,0xF0,0x38,0x1F,
  0x00,0x07,0xFC,0x00,0x7F,0xC0,0x00,0x1E,0x0F,0x81,0xE3,0xF8,0x00,0xF0,0x00,0xFF,
  0x00,0x07,0xFC,0x00,0x7F,0xC0,0x00,0x1E,0x0F,0xFF,0xE3,0xFF,0x80,0xF0,0x1F,0xFF,
  0x00,0x0F,0xFC,0x3F,0xFF,0xFF,0xFE,0x1E,0x0F,0xFF,0xE1,0xFF,0xE0,0xF0,0x3F,0xFF,
  0x00,0x0F,0xF8,0x7F,0xFF,0xFF,0xFC,0x1E,0x0F,0xFF,0xE0,0x7F,0xF0,0xF0,0x7F,0xFF,
  0x00,0x0F,0xF8,0xFF,0xFF,0xFF,0xF8,0x1E,0x0F,0x80,0x03,0x8F,0xF0,0xF0,0x7C,0x1F,
  0x00,0x1F,0xF9,0xFF,0xFF,0xFF,0xF8,0x1E,0x0F,0x81,0xE3,0xC0,0xF0,0xF0,0x78,0x1F,
  0x00,0x1F,0xF3,0xFF,0xFF,0xFF,0xF0,0x1E,0x0F,0x81,0xE3,0xC0,0xF0,0xF0,0x78,0xFF,
  0x00,0x1F,0xF3,0xFF,0xFF,0xFF,0xE0,0x1E,0x07,0xFF,0xE3,0xFF,0xF0,0xFE,0x7F,0xFF,
  0x00,0x3F,0xF7,0xFF,0xFF,0xFF,0xC0,0x1E,0x07,0xFF,0xE3,0xFF,0xF0,0xFE,0x7F,0xFF,
  0x00,0x3F,0xEF,0xFF,0xFF,0xFF,0x80,0x1E,0x07,0xFF,0xC3,0xFF,0xF0,0x7E,0x7F,0xFF,
  0x00,0x3F,0xFF,0xFF,0xFF,0xFF,0x00,0x1E,0x03,0xFF,0x81,0xFF,0xE0,0x3E,0x3F,0x06,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
};


void setup() {


  Serial.begin(9600);         //Initialize serial port

  oled.init();                //Initialize OLED Display
  //oled.setBatteryVisible(true);

  oled.println("Starting ....");
  oled.display();

  EasyBuzzer.setPin(buzzer);  //Initialize buzzer
  
  testingBuzzer();
  testingRelay();

  while (!Serial);
  
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

  output = "Timestamp [ms], raw temperature [°C], pressure [hPa], raw relative humidity [%], gas [Ohm], IAQ, IAQ accuracy, temperature [°C], relative humidity [%], Static IAQ, CO2 equivalent, breath VOC equivalent";
  Serial.println(output);
  
  // Testing buzzer
  pinMode(buzzer, OUTPUT);

  //Initialize neopixels
  neopixelLEDs.begin(); // Initialize NeoPixel strip object (REQUIRED)
  neopixelLEDs.setBrightness(128);
  neopixelLEDs.show();  // Initialize all pixels to 'off'

  //Testing LEDs
  testLEDs();

  oled.clearDisplay();
  //oled.drawBitmap(0 , -21 , TL , 128 , 64 , WHITE );
  oled.setCursor(0,0);
  oled.println("Inter. of Home Things");
  oled.display();
  oled.setTextSize(1);
  oled.drawBitmap(0 , 0 , TL , 128 , 64 , WHITE );
  oled.display();
  delay(8000);

}

int data_index = 0;

void loop() {

  output2 = "";
  if (iaqSensor.run()) { // If new data is available
    Serial.println(output2);     
  } else {
      checkIaqSensorStatus();
  }
  oled.clearDisplay();
  preHeatSensor();


  // checking values for alarms

  int rojoRGB[3] = {255, 0, 0};
  int celesteRGB[3] = {0, 181, 255};
  
  //checksAlarm(temp, TEMP_MAX, TEMP_MIN, "Temperatura", rojoRGB, celesteRGB);
  //checksAlarm(hume, HUME_MAX, HUME_MIN, "Humedad", rojoRGB, celesteRGB);
  //checksAQI(sAQI, SAQI_MAX, SAQI_MIN);


  if (getTouch(pin_boton_enter))
  {
    displayMenu();
  }


  delay(10);
}

void testingBuzzer() {
  EasyBuzzer.beep(840);    // Frequency in hertz(HZ).
  delay(500);
  EasyBuzzer.stopBeep();
  delay(250);
  EasyBuzzer.beep(840);    // Frequency in hertz(HZ).
  delay(500);
  EasyBuzzer.stopBeep();
  delay(250);
}

void beepBuzzer() {
  EasyBuzzer.beep(1000);    // Frequency in hertz(HZ).
  delay(500);
  EasyBuzzer.stopBeep();
  delay(250);
}

void flashLED (int led) {
  uint32_t color = neopixelLEDs.Color(255, 0, 0);   //Set color red
  for (int i = 0; i < 3; i++) { // For each pixel in strip...
    neopixelLEDs.setPixelColor(led, color); //  Set pixel's color (in RAM)
    delay(50);
    neopixelLEDs.show();                          //  Update strip to match
    delay(500);
    //Turning LEDs off
    neopixelLEDs.clear();
    neopixelLEDs.show();
    delay(250);
  }

}

void testingRelay() {
  digitalWrite(pinRelay, HIGH);
  delay(1000);
  digitalWrite(pinRelay, LOW);
  delay(1000);
}

void testLEDs() {
  for (int i = 0; i < neopixelLEDs.numPixels(); i++) { // For each pixel in strip...
    neopixelLEDs.setPixelColor(i, neopixelLEDs.Color(255, 0, 0)); //  Set pixel's color (in RAM)
    neopixelLEDs.show();                          //  Update strip to match
    delay(250);
    neopixelLEDs.setPixelColor(i, neopixelLEDs.Color(0, 255, 0)); //  Set pixel's color (in RAM)
    neopixelLEDs.show();                          //  Update strip to match
    delay(250);
    neopixelLEDs.setPixelColor(i, neopixelLEDs.Color(0, 0, 255)); //  Set pixel's color (in RAM)
    neopixelLEDs.show();                          //  Update strip to match
    delay(250);
  }
  //Turning LEDs off
  neopixelLEDs.clear();
  neopixelLEDs.show();
  delay(250);
}

void testTouchButton (int dataTouchButton, int LED) {
  if (dataTouchButton < 12) {
    neopixelLEDs.setPixelColor(LED, neopixelLEDs.Color(0, 0, 255)); //  Set pixel's color (in RAM)
    neopixelLEDs.show();                          //  Update strip to match
    delay(1000);
    //Turning LEDs off
    neopixelLEDs.clear();
    neopixelLEDs.show();
    delay(250);
  }
}

void setLedsRGB(int colors[3], int last_led, int delay1) {

  for (int i = 0; i < last_led; i++) {
    neopixelLEDs.setPixelColor(i, neopixelLEDs.Color(colors[0], colors[1], colors[2]));
  }

  neopixelLEDs.show();
  delay(delay1);
  neopixelLEDs.clear();
  neopixelLEDs.show();
}

void setBuzzer(int frequency, int count, int delay1) {
  for (int i = 0; i < count; i++) {
    EasyBuzzer.beep(frequency);
    delay(delay1);
    EasyBuzzer.stopBeep();
    delay(delay1);
  }
}

void setLedsAndBuzzer(int colors[3], int last_led, int frequency, int count, int delay1) {

  for (int i = 0; i < count; i++) {
    for (int i = 0; i < last_led; i++) {
      neopixelLEDs.setPixelColor(i, neopixelLEDs.Color(colors[0], colors[1], colors[2]));
    }
    neopixelLEDs.show();
    delay(delay1);
    EasyBuzzer.beep(frequency);
    delay(delay1);
    neopixelLEDs.clear();
    neopixelLEDs.show();
    EasyBuzzer.stopBeep();
    delay(delay1);
  }
}

void checksAQI(int value, int max, int min) {
  bool check = false;
  
  int rojoRGB[3] = {255, 0, 0};
  int celesteRGB[3] = {0, 181, 255};
  
  if (max != -999) {
    if (value > max) {

      check = true;
      oled.clearDisplay();
      oled.setCursor(0, 0);
      oled.println("Niveles de sAQI altos");
      oled.print("Valor: ");
      oled.println(value);

      oled.display();

      setLedsAndBuzzer(rojoRGB, PIXEL_COUNT, 1000, 6, 100);
    }
  }
  if (min != -999) {

      check = true;
    if (value < min) {
      oled.clearDisplay();
      oled.setCursor(0, 0);
      oled.println("Niveles de sAQI bajos");
      oled.print("Valor: ");
      oled.println(value);

      oled.display();

      setLedsAndBuzzer(celesteRGB, PIXEL_COUNT, 500, 3, 200);
    }
  }
  if (!check){
    oled.clearDisplay();
    oled.setCursor(0, 0);
    oled.println("Niveles de sAQI:");
    oled.println(value);
    if (value > 300){
      int colors [3] = {126,0,35};
      setLedsRGB(colors, PIXEL_COUNT, 1000);
    }else if (value > 200){
      int colors [3] = {143,63,151};
      setLedsRGB(colors, PIXEL_COUNT, 1000);
    }else if (value > 150){
      int colors [3] = {255,0,0};
      setLedsRGB(colors, PIXEL_COUNT, 1000);
    }else if (value > 100){
      int colors [3] = {255,126,0};
      setLedsRGB(colors, PIXEL_COUNT, 1000);
    }else if (value > 50){
      int colors [3] = {255,255,0};
      setLedsRGB(colors, PIXEL_COUNT, 1000);
    }else if (value > 0){
      int colors [3] = {0,228,0};
      setLedsRGB(colors, PIXEL_COUNT, 1000);
    }
  }
}

void checksAlarm(int value, int max, int min, String nombre, int colorsHigh [3], int colorsLow [3]) {
  bool check = false;
  if (max != -999) {
    if (value > max) {

      oled.clearDisplay();
      oled.setCursor(0, 0);
      oled.print("Nivel de ");
      oled.println(nombre);
      oled.println("ALTOS");
      oled.print("Valor: ");
      oled.println(value);

      oled.display();

      setLedsAndBuzzer(colorsHigh, PIXEL_COUNT, 1000, 6, 100);
    }
  }
  if (min != -999) {
    if (value < min) {

      oled.clearDisplay();
      oled.setCursor(0, 0);
      oled.print("Nivel de ");
      oled.print(nombre);
      oled.println("BAJOS");
      oled.print("Valor: ");
      oled.println(value);


      oled.display();

      setLedsAndBuzzer(colorsLow, PIXEL_COUNT, 500, 3, 200);
    }
  }
}


// Funciones de ayuda en el sensor BME680
void checkIaqSensorStatus(void){
  if (iaqSensor.status != BSEC_OK) {
    if (iaqSensor.status < BSEC_OK) {
      oled.clearDisplay();
      oled.setCursor(0,9);
      oled.print("BSEC error code: " + String(iaqSensor.status));
      oled.display();
      delay(5000);
      oled.clearDisplay();
      output = "BSEC error code : " + String(iaqSensor.status);
      Serial.println(output);
    } else {
      output = "BSEC warning code : " + String(iaqSensor.status);
      oled.clearDisplay();
      oled.setCursor(0,9);
      oled.print("BSEC warning code: " + String(iaqSensor.status));
      oled.display();
      delay(5000);
      oled.clearDisplay();
      Serial.println(output);
    }
  }

  if (iaqSensor.bme680Status != BME680_OK) {
    if (iaqSensor.bme680Status < BME680_OK) {
      output = "BME680 error code : " + String(iaqSensor.bme680Status);
      oled.clearDisplay();
      oled.setCursor(0,9);
      oled.print("BME680 error code: " + String(iaqSensor.bme680Status));
      oled.display();
      delay(5000);
      oled.clearDisplay();
      Serial.println(output);
    } else {
      output = "BME680 warning code : " + String(iaqSensor.bme680Status);
      oled.clearDisplay();
      oled.setCursor(0,9);
      oled.print("BSEC warning code: " + String(iaqSensor.bme680Status));
      oled.display();
      delay(5000);
      oled.clearDisplay();
      Serial.println(output);
    }
  }
}

bool getTouch(int pin){

  int valor = touchRead(pin);
  if (valor < touch_treshold)
  {
    delay(50);
    valor = touchRead(pin);
    if (valor < touch_treshold)
    {
      return true;
    }
  }
  return false;
}

int menu_value = 0;

void displayMenu(){
  
  while(!getTouch(pin_boton_cancel)){
    
    oled.clearDisplay();
    oled.setCursor(0, 0);
    
    String menu_string = "";

    switch(menu_value){
      case 0:
        menu_string = "\n <  Ver Datos  >";
        break;
      case 1:
        menu_string = "\n<Configurar Alarmas>";
        break;
      case 2:
        menu_string = "\n<  Tocar musica  >";
        break;
      case 3:
        menu_string = "\n< Jugar con Leds >";
        break;
      case 4:
        menu_string = "\n< Luz con Timer >";
        break;
    }

    oled.print(menu_string);
    oled.display();

    if(getTouch(pin_boton_derecha)){
      menu_value = (menu_value == 4) ? 0 : menu_value + 1;
    }

    if(getTouch(pin_boton_izquierda)){
      menu_value = (menu_value == 0) ? 4 : menu_value - 1;
    }

    if (getTouch(pin_boton_enter)){
      switch (menu_value){
      case 0:
        menuVerDatos();
        break;
      case 1:
        menuConfigurarAlarmas();
        break;
      case 2:
        tocarMusica();
        break;
      case 3:
        jugarConLeds();
        break;
      case 4:
        luzConTimer();
        break;
      
      }
    }
    
    delay(200);

  }

}

void menuVerDatos(){}
void menuConfigurarAlarmas(){}
void tocarMusica(){}
void jugarConLeds(){}

void luzConTimer(){
  int r = 100;
  int g = 100;
  int b = 100;
  int index = 0;
  
  oled.clearDisplay();
  oled.setCursor(0, 0);
  oled.println("Elija Color (RGB)");

  while(!getTouch(pin_boton_cancel)){
    oled.clearDisplay();
    oled.setCursor(0, 0);
    oled.println("Elija Color (RGB)");
    oled.print("R:  ");
    oled.println(r);
    oled.print("G:  ");
    oled.println(g);
    oled.print("B:  ");
    oled.println(b);
    oled.display();
    delay(100);

    switch (index){
      case 0:
      oled.clearDisplay();
      oled.setCursor(0, 0);
      oled.println("Elija Color (RGB)");
      oled.println();
      oled.print("G:  ");
      oled.println(g);
      oled.print("B:  ");
      oled.println(b);
      oled.display();
      delay(50);
      break;

      case 1:
      oled.clearDisplay();
      oled.setCursor(0, 0);
      oled.println("Elija Color (RGB)");
      oled.print("R:  ");
      oled.println(r);
      oled.println();
      oled.print("B:  ");
      oled.println(b);
      oled.display();
      delay(50);
      break;

      case 2:
      oled.clearDisplay();
      oled.setCursor(0, 0);
      oled.println("Elija Color (RGB)");
      oled.print("R:  ");
      oled.println(r);
      oled.print("G:  ");
      oled.println(g);
      oled.println();
      oled.display();
      delay(50);
      break;
    
    }
    

    if(getTouch(pin_boton_derecha)){
      index = (index == 2) ? 0 : index + 1;
    }
    if(getTouch(pin_boton_izquierda)){
      index = (index == 0) ? 2 : index - 1;
    }
    if(getTouch(pin_boton_arriba)){
      switch (index){
      case 0:
        r = (r >= 255) ? 255 : r + 5;
      break;
      case 1:
        g = (g >= 255) ? 255 : g + 5;
      break;
      case 2:
        b = (b >= 255) ? 255 : b + 5;
      break;
      }
    }
    if(getTouch(pin_boton_abajo)){
      switch (index){
      case 0:
        r = (r <= 0) ? 0 : r - 5;
      break;
      case 1:
        g = (g <= 0) ? 0 : g - 5;
      break;
      case 2:
        b = (b <= 0) ? 0 : b - 5;
      break;
      }
    }

    int tiempo = 0;
    if(getTouch(pin_boton_enter)){
      while (!getTouch(pin_boton_cancel)){
        oled.clearDisplay();
        oled.setCursor(0, 0);
        oled.println("Timer en Segundos: ");
        oled.println();
        oled.print("      ");
        oled.println(tiempo);
        oled.display();
        delay(50);

        if(getTouch(pin_boton_arriba)){
          tiempo = tiempo + 10;
        }
        if(getTouch(pin_boton_abajo)){
          tiempo = tiempo - 10;
        }

        if(getTouch(pin_boton_enter)){
          oled.clearDisplay();
          oled.setCursor(0, 0);
          int tiempo_actual = millis() / 1000;
          oled.println("Luz con timer");
          oled.display();
          delay(50);
          for (int i = 0; i < PIXEL_COUNT; i++) {
            neopixelLEDs.setPixelColor(i, neopixelLEDs.Color(r, g, b));
          }
          neopixelLEDs.show();
          while (!getTouch(pin_boton_cancel) && (tiempo_actual + tiempo > millis()/1000)) {
            oled.clearDisplay();
            oled.setCursor(0, 0);
            oled.println("Luz con timer");
            oled.println(tiempo_actual + tiempo - millis()/1000);
            oled.display();
          }
          neopixelLEDs.clear();
          neopixelLEDs.show();
          
        }
      }
      delay(100);
    }

  }


}

void errLeds(void){

 for (int i = 0; i < neopixelLEDs.numPixels(); i++) { // For each pixel in strip...
    neopixelLEDs.setPixelColor(i, neopixelLEDs.Color(255, 0, 0)); //  Set pixel's color (in RAM)
    neopixelLEDs.show();                          //  Update strip to match
    delay(250);
   
  }
  //Turning LEDs off
  neopixelLEDs.clear();
  neopixelLEDs.show();
  delay(250);

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
  //if (iaqSensor.run()) { // If new data is available
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
    //Serial.println(output2);
    oled.clearDisplay();
    oled.setCursor(0, 0);
    oled.println("Tesla Lab Data");
  
    oled.print("T ");
    oled.print(iaqSensor.temperature);
    oled.println(" *C");

    oled.print("H ");
    oled.print(iaqSensor.humidity);
    oled.println(" %");

    oled.print("sAQI ");
    oled.println(iaqSensor.staticIaq);

    oled.display();
    
 // } else {
 //   checkIaqSensorStatus();
  //}
}