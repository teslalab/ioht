//Libreria para usar las propiedades de la pantalla OLED
  #include <Adafruit_FeatherOLED.h> 
  Adafruit_FeatherOLED oled = Adafruit_FeatherOLED();

  // Definiciones para el uso de la libreria BSEC de Bosch
  #include "bsec.h"
  void checkIaqSensorStatus(void);
  void errLeds(void);
  Bsec bme;
  String output;
  
  //Mediciones disponibles para BME680
  double temp, hume, pres = 0;
  double aqi, sAQI, AQIa = 0;
  double CO2e, VOCe, gas = 0;

     
void setup() {
  Serial.begin(115200);
  
  //Setup para BME680
  Wire.begin(21,22); // Pines I2C del ESP32
  bme.begin(0x77, Wire); 
  bmeSetup();
  
  oled.init();
  oled.clearDisplay();
}

void loop() {
  oled.clearDisplay();
  datosAmbientales(); 
  Serial.println("Se repite");
  //delay(1000); 
}

void datosAmbientales(){
  int i = 9;
  
  //temp = bme.temperature;
  // se detiene calculando la cantidad de ((lineas * 8) + 9 )
  for(i; i > -72 ; i--){
  bmeData();
  oled.setCursor(0,i);
  oled.clearDisplay();
  oled.setTextSize(1); 
  oled.fillRect(0,0,128,8,WHITE);
  oled.print("CO2e: ");
  oled.println(bme.co2Equivalent);
  
  oled.print("VOCe: ");
  oled.println(bme.breathVocEquivalent);
  
  oled.print("GAS: ");
  oled.println(bme.gasResistance);
    
  oled.print("sAQI: ");
  oled.println(bme.iaq);
  
  oled.print("sAQI: ");
  oled.println(bme.staticIaq);
      
  oled.print("Accuracy: ");
  oled.println(bme.iaqAccuracy);
  
  oled.print("Temperatura: ");
  oled.println(bme.temperature);
  
  oled.print("Humedad: ");
  oled.println(bme.humidity);

  oled.print("Presion: ");
  oled.println(bme.pressure);
  
  oled.display();
  delay(200);
  //bme.temperature;
  }
  
}

void bmeData(){
    if (bme.run()) { // If new data is available
    output += ", " + String(bme.rawTemperature);output += ", " + String(bme.pressure);
    output += ", " + String(bme.rawHumidity);output += ", " + String(bme.gasResistance);
    output += ", " + String(bme.iaq);output += ", " + String(bme.iaqAccuracy);
    output += ", " + String(bme.temperature);output += ", " + String(bme.humidity);
    output += ", " + String(bme.staticIaq);output += ", " + String(bme.co2Equivalent);
    output += ", " + String(bme.breathVocEquivalent); Serial.println(output);     
  } else {
      checkIaqSensorStatus();
  }
  }

/* Esta funcion nos muestra el estado
del sensor BME680*/
void checkIaqSensorStatus(void)
{
  if (bme.status != BSEC_OK) {
    if (bme.status < BSEC_OK) {
      output = "BSEC error code : " + String(bme.status);
      Serial.println(output);
      for (;;)
        errLeds(); /* Halt in case of failure */
    } else {
      output = "BSEC warning code : " + String(bme.status);
      Serial.println(output);
    }
  }

  if (bme.bme680Status != BME680_OK) {
    if (bme.bme680Status < BME680_OK) {
      output = "BME680 error code : " + String(bme.bme680Status);
      Serial.println(output);
      for (;;)
        errLeds(); /* Halt in case of failure */
    } else {
      output = "BME680 warning code : " + String(bme.bme680Status);
      Serial.println(output);
    }
  }
}

/* Esta funcion nos muestra posibles 
errores en el sensor BME680 NO MODIFICAR,
CONFIGURADA PARA IoHT*/

void errLeds(void)
{
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
  delay(100);
  digitalWrite(2, LOW);
  delay(100);
}

void bmeSetup(){
  
   output = "\n Versión de la libreria BSEC" + String(bme.version.major) + "." + String(bme.version.minor) + "." + String(bme.version.major_bugfix) + "." + String(bme.version.minor_bugfix);
   Serial.println(output); // Informacion del sensor y la libreria
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

  bme.updateSubscription(sensorList, 10, BSEC_SAMPLE_RATE_LP);
  checkIaqSensorStatus();

  // Print the header
  output = "Timestamp [ms], raw temperature [°C], pressure [hPa], raw relative humidity [%], gas [Ohm], IAQ, IAQ accuracy, temperature [°C], relative humidity [%], Static IAQ, CO2 equivalent, breath VOC equivalent";
  Serial.println(output);

}
