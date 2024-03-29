# IoHT (Internet of Home Things)

En este repositorio encontrara información acerca del uso de la tarjeta diseñada por Ángel Isidro desarrollador de dispositivos IoT de Tesla Lab, tambien encontrara algunas librerias y ejemplos desarrollados por Gabriel Monzón desarrollador de software de Tesla Lab.

![](/img/ioht.jpeg)
![](/img/logos.png)

## Dispositivos en la tarjeta
- ESP32
- Neopixel
- BME680
- Pantalla OLED 128x32
- Buzzer
- Sensor de luz 
- Tactile Push Buttons
- Pines GPIO

## Instalación

Para la instalación en Arduino IDE debemos seguir los siguientes pasos:

1. Abrir ARDUINO IDE
2. Debemos ir a la pestaña ***Archivo***->***Preferencias***.
	- En la pestaña ***Ajustes*** buscamos la opción: ***Gestor de URLs Adicionales de Tarjetas:***
	- Pegamos la siguiente URL: `https://dl.espressif.com/dl/package_esp32_index.json`
	- Luego precionamos la opción ***OK*** y automaticamente se cerrara la ventana.
3. Debemos ir a la pestaña ***Herramientas***->***Placa***->***Gestor de tarjetas***.
	- Colocamos en la barra de busqueda ***ESP32***.
	- Seleccionamos la opción que nos muestre ***esp32***.
	- Instalamos y luego precionamos la opción de ***Cerrar***.

Con esto tendremos completa la instalación de nuestra tarjeta ESP32 y lista para ser programada.	 

## Instalación USB Driver ***(Si no reconoce el puerto COM)***

En algunos casos nuestra PC no es capaz de reconocer el dispositivo USB que nosotros conectamos, es por ello que dejaremos un archivo para la instalación del driver con ello nuestra PC deberá ser capaz de reconocer el dispositivo conectado, en caso de que el problema persista asegúrese de que su dispositivo no se este sobre calentando.

[Descarga este driver para Windows][DRIVER_USB]

[DRIVER_USB]: https://drive.google.com/file/d/1yuZ91mJfR12bhASqD9ffsebzsEOJz8Om/view?usp=sharing

1. Descomprimir el archivo ZIP descargado 
2. Seleccionar el archivo dependiendo de nuestra instalación de Windows
	- Para 32 Bits `CP210xVCPInstaller_x86`
	- Para 64 Bits `CP210xVCPInstaller_x64`
3. Brindar permisos de Administrador	
4. Click en la opción ***Siguiente***
5. Click en la opción ***Finalizar***

Con esto estaría lista la instalación del Driver USB para nuestra tarjeta en el Sistema Operativo Windows.

# Mapeo de Pines

### ESP32 - Touch Sensor
Touch | GPIO | *** | Touch | GPIO
--- | --- | --- | --- | ---
UP | GPIO4 | | RIGHT | GPIO15 
DOWN | GPIO32 | | CANCEL | GPIO12
LEFT | GPIO2 | | ENTER | GPIO13

### IoHT - Dispositivos
Dispositivo | GPIO | GPIO | Cantidad 
--- | --- | --- | --- 
NeoPixel | GPIO14 | | 6 
Sensor de Luz | GPIO34 | | 1 
Buzzer | GPIO27 | | 1 
BME680 | SDA | SCL | 1 
Relay | GPIO35 | | 1 
OLED | SDA | SCL | 1 

## Protocolo de comunicación I2C
Nombre | GPIO
--- | ---
SDA | GPIO21
SCL | GPIO22

