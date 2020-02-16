Ejemplo para la configuración con sensor de efecto Hall, BME280 (temperatura, presión y humedad), envío de heartbeat periódico, y activación/desactivación del LED mediante downlink.
El nodo se activa por ABP.
Envía la tensión de la batería, las lecturas del BME280, y el estado de la puerta cada vez que éste cambia, durmiendo entre cada envío por un máximo de 1 minuto, que es el tiempo predeterminado para el heartbeat.
Con cada heartbeat se envía la misma carga de pago (tensión de la batería, lecturas del BME280 y estado de la puerta).

Si tiene problemas para compilarlo por un error de tipo "segmentation fault", reduzca la versión de las placas Arduino a la versión 1.6.21.

Requiere las siguientes librerías:
* MCCI LoRaWAN LMIC Library by IBM, Matthis Kooijman, Terry Moore, ChaeHee Won, Frank Rose. Versión 2.3.2.
* Low-Power by Rocket Scream Electronics. Versión 1.6.0.
* CayenneLPP by Electronics Cats. Versión 1.0.1
* Adafruit Unified Sensor by Adafruit. Version 1.0.3.
* Adafuir BME280 Library by Adafruit. Version 1.0.8.
   * En el archivo Adafruit_BME280.h hay que cambiar la siguiente línea
   ```
   #define BME280_ADDRESS                (0x76)
   ```
   
Para reducir los requisitos de memoria FLASH de la librería MCCI se debe modificar su archivo MCCI_LoRaWAN_LMIC_library\project_config\lmic_project_config.h del siguiente modo:
```
// project-specific definitions
#define CFG_eu868 1
//#define CFG_us915 1
//#define CFG_au921 1
//#define CFG_as923 1
// #define LMIC_COUNTRY_CODE LMIC_COUNTRY_CODE_JP	/* for as923-JP */
//#define CFG_in866 1
#define CFG_sx1276_radio 1
//#define LMIC_USE_INTERRUPTS
#define DISABLE_PING
#define DISABLE_BEACONS
```
