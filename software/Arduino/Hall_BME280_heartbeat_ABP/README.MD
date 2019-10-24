Ejemplo para la configuración con sensor de efecto Hall y envío de heartbeat periódico.
Se activa por ABP.
Envía la tensión de la batería y el estado de la puerta cada vez que éste cambia, durmiendo entre cada envío.

Requiere las siguientes librerías:
* MCCI LoRaWAN LMIC Library by IBM, Matthis Kooijman, Terry Moore, ChaeHee Won, Frank Rose. Versión 2.3.2.
* Low-Power by Rocket Scream Electronics. Versión 1.6.0.
* CayenneLPP by Electronics Cats. Versión 1.0.1
   
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
