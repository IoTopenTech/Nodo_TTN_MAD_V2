Este ejemplo puede funcionar en dos modos diferentes:
* Modo Heartbeat: Envía cada 15' la tensión de la batería 
* Modo Tracking: Envía cada 5' la tensión de la batería, la posición GPS y la MAC de los 2 puntos de acceso con mejor RSSI. La adquisición de las direcciones MAC permitiría obtener una geolocalización aproximada, incluso en interiores o zonas sin cobertura GPS, mediante el uso de servicios como http://api.mylnikov.org/geolocation/wifi?v=1.1&data=open&bssid={wifi-bssid}

![Identificación de elementos](https://github.com/IoTopenTech/Nodo_TTN_MAD_V2_2/blob/master/rsc/TTN_MAD_V2_2_dondeestamimochila.jpg)

![Nodo completo](https://github.com/IoTopenTech/Nodo_TTN_MAD_V2_2/blob/master/rsc/TTN_MAD_V2_2_dondeestamimochila_completo.jpg)

La carga de pago usa el estándar Cayenne LPP. Como no existe un modelo de datos para direcciones MAC en LPP, se codifican los 6 bytes de cada MAC en 3 paquetes de 2 bytes, que se envían cada uno como datos de luminosidad.

![Integración con MyDevices](https://github.com/IoTopenTech/Nodo_TTN_MAD_V2_2/blob/master/rsc/TTN_MAD_V2_2_dondeestamimochila_mydevices.jpg)

Inicialmente arranca en el modo Heartbeat, y se puede intercambiar entre uno y otro modo usando cualquiera de estos dos métodos:
* Usando el botón: Con una triple pulsación del botón en menos de 2'' pero con más de 500ms de separación entre pulsación y pulsación. El botón está conectado a los bornes SW y GND del terminal de tornillos verde. El botón elegido es el mismo que para la extensión WiFi (SMD 3*6*2,5).
* Mediante downlink: Se recomienda usar downlink con ACK para asegurarse de que es efectivamente recibido por el nodo. Un downlink con la carga de pago 02 00 64 FF activará el modo Tracking, y usando la carga de pago 02 00 00 FF se activará el modo Heartbeat.

En modo Tracking el LED del nodo parpadea al menos cada 8 segundos en la fase dormida, y cada segundo aproximadamente si está en las fases de adquisición de GPS o WiFi.

Requiere las extensiones WIFI, BOOSTER y GPS, y adicionalmente un botón SMD 3*6*2,5 como el de la extensión WIFI..

Puede configurarse a través del portal cautivo WiFi para unirse a la red por OTAA o ABP.
Sin embargo, en modo ABP no almacena de forma no volátil los contadores, por lo que ante un fallo de alimentación se reiniciarían; en caso de configurarlo en modo ABP se recomienda deshabilitar la comprobación de contador en la consola de The Things Network.

Requiere las siguientes librerías:
* MCCI LoRaWAN LMIC Library by IBM, Matthis Kooijman, Terry Moore, ChaeHee Won, Frank Rose. Versión 2.3.2.
* Low-Power by Rocket Scream Electronics. Versión 1.6.0.
* NeoGPS by SlashDevin 4.2.9

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
Se ha probado con un GPS G28U7FTTL.
Cuando se usa el transmisor LoRa se deshabilita el GPS porque el conversor de tensión no es capaz de suministrar corriente suficiente para ambos. Tampoco se utilizan simultáneamente el GPS y el ESP03 por el mismo motivo.
El GPS está conectado al puerto Serial, por lo que el archivo GPSport.h de la librería NeoGPS debe modificarse para que su contenido sea el siguiente:
```
#ifndef GPSport_h
#define GPSport_h

#define gpsPort Serial
#define GPS_PORT_NAME "Serial"
#define DEBUG_PORT Serial

#endif
```
La tensión de la batería se mide a través del pin A1 intercalando una resistencia de 1M entre el borne 3v3 del terminal de tornillos y el orificio A1 de la sub-placa de expansión.

Se ha incluido también una resistencia de 1M en configuración puldown entre el pin PIR de la sub-placa de expansión y el ping GND de la sub-placa de expansión para que el GPS arranque inicialmente deshabilitado.

Para entrar en el modo de configuración por WiFi hay que mantener pulsado el botón del pcb (no el de cambio entre los modos Heartbeat y Tracking) durante el arranque del nodo.

Cuando el ESP03 arranca empieza a mandar por el puerto serie "IoT" cada segundo, y espera recibir un "1" para entrar en el modo de configuración por WiFi, o un "2" para entrar en el modo de adquisición de las direcciones MAC de los 2 puntos de acceso con mejor RSSI.

El modelo 3D de la caja está diseñado con TinkerCAD y disponible en  https://www.tinkercad.com/things/hX5CWUOLzT4 

El asa se sujeta mediante 2 tuercas M2 y dos tornillos M2x5mm.
El soporte para el botón conviene fijarlo con pegamento termofusible.

![Piezas 3D](https://github.com/IoTopenTech/Nodo_TTN_MAD_V2_2/blob/master/rsc/TTN_MAD_V2_2_dondeestamimochila_3d.jpg)

