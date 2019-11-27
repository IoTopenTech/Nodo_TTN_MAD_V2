En este ejemplo utilizamos un GPS.
El nodo envía un heartbeat cada 24 segundos con la tensión de la batería.
Cada 12 ciclos de 24 segundos, incluye en el heartbeat la posición GPS (si consigue hacer un fix antes de 5 minutos).
El usuario puede enviar un downlink con la carga de pago 02 00 64 FF para activar el modo de seguimiento; en este modo el GPS se habilita y se envía un paquete cada minuto.
Enviando un downlink con la carga de pago 02 00 00 FF se desactiva el modo de seguimiento.
Se ha probado con un GPS VK2828U7G5LF.
El GPS está conectado al puerto Serial, por lo que el archivo GPSport.h de la librería NeoGPS debe modificarse para que su contenido sea el siguiente:
#ifndef GPSport_h
#define GPSport_h

#define gpsPort Serial
#define GPS_PORT_NAME "Serial"
#define DEBUG_PORT Serial

#endif
