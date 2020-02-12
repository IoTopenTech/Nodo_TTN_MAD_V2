En este ejemplo utilizamos un GPS para enviar información a ttnmapper.

Este ejemplo está dirigido a usuarios que sólo disponen de un gateway Single Channel.

El nodo envía la información siempre por el canal 868.1 y en SF7.

Adicionalmente puede conectarse el pin de entrada de la batería al pin A1 mediante una resistencia de 1M para poder medir la tensión, y pulsando el botón (conectado al pin 8) se fuerza el envío de un paquete.

Se ha probado con un GPS VK2828U7G5LF y un GN-701.

Si utiliza el model GN-701 tendrá que retirar una resistencia para habilitar el pin EN (más información en el Wiki).

Cuando se usa el transmisor LoRa se deshabilita el GPS porque el conversor de tensión no es capaz de suministrar corriente suficiente para ambos.

El GPS está conectado al puerto Serial, por lo que el archivo GPSport.h de la librería NeoGPS debe modificarse para que su contenido sea el siguiente:
```
#ifndef GPSport_h
#define GPSport_h

#define gpsPort Serial
#define GPS_PORT_NAME "Serial"
#define DEBUG_PORT Serial

#endif
```
Además, en la librería NeoGPS, debe habilitarse la extracción del valor hdop (horizontal dilution of precision) descomentando la siguiente línea del archivo GPSfix_cfg.h:
```
#define GPS_FIX_HDOP
```
Por último, en la consola de The Thnings Network, debe utilizarse el siguiente decoder en la sección Payload formats:
```
function Decoder(bytes, port) {
    var decoded = {};
    //JavaScript Numbers are Always 64-bit Floating Point
    decoded.latitude = ((((bytes[0]<<(24+32))>>>0) + ((bytes[1]<<(16+32))>>>0) + ((bytes[2]<<(8+32))>>>0) + ((bytes[3]<<(32))>>>0))>>32)/10000000;
    decoded.longitude = ((((bytes[4]<<(24+32))>>>0) + ((bytes[5]<<(16+32))>>>0) + ((bytes[6]<<(8+32))>>>0) + ((bytes[7]<<(32))>>>0))>>32)/10000000;
    decoded.altitude = (((bytes[8]<<24)>>>0) + ((bytes[9]<<16)>>>0) + ((bytes[10]<<8)>>>0) + bytes[11]);
    decoded.hdop=(((bytes[12]<<8)>>>0) + bytes[13])/1000;
    decoded.bateria=(((bytes[14]<<8)>>>0) + bytes[15])*3.3/1023.0;
    return decoded;
}
```
