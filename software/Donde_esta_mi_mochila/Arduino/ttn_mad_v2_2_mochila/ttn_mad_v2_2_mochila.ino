/*
   El nodo enviará un heartbeat cada 15' con el estado de la batería
   Si el usuario pulsa el botón 3 veces en menos de 2'', 
   dejando una separación de más de 500ms entre pulsación y pulsación, 
   se activará el modo tracking
   Con otra triple pulsación se desactivará
   Utilizo 2 ciclos de sueño alternos, uno de 500ms y otro de 8s para poder detectar
   si la triple pulsación en caso de sueño es rápida
   El modo tracking también se puede activar/desactivar remotamente con un downlink (con ACK)
   En el modo tracking el nodo enviará cada 5':
    El estado de la batería
    Las 2 MAC de AP WiFi con mejor RSSI disfrazadas como datos de luminosidad en cayenne lpp
    La posición GPS si consigue hacer FIX (intentará hacer FIX durante un máximo de 2')
*/
#define  BOTON_SOS  3
#define  BOTON_CONF 8
#define  LED        4
#define  EN_ESP03   A0
#define  EN_GPS     A3
#define  BATT       A1

#define  CICLOS_HEARTBEAT 105 //aprox 15'
#define  CICLOS_TRACKING 35 //aprox 5'
int ciclo = 0;

#include <lmic.h>
#include <hal/hal.h>

#include <SPI.h>
#include <EEPROM.h>
#include "LowPower.h"

#include <CayenneLPP.h>
CayenneLPP lpp(45);//Sólo voy a pasar la tensión [4], la entrada del botón[3], el GPS [14] y las MAC[2x12]

#include <NMEAGPS.h>
#define gpsPort Serial
NMEAGPS  gps; // This parses the GPS characters
gps_fix  fix; // This holds on to the latest values
float latitud, longitud, altitud;

unsigned long arranqueCrono;
volatile unsigned long cronoPulsacionAnterior = 0;
volatile unsigned long cicloPulsacionAnterior = 0;
volatile byte pulsacionesConsecutivas = 0;
volatile bool pulsacionRecienIncrementada = false;
volatile unsigned long pulsaciones = 0;
volatile bool envioEnCurso = false;
bool modoTracking = false;
bool GPSarrancado = false;
bool ESP03arrancado = false;
bool esperarUnPoco = false;

bool otaa_abp;


unsigned long DEVADDR = 0;
byte NWKSKEY[16] ;
byte APPSKEY[16];

void os_getDevEui (u1_t* buf) {
  for (int i = 0; i < 8; i++) {
    buf[7 - i] = NWKSKEY[i];
  }
}
void os_getArtEui (u1_t* buf) {
  for (int i = 0; i < 8; i++) {
    buf[7 - i] = NWKSKEY[8 + i];
  }
}
void os_getDevKey (u1_t* buf) {
  for (int i = 0; i < 16; i++) {
    buf[i] = APPSKEY[i];
  }
}

static osjob_t sendjob;

// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 9,
  .dio = {2, 7, LMIC_UNUSED_PIN},
};

void onEvent (ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev) {
    /*
      case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
      case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
      case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
      case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;

      case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    */
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      {
        u4_t netid = 0;
        devaddr_t devaddr = 0;
        u1_t nwkKey[16];
        u1_t artKey[16];
        LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
        /*
          Serial.print("netid: ");
          Serial.println(netid, DEC);
          Serial.print("devaddr: ");
          Serial.println(devaddr, HEX);
          Serial.print("artKey: ");
          for (int i = 0; i < sizeof(artKey); ++i) {
          Serial.print(artKey[i], HEX);
          }
          Serial.println("");
          Serial.print("nwkKey: ");
          for (int i = 0; i < sizeof(nwkKey); ++i) {
          Serial.print(nwkKey[i], HEX);
          }
          Serial.println("");
        */
      }
      // Disable link check validation (automatically enabled
      // during join, but because slow data rates change max TX
      // size, we don't use it in this example.
      LMIC_setLinkCheckMode(0);
      break;
    /*
      || This event is defined but not used in the code. No
      || point in wasting codespace on it.
      ||
      || case EV_RFU1:
      ||     Serial.println(F("EV_RFU1"));
      ||     break;
    */
    /*
      case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
      case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    */
    case EV_TXCOMPLETE:
      envioEnCurso = false;
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));

      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));
      if (LMIC.dataLen) {
        Serial.println(F("Downlink recibido"));
        //El downlink tendrá el formato canal (02), tipo de valor (00=digital), valor (0 o 100), FF

        if (LMIC.frame[LMIC.dataBeg + 2] == 100) {
          modoTracking = true;
          ciclo = CICLOS_TRACKING;
          //digitalWrite(LED, HIGH);
        } else {
          modoTracking = false;
          //digitalWrite(LED, LOW);
        }
        esperarUnPoco = true;
        arranqueCrono = millis();
      }

      // Schedule next transmission
      //os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);

      break;
    /*
      case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
      case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    */
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    /*
      case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
      case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    */
    /*
      || This event is defined but not used in the code. No
      || point in wasting codespace on it.
      ||
      || case EV_SCAN_FOUND:
      ||    Serial.println(F("EV_SCAN_FOUND"));
      ||    break;
    */

    case EV_TXSTART:
      Serial.println(F("EV_TXSTART"));
      break;
    default:
      Serial.print(F("Unknown event: "));
      Serial.println((unsigned) ev);
      break;
  }
}

void do_send(osjob_t* j) {
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    envioEnCurso = true;
    lpp.addAnalogInput(1, analogRead(BATT) * 3.3 / 1000.F);
    lpp.addDigitalOutput(2, 0);//Canal para habilitar el modo de tracking a distancia mediante un downlink

    LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);

    Serial.println(F("Packet queued"));

  }
}
void setup() {
  Serial.begin(9600);
  pinMode(BOTON_SOS, INPUT_PULLUP);//BOTON SOS
  pinMode(BOTON_CONF, INPUT_PULLUP);//BOTON INTERNO
  pinMode(LED, OUTPUT);//LED
  //pinMode(5, OUTPUT);//BUZZER
  //pinMode(6, INPUT);//External
  pinMode(EN_ESP03, OUTPUT);//EN_ESP03
  digitalWrite(EN_ESP03, LOW);
  //pinMode(A1, INPUT);//EXTERNAL
  //pinMode(A2, INPUT);//INT1
  pinMode(EN_GPS, OUTPUT);//Enable GPS
  digitalWrite(EN_GPS, LOW);

  //Avisos de encendido

  digitalWrite(LED, HIGH);
  delay(1000);
  digitalWrite(LED, LOW);
  byte entradaSerial[16];
  // ¿Entrar en el modo de configuración?

  if ( digitalRead(BOTON_CONF) == LOW) {
    delay(500);
    if ( digitalRead(BOTON_CONF) == LOW) {
      //Entrar en modo de programación
      digitalWrite(EN_ESP03, HIGH);
      delay(1000);//Espero a que arranque el esp03

      digitalWrite(LED, HIGH);

      bool configuracionTerminada = false;
      while (!configuracionTerminada) {
        if (Serial.available()) {
          delay(100);
          Serial.readBytes(entradaSerial, 16);
          if (entradaSerial[0] == 'I' && entradaSerial[1] == 'o' && entradaSerial[2] == 'T') {
            Serial.write("1");//Entrar en modo configuración
          } else if (entradaSerial[0] == 'A' && entradaSerial[1] != 'B') {
            //No hago nada
          } else if (entradaSerial[0] == 'O' && entradaSerial[1] == 'T' && entradaSerial[2] == 'A' && entradaSerial[3] == 'A') {
            //digitalWrite(6, LOW);
            int i = 0;
            for (int j = 0 ; j < 4 ; i++, j++) {
              EEPROM.write( i, entradaSerial[j] );
            }
            while (!Serial.available()) {}
            delay(100);
            Serial.readBytes(entradaSerial, 8);
            for (int j = 0 ; j < 8 ; i++, j++) {
              EEPROM.write( i, entradaSerial[j] );
            }
            while (!Serial.available()) {}
            delay(100);
            Serial.readBytes(entradaSerial, 8);
            for (int j = 0 ; j < 8 ; i++, j++) {
              EEPROM.write( i, entradaSerial[j] );
            }
            while (!Serial.available()) {}
            delay(100);
            Serial.readBytes(entradaSerial, 16);
            for (int j = 0 ; j < 16 ; i++, j++) {
              EEPROM.write( i, entradaSerial[j] );
            }
            Serial.write("B");
            digitalWrite(LED, LOW);
            delay(5000);
            digitalWrite(EN_ESP03, LOW);
            configuracionTerminada = true;
          } else if (entradaSerial[0] == 'A' && entradaSerial[1] == 'B' && entradaSerial[2] == 'P') {
            int i = 0;
            for (int j = 0 ; j < 3 ; i++, j++) {
              EEPROM.write( i, entradaSerial[j] );
            }

            while (!Serial.available()) { }
            delay(100);
            Serial.readBytes(entradaSerial, 4);
            for (int j = 0 ; j < 4 ; i++, j++) {
              EEPROM.write( i, entradaSerial[j] );
            }

            while (!Serial.available()) {}
            delay(100);
            Serial.readBytes(entradaSerial, 16);
            for (int j = 0 ; j < 16 ; i++, j++) {
              EEPROM.write( i, entradaSerial[j] );
            }

            while (!Serial.available()) {
            }
            delay(100);
            Serial.readBytes(entradaSerial, 16);
            for (int j = 0 ; j < 16 ; i++, j++) {
              EEPROM.write( i, entradaSerial[j] );
            }
            Serial.write("B");
            digitalWrite(LED, LOW);

            delay(5000);
            digitalWrite(EN_ESP03, LOW);

            delay(100);

            configuracionTerminada = true;
          }
        }
      }

    }
  }

  //Arranque normal (sin entrar en modo configuración)
  //Leer la configuración de la eeprom
  if (EEPROM.read(0) == 'O' &&  EEPROM.read(1) == 'T' && EEPROM.read(2) == 'A' && EEPROM.read(3) == 'A' ) {
    otaa_abp = true;
    //Serial.println("OTAA");
    EEPROM.get(4, entradaSerial);
    for (int j = 7; j >= 0; j--) {
      NWKSKEY[j] = entradaSerial[j];
    }
    EEPROM.get(12, entradaSerial);
    for (int j = 7; j >= 0; j--) {
      NWKSKEY[8 + j] = entradaSerial[j];
    }
    EEPROM.get(20, entradaSerial);
    for (int j = 0; j < 16; j++) {
      APPSKEY[j] = entradaSerial[j ];
    }
  } else if (EEPROM.read(0) == 'A' &&  EEPROM.read(1) == 'B' && EEPROM.read(2) == 'P') {
    otaa_abp = false;
    //Serial.println("ABP");
    EEPROM.get(3, entradaSerial);
    for (int j = 0; j < 4; j++) {
      DEVADDR *= 256;
      DEVADDR += entradaSerial[j];
    }

    EEPROM.get(7, entradaSerial);
    //Serial.println("NWKSKEY");
    for (int j = 0; j < 16; j++) {
      NWKSKEY[j] = entradaSerial[j ];

    }
    //Serial.println("APPSKEY");
    EEPROM.get(23, entradaSerial);
    for (int j = 0; j < 16; j++) {
      APPSKEY[j] = entradaSerial[j];

    }
  } else {
    //El nodo no está configurado
    while (true) {
      digitalWrite(LED, HIGH);
      delay(200);
      digitalWrite(LED, LOW);
      delay(200);
    }
  }


  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();
  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
  if (otaa_abp == false) {
    LMIC_setSession (0x13, DEVADDR, NWKSKEY, APPSKEY);




#if defined(CFG_eu868)
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.
#elif defined(CFG_us915)
    // NA-US channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    LMIC_selectSubBand(1);
#endif

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7, 14);
  }
  // Start job


  attachInterrupt(digitalPinToInterrupt(3), toggleTracking, FALLING);

  do_send(&sendjob);

}

void toggleTracking() {
  //No puedo usar millis porque si le despierto de un sleep apenas habrá avanzado
  //Si la diferencia de tiempo respecto a la pulsación anterior es mayor que 750, reinicio el contador
  //Si es menor que 750 pero mayor que 100 (tiempo antirrebote)y no viene de estar dormido dormido avanzo el contador
  //si es menor que 25 o viene de estar dormido (ciclo!=0), me fijo en el ciclo: si es el ciclo consecutivo, avanzo el contador, si no, lo reinicio
  //En este último caso no se me ha ocurrido como implementar el antirrebote
  pulsaciones++;
  unsigned long cronoActual = millis();
  if (cronoActual - cronoPulsacionAnterior > 1000L) {
    cronoPulsacionAnterior = cronoActual;
    cicloPulsacionAnterior = ciclo;
    pulsacionesConsecutivas = 1;
  } else if (ciclo == 0) {
    if (cronoActual - cronoPulsacionAnterior > 500L) {
      cronoPulsacionAnterior = cronoActual;
      cicloPulsacionAnterior = ciclo;
      pulsacionesConsecutivas++;
    } else {
      //rebote
      return;
    }
  } else {
    pulsacionRecienIncrementada = true;
    if (ciclo == cicloPulsacionAnterior ) {
      cronoPulsacionAnterior = cronoActual;
      cicloPulsacionAnterior = ciclo;
      pulsacionesConsecutivas++;
    } else {
      cronoPulsacionAnterior = cronoActual;
      cicloPulsacionAnterior = ciclo;
      pulsacionesConsecutivas = 1;
    }
  }

  if (pulsacionesConsecutivas == 3) {
    pulsacionesConsecutivas = 0;
    if (modoTracking) {
      modoTracking = false;
      ciclo = 0;
      digitalWrite(LED, LOW);
      digitalWrite(EN_ESP03, LOW);
      digitalWrite(EN_GPS, LOW);
    } else {
      modoTracking = true;
      ciclo = CICLOS_TRACKING;
      digitalWrite(LED, HIGH);
    }
    return;
  }
}

void loop() {
  os_runloop_once();
  /*
    Serial.print(pulsaciones);
    Serial.print("-");
    Serial.print(pulsacionesConsecutivas);
    Serial.print("-");
    Serial.print(cronoPulsacionAnterior);
    Serial.print("-");
    Serial.print(cicloPulsacionAnterior);
    Serial.print("-");
    Serial.print(ciclo);
    Serial.println();
    Serial.flush();
  */
  if (esperarUnPoco) {
    //He metido esto porque parece que LMIC contesta inmediatamente
    //el ack de un downlink, en lugar de ponerlo como pigpack en el uplink siguiente
    //Por lo que no puedo dormirme inmediatamente
    //Quizás actúe así porque no tengo programado el envío de otro paquete
    if (millis() - arranqueCrono > 20 * 1000L) {
      //Serial.println("FIN DE LA ESPERA");
      esperarUnPoco = false;
    }
  } else if ((LMIC.opmode & OP_TXRXPEND) || envioEnCurso) {
    //No hago nada
  } else {
    if (modoTracking) {
      digitalWrite(LED, HIGH);
      delay(50);
      digitalWrite(LED, LOW);
      if (ciclo == CICLOS_TRACKING) {
        //Aranco el GPS y espero por un fix del GPS un máximo de 2 minutos
        //Una vez logrado el FIX o superado el plazo anterior,
        //apago el GPS, enciendo el ESP03, obtengo las MACs y envío el paquete
        arranqueCrono = millis();
        GPSarrancado = true;
        digitalWrite(EN_GPS, HIGH);
        ciclo = 0;
        //Serial.print("CICLO: ");
        //Serial.println(ciclo);
      }
      if (!GPSarrancado && !ESP03arrancado) {
        ciclo++;
        //Serial.print("CICLO: ");
        //Serial.println(ciclo);
        if (!pulsacionRecienIncrementada) {
          LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
        } else {
          pulsacionRecienIncrementada = false;
          LowPower.powerDown(SLEEP_500MS, ADC_OFF, BOD_OFF);
          
            ciclo--;
          
          LowPower.powerDown(SLEEP_500MS, ADC_OFF, BOD_OFF);
        }
        //unsigned long crono=millis();
        //while (millis()-crono < 8*1000){}
      } else if (GPSarrancado) {
        //Serial.print("GPS: ");
        //Serial.print(millis() - arranqueCrono);
        //Serial.println("");
        delay(500);
        bool FIXconseguido = false;
        while (gps.available(Serial)) {
          fix = gps.read();
          if (fix.valid.location ) {
            latitud = (float)fix.latitude();
            longitud = (float)fix.longitude();
            altitud = (float)fix.altitude();
            FIXconseguido = true;
            break;
          }
        }
        if (FIXconseguido ||  ((millis() - arranqueCrono) > 2 * 60 * 1000L)) {
          lpp.reset();
          if (FIXconseguido) {
            lpp.addGPS(3, latitud, longitud, altitud);
          }
          GPSarrancado = false;
          ESP03arrancado = true;
          digitalWrite(EN_GPS, LOW);
          delay(1000);
          digitalWrite(EN_ESP03, HIGH);
          delay(1000);
          arranqueCrono = millis();
        }
      } else { //ESP03arrancado=true
        //Serial.print("ESP03 ARRANCADO");
        //Serial.println("");
        delay(500);
        bool MACconseguida = false;
        byte entradaSerial[7];
        while (Serial.available()) {
          delay(100);
          Serial.readBytes(entradaSerial, 7);
          //El ESP03 está constantemente enviando "IoT" y esperando 1 segundo hasta que reciba un modo (1 o 2)
          if (entradaSerial[0] == 'I' && entradaSerial[1] == 'o' && entradaSerial[2] == 'T') {

            Serial.write("2");//Entrar en modo SCAN
            Serial.println("");
            //Vacío todo el buffer de entrada Serial
            while (Serial.available() > 0) {
              char t = Serial.read();
            }
            //Serial.println("CONTESTANDO");
            //Serial.println("");

            break;
          } else if (entradaSerial[6] == '#') {
            lpp.addLuminosity(4, ((entradaSerial[0] << 8) + (entradaSerial[1])));
            lpp.addLuminosity(5, ((entradaSerial[2] << 8) + (entradaSerial[3])));
            lpp.addLuminosity(6, ((entradaSerial[4] << 8) + (entradaSerial[5])));
            //Necesito al menos 2 MAC para poder acceder a los servicios de geolocalización
            Serial.readBytes(entradaSerial, 7);
            if (entradaSerial[6] == '#') {
              lpp.addLuminosity(7, ((entradaSerial[0] << 8) + (entradaSerial[1])));
              lpp.addLuminosity(8, ((entradaSerial[2] << 8) + (entradaSerial[3])));
              lpp.addLuminosity(9, ((entradaSerial[4] << 8) + (entradaSerial[5])));
              MACconseguida = true;

              //Serial.println("");
              //Serial.println("MAC conseguida");
              //Serial.println("");
              break;
            }
          }
        }
        if (MACconseguida || (millis() - arranqueCrono > 20 * 1000L)) {
          //Salgo
          digitalWrite(EN_ESP03, LOW);
          ESP03arrancado = false;
          delay(2000);
          //Serial.println("");
          //Serial.println("ENVIANDO GPS");
          //Serial.println("");
          do_send(&sendjob);
        }
      }
    } else {
      if (ciclo == CICLOS_HEARTBEAT) {
        ciclo = 0;
        //Serial.print("CICLO: ");
        //Serial.println(ciclo);
        lpp.reset();
        //Serial.println("");
        //Serial.println("HEARTBEAT");
        //Serial.println("");
        do_send(&sendjob);
      } else {
        ciclo++;
        //Serial.print("CICLO: ");
        //Serial.println(ciclo);
        //unsigned long crono=millis();
        //while (millis()-crono < 8*1000){}
        if (!pulsacionRecienIncrementada) {
          LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
        } else {
          pulsacionRecienIncrementada = false;

          LowPower.powerDown(SLEEP_500MS, ADC_OFF, BOD_OFF);
          
            ciclo--;
          
          LowPower.powerDown(SLEEP_500MS, ADC_OFF, BOD_OFF);
        }
      }
    }
  }

}
