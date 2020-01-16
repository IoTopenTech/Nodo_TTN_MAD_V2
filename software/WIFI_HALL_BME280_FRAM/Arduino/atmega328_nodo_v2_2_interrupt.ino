#include <lmic.h>
#include <hal/hal.h>

#include <SPI.h>
#include <EEPROM.h>
#include "LowPower.h"

//#include <CayenneLPP.h>
//CayenneLPP lpp(18);//Sólo voy a pasar la tensión y el GPS
#include <Wire.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
//En el archivo Adafruit_BME280.h hay que cambiar la siguiente línea
// #define BME280_ADDRESS                (0x76)

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C

/*
  //#include <NMEAGPS.h>
  #define gpsPort Serial


  NMEAGPS  gps; // This parses the GPS characters
  gps_fix  fix; // This holds on to the latest values
*/
//float latitud, longitud, altitud;
unsigned long ultimo_upload;

bool envioEnCurso = true;//Para forzar la asignación del attachInterrupt

volatile int cambiosDeEstado = 0;
bool habilitar_GPS = false;
bool enviar_GPS = false;
/*
  #include <HDC2010.h>
  #define ADDR 0x40
  HDC2010 sensor(ADDR);
*/


#define FRAMADDRESS   0x50

void FRAMWrite(uint8_t direccion, uint32_t valor) {
  Wire.begin();
  uint8_t dato;
  Wire.beginTransmission(FRAMADDRESS);
  Wire.write(direccion);

  for (int i = 0; i < 4; i++) {
    dato = ((valor) & 0xFF);
    Wire.write(dato);
    valor = valor >> 8;
  }
  Wire.endTransmission();
}

uint32_t FRAMRead(uint8_t direccion) {
  Wire.begin();
  Wire.beginTransmission(FRAMADDRESS);
  Wire.write(direccion);
  Wire.endTransmission();
  Wire.requestFrom(FRAMADDRESS, 4);
  return Wire.read() + ((Wire.read() << 8) & 0xFF00) + ((Wire.read() << 16) & 0xFF0000) + ((Wire.read() << 24) & 0xFF000000);
}

/*
  #include "BMA400.h"
*/




//String entradaSerial = "";

bool otaa_abp; //OTAA=TRUE ABP=FALSE
//byte DEVEUI[8] ;
//byte APPEUI[8] ;
//byte APPKEY[16]; //Uso APPSKEY para ahorrar un poco de memoria
unsigned long DEVADDR = 0;
byte NWKSKEY[16] ;
byte APPSKEY[16];
//unsigned long DEVADDR = 0x26011EB9;
//byte NWKSKEY[16]= { 0x28, 0x47, 0xB0, 0xAD, 0x02, 0xB1, 0x2D, 0xB0, 0x75, 0x54, 0x6E, 0x01, 0x17, 0x37, 0x1D, 0xEC };
//byte APPSKEY[16]={ 0x6C, 0x5C, 0x96, 0xC0, 0x20, 0x2F, 0xCD, 0xCE, 0x03, 0x9E, 0xB2, 0x8B, 0x8F, 0xA2, 0xF9, 0x8B };

long readVcc() {
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2);
  ADCSRA |= _BV(ADSC);
  while (bit_is_set(ADCSRA, ADSC));

  long result = ADCL;
  result |= ADCH << 8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  return result;
}

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
//static uint8_t mydata[] = "Hello, world!";
static osjob_t sendjob;

//const unsigned TX_INTERVAL = 60;

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
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));

      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));
      if (LMIC.dataLen) {
        //El downlink tendrá el formato canal (02), tipo de valor (00=digital), valor (0 o 100), FF
        if (LMIC.frame[LMIC.dataBeg + 2] == 100) {
          digitalWrite(4, HIGH);
          habilitar_GPS = true;
          ultimo_upload = millis();
        } else {
          digitalWrite(4, LOW);
          habilitar_GPS = false;
        }
      }
      // Schedule next transmission
      //os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
      if (otaa_abp == false) {
        // digitalWrite(A3, HIGH);
        //delay(100);
        FRAMWrite(0, LMIC.seqnoUp);
        //if (habilitar_GPS == false) {
        //  digitalWrite(A3, LOW);
        //}
      }

      //Serial.println(LMIC.seqnoUp);
      //envioEnCurso = false;
      //digitalWrite(4, HIGH);
      if (habilitar_GPS == false) {
        Serial.println(F("Me voy a echar una siestecita"));
        delay(1000);// Para que terminen de imprimirse los mensajes en el terminal
        //digitalWrite(A2, HIGH);
        //El hall devuelve 1 en ausencia del imán y 0 en presencia del imán
        if (envioEnCurso == true) {
          //Ha terminado el envío de un paquete correspondiente a un cambio de estado de la puerta
          if (cambiosDeEstado == 0) {
            //No se han producido más cambios de estado desde el que provocó el envío del paquete
            envioEnCurso = false;
          } else {
            envioEnCurso = true;//Forzas un nuevo envío
          }
          //delay(100);
        }

        if (envioEnCurso == false) {
          for (byte contador = 0; contador < 8; contador++) {
            if (envioEnCurso == false) {
              LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
            }
          }
        }
        //Envío el heartbeat
        os_setCallback (&sendjob, do_send);
      } else {
        Serial.println(F("GPS Habilitado"));
      }

      //if (habilitar_GPS == true) {
      //  digitalWrite(A3, HIGH);
      //}


      //
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
    // Prepare upstream data transmission at the next possible time.
    //LMIC_setTxData2(1, mydata, sizeof(mydata) - 1, 0);
    //Añado a la carga de pago el estado de la batería
    /*
      lpp.reset();
      //lpp.addDigitalInput(1, 1);
      lpp.addAnalogInput(2, readVcc() / 1000.0);
      lpp.addDigitalOutput(3, 0);
      if (enviar_GPS == true ) {
      //digitalWrite(A3, LOW);
      lpp.addGPS(4, latitud, longitud, altitud);
      habilitar_GPS = false;
      enviar_GPS = false;
      }
    */
    /*
      lpp.addTemperature(5, bme280.readTempC());
      lpp.addBarometricPressure(6, bme280.readPressure());
      lpp.addRelativeHumidity(7, bme280.readHumidity());
    */
    // Initialize I2C communication
    //sensor.begin();
    /*
        // Begin with a device reset
        sensor.reset();
        // Configure Measurements
        sensor.setMeasurementMode(TEMP_AND_HUMID);  // Set measurements to temperature and humidity
        sensor.setRate(ONE_HZ);                     // Set measurement frequency to 1 Hz
        sensor.setTempRes(FOURTEEN_BIT);
        sensor.setHumidRes(FOURTEEN_BIT);
        //begin measuring
        sensor.triggerMeasurement();
        delay(1000);
        // read temperature and humidity
        float temperature = 0, humidity = 0;
        temperature = sensor.readTemp();
        humidity = sensor.readHumidity();
        lpp.addTemperature(4, temperature);
        lpp.addRelativeHumidity(5, humidity);


          bma400.initialize();
          float x = 0, y = 0, z = 0;
          bma400.getAcceleration(&x, &y, &z);
          lpp.addAccelerometer(6,  x / 1000.0,  y / 1000.0,  z / 1000.0);

    */
    //digitalWrite(A3, LOW);



    unsigned char carga_de_pago[25];

    carga_de_pago[0] = 0x01;
    carga_de_pago[1] = 0x66; //Digital input
    carga_de_pago[2] = digitalRead(3); //Digital input

    carga_de_pago[3] = 0x02;
    carga_de_pago[4] = 0x01; //Digital output
    carga_de_pago[5] = 0x00; //Digital input

    bme.begin();
    bme.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1, // temperature
                    Adafruit_BME280::SAMPLING_X1, // pressure
                    Adafruit_BME280::SAMPLING_X1, // humidity
                    Adafruit_BME280::FILTER_OFF   );
    carga_de_pago[6] = 0x03;
    carga_de_pago[7] = 0x68; //Humidity
    carga_de_pago[8] = (unsigned char)(bme.readHumidity()) * 2;

    carga_de_pago[10] = 0x02; //Batería: Analog input 0.01 Signed

    carga_de_pago[14] = 0x67; //Temperature sensor 0.1 °C Signed MSB

    carga_de_pago[18] = 0x73; //Barometer

    carga_de_pago[22] = 0x02; //Cambios de estado entre dos mensajes

    int help[4];
    help[0] = (int)(readVcc() / 10.F);    // readVCC returns  mVolt need just 10mVolt steps
    help[1] = (int)(bme.readTemperature() * 10.);
    help[2] = (int)(bme.readPressure() / 10.);
    help[3] = cambiosDeEstado;
    cambiosDeEstado = 0;
    for (byte i = 0; i < 4; i++) {
      carga_de_pago[9 + (4 * i)] = 4 + i;
      carga_de_pago[11 + (4 * i)] = help[i] >> 8;
      carga_de_pago[12 + (4 * i)] = help[i];
    }


    LMIC_setTxData2(1, carga_de_pago, 25, 0);

    //LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);

    Serial.println(F("Packet queued"));
  }
  // Next TX is scheduled after TX_COMPLETE event.
}
void enviarMensaje() {
  envioEnCurso = true;
  cambiosDeEstado += 100;
  if (digitalRead(3) == HIGH) {
    attachInterrupt(digitalPinToInterrupt(3), enviarMensaje, FALLING);
  } else {
    attachInterrupt(digitalPinToInterrupt(3), enviarMensaje, RISING);
  }
}
void setup() {
  Serial.begin(9600);
  pinMode(3, INPUT_PULLUP);//SENSOR PUERTA
  pinMode(8, INPUT_PULLUP);//BOTON
  pinMode(4, OUTPUT);//LED
  pinMode(5, OUTPUT);//BUZZER
  pinMode(6, INPUT);//External
  pinMode(A0, OUTPUT);//EN_ESP03
  digitalWrite(A0, LOW);
  pinMode(A1, INPUT);//EXTERNAL
  pinMode(A2, INPUT);//INT1
  pinMode(A3, INPUT);//PIR

  //Wire.begin();

  bool status;
  status = bme.begin();
  if (!status) {
    Serial.println(F("Could not find a valid BME280 sensor, check wiring!"));
    while (1);
  }

  //Avisos de encendido
  tone(5, 4000);
  digitalWrite(4, HIGH);
  delay(1000);
  digitalWrite(4, LOW);
  noTone(5);

  byte entradaSerial[16];
  // ¿Entrar en el modo de configuración?



  if ( digitalRead(8) == LOW) {
    delay(500);
    if ( digitalRead(8) == LOW) {
      //Entrar en modo de programación
      digitalWrite(A0, HIGH);
      delay(1000);//Espero a que arranque el esp03

      digitalWrite(4, HIGH);
      bool configuracionTerminada = false;
      while (!configuracionTerminada) {
        if (Serial.available()) {
          delay(100);
          Serial.readBytes(entradaSerial, 16);
          if (entradaSerial[0] == 'A' && entradaSerial[1] != 'B') {
            digitalWrite(6, HIGH);
          } else if (entradaSerial[0] == 'O' && entradaSerial[1] == 'T' && entradaSerial[2] == 'A' && entradaSerial[3] == 'A') {
            digitalWrite(6, LOW);
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
            digitalWrite(4, LOW);
            delay(5000);
            digitalWrite(A0, LOW);
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
            digitalWrite(4, LOW);

            delay(5000);
            digitalWrite(A0, LOW);

            delay(100);
            FRAMWrite(0, 0);

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
    digitalWrite(A0, HIGH);
    while (true) {
      digitalWrite(4, HIGH);

      delay(200);
      digitalWrite(4, LOW);

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

    delay(100);
    LMIC.seqnoUp = FRAMRead(0) + 1;



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

  if (digitalRead(3) == HIGH) {
    attachInterrupt(digitalPinToInterrupt(3), enviarMensaje, FALLING);
  } else {
    attachInterrupt(digitalPinToInterrupt(3), enviarMensaje, RISING);
  }
  do_send(&sendjob);

}



void loop() {
  os_runloop_once();
  if (habilitar_GPS == true) {
    /*
      while (gps.available(Serial)) {
      fix = gps.read();
      if (fix.valid.location ) {
        latitud = (float)fix.latitude();
        longitud = (float)fix.longitude();
        altitud = (float)fix.altitude();
        enviar_GPS = true;
        os_setCallback (&sendjob, do_send);
        ultimo_upload = millis();
      }
      }
    */
    if (millis() - ultimo_upload > 60. * 1000) {
      ultimo_upload = millis();
      os_setCallback (&sendjob, do_send);
    }
  }

  /*
    //Preparar la carga de pago
    lpp.reset();
    lpp.addDigitalInput(1, 1);
    attachInterrupt(digitalPinToInterrupt(3), enviarMensaje, FALLING);
    Serial.println("Me voy a dormir hasta que se abra la puerta");
    delay(100);
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_ON);
    //Se ha abierto la puerta
    // Deshabilitamos las interrupciones al despertarnos
    detachInterrupt(0);
    envioEnCurso = true;
    //Esperamos a que concluya el envío
    while (envioEnCurso) {
    os_runloop_once();
    }

    Serial.println(F("HOLA"));
    //Preparar la carga de pago
    lpp.reset();
    lpp.addDigitalInput(1, 0);
    if (digitalRead(3) == HIGH) {
    do_send(&sendjob);
    } else {
    attachInterrupt(digitalPinToInterrupt(3), enviarMensaje, RISING);
    Serial.println("Me voy a dormir hasta que se cierre la puerta");
    delay(100);
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_ON);
    }
    //Se ha cerrado la puerta
    // Deshabilitamos las interrupciones al despertarnos
    detachInterrupt(0);
    envioEnCurso = true;
    //Esperamos a que concluya el envío
    while (envioEnCurso) {
    os_runloop_once();
    }
  */

}
