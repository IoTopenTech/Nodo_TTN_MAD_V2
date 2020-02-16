


/*******************************************************************************
   Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
   Copyright (c) 2018 Terry Moore, MCCI
   Permission is hereby granted, free of charge, to anyone
   obtaining a copy of this document and accompanying files,
   to do whatever they want with them without any restriction,
   including, but not limited to, copying, modification and redistribution.
   NO WARRANTY OF ANY KIND IS PROVIDED.
   This example sends a valid LoRaWAN packet with payload "Hello,
   world!", using frequency and encryption settings matching those of
   the The Things Network.
   This uses ABP (Activation-by-personalisation), where a DevAddr and
   Session keys are preconfigured (unlike OTAA, where a DevEUI and
   application key is configured, while the DevAddr and session keys are
   assigned/generated in the over-the-air-activation procedure).
   Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
   g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
   violated by this sketch when left running for longer)!
   To use this sketch, first register your application and device with
   the things network, to set or generate a DevAddr, NwkSKey and
   AppSKey. Each device should have their own unique values for these
   fields.
   Do not forget to define the radio type correctly in
   arduino-lmic/project_config/lmic_project_config.h or from your BOARDS.txt.
 *******************************************************************************/

// References:
// [feather] adafruit-feather-m0-radio-with-lora-module.pdf

#include <Wire.h>
#include <SPI.h>
#include "LowPower.h"
#include <lmic.h>
#include <hal/hal.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <CayenneLPP.h>
CayenneLPP lpp(22);
//En el archivo Adafruit_BME280.h hay que cambiar la siguiente línea
// #define BME280_ADDRESS                (0x76)
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme; // I2C

boolean envioEnCurso = false;
boolean puertaAbierta;
boolean interrumpido = false;
byte minutosHearbeat = 1;
//
// For normal use, we require that you edit the sketch to replace FILLMEIN
// with values assigned by the TTN console. However, for regression tests,
// we want to be able to compile these scripts. The regression tests define
// COMPILE_REGRESSION_TEST, and in that case we define FILLMEIN to a non-
// working but innocuous value.
//
#ifdef COMPILE_REGRESSION_TEST
# define FILLMEIN 0
#else
# warning "You must replace the values marked FILLMEIN with real values from the TTN control panel!"
# define FILLMEIN (#dont edit this, edit the lines that use FILLMEIN)
#endif

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8]={ FILLMEIN };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8]={ FILLMEIN };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = { FILLMEIN };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}


//static uint8_t mydata[] = "Hello, world!";
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
//const unsigned TX_INTERVAL = 60;

// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 9,
  .dio = {2, 7, LMIC_UNUSED_PIN},
};

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
void enviarMensaje() {
  envioEnCurso = true;
  interrumpido = true;
  os_setCallback (&sendjob, do_send);
}

void onEvent (ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      //Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      //Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      //Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      //Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      //Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      //Serial.println(F("EV_JOINED"));
      break;
    /*
      || This event is defined but not used in the code. No
      || point in wasting codespace on it.
      ||
      || case EV_RFU1:
      ||     Serial.println(F("EV_RFU1"));
      ||     break;
    */
    case EV_JOIN_FAILED:
      //Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      //Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));
      if (LMIC.dataLen) {
        //Serial.println(F("Received "));
        //Serial.println(LMIC.dataLen);
        //Serial.println(F(" bytes of payload"));
        //El downlink tendrá el formato canal (06), tipo de valor (00=digital), valor (0 o 100=0x64), FF (ACK)
        if (LMIC.frame[LMIC.dataBeg + 2] == 100) {
          digitalWrite(4, HIGH);          
        } else {
          digitalWrite(4, LOW);          
        }
      }
      // Schedule next transmission
      //os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
      envioEnCurso = false;
      break;
    case EV_LOST_TSYNC:
      //Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      //Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      //Serial.println(F("EV_LINK_ALIVE"));
      break;
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
    lpp.reset();
    lpp.addAnalogInput(1, readVcc() / 1000.F);
    lpp.addDigitalInput(2, puertaAbierta);
    //LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
    if (bme.begin()) {
      bme.setSampling(Adafruit_BME280::MODE_FORCED,
                      Adafruit_BME280::SAMPLING_X1, // temperature
                      Adafruit_BME280::SAMPLING_X1, // pressure
                      Adafruit_BME280::SAMPLING_X1, // humidity
                      Adafruit_BME280::FILTER_OFF   );
      lpp.addTemperature(3, bme.readTemperature());
      lpp.addBarometricPressure(4, bme.readPressure() / 100.0F);
      lpp.addRelativeHumidity(5, bme.readHumidity());
    }
    lpp.addDigitalOutput(6, digitalRead(4));
    LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);
    Serial.println(F("Packet queued"));
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
  pinMode(4,OUTPUT);
  digitalWrite(4,LOW);
  //    pinMode(13, OUTPUT);
  while (!Serial); // wait for Serial to be initialized
  Serial.begin(115200);
  delay(100);     // per sample code on RF_95 test
  Serial.println(F("Starting"));

#ifdef VCC_ENABLE
  // For Pinoccio Scout boards
  pinMode(VCC_ENABLE, OUTPUT);
  digitalWrite(VCC_ENABLE, HIGH);
  delay(1000);
#endif
  bool status;
  status = bme.begin();
  if (!status) {
    Serial.println(F("Could not find a valid BME280 sensor!"));    
    while (1);
  }else{
    Serial.println(F("BME280 inicializado!"));
  }
  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();



  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100); //Para mejorar la recepción de los downlinks
  //Conecto en el pin 3 un interruptor a GND normalmente abierto
  pinMode(3, INPUT);
  delay(100);
  // Start job
  puertaAbierta = digitalRead(3);
  enviarMensaje();
  //do_send(&sendjob);
}

void loop() {
  //Esperamos a que concluya el envío
  while (envioEnCurso) {
    os_runloop_once();
  }
  delay(100);
  interrumpido = false;
  if (puertaAbierta == digitalRead(3)) {
    //La puerta no ha cambiado de estado
    attachInterrupt(digitalPinToInterrupt(3), enviarMensaje, (puertaAbierta == 1 ? FALLING : RISING));
    for (byte minutos = 0; minutos < minutosHearbeat; minutos++) {
      if (interrumpido == true) {
        break;        
      }
      for (byte contador = 0; contador < 7; contador++) {
        if (interrumpido == true) {
          break;
        }
        LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
      }
    }
    if (!interrumpido) {
      enviarMensaje();
    } else {
      puertaAbierta = !puertaAbierta;
    }
  } else {
    //La puerta ha cambiado de estado durante el envío del mensaje de estado anterior
    puertaAbierta = !puertaAbierta;
    enviarMensaje();
  }
}
