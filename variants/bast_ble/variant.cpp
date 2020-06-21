#include "Arduino.h"

PinDescription g_APinDescription[] = {
  
  // D0 - D8
  P0_10,  NULL, NULL,    // D0/TX
  P0_9,  NULL, NULL,     // D1/RX
  P1_11, NULL, NULL,     // D2/SDA
  P1_10, NULL, NULL,     // D3/SCL
  P1_0, NULL, NULL,      // D4/SCK
  P1_2, NULL, NULL,      // D5
  P1_13, NULL, NULL,     // D6
  P1_6,  NULL, NULL,     // D7/MOSI
  P0_15,  NULL, NULL,    // D8/MISO

  // D9 - D13
  P0_6, NULL, NULL,      // D9
  P1_9, NULL, NULL,      // D10
  P0_8,  NULL, NULL,     // D11
  P0_4, NULL, NULL,      // D12
  P0_7, NULL, NULL,      // D13

  // D14 - LED
  P0_24, NULL, NULL,      // D14

  // D15..D21 - A0 - A5
  P0_2,  NULL, NULL,     // A0
  P0_28,  NULL, NULL,    // A1
  P0_5,  NULL, NULL,     // A2
  P0_3, NULL, NULL,      // A3
  P0_30, NULL, NULL,     // A4
  P0_29, NULL, NULL,     // A5
  P0_26, NULL, NULL,     // VBAT CHANGE!!

  // 22..26 - FLASH SPI
  P0_17,  NULL, NULL,     // QSPI_DATA0
  P0_22,  NULL, NULL,     // QSPI_DATA1
  P1_4, NULL, NULL,       // QSPI_DATA2
  P0_13,  NULL, NULL,     // QSPI_SCK
  P0_20, NULL, NULL,      // QSPI_CS
  

};

extern "C" {
  unsigned int PINCOUNT_fn() {
    return (sizeof(g_APinDescription) / sizeof(g_APinDescription[0]));
  }
}

#include "nrf_rtc.h"

void initVariant() {
  // turn power LED on
  //pinMode(LED_PWR, OUTPUT);
  //digitalWrite(LED_PWR, HIGH);

  // Errata Nano33BLE - I2C pullup is on SWO line, need to disable TRACE
  // was being enabled by nrfx_clock_anomaly_132
  //CoreDebug->DEMCR = 0;
  //NRF_CLOCK->TRACECONFIG = 0;

  // FIXME: bootloader enables interrupt on COMPARE[0], which we don't handle
  // Disable it here to avoid getting stuck when OVERFLOW irq is triggered
  //nrf_rtc_event_disable(NRF_RTC1, NRF_RTC_INT_COMPARE0_MASK);
  //nrf_rtc_int_disable(NRF_RTC1, NRF_RTC_INT_COMPARE0_MASK);

  // FIXME: always enable I2C pullup and power @startup
  // Change for maximum powersave
  //pinMode(PIN_ENABLE_SENSORS_3V3, OUTPUT);
  //pinMode(PIN_ENABLE_I2C_PULLUP, OUTPUT);

  //digitalWrite(PIN_ENABLE_SENSORS_3V3, HIGH);
  //digitalWrite(PIN_ENABLE_I2C_PULLUP, HIGH);
 
  NRF_PWM_Type* PWM[] = {
    NRF_PWM0, NRF_PWM1, NRF_PWM2
#ifdef NRF_PWM3
    ,NRF_PWM3
#endif
  };

  for (unsigned int i = 0; i < (sizeof(PWM)/sizeof(PWM[0])); i++) {
    PWM[i]->ENABLE = 0;
    PWM[i]->PSEL.OUT[0] = 0xFFFFFFFFUL;
  } 
}

#ifdef SERIAL_CDC

static void utox8(uint32_t val, uint8_t* s) {
  for (int i = 0; i < 16; i=i+2) {
    int d = val & 0XF;
    val = (val >> 4);

    s[15 - i -1] = d > 9 ? 'A' + d - 10 : '0' + d;
    s[15 - i] = '\0';
  }
}

uint8_t getUniqueSerialNumber(uint8_t* name) {
  #define SERIAL_NUMBER_WORD_0  NRF_FICR->DEVICEADDR[1]
  #define SERIAL_NUMBER_WORD_1  NRF_FICR->DEVICEADDR[0]

  utox8(SERIAL_NUMBER_WORD_0, &name[0]);
  utox8(SERIAL_NUMBER_WORD_1, &name[16]);

  return 32;
}

void _ontouch1200bps_() {
  __disable_irq();
  NRF_POWER->GPREGRET = DFU_MAGIC_SERIAL_ONLY_RESET;
  NVIC_SystemReset();
}

#endif
