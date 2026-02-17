/*
 * boneIO Edge Temp — ATtiny402 SHT40 Modbus RTU Sensor
 *
 * Copyright (C) 2025 boneIO Sp. z o.o.
 * License: GPL-3.0 (see LICENSE)
 * Made in Poland
 *
 * Registers (Holding, FC 0x03/0x04 read, FC 0x06 write):
 *   0x0000 (0)   - Humidity      [0.1 %RH, S_WORD]              (R)
 *   0x0001 (1)   - Temperature   [0.1 °C,  S_WORD]              (R)
 *   0x0064 (100) - Slave ID      [1–247]                        (R/W)
 *   0x0065 (101) - Baud Rate     [0=2400,1=4800,2=9600,3=19200] (R/W)
 *   0x0066 (102) - LED mode      [0=off, 1=on, 2=auto, when Modbus
 * communication is active]                                      (R/W)
 *   0x0067 (103) - FW version    [e.g. 0x0001 = v0.1]           (R)
 *   0x0068 (104) - Temp cal      [0.1 °C offset, S_WORD]        (R/W)
 *   0x0069 (105) - Hum cal       [0.1 %RH offset, S_WORD]       (R/W)
 *
 * Hardware (boneIO edge-temp PCB):
 *   PA1 = SDA  (soft I2C → SHT40, 47Ω series, 4.7kΩ pull-up)
 *   PA2 = SCL  (soft I2C → SHT40, 47Ω series, 4.7kΩ pull-up)
 *   PA3 = LED  (active low, 220Ω to VCC)
 *   PA6 = TXD  (USART0 → THVD1406 auto-dir RS485)
 *   PA7 = RXD  (USART0 ← THVD1406)
 *   PA0 = UPDI
 *
 * Clock: 3.33 MHz (20 MHz / 6, ATtiny402 default)
 */

#ifdef F_CPU
#undef F_CPU
#endif
#define F_CPU 3333333UL

#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdbool.h>
#include <util/delay.h>

/* ═══════════════════════════════════════════════════════════════
 *  Configuration
 * ═══════════════════════════════════════════════════════════════ */

#define FW_VERSION 0x0001 /* v0.1 — high byte = major, low = minor */

#define DEFAULT_SLAVE_ID 1
#define DEFAULT_BAUD_IDX 2 /* 9600 */
#define DEFAULT_LED_MODE 2 /* auto */

#define EEPROM_ADDR_ID 0x00
#define EEPROM_ADDR_BAUD 0x01
#define EEPROM_ADDR_LED 0x02
#define EEPROM_ADDR_CAL_T 0x04 /* int16_t, 2 bytes (0x04-0x05) */
#define EEPROM_ADDR_CAL_H 0x06 /* int16_t, 2 bytes (0x06-0x07) */

#define SHT40_ADDR 0x44
#define SHT40_CMD_MEASURE 0xFD /* High precision, no heater */

#define SDA_PIN 1 /* PA1 */
#define SCL_PIN 2 /* PA2 */
#define LED_PIN 3 /* PA3 */

#define RX_BUF_SIZE 16

/* Register addresses */
#define REG_HUMIDITY 0x0000    /* 0   */
#define REG_TEMPERATURE 0x0001 /* 1   */
#define REG_SLAVE_ID 0x0064    /* 100 */
#define REG_BAUDRATE 0x0065    /* 101 */
#define REG_LED_MODE 0x0066    /* 102 */
#define REG_FW_VERSION 0x0067  /* 103 */
#define REG_CAL_TEMP 0x0068    /* 104 — calibration offset [0.1°C] */
#define REG_CAL_HUM 0x0069     /* 105 — calibration offset [0.1%RH] */

#define CAL_MAX 500 /* max ±50.0 units */

/* Modbus function codes */
#define FC_READ_HOLDING 0x03
#define FC_READ_INPUT 0x04
#define FC_WRITE_SINGLE 0x06

/* Modbus exception codes */
#define EXC_ILLEGAL_FUNC 0x01
#define EXC_ILLEGAL_ADDR 0x02
#define EXC_ILLEGAL_VALUE 0x03

/* LED modes */
#define LED_OFF 0
#define LED_ON 1
#define LED_AUTO 2

/* Measurement interval: ~3 seconds (loop iterations) */
#define MEASURE_INTERVAL 30000U

/* ═══════════════════════════════════════════════════════════════
 *  Baud rate tables
 * ═══════════════════════════════════════════════════════════════ */

/* USART BAUD = F_CPU * 64 / (16 * baud) */
static const uint16_t uart_baud_reg[] = {
    5556, /* 0: 2400  */
    2778, /* 1: 4800  */
    1389, /* 2: 9600  */
    694   /* 3: 19200 */
};

/* TCA0 period for 3.5 char silence (TCA0 @ F_CPU/64 ≈ 52083 Hz) */
static const uint16_t silence_period[] = {
    834, /* 0: 2400  */
    417, /* 1: 4800  */
    208, /* 2: 9600  */
    104  /* 3: 19200 */
};

#define BAUD_COUNT 4

/* ═══════════════════════════════════════════════════════════════
 *  Global state
 * ═══════════════════════════════════════════════════════════════ */

static uint8_t slave_id;
static uint8_t baud_idx;
static uint8_t led_mode;
static int16_t cal_temp = 0; /* calibration offset [0.1°C]  */
static int16_t cal_hum = 0;  /* calibration offset [0.1%RH] */

static volatile int16_t reg_humidity = 0;
static volatile int16_t reg_temperature = 0;

/* RX buffer (filled by ISR) */
static volatile uint8_t rx_buf[RX_BUF_SIZE];
static volatile uint8_t rx_len = 0;
static volatile bool frame_ready = false;

/* ═══════════════════════════════════════════════════════════════
 *  LED (active LOW on PA3 — pin LOW = LED ON)
 * ═══════════════════════════════════════════════════════════════ */

static inline void led_init(void) {
  PORTA.DIRSET = (1 << LED_PIN);
  PORTA.OUTSET = (1 << LED_PIN); /* OFF (HIGH = off) */
}

static inline void led_hw_on(void) { PORTA.OUTCLR = (1 << LED_PIN); }
static inline void led_hw_off(void) { PORTA.OUTSET = (1 << LED_PIN); }

/* Apply LED mode (call after mode change) */
static void led_apply_mode(void) {
  if (led_mode == LED_ON)
    led_hw_on();
  else if (led_mode == LED_OFF)
    led_hw_off();
  /* LED_AUTO: managed by blink calls */
}

/* Blink LED only in AUTO mode */
static void led_blink(void) {
  if (led_mode != LED_AUTO)
    return;
  led_hw_on();
  _delay_ms(50);
  led_hw_off();
}

/* ═══════════════════════════════════════════════════════════════
 *  Soft I2C (bit-banged, open-drain)
 * ═══════════════════════════════════════════════════════════════ */

static inline void sda_high(void) { PORTA.DIRCLR = (1 << SDA_PIN); }
static inline void sda_low(void) {
  PORTA.OUTCLR = (1 << SDA_PIN);
  PORTA.DIRSET = (1 << SDA_PIN);
}
static inline void scl_high(void) { PORTA.DIRCLR = (1 << SCL_PIN); }
static inline void scl_low(void) {
  PORTA.OUTCLR = (1 << SCL_PIN);
  PORTA.DIRSET = (1 << SCL_PIN);
}
static inline uint8_t sda_read(void) { return (PORTA.IN >> SDA_PIN) & 1; }

static inline void i2c_delay(void) { _delay_us(5); }

static void i2c_start(void) {
  sda_high();
  scl_high();
  i2c_delay();
  sda_low();
  i2c_delay();
  scl_low();
  i2c_delay();
}

static void i2c_stop(void) {
  sda_low();
  i2c_delay();
  scl_high();
  i2c_delay();
  sda_high();
  i2c_delay();
}

static bool i2c_write_byte(uint8_t data) {
  for (uint8_t i = 0; i < 8; i++) {
    if (data & 0x80)
      sda_high();
    else
      sda_low();
    data <<= 1;
    i2c_delay();
    scl_high();
    i2c_delay();
    scl_low();
  }
  sda_high();
  i2c_delay();
  scl_high();
  i2c_delay();
  bool ack = !sda_read();
  scl_low();
  i2c_delay();
  return ack;
}

static uint8_t i2c_read_byte(bool send_ack) {
  uint8_t data = 0;
  sda_high();
  for (uint8_t i = 0; i < 8; i++) {
    data <<= 1;
    scl_high();
    i2c_delay();
    if (sda_read())
      data |= 1;
    scl_low();
    i2c_delay();
  }
  if (send_ack)
    sda_low();
  else
    sda_high();
  i2c_delay();
  scl_high();
  i2c_delay();
  scl_low();
  i2c_delay();
  sda_high();
  return data;
}

/* ═══════════════════════════════════════════════════════════════
 *  SHT40 sensor
 * ═══════════════════════════════════════════════════════════════ */

/* Sensirion CRC-8: polynomial 0x31, init 0xFF */
static uint8_t sht40_crc8(uint8_t msb, uint8_t lsb) {
  uint8_t crc = 0xFF;
  crc ^= msb;
  for (uint8_t b = 0; b < 8; b++)
    crc = (crc & 0x80) ? ((crc << 1) ^ 0x31) : (crc << 1);
  crc ^= lsb;
  for (uint8_t b = 0; b < 8; b++)
    crc = (crc & 0x80) ? ((crc << 1) ^ 0x31) : (crc << 1);
  return crc;
}

/* Read SHT40, returns true on success */
static bool sht40_read(int16_t *temp_out, int16_t *hum_out) {
  /* Send measurement command (0xFD = high precision) */
  i2c_start();
  if (!i2c_write_byte(SHT40_ADDR << 1)) {
    i2c_stop();
    return false;
  }
  if (!i2c_write_byte(SHT40_CMD_MEASURE)) {
    i2c_stop();
    return false;
  }
  i2c_stop();

  /* Wait for measurement (~10 ms high precision) */
  _delay_ms(12);

  /* Read 6 bytes: T_MSB, T_LSB, T_CRC, H_MSB, H_LSB, H_CRC */
  i2c_start();
  if (!i2c_write_byte((SHT40_ADDR << 1) | 1)) {
    i2c_stop();
    return false;
  }

  uint8_t t_msb = i2c_read_byte(true);
  uint8_t t_lsb = i2c_read_byte(true);
  uint8_t t_crc = i2c_read_byte(true);
  uint8_t h_msb = i2c_read_byte(true);
  uint8_t h_lsb = i2c_read_byte(true);
  uint8_t h_crc = i2c_read_byte(false); /* NACK last byte */
  i2c_stop();

  /* Verify CRC */
  if (sht40_crc8(t_msb, t_lsb) != t_crc)
    return false;
  if (sht40_crc8(h_msb, h_lsb) != h_crc)
    return false;

  uint16_t t_raw = ((uint16_t)t_msb << 8) | t_lsb;
  uint16_t h_raw = ((uint16_t)h_msb << 8) | h_lsb;

  /* Convert to 0.1 °C:  T = (175 * t_raw / 65535 - 45) * 10
   *                        = 1750 * t_raw / 65535 - 450          */
  int32_t t_val = ((int32_t)1750 * t_raw) / 65535 - 450;

  /* Convert to 0.1 %RH: H = (125 * h_raw / 65535 -  6) * 10
   *                        = 1250 * h_raw / 65535 -  60          */
  int32_t h_val = ((int32_t)1250 * h_raw) / 65535 - 60;

  /* Clamp humidity to 0.0–100.0 % */
  if (h_val < 0)
    h_val = 0;
  if (h_val > 1000)
    h_val = 1000;

  *temp_out = (int16_t)t_val;
  *hum_out = (int16_t)h_val;
  return true;
}

/* ═══════════════════════════════════════════════════════════════
 *  UART (USART0, PA6=TX, PA7=RX)
 * ═══════════════════════════════════════════════════════════════ */

static void uart_init(void) {
  PORTA.DIRSET = PIN6_bm; /* TX output */
  USART0.BAUD = uart_baud_reg[baud_idx];
  USART0.CTRLC = USART_CHSIZE_8BIT_gc; /* 8N1 */
  USART0.CTRLB = USART_TXEN_bm | USART_RXEN_bm;
  USART0.CTRLA = USART_RXCIE_bm; /* RX interrupt */
}

static void uart_tx(uint8_t c) {
  while (!(USART0.STATUS & USART_DREIF_bm))
    ;
  USART0.TXDATAL = c;
}

static void uart_tx_flush(void) {
  while (!(USART0.STATUS & USART_TXCIF_bm))
    ;
  USART0.STATUS = USART_TXCIF_bm;
}

/* ═══════════════════════════════════════════════════════════════
 *  Silence timer (TCA0) — Modbus 3.5 char gap detection
 * ═══════════════════════════════════════════════════════════════ */

static void silence_timer_init(void) {
  TCA0.SINGLE.PER = silence_period[baud_idx];
  TCA0.SINGLE.INTCTRL = TCA_SINGLE_OVF_bm;
  TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV64_gc; /* disabled */
}

static inline void silence_timer_restart(void) {
  TCA0.SINGLE.CNT = 0;
  TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;
  TCA0.SINGLE.CTRLA |= TCA_SINGLE_ENABLE_bm;
}

static inline void silence_timer_stop(void) {
  TCA0.SINGLE.CTRLA &= ~TCA_SINGLE_ENABLE_bm;
}

/* TCA0 overflow = 3.5 char silence = end of Modbus frame */
ISR(TCA0_OVF_vect) {
  TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;
  silence_timer_stop();
  if (rx_len >= 4) { /* min: addr + func + crc(2) */
    frame_ready = true;
  } else {
    rx_len = 0; /* runt frame, discard */
  }
}

/* USART0 RX complete */
ISR(USART0_RXC_vect) {
  uint8_t data = USART0.RXDATAL;
  if (frame_ready)
    return; /* previous frame not processed yet */
  if (rx_len < RX_BUF_SIZE) {
    rx_buf[rx_len++] = data;
  }
  silence_timer_restart();
}

/* ═══════════════════════════════════════════════════════════════
 *  Modbus CRC-16
 * ═══════════════════════════════════════════════════════════════ */

static uint16_t modbus_crc16(const uint8_t *buf, uint8_t len) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= buf[i];
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 1) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

/* ═══════════════════════════════════════════════════════════════
 *  Modbus TX helpers
 * ═══════════════════════════════════════════════════════════════ */

static void mb_send(const uint8_t *buf, uint8_t len) {
  uint16_t crc = modbus_crc16(buf, len);
  for (uint8_t i = 0; i < len; i++)
    uart_tx(buf[i]);
  uart_tx(crc & 0xFF);
  uart_tx(crc >> 8);
  uart_tx_flush();
}

static void mb_send_raw(const uint8_t *buf, uint8_t len) {
  for (uint8_t i = 0; i < len; i++)
    uart_tx(buf[i]);
  uart_tx_flush();
}

static void mb_exception(uint8_t func, uint8_t code) {
  uint8_t resp[3] = {slave_id, (uint8_t)(func | 0x80), code};
  mb_send(resp, 3);
}

/* ═══════════════════════════════════════════════════════════════
 *  Modbus register access
 * ═══════════════════════════════════════════════════════════════ */

static bool reg_read(uint16_t addr, uint16_t *val) {
  switch (addr) {
  case REG_HUMIDITY:
    *val = (uint16_t)reg_humidity;
    return true;
  case REG_TEMPERATURE:
    *val = (uint16_t)reg_temperature;
    return true;
  case REG_SLAVE_ID:
    *val = (uint16_t)slave_id;
    return true;
  case REG_BAUDRATE:
    *val = (uint16_t)baud_idx;
    return true;
  case REG_LED_MODE:
    *val = (uint16_t)led_mode;
    return true;
  case REG_FW_VERSION:
    *val = FW_VERSION;
    return true;
  case REG_CAL_TEMP:
    *val = (uint16_t)cal_temp;
    return true;
  case REG_CAL_HUM:
    *val = (uint16_t)cal_hum;
    return true;
  default:
    return false;
  }
}

/* ═══════════════════════════════════════════════════════════════
 *  Modbus frame processing
 * ═══════════════════════════════════════════════════════════════ */

static void process_modbus(void) {
  uint8_t len = rx_len;

  if (len < 6)
    return;

  /* Address check */
  if (rx_buf[0] != slave_id)
    return;

  /* CRC check */
  uint16_t rx_crc = ((uint16_t)rx_buf[len - 1] << 8) | rx_buf[len - 2];
  if (modbus_crc16((const uint8_t *)rx_buf, len - 2) != rx_crc)
    return;

  uint8_t func = rx_buf[1];

  /* Turnaround delay for RS485 */
  _delay_ms(2);

  switch (func) {

  /* ── Read Holding / Input Registers ────────────────────── */
  case FC_READ_HOLDING:
  case FC_READ_INPUT: {
    if (len < 8)
      return;

    uint16_t start = ((uint16_t)rx_buf[2] << 8) | rx_buf[3];
    uint16_t qty = ((uint16_t)rx_buf[4] << 8) | rx_buf[5];

    if (qty == 0 || qty > 6) {
      mb_exception(func, EXC_ILLEGAL_VALUE);
      break;
    }

    /* Build response: [addr][func][byte_cnt][data...] */
    uint8_t resp[3 + 12]; /* header(3) + max 6 regs × 2 bytes */
    uint8_t bc = 0;

    for (uint16_t i = 0; i < qty; i++) {
      uint16_t val;
      if (!reg_read(start + i, &val)) {
        mb_exception(func, EXC_ILLEGAL_ADDR);
        return;
      }
      resp[3 + bc++] = (uint8_t)(val >> 8);
      resp[3 + bc++] = (uint8_t)(val & 0xFF);
    }

    resp[0] = slave_id;
    resp[1] = func;
    resp[2] = bc;
    mb_send(resp, 3 + bc);
    led_blink();
    break;
  }

  /* ── Write Single Register ─────────────────────────────── */
  case FC_WRITE_SINGLE: {
    if (len < 8)
      return;

    uint16_t reg_addr = ((uint16_t)rx_buf[2] << 8) | rx_buf[3];
    uint16_t value = ((uint16_t)rx_buf[4] << 8) | rx_buf[5];

    switch (reg_addr) {

    case REG_SLAVE_ID:
      if (value < 1 || value > 247) {
        mb_exception(func, EXC_ILLEGAL_VALUE);
        break;
      }
      /* Echo response with OLD address first */
      mb_send_raw((const uint8_t *)rx_buf, 8);
      _delay_ms(5);
      slave_id = (uint8_t)value;
      eeprom_update_byte((uint8_t *)EEPROM_ADDR_ID, slave_id);
      break;

    case REG_BAUDRATE:
      if (value >= BAUD_COUNT) {
        mb_exception(func, EXC_ILLEGAL_VALUE);
        break;
      }
      /* Echo response at OLD baud rate first */
      mb_send_raw((const uint8_t *)rx_buf, 8);
      _delay_ms(5);
      baud_idx = (uint8_t)value;
      eeprom_update_byte((uint8_t *)EEPROM_ADDR_BAUD, baud_idx);
      USART0.BAUD = uart_baud_reg[baud_idx];
      TCA0.SINGLE.PER = silence_period[baud_idx];
      break;

    case REG_LED_MODE:
      if (value > LED_AUTO) {
        mb_exception(func, EXC_ILLEGAL_VALUE);
        break;
      }
      led_mode = (uint8_t)value;
      eeprom_update_byte((uint8_t *)EEPROM_ADDR_LED, led_mode);
      led_apply_mode();
      /* Echo response */
      mb_send_raw((const uint8_t *)rx_buf, 8);
      break;

    case REG_FW_VERSION:
      /* Read-only register */
      mb_exception(func, EXC_ILLEGAL_ADDR);
      break;

    case REG_CAL_TEMP: {
      int16_t sv = (int16_t)value;
      if (sv < -CAL_MAX || sv > CAL_MAX) {
        mb_exception(func, EXC_ILLEGAL_VALUE);
        break;
      }
      cal_temp = sv;
      eeprom_update_word((uint16_t *)EEPROM_ADDR_CAL_T, (uint16_t)cal_temp);
      mb_send_raw((const uint8_t *)rx_buf, 8);
      break;
    }

    case REG_CAL_HUM: {
      int16_t sv = (int16_t)value;
      if (sv < -CAL_MAX || sv > CAL_MAX) {
        mb_exception(func, EXC_ILLEGAL_VALUE);
        break;
      }
      cal_hum = sv;
      eeprom_update_word((uint16_t *)EEPROM_ADDR_CAL_H, (uint16_t)cal_hum);
      mb_send_raw((const uint8_t *)rx_buf, 8);
      break;
    }

    default:
      mb_exception(func, EXC_ILLEGAL_ADDR);
      break;
    }
    break;
  }

  default:
    mb_exception(func, EXC_ILLEGAL_FUNC);
    break;
  }
}

/* ═══════════════════════════════════════════════════════════════
 *  EEPROM load / defaults
 * ═══════════════════════════════════════════════════════════════ */

static void load_config(void) {
  slave_id = eeprom_read_byte((const uint8_t *)EEPROM_ADDR_ID);
  if (slave_id < 1 || slave_id > 247)
    slave_id = DEFAULT_SLAVE_ID;

  baud_idx = eeprom_read_byte((const uint8_t *)EEPROM_ADDR_BAUD);
  if (baud_idx >= BAUD_COUNT)
    baud_idx = DEFAULT_BAUD_IDX;

  led_mode = eeprom_read_byte((const uint8_t *)EEPROM_ADDR_LED);
  if (led_mode > LED_AUTO)
    led_mode = DEFAULT_LED_MODE;

  cal_temp = (int16_t)eeprom_read_word((const uint16_t *)EEPROM_ADDR_CAL_T);
  if (cal_temp < -CAL_MAX || cal_temp > CAL_MAX)
    cal_temp = 0;

  cal_hum = (int16_t)eeprom_read_word((const uint16_t *)EEPROM_ADDR_CAL_H);
  if (cal_hum < -CAL_MAX || cal_hum > CAL_MAX)
    cal_hum = 0;
}

/* ═══════════════════════════════════════════════════════════════
 *  Main
 * ═══════════════════════════════════════════════════════════════ */

int main(void) {
  /* Clock: 20 MHz / 6 = 3.33 MHz (already default, set explicitly) */
  _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, CLKCTRL_PDIV_6X_gc | CLKCTRL_PEN_bm);

  load_config();
  led_init();
  led_apply_mode();
  uart_init();
  silence_timer_init();

  /* I2C pins: both start as inputs (open-drain high) */
  PORTA.DIRCLR = (1 << SDA_PIN) | (1 << SCL_PIN);

  sei();

  /* Startup indicator: 3 quick blinks = MCU is alive */
  for (uint8_t i = 0; i < 3; i++) {
    led_hw_on();
    _delay_ms(100);
    led_hw_off();
    _delay_ms(100);
  }

  /* Initial measurement */
  _delay_ms(50);
  {
    int16_t t, h;
    if (sht40_read(&t, &h)) {
      reg_temperature = t + cal_temp;
      int16_t hc = h + cal_hum;
      if (hc < 0)
        hc = 0;
      if (hc > 1000)
        hc = 1000;
      reg_humidity = hc;
      /* Clamp temperature to -40.0 .. +125.0 °C */
      int16_t tc = reg_temperature;
      if (tc < -400)
        tc = -400;
      if (tc > 1250)
        tc = 1250;
      reg_temperature = tc;
    } else {
      /* Sensor error: long blink = I2C problem */
      led_hw_on();
      _delay_ms(1000);
      led_hw_off();
    }
  }

  uint16_t measure_cnt = 0;

  for (;;) {
    /* ── Process completed Modbus frame ──────────────────── */
    if (frame_ready) {
      process_modbus();
      rx_len = 0;
      frame_ready = false;
    }

    /* ── Periodic SHT40 measurement (~3s interval) ────────── */
    if (++measure_cnt >= MEASURE_INTERVAL) {
      measure_cnt = 0;

      int16_t t, h;
      if (sht40_read(&t, &h)) {
        reg_temperature = t + cal_temp;
        int16_t hc = h + cal_hum;
        if (hc < 0)
          hc = 0;
        if (hc > 1000)
          hc = 1000;
        reg_humidity = hc;
        /* Clamp temperature to -40.0 .. +125.0 °C */
        int16_t tc = reg_temperature;
        if (tc < -400)
          tc = -400;
        if (tc > 1250)
          tc = 1250;
        reg_temperature = tc;
      }
    }

    /* Loop pacing: 100µs/iter → 30000 × 100µs ≈ 3s between measurements.
       UART RX is interrupt-driven, so no bytes are lost. */
    _delay_us(100);
  }
}
