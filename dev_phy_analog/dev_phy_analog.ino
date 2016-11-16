// By Bontor Humala
// PHY layer of Volcano, default is off (idle)
// LOG
// 11/7
//   (ok) initial version, use ADC to sample - can detect edge
// 11/8
//   (?) if rx_len is wrong (corrupt due to missed edge detection), returns to idle state
//   (?) fix moving window, previously all data are discarded once buffer is full. needs FIFO buffer
// 11/9
//   (ok) in transmission, tx_buffer elements must be changed to bits
//   (ok) able to tx properly, detect edges, but every bit is detected as 1. need preamble (i.e. starting bit must be 0 -- rising edge)
// 11/10
//   (ok) adding preamble 01..01, according to PREAMBLE_LEN
//   (ok) fix edge detection, previously assigning in_bit always 1 because variable is uint (never negative thus never 0)
//   (ok) fix edge detection, edge is only considered if 0-th bit and PULSE_LEN-th bit exceeds EDGE_THRESHOLD
// 11/11
//   (bug) able to receive bits, but really depends on timing. need to check tx output and also adc sampling period
// 11/15
//   (?) shorter interrupt to avoid inconsistent timing, use preamble and level detection

#include <stdint.h>

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

#define SERIAL_PLOT
#define DEBUG
#define DEBUG_RX
//#define DEBUG_UPDATE
//#define DEBUG_TX
//#define DEBUG_OSC
#define RX_NODE
//#define TX_NODE
//#define TX_NODE_LOOP
//#define PROFILING

#define PHY_IDLE 0
#define PHY_RX 1
#define PHY_TX_RX 2
#define PHY_PREAMBLE_RX 3
#define PHY_PREAMBLE_TX 4
#define TIMER2COUNT 100 // Timer2 runs at 16MHz, prescaler 64, (4 us per tick), in order to interrupt every 400us -> count 100 (CTC)
#define PHY_SAFE_IDLE 5 // minimum idle pulse period to ensure it is safe to transmit
#define MAX_PHY_BUFFER 263 // 263 (maximum MAC packet), assume no additional PHY bytes
#define ACK_PHY_BUFFER 8 // 8 (ACK in MAC), assume no additional PHY bytes
#define BIT_LEN 2
#define PREAMBLE_LEN 6
#define BYTE_LEN 8
#define EDGE_THRESHOLD 10 // minimum range in PULSE_WINDOW_LEN to be considered an edge 
#define NEG_EDGE_THRESHOLD -10

// rx tx buffer and iterator
uint8_t rx_buffer[MAX_PHY_BUFFER];
uint8_t tx_buffer[MAX_PHY_BUFFER];
uint8_t preamble_buffer[PREAMBLE_LEN];
uint16_t rx_iter;
uint16_t tx_iter;
// maximum iteration for rx_buffer and tx_buffer
uint16_t rx_len;
uint16_t tx_len;
uint8_t preamble_iter;
// bit array representation of byte in tx_buffer
uint8_t tx_bits_buffer[BYTE_LEN];
uint8_t tx_bits_iter;
uint8_t tx_bit_buffer;
// decoding bit to byte for rx_buffer
uint8_t rx_bits_buffer[BYTE_LEN];
uint8_t rx_bits_iter;
// update sampling_buffer every sampling period
uint16_t sampling_buffer;
uint16_t prev_sampling;
uint8_t phy_state;
uint16_t rx_threshold;
uint8_t idle_counter;

uint8_t tx_pin;
uint8_t rx_pin;

// PUBLIC
bool phy_sense();
int16_t phy_rx(uint8_t *data);
void phy_tx(uint8_t *data, uint16_t data_len);
void _phy_fsm_control(); // needs to be called from loop() by the application

// PRIVATE
void _phy_update();
void _phy_idle();
void _phy_rx();
void _phy_tx_rx();
void _phy_preamble_rx();
void _phy_preamble_tx();
uint8_t _bits_byte(uint8_t *bits);
void _byte_bits(uint8_t tx_byte, uint8_t *bits);

#ifdef RX_NODE // address is 0x88
uint8_t test_rx[4];
#endif
#ifdef TX_NODE // address is 0x77
uint8_t test_tx[4] = {0x01,0x55,0x00,0xFF};
unsigned long tx_loop_time;
#endif
#ifdef PROFILING
unsigned long loop_time;
bool is_time;
#endif

#ifdef DEBUG_OSC
uint8_t osc_pin1;
uint8_t osc_pin2;
bool osc_pin1_state;
bool osc_pin2_state;
#endif

// update pulse_iter and sample ADC every PHY_SAMPLE_PERIOD
ISR(TIMER2_COMPA_vect)
{
#ifndef PROFILING
  _phy_fsm_control();
#endif
#ifdef PROFILING
  is_time = true;
#endif
#ifdef DEBUG_OSC
  osc_pin1_state = !osc_pin1_state;
  digitalWrite(osc_pin1, osc_pin1_state);
#endif
}

void setup() {
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);
  Serial.begin(115200);
#ifdef RX_NODE
  Serial.println("RX Node\n");
#endif
#ifdef TX_NODE
  Serial.println("TX Node\n");
#endif
  phy_state = PHY_IDLE;
  tx_pin = A1;
  rx_pin = A0;
  pinMode(tx_pin, OUTPUT);
#ifdef DEBUG_OSC
  osc_pin1 = 12;
  osc_pin2 = 13;
  pinMode(osc_pin1, OUTPUT);
  pinMode(osc_pin2, OUTPUT);
  osc_pin1_state = LOW;
  osc_pin2_state = LOW;
  digitalWrite(osc_pin1, osc_pin1_state);
  digitalWrite(osc_pin2, osc_pin2_state);
#endif
#ifdef TX_NODE
  phy_tx(test_tx, 4);
  tx_loop_time = millis();
#endif
  _initialize_timer();
}

void loop() {
#ifdef PROFILING
  if (is_time) {
    is_time = false;
    _phy_fsm_control();  
  }
#endif
#ifdef TX_NODE_LOOP
  if ((millis() - tx_loop_time) >= 1000) {
    phy_tx(test_tx, 1);
    tx_loop_time = millis();
  }
#endif
}

// returns true if medium is idle, false otherwise
bool phy_sense() {
  bool is_idle = false;
  if (phy_state == PHY_IDLE) {
    is_idle = true;
  }
  return is_idle;
}

// data will be filled with received frame
// must be read before being overwritten (rx_iter larger than MAX_PHY_BUFFER)
int16_t phy_rx(uint8_t *data) {
  if ((rx_iter != 0) || (rx_len == 0)) { // rx_iter is not finished yet OR rx_len is not updated
    return -1;
  }
  uint16_t rx_byte_len = rx_len;
  memcpy(data, rx_buffer, MAX_PHY_BUFFER);
  rx_iter = 0;
  return rx_byte_len;
}

// data contains frame to send, will be transferred to tx_buffer
void phy_tx(uint8_t *data, uint16_t data_len) {
  memcpy(tx_buffer, data, data_len);
  tx_len = data_len;
  tx_iter = 0;
}

// called every PHY_SAMPLE_PERIOD
void _phy_fsm_control() {
#ifdef PROFILING
  loop_time = micros();
#endif
  _phy_update();
  switch (phy_state) {
    case PHY_IDLE:
      _phy_idle();
      break;
    case PHY_PREAMBLE_RX:
      _phy_preamble_rx();
      break;
    case PHY_RX:
      _phy_rx();
      break;
    case PHY_PREAMBLE_TX:
      _phy_preamble_tx();
      break;
    case PHY_TX_RX:
      _phy_tx_rx();
      break;
  }
#ifdef PROFILING
#ifdef RX_NODE
  if (phy_state == PHY_RX) {
#endif
    Serial.println(micros() - loop_time);
#ifdef RX_NODE
  }
#endif    
#endif
}

// sample rx pin and update tx pin
void _phy_update() {
  sampling_buffer = analogRead(rx_pin); // sample rx_pin
  // update tx_pin
#ifdef DEBUG_UPDATE
  Serial.print("txb: "); Serial.println(tx_bit_buffer);
#endif
  digitalWrite(tx_pin, tx_bit_buffer);
}

// if find an edge, go to rx
// if there is something to send, go to tx
void _phy_idle() {
  rx_iter = 0;
  int16_t sampling_diff = prev_sampling - sampling_buffer;
  if ((sampling_diff > EDGE_THRESHOLD) || (sampling_diff < NEG_EDGE_THRESHOLD)) { // rising edge of preamble
    idle_counter = 0;
    preamble_buffer[preamble_iter] = sampling_buffer;
    preamble_iter++;
    if (preamble_iter == PREAMBLE_LEN) {
      uint16_t sum_preamble = 0;
      for (uint8_t i=0; i<PREAMBLE_LEN; i++) {
        sum_preamble = sum_preamble + preamble_buffer[i];
      }
      rx_threshold = sum_preamble/6;
      rx_len = 0; // reset rx_buffer
      idle_counter = 0; // reset idle_counter to hold back transmission
      phy_state = PHY_RX;
#ifdef DEBUG
      Serial.print("PHY_RX, thr: ");Serial.println(rx_threshold);
#endif
      return;
    }
  }
  idle_counter++;
  if ((idle_counter > PHY_SAFE_IDLE) && (tx_len > 0)) { // check tx buffer and ensure safe to send
#ifdef DEBUG
    Serial.println("PHY_PRE_TX");
#endif
    phy_state = PHY_PREAMBLE_TX;
    return;
  }
  prev_sampling = sampling_buffer;
}

void _phy_preamble_rx() {
  
}

// update rx_buffer and increment rx_iter
// rx_len is in dataframe[4]
void _phy_rx() {
  int8_t in_bit = -1;
  if (sampling_buffer > rx_threshold) in_bit = 1;
  else if (sampling_buffer < rx_threshold) in_bit = 0;
  if (in_bit > -1) {
#ifdef DEBUG_RX
//    Serial.print("bit: ");Serial.println(in_bit);
#endif
    rx_bits_buffer[tx_bits_iter] = in_bit;
    rx_bits_iter++;
    if (rx_bits_iter == BYTE_LEN) {
      rx_buffer[rx_iter] = _bits_byte(rx_bits_buffer);
#ifdef DEBUG_RX
      Serial.print("OK, 1 byte: ");Serial.println(rx_buffer[rx_iter]);
#endif
      rx_bits_iter = 0;
      if ((rx_iter == 2) && (rx_buffer[1] == tx_buffer[2]) && (rx_buffer[2] == tx_buffer[1]) && (tx_len > 0)) { // if there is something to transmit to the transmitting node
        if (phy_state != PHY_TX_RX) { // do not reset tx_iter if already in PHY_TX_RX
          tx_iter = 0;
          phy_state = PHY_PREAMBLE_TX;
        }
        return;
      } else if (rx_iter == 4) { // get rx_len
        rx_len = rx_buffer[rx_iter];
      }
      if (rx_iter > 4) { // rx_len has been received
        if ((rx_iter == rx_len) && (phy_state != PHY_TX_RX)) { // finished receiving
#ifdef DEBUG
          Serial.println("LEN, PHY_IDLE");
#endif
          phy_state = PHY_IDLE;
        }
      }
      rx_iter++;
    }
  }
  if ((rx_iter > MAX_PHY_BUFFER) && (phy_state != PHY_TX_RX)) { // rx_len is corrupted, still in rx even if no edge is found after 1 period
#ifdef DEBUG
    Serial.println("corrupt, PHY_IDLE");
#endif
    phy_state = PHY_IDLE;
  }
}

// send preamble before entering tx_rx
void _phy_preamble_tx() {
#ifdef DEBUG_PRE_TX
  Serial.print("PRETX: ");Serial.println(preamble_iter);
#endif
  if ((preamble_iter & 0x01) == 0) {
    tx_bit_buffer = LOW;
  } else {
    tx_bit_buffer = HIGH;
  }
  preamble_iter++;
  if (preamble_iter == PREAMBLE_LEN) { // preamble finished
    preamble_iter = 0;
    tx_iter = 0;
#ifdef DEBUG
    Serial.println("TX_RX");
#endif
    phy_state = PHY_TX_RX;
  }
}

// update tx_bit_buffer and increment tx_iter
// meanwhile, still receive data (full duplex)
void _phy_tx_rx() {
#ifdef DEBUG_TX
  Serial.print("TXRX: ");Serial.print(tx_bits_buffer[tx_bits_iter]);Serial.print(", tbi: ");Serial.print(tx_bits_iter);Serial.print(", tb: ");Serial.print(tx_buffer[tx_iter]);Serial.print(", tx_iter: ");Serial.println(tx_iter);
#endif
  if (tx_bits_iter == 0) { // start of a byte, convert byte to bits
    _byte_bits(tx_buffer[tx_iter], tx_bits_buffer);
    if (tx_iter == tx_len) { // finished transmission
      tx_len = 0;
#ifdef DEBUG
      Serial.println("IDLE");
#endif
      phy_state = PHY_IDLE;
      tx_bit_buffer = LOW;
#ifdef DEBUG
      Serial.println("FINISHED TRANSMISSION");
#endif
      return;
    }
  }
  if (tx_bits_buffer[tx_bits_iter] == 0) { // transmit according to tx_buffer
    tx_bit_buffer = LOW;
  } else {
    tx_bit_buffer = HIGH;
  }
  tx_bits_iter++;
  if (tx_bits_iter == BYTE_LEN) { // restart after transmit one byte
    tx_bits_iter = 0;
    tx_iter++;
  }
//  _phy_preamble_rx(); // receive data
}

// transform bit array to byte
// LSB is at index 0
uint8_t _bits_byte(uint8_t *bits) {
  uint8_t sum = 0;
  for (uint8_t i = 0; i < BYTE_LEN; i++) {
    sum += bits[i] << i;
  }
  return sum;
}

void _byte_bits(uint8_t tx_byte, uint8_t *bits) {
  for (uint8_t i = 0; i < BYTE_LEN; i++) {
    bits[i] = (tx_byte >> i) & 0x01;
  }
}

void _initialize_timer()
{
  noInterrupts();
  TCCR2A = 0x00;                // Timer2 disable interrupt
  TCCR2B = 0x00;                // Timer2 disable interrupt
  OCR2A = TIMER2COUNT;          // Timer2 compare match value
  TCCR2A |= (1 << WGM21);       // Timer2 Control Reg A: Set to CTC mode
  TCCR2B = _BV(CS22);           // Timer2 Control Reg B: Timer Prescaler set to 64
  TIMSK2 |= (1 << OCIE2A);      // Timer2 INT Reg: Timer2 CTC Interrupt Enable
  interrupts();
}
