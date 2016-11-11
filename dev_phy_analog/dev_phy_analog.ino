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
//   able to tx properly, detect edges, but every bit is detected as 1. need preamble (i.e. starting bit must be 0 -- rising edge)
// 11/10
//   (ok) adding preamble 01..01, according to PREAMBLE_LEN
//   (ok) fix edge detection, previously assigning in_bit always 1 because variable is uint (never negative thus never 0)
//   (ok) fix edge detection, edge is only considered if 0-th bit and PULSE_LEN-th bit exceeds EDGE_THRESHOLD
// 11/11
//   (?) how to merge preamble_rx and rx inside phy_tx_rx?

#include <stdint.h>

#define SERIAL_PLOT
//#define DEBUG
//#define DEBUG_RX
//#define DEBUG_TX
#define RX_NODE
//#define TX_NODE
#define TEST_TX 0b10100101

#define PHY_IDLE 0
#define PHY_RX 1
#define PHY_TX_RX 2
#define PHY_PREAMBLE_RX 3
#define PHY_PREAMBLE_TX 4
#define PHY_SAMPLE_PERIOD 500 // phy sensing (sampling) period
#define PHY_PULSE_WIDTH 5000 // pulse width
#define TIMER2COUNT 192 // Timer2 runs at 16MHz, in order to interrupt every 500us -> count 63x
#define PHY_SAFE_IDLE 5*PERIOD_LEN // minimum idle pulse period to ensure it is safe to transmit
#define MAX_PHY_BUFFER 263 // 263 (maximum MAC packet), assume no additional PHY bytes
#define ACK_PHY_BUFFER 8 // 8 (ACK in MAC), assume no additional PHY bytes
#define BIT_LEN 2
#define PREAMBLE_LEN 2
#define BYTE_LEN 8
#define PULSE_LEN (PHY_PULSE_WIDTH/PHY_SAMPLE_PERIOD) // pulse window size, also used to indicate new pulse
#define PERIOD_LEN (BIT_LEN*PULSE_LEN) // ppm window size (2 consecutive pulse)
#define NO_EDGE_PERIOD_LEN 2*PERIOD_LEN // maximum no edge is 2*period
#define EDGE_THRESHOLD 10 // minimum range in PULSE_WINDOW_LEN to be considered an edge 
#define NEG_EDGE_THRESHOLD -10
#define MID_BIT (PULSE_LEN>>1)

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
// encoding buffer and iterator
uint8_t encode_buffer[BIT_LEN];
uint8_t encode_iter;
// bit array representation of byte in tx_buffer
uint8_t tx_bits_buffer[BYTE_LEN];
uint8_t tx_bits_iter;
// decoding bit to byte for rx_buffer
uint8_t decode_buffer[BYTE_LEN];
uint8_t decode_iter;
// update sampling_buffer every PHY_SAMPLE_PERIOD
// update tx pin every pulse period
uint16_t sampling_buffer[PULSE_LEN];
uint8_t sampling_iter;
uint8_t pulse_iter;
// safe idle pulse period
uint8_t idle_counter;
// time of last detected edge
unsigned long prev_edge;
uint8_t phy_state;
// no edge counter
uint8_t no_edge_count;

uint8_t tx_pin;
uint8_t rx_pin;
unsigned long timestamp;
// PUBLIC
bool phy_sense();
int16_t phy_rx(uint8_t *data);
void phy_tx(uint8_t *data, uint16_t data_len);

// PRIVATE
void _phy_update();
void _phy_idle();
void _phy_rx();
void _phy_tx_rx();
void _phy_preamble_rx();
void _phy_preamble_tx();
void _phy_fsm_control();
void _encode_zero();
void _encode_one();
void _encode_idle();
int8_t _detect_edge();
uint16_t _get_min(uint8_t *arr, uint8_t len);
uint16_t _get_max(uint8_t *arr, uint8_t len);
uint8_t _bits_byte(uint8_t *bits);
void _byte_bits(uint8_t tx_byte, uint8_t *bits);
void _push_sampling_buffer(uint16_t data);
void _empty_array(uint8_t *buff, uint8_t len);

#ifdef RX_NODE // address is 0x88
uint8_t test_rx[1];
#endif
#ifdef TX_NODE // address is 0x77
uint8_t test_tx[1] = {TEST_TX};
#endif

ISR(TIMER2_OVF_vect)
{
  _phy_fsm_control();
  TCNT2 = TIMER2COUNT;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
#ifdef RX_NODE
//  Serial.println("RX Node\n");
#endif
#ifdef TX_NODE
//  Serial.println("TX Node\n");
#endif
  phy_state = PHY_IDLE;
  tx_pin = A1;
  rx_pin = A0;
  pinMode(tx_pin, OUTPUT);
  _initialize_timer();

#ifdef RX_NODE
  int16_t test_rx_size = 0;
  do {
    test_rx_size = phy_rx(test_rx);
    if (test_rx_size > -1) {
//      Serial.print("rx: ");
      for (int i = 0; i < test_rx_size; i++) {
//        Serial.println(test_rx[i]);
      }
//      Serial.println("\nfinished rx \n");
    }
  } while (test_rx_size == -1);
#endif
#ifdef TX_NODE
//  Serial.println("Transmitting\n");
  phy_tx(test_tx, 1);
#endif
}

void loop() {
  // put your main code here, to run repeatedly:

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

// sample rx pin every PHY_SAMPLE_PERIOD
// update tx pin every PULSE_WINDOW_LEN
void _phy_update() {
  _push_sampling_buffer(analogRead(rx_pin)); // sample rx_pin
  if (pulse_iter == 0) { // update tx_pin
#ifdef DEBUG_TX
    Serial.print("eb0: "); Serial.print(encode_buffer[0]);Serial.print("eb1: "); Serial.print(encode_buffer[1]);Serial.print("ei: ");Serial.print(encode_iter & 0x01);Serial.print(", tx: "); Serial.println(encode_buffer[encode_iter & 0x01]);
#endif
    digitalWrite(tx_pin, encode_buffer[encode_iter & 0x01]);
    encode_iter++;
    if (phy_state == PHY_IDLE) {
      idle_counter++;
    }
  }
  pulse_iter++;
}

// if find an edge, go to rx
// if there is something to send, go to tx
void _phy_idle() {
  rx_iter = 0;
  _encode_idle();
  int8_t in_bit;
  in_bit = _detect_edge();
  if (in_bit > -1) { // check incoming bit
    idle_counter = 0; // reset idle_counter to hold back transmission
#ifdef DEBUG   
    Serial.println("PRE_RX");
#endif
    phy_state = PHY_PREAMBLE_RX;
    return;
  }
  if ((idle_counter > PHY_SAFE_IDLE) && (tx_len > 0) && (pulse_iter == PULSE_LEN)) { // check tx buffer and ensure safe to send
    encode_iter = 0;
#ifdef DEBUG
    Serial.println("PRE_TX");
#endif
    phy_state = PHY_PREAMBLE_TX;
    return;
  }
}

void _phy_preamble_rx() {
  int8_t in_bit;
  bool is_preamble = true;
  uint8_t i = 0;
  in_bit = _detect_edge();
  if (in_bit > -1) { // check incoming bit
    if (preamble_iter < PREAMBLE_LEN) { // receive preamble bits
      preamble_buffer[preamble_iter] = in_bit;
      preamble_iter++;
    }
    else if (preamble_iter == PREAMBLE_LEN) { // check all preamble bits
#ifdef DEBUG_RX
      Serial.print("pre: ");
#endif
      while ((i < PREAMBLE_LEN) && is_preamble) {
#ifdef DEBUG_RX
        Serial.print(preamble_buffer[i]);
#endif
        if (preamble_buffer[i] != (i & 0x01)) {// preamble should be 0101...01 (depends on PREAMBLE_LEN)
          is_preamble = false;
        }
        i++;
      }
      if (is_preamble) {
#ifdef DEBUG_RX
        Serial.println("ok, PHY_RX");
#endif
        decode_buffer[decode_iter & 0x01] = in_bit;
        decode_iter++;
        idle_counter = 0; // reset idle_counter to hold back transmission
        rx_len = 0; // reset rx_buffer
        phy_state = PHY_RX;
        no_edge_count = 0;
        return;
      }
      else {
#ifdef DEBUG_RX
        Serial.println("bad, IDLE");
#endif
        phy_state = PHY_IDLE;
        preamble_iter = 0;
        _empty_array(preamble_buffer, PREAMBLE_LEN);
      }
    }
    else {
#ifdef DEBUG
      Serial.println("strange, IDLE");
#endif
      phy_state = PHY_IDLE;
      preamble_iter = 0;
      _empty_array(preamble_buffer, PREAMBLE_LEN);
    }
  }
  if ((no_edge_count > NO_EDGE_PERIOD_LEN) && (phy_state != PHY_TX_RX)) { // rx_len is corrupted, still in rx even if no edge is found after 1 period
#ifdef DEBUG
    Serial.println("corrupt, PHY_IDLE");
#endif
    phy_state = PHY_IDLE;
  }
}

// update rx_buffer and increment rx_iter
// rx_len is in dataframe[4]
void _phy_rx() {
  int8_t in_bit;
  in_bit = _detect_edge();
  if (in_bit > -1) { // check incoming bit
    decode_buffer[decode_iter] = in_bit;
    decode_iter++;
    if (decode_iter == BYTE_LEN) {
      rx_buffer[rx_iter] = _bits_byte(decode_buffer);
      decode_iter = 0;
      if ((rx_iter == 1) && (rx_buffer[rx_iter] == tx_buffer[1]) && (tx_len > 0)) { // if there is something to transmit to the transmitting node
        if (phy_state != PHY_TX_RX) { // do not reset tx_iter if already in PHY_TX_RX
          tx_iter = 0;
          encode_iter = 0;
          phy_state = PHY_PREAMBLE_TX;
        }
        return;
      } else if (rx_iter == 4) { // get rx_len
        rx_len = rx_buffer[rx_iter];
      } else {
#ifdef DEBUG
        Serial.print("OK, 1 byte: ");Serial.println(rx_buffer[rx_iter]);
#endif
      }
    }
    idle_counter = 0; // reset idle_counter to hold back transmission
  } 
  if (rx_iter > 4) { // rx_len has been received
    if ((rx_iter == rx_len) && (phy_state != PHY_TX_RX)) { // finished receiving
#ifdef DEBUG
      Serial.println("LEN, PHY_IDLE");
#endif
      phy_state = PHY_IDLE;
    }
  }
  if ((no_edge_count > NO_EDGE_PERIOD_LEN) && (phy_state != PHY_TX_RX)) { // rx_len is corrupted, still in rx even if no edge is found after 1 period
#ifdef DEBUG
    Serial.println("corrupt, PHY_IDLE");
#endif
    phy_state = PHY_IDLE;
  }
  rx_iter++;
}

// send preamble before entering tx_rx
void _phy_preamble_tx() {
#ifdef DEBUG_TX
  Serial.print("PRETX: ");Serial.print(encode_iter & 0x01);Serial.print(", prei: ");Serial.print(preamble_iter);Serial.print(", puli: ");Serial.println(pulse_iter);
#endif
  if (((encode_iter & 0x01) == 0) && (pulse_iter == PULSE_LEN)) { // need to update encode_buffer at the end of encode_buffer
    if (preamble_iter == 0) {
      _encode_zero();
    } else {
      _encode_one();
    }
    preamble_iter++;
  }
  if (preamble_iter == PREAMBLE_LEN) { // preamble finished
    preamble_iter = 0;
    tx_iter = 0;
    encode_iter = 0;
#ifdef DEBUG
    Serial.println("TX_RX");
#endif
    phy_state = PHY_TX_RX;
  }
}

// update encode_buffer and increment tx_iter
// meanwhile, still receive data (full duplex)
void _phy_tx_rx() {
#ifdef DEBUG_TX
  Serial.print("TXRX: ");Serial.print(encode_iter & 0x01);Serial.print(", tx_bits_iter: ");Serial.print(tx_bits_iter);Serial.print(", tx_iter: ");Serial.print(tx_iter);Serial.print(", puli: ");Serial.println(pulse_iter);
#endif
  if (((encode_iter & 0x01) == 0) && (pulse_iter == PULSE_LEN)) { // need to update encode_buffer at the start of a bit (end of encode_buffer)
    if (tx_bits_iter == 0) { // start of a byte, convert byte to bits
      _byte_bits(tx_buffer[tx_iter], tx_bits_buffer);
      if (tx_iter == tx_len) { // finished transmission
        tx_len = 0;
#ifdef DEBUG
        Serial.println("IDLE");
#endif
        phy_state = PHY_IDLE;
#ifdef DEBUG
        Serial.println("FINISHED TRANSMISSION");
#endif
        return;
      }
      tx_iter++;
    }
    if (tx_bits_buffer[tx_bits_iter] == 0) { // transmit according to tx_buffer
      _encode_zero();
    } else {
      _encode_one();
    }
    tx_bits_iter++;
    if (tx_bits_iter == BYTE_LEN) { // restart after transmit one byte
      tx_bits_iter = 0;
    }
  }
//  _phy_preamble_rx(); // receive data
}

// called every PHY_SAMPLE_PERIOD
void _phy_fsm_control() {
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
  if (pulse_iter == PULSE_LEN) {
    pulse_iter = 0;
  }  
}

// idle
void _encode_idle() {
  encode_buffer[0] = LOW;
  encode_buffer[1] = LOW;
}

// rising edge
void _encode_one() {
  encode_buffer[0] = LOW;
  encode_buffer[1] = HIGH;
}

// falling edge
void _encode_zero() {
  encode_buffer[0] = HIGH;
  encode_buffer[1] = LOW;
}

// detect rising (1) or falling (0) edge using range in the sampling_buffer
// avoid transistional edge between two consecutive identical bit:
//   i.e. 00: HIGH (falling) LOW (rising) HIGH (falling) LOW -> falling, falling
//   ensure that the edge is detected after one PHY_PULSE_WIDTH
int8_t _detect_edge() {
  int8_t in_bit = -1;
  int sample_diff = sampling_buffer[MID_BIT] - sampling_buffer[MID_BIT-1];
  if (sample_diff < NEG_EDGE_THRESHOLD) { // falling edge
    if (no_edge_count >= 8) { // no edge in vicinity (i.e. not a repetition or transition)
      in_bit = 0;
      no_edge_count = 0;
    }
  } else if (sample_diff > EDGE_THRESHOLD) { // positive edge
    if (no_edge_count >= 8) {
      in_bit = 1;
      no_edge_count = 0;
    }
  }
#ifdef DEBUG_RX
  if ((phy_state == PHY_RX) || (phy_state == PHY_PREAMBLE_RX)) {
    Serial.print("sb: ");
    for (uint8_t j = 0; j < PULSE_LEN; j++) {
      Serial.print(sampling_buffer[j]); Serial.print(", ");
    }
    Serial.print("\nde: "); Serial.println(in_bit);
  }
  else {
    Serial.print(in_bit);
  }
#endif
#ifdef SERIAL_PLOT
  if ((phy_state == PHY_RX) || (phy_state == PHY_PREAMBLE_RX)) {
    Serial.print(sample_diff);
    Serial.print(",");
    Serial.println(in_bit);
  }
#endif
  if (in_bit == -1) {
    no_edge_count++;
  }
  return in_bit;
}

// get maximum value in an array
uint16_t _get_max(uint16_t *arr, uint8_t len) {
  uint16_t max_val = 0;
  for (uint8_t i = 0; i < len; i++) {
    if (arr[i] > max_val) {
      max_val = arr[i];
    }
  }
  return max_val;
}

// get minimum value in an array
uint16_t _get_min(uint16_t *arr, uint8_t len) {
  uint16_t min_val = 9999;
  for (uint8_t i = 0; i < len; i++) {
    if (arr[i] < min_val) {
      min_val = arr[i];
    }
  }
  return min_val;
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

// FIFO buffer push implementation for sampling_buffer
void _push_sampling_buffer(uint16_t data) {
  if (sampling_iter == PULSE_LEN) {
    for (uint8_t i = 0; i < PULSE_LEN - 1; i++) { // kick earlist sample at index 0 and shift the remaining
      sampling_buffer[i] = sampling_buffer[i + 1];
    }
    sampling_buffer[PULSE_LEN - 1] = data; // add new data to last position in the buffer
  } else {
    sampling_buffer[sampling_iter] = data;
    sampling_iter++;
  }
}

void _empty_array(uint8_t *buff, uint8_t len) {
  for (uint8_t i = 0; i < len; i++) {
    buff[i] = 0;
  }
}

void _initialize_timer()
{
  noInterrupts();
  TCCR2B = 0x00;        //Disbale Timer2 while we set it up
  TCNT2  = TIMER2COUNT;         //Reset Timer Count to 130 out of 255
  TIFR2  = 0x00;        //Timer2 INT Flag Reg: Clear Timer Overflow Flag
  TIMSK2 = 0x01;        //Timer2 INT Reg: Timer2 Overflow Interrupt Enable
  TCCR2A = 0x00;        //Timer2 Control Reg A: Wave Gen Mode normal
  TCCR2B = 0x05;        //Timer2 Control Reg B: Timer Prescaler set to 128
  interrupts();
}
