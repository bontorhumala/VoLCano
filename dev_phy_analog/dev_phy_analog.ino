// By Bontor Humala
// PHY layer of Volcano, default is off (idle)
// Log:
// - initial version, use ADC to sample

#include <TimerOne.h>

#define PHY_IDLE 0
#define PHY_RX 1
#define PHY_TX_RX 2
#define PHY_SAMPLE_PERIOD 200 // phy sensing (sampling) period
#define PHY_PULSE_PERIOD 1000 // pulse width as well
#define PHY_SAFE_IDLE 5*PHY_PULSE_PERIOD // minimum idle pulse period to ensure it is safe to transmit
#define MAX_PHY_BUFFER 263 // 263 (maximum MAC packet), assume no additional PHY bytes
#define ACK_PHY_BUFFER 8 // 8 (ACK in MAC), assume no additional PHY bytes
#define BIT_LEN 2
#define BYTE_LEN 8
#define PULSE_WINDOW_LEN (PHY_PULSE_PERIOD/PHY_SAMPLE_PERIOD) // sampling window size, also used to indicate new pulse
#define EDGE_THRESHOLD 100 // minimum range in PULSE_WINDOW_LEN to be considered an edge 

// rx tx buffer and iterator
uint8_t rx_buffer[MAX_PHY_BUFFER];
uint8_t tx_buffer[MAX_PHY_BUFFER];
uint16_t rx_iter;
uint16_t tx_iter;
// maximum iteration for rx_buffer and tx_buffer
uint16_t rx_len;
uint16_t tx_len;
// encoding buffer and iterator
uint8_t encode_buffer[BIT_LEN];
uint8_t encode_iter;
// decoding bit to byte for rx_buffer
uint8_t decode_buffer[BYTE_LEN];
uint8_t decode_iter;
// update sampling_buffer every PHY_SAMPLE_PERIOD
// update tx pin every pulse period
uint16_t sampling_buffer[PULSE_WINDOW_LEN];
uint8_t pulse_iter;
uint8_t max_pulse_len;
// safe idle pulse period
uint8_t idle_counter;
// time of last detected edge
unsigned long prev_edge;

// PUBLIC
bool phy_sense();
uint8_t phy_rx(uint8_t *data);
bool phy_tx(uint8_t *data, uint8_t count);

// PRIVATE
void _phy_update();
void _phy_idle();
void _phy_rx();
void _phy_tx_rx();
void _phy_fsm_control();
void _encode_zero();
void _encode_one();
void _encode_idle();
int8_t _detect_edge();
uint16_t _get_min(uint8_t *arr, uint8_t len)
uint16_t _get_max(uint8_t *arr, uint8_t len)
uint8_t _bits_byte(uint8_t *bits);

void setup() {
  // put your setup code here, to run once:
  phy_state = PHY_IDLE;
  pinMode(tx_pin, OUTPUT);
  digitalWrite(tx_pin, HIGH);
  rx_iter = 0;
  tx_iter = 0;
  rx_len = 0;
  tx_len = 0;
  encode_iter = 0; // alternated between 0 and 1, over encode_buffer
  decode_iter = 0; // alternated between 0 and 1, over decode_buffer
  pulse_iter = 0; // counts until PULSE_WINDOW_LEN to update tx output
  tx_encode_iter = 0; // counts until 2*PULSE_WINDOW_LEN to update encode_buffer
  max_pulse_len = PULSE_WINDOW_LEN; // update tx pin and detect adge
  max_tx_encode_len = BIT_LEN*PULSE_WINDOW_LEN; // update encode_buffer
  idle_counter = 0;
  prev_edge = 0;
  Timer1.initialize(PHY_SAMPLE_PERIOD);
  Timer1.attachInterrupt(_phy_fsm_control, PHY_SAMPLE_PERIOD);
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
int8_t phy_rx(uint8_t *data) {
  if ((rx_iter != 0) || (rx_len == 0)) { // rx_iter is not finished yet OR rx_len is not updated
    return -1;
  }
  uint16_t rx_byte_len = rx_len;
  memcpy(data, rx_buffer, MAX_PHY_BUFFER);
  rx_iter = 0;
  return rx_byte_len;
}

// data contains frame to send, will be transferred to tx_buffer
void phy_tx(uint8_t *data, uint8_t count) {
  memcpy(tx_buffer, data, count);
  tx_len = count;
  tx_iter = 0;
}

// sample rx pin every PHY_SAMPLE_PERIOD
// update tx pin every PULSE_WINDOW_LEN
void _phy_update() {
  sampling_buffer[pulse_iter] = analogRead(rx_pin); // sample rx_pin
  pulse_iter++;
  if (pulse_iter == max_pulse_len) { // update tx_pin
    digitalWrite(tx_pin, encode_buffer[encode_iter & 0x01]);
    encode_iter++;
    pulse_iter = 0;
    if (phy_state == PHY_IDLE) {
      idle_counter++;
    }
  }
}

// if find an edge, go to rx
// if there is something to send, go to tx
uint8_t _phy_idle() {
  rx_iter = 0;
  _encode_idle();
  uint8_t in_bit;
  in_bit = _detect_edge();
  if (in_bit > -1) { // check incoming bit
    decode_buffer[decode_iter & 0x01] = in_bit;
    decode_iter++;
    idle_counter = 0; // reset idle_counter to hold back transmission
    rx_len = 0; // reset rx_buffer
    phy_state = PHY_RX;
  }
  if ((idle_counter > PHY_SAFE_IDLE) && (tx_len > 0)) { // check tx buffer and ensure safe to send
    tx_iter = 0;
    phy_state = PHY_TX_RX;
  }
}

// update rx_buffer and increment rx_iter
// rx_len is in dataframe[4]
uint8_t _phy_rx() {
  in_bit = _detect_edge();
  if (in_bit > -1) { // check incoming bit
    decode_buffer[decode_iter & 0x01] = in_bit;
    decode_iter++;
    idle_counter = 0; // reset idle_counter to hold back transmission
    phy_state = PHY_RX;
  }
  if (decode_iter == BYTE_LEN) {
    rx_buffer[rx_iter] = _bits_byte(decode_buffer);
    decode_iter = 0;
    if ((rx_iter == 1) && (rx_buffer[rx_iter] == tx_buffer[1]) && (tx_len > 0)) { // if there is something to transmit to the transmitting node
      tx_iter = 0;
      phy_state = PHY_TX_RX;
    } else if (rx_iter == 4) { // get rx_len
      rx_len = rx_buffer[rx_iter];
    }   
  }
  if (rx_iter > 4) { // rx_len has been received
    if (rx_iter == rx_len) { // finished receiving
      phy_state = PHY_IDLE;
    }
  }
  rx_iter++;
}

// update encode_buffer and increment tx_iter
// meanwhile, still receive data (full duplex)
uint8_t _phy_tx_rx() {
  tx_encode_iter++
  if (tx_encode_iter == max_tx_encode_len) { // need to update encode_buffer after 2 pulse
    tx_encode_iter = 0;
    if (tx_buffer[tx_iter] == 0) { // transmit according to tx_buffer
      _encode_zero();
    } else {
      _encode_one();
    }
  }
  if (tx_iter == tx_len) { // finished transmission
    tx_len = 0;
    phy_state = PHY_IDLE;
  } else {
    tx_iter++;
  }
  _phy_rx(); // receive data
}

// called every SAMPLING_WINDOW_LEN, by _phy_update()
void _phy_fsm_control() {
  _phy_update();
  switch(phy_state){
    case PHY_IDLE:
      _phy_idle();
      break;
    case PHY_RX:
      _phy_rx();
      break;
    case PHY_TX_RX:
      _phy_tx_rx();
      break;
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

// detect rising (1) or falling (0) edge
// avoid transistional edge between two consecutive identical bit: 
//   i.e. 00: HIGH (falling) LOW (rising) HIGH (falling) LOW -> falling, falling
//   ensure that the edge is detected after one PHY_PULSE_PERIOD
int8_t _detect_edge() {
  int8_t in_bit = -1;
  uint16_t max_sample = _get_max(sampling_buffer, PHY_PULSE_PERIOD);
  uint16_t min_sample = _get_min(sampling_buffer, PHY_PULSE_PERIOD);  
  if ((max_sample-min_sample) > EDGE_THRESHOLD) { // the range is big, must have been an edge
    if ((micros()-prev_edge) > PHY_PULSE_PERIOD) { // no edge in vicinity, must be considered
      if ((sampling_buffer[PULSE_WINDOW_LEN-1]-sampling_buffer[0]) < 0) { // falling edge, buffer 0-th is larger
        in_bit = 0;
      } else {
        in_bit = 1;
      }
      prev_edge = micros();
    }
  }
  return in_bit;
}

// get maximum value in an array
uint16_t _get_max(uint8_t *arr, uint8_t len) {
  uint16_t max_val = 0;
  for (uint8_t i=0; i<len; i++) {
    if (arr[i]>max_val) {
      max_val = arr[i];
    }
  }
  return max_val;
}

// get minimum value in an array
uint16_t _get_min(uint8_t *arr, uint8_t len) {
  uint16_t min_val = 9999;
  for (uint8_t i=0; i<len; i++) {
    if (arr[i]<min_val) {
      min_val = arr[i];
    }
  }
  return min_val;
}

// transform bit array to byte
// LSB is at index 0
uint8_t _bits_byte(uint8_t *bits) {
  uint8_t sum = 0;
  for(uint8_t i = 0; i < BYTE_LEN; i++) {
    sum += bits[i] << i;
  }
  return sum;

}
