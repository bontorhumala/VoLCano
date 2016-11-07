#include <TimerOne.h>

#define PHY_SYNC 0
#define PHY_IDLE 1
#define PHY_RX 2
#define PHY_TX_RX 3
#define PHY_UPDATE_PERIOD 50 // phy sensing (sampling) period
#define PHY_PULSE_PERIOD 500 // pulse period as well as FSM update period
#define MAX_PHY_BUFFER 263 // 263 (maximum MAC packet), assume no additional PHY bytes
#define ACK_PHY_BUFFER 8 // 8 (ACK in MAC), assume no additional PHY bytes
#define SYMBOL_LEN 2
#define PULSE_WINDOW_LEN PHY_PULSE_PERIOD/PHY_UPDATE_PERIOD

uint8_t rx_buffer[MAX_PHY_BUFFER];
uint8_t tx_buffer[MAX_PHY_BUFFER];
uint8_t encode_buffer[BIT_LEN];
uint8_t encode_iter;
uint8_t decode_buffer[BIT_LEN];
uint8_t sampling_buffer[SYMBOL_LEN*PULSE_WINDOW_LEN];
uint16_t rx_iter;
uint16_t tx_iter;
uint16_t rx_len;
uint16_t tx_len;
uint8_t sampling_iter;
uint8_t pulse_counter;
uint8_t max_pulse_len;
uint8_t max_sampling_len;

// PUBLIC
bool phy_sense();
uint8_t phy_rx(uint8_t *data);
bool phy_tx(uint8_t *data, uint8_t count);

// PRIVATE
void _phy_update();
void _phy_rx();
void _phy_tx_rx();
void _phy_fsm_control();
void _shift_forward();
void _shift_backward();
void _encode_zero();
void _encode_one();
int8_t _get_consistency(uint8_t *buffer);

void setup() {
  // put your setup code here, to run once:
  phy_state = PHY_IDLE;
  pinMode(rx_pin, INPUT);
  digitalWrite(rx_pin, HIGH);
  pinMode(tx_pin, OUTPUT);
  digitalWrite(tx_pin, HIGH);
  rx_iter = 0;
  tx_iter = 0;
  rx_len = 0;
  tx_len = 0;
  encode_iter = 0; // alternated between 0 and 1, over encode_buffer
  idle_bit = 0; // alternated between 0 and 1
  pulse_iter = 0; // counts until PULSE_WINDOW_LEN to update tx output
  sampling_iter = 0; // counts until 2*PULSE_WINDOW_LEN to execute fsm and check sampling_buffer
  max_pulse_len = PULSE_WINDOW_LEN; // update tx pin
  max_sampling_len = SYMBOL_LEN*PULSE_WINDOW_LEN; // call fsm, check sampling_buffer
  Timer1.initialize(PHY_UPDATE_PERIOD);
  Timer1.attachInterrupt(_phy_update, PHY_UPDATE_PERIOD);
}

void loop() {
  // put your main code here, to run repeatedly:

}

// increase pulse_iter so that tx is updated earlier
void _shift_forward() {
  pulse_iter++;
}

// decrease pulse_iter so that tx is updated later
void _shift_backward();
  pulse_iter--;
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
uint8_t phy_rx(uint8_t *data) {
  uint16_t rx_len = rx_iter;
  memcpy(data, rx_buffer, MAX_PHY_BUFFER);
  rx_iter = 0;
  return rx_len;
}

// data contains frame to send, will be transferred to tx_buffer
void phy_tx(uint8_t *data, uint8_t count) {
  memcpy(tx_buffer, data, count);
  tx_len = count;
  tx_iter = 0;
}

// sample rx pin every PHY_UPDATE_PERIOD
// update tx pin and execute fsm every PULSE_WINDOW_LEN
void _phy_update() {
  sampling_buffer[sampling_iter] = digitalRead(rx_pin); // sample rx_pin
  sampling_iter++;
  if (sampling_iter == max_sampling_len) { // reset sampling_buffer
    sampling_iter = 0;
  }
  pulse_iter++;
  if (pulse_iter == max_pulse_len) { // update tx_pin and execute fsm
    _phy_fsm_control();
    digitalWrite(tx_pin, encode_buffer[encode_iter & 0x01]);
    encode_iter++;
    pulse_iter = 0;
  }
}

// perform synchronization every PULSE_WINDOW_LEN
void _phy_sync() {
  _transmit_idle();
  // synchronization
  int8_t consistency;
  consistency = _get_consistency(sampling_buffer);
  if (consistency > 0) { // too fast
    _shift_backward();
  } else if (consistency < 0) { // too slow
    _shift_forward();
  } else { // consistent, move to IDLE
    if (encode_buffer[encode_iter & 0x01] != sampling_buffer[max_sampling_len-1]) { // aligned, last rx is different than current tx
      phy_state = PHY_IDLE;
    } else { // totally off-phase, just split direction
      if (encode_buffer[encode_iter & 0x01]) {
        _shift_forward();
      } else{
        _shift_backward();
      }
    }
  }
}

// idle, keeps on transmitting idle (to help new node to adapt) 
// if found 0xFF, go to rx
// if there is something to send, go to tx
uint8_t _phy_idle() {
  _transmit_idle();
  if (tx_len > 0) { // check tx buffer
    phy_state = PHY_TX_RX;
  }
  // wait for 0xFF, how to separate idle and tx?
}

// decode rx signal
// update rx_buffer and increment rx_iter
// rx_len is in dataframe[4]
uint8_t _phy_rx() {
  if (rx_iter == rx_len) { // finished receiving
    rx_len = 0;
    phy_state = PHY_IDLE;
  }
  else {
    rx_iter++;
  }
}

// update encode_buffer and increment tx_iter
// meanwhile, still receive data (full duplex)
uint8_t _phy_tx_rx() {
  if (tx_buffer[tx_iter] == 0) { // transmit according to tx_buffer
    _encode_zero();
  } else {
    _encode_one();
  }
  if (tx_iter == tx_len) { // finished transmission
    tx_len = 0;
    phy_state = PHY_IDLE;
  }
  else {
    tx_iter++;
  }
  _phy_rx(); // receive data
}

// called every SAMPLING_WINDOW_LEN, by _phy_update()
void _phy_fsm_control() {
  switch(phy_state){
    case PHY_SYNC:
      _phy_sync();
      break;
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

// returns 0 if consistent, -1 if too slow, 1 if too fast
int8_t _get_consistency(uint8_t *sample) {
  bool inconsistent = false;
  uint8_t i = 0;
  uint8_t max_sampling = max_sampling_len-1;
  while (!inconsistent && (i<max_sampling)) {
    inconsistent = sample[i] ^ sample[i+1];
    i++;
  }
  if (!inconsistent) {
    return 1;
  } 
  if (i<max_pulse_len) { // inconsistency happens early, too fast
    return 1;
  } else { // inconsistency happens late, too slow
    return -1;
  }
}

void _transmit_idle() {
  // transmit idle_symbols
  if (sampling_iter == 0) {
    if ((idle_bit & 0x01) == 0) {
      _encode_zero();
    } else {
      _encode_one();
    }
    idle_bit++;
  }
}
