pt#include <TimerOne.h>
#include <PinChangeInt.h>
#include <PinChangeIntConfig.h>

#define PHY_IDLE 0
#define PHY_RX 1
#define PHY_TX_RX 2
#define PHY_PULSE_PERIOD 500 // pulse period as well as FSM update period
#define MAX_PHY_BUFFER 263 // 263 (maximum MAC packet), assume no additional PHY bytes
#define ACK_PHY_BUFFER 8 // 8 (ACK in MAC), assume no additional PHY bytes
#define BIT_SYMBOL_LEN 2 // 0: HIGH LOW, 1: LOW, HIGH
#define SYNC_BIT_LEN 2 // in edge detection method, only need to ensure it is 01
#define PPM_SYNC_OK 0 // okay, in sync
#define PPM_SYNC_INC 1 // not sync, needs faster sampling

uint8_t rx_buffer[MAX_PHY_BUFFER];
uint8_t tx_buffer[MAX_PHY_BUFFER];
uint8_t encode_buffer[BIT_SYMBOL_LEN];
uint8_t idle_buffer[SYNC_BIT_LEN]; // for synchronization
uint16_t rx_iter;
uint16_t tx_iter;
uint16_t tx_len;
uint8_t encode_iter; // 0 or 1
uint8_t fsm_counter;
uint8_t max_fsm_counter;

// PUBLIC
bool phy_sense();
uint8_t phy_rx(uint8_t *data);
bool phy_tx(uint8_t *data, uint8_t count);

// PRIVATE
void _phy_generate_pulse();
void _phy_idle();
void _phy_rx();
void _phy_tx_rx(uint8_t bit_to_send);
void _phy_fsm_control();
void _shift_forward();
void _encode_one(); // set encode_buffer to LOW HIGH
void _encode_zero(); // set encode_buffer to HIGH LOW
void _int_one(); // detect rising edge -> 1
void _int_zero(); // detect falling edge -> 0

void setup() {
  // put your setup code here, to run once:
  phy_state = PHY_IDLE;
  pinMode(rx_pin, INPUT);
  digitalWrite(rx_pin, HIGH);
  pinMode(tx_pin, OUTPUT);
  digitalWrite(tx_pin, HIGH);
  rx_iter = 0;
  tx_iter = 0;
  symbol_iter = 0;
  PCintPort::attachInterrupt(rx_pin, _int_one, RISING); // rising edge is 1 bit
  PCintPort::attachInterrupt(rx_pin, _int_zero, FALLING); // falling edge is 0 bit
  Timer1.initialize(PHY_PULSE_PERIOD);
  Timer1.attachInterrupt(_phy_fsm_control, PHY_PULSE_PERIOD);
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

// send data in encode_buffer every PHY_PULSE_PERIOD
void _phy_generate_pulse() {
  digitalWrite(tx_pin, encode_buffer[encode_iter & 0x01]);
  encode_iter++;
}

// increment the sync_iter
void _shift_forward() {
  rx_iter++;
}

uint8_t _is_ppm_sync() {
  uint8_t is_sync = PPM_SYNC_INC;
  if ((idle_buffer[0] == 0) && (idle_buffer[1] == 1)) { // iterator is in sync 01...01
    is_sync = PPM_SYNC_OK;
  } else {
    is_sync = PPM_SYNC_INC;    
  }
  return is_sync;
}

// transmit IDLE symbols and synchronize
void _phy_idle() {
  uint8_t sync_status;
  if (rx_iter & 0x01 == 0x01) {// all idle bits are received
    sync_status = _is_ppm_sync();
    if (sync_status == PPM_SYNC_OK) { // received bits are idle pattern
      phy_state = PHY_RX;
    }
    rx_iter = 0; // either not sync or moving to PHY_RX, rx_iter must be returned to 0
  }
  if ((rx_iter & 0x01) == 0)  { // 1st idle bit, send 0
    _encode_zero();
  } else { // 2nd idle bit, send 1
    _encode_one();
  }
}

// check if receiving 0xff
uint8_t _phy_rx() {
    
}

// update encode_buffer using values from tx_buffer
// do this until tx_len is zero, ie all bits are transmitted
uint8_t _phy_tx_rx() {
  if (tx_len > 0) { // transmit bits in tx_buffer
    if (tx_buffer[tx_iter] == 0) {
      _encode_zero();
    } else {
      _encode_one();
    }
    tx_iter++;
    tx_len--;
  } else { // all bits are transmitted
    phy_state = PHY_IDLE;
  }
}

// called every 500us, by _phy_update()
void _phy_fsm_control() {
  _phy_generate_pulse();
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

void _encode_one() {
  encode_buffer[0] = LOW;
  encode_buffer[1] = HIGH;
}

void _encode_zero() {
  encode_buffer[0] = HIGH;
  encode_buffer[1] = LOW;
}

void _int_one() {
  if (phy_state == PHY_IDLE) { //  put to idle_buffer
    idle_buffer[rx_iter & 0x01] = 1;
  } else { // put to rx_buffer
    rx_buffer[rx_iter] = 1;
  }
  rx_iter++;
}

void _int_zero() {
  if (phy_state == PHY_IDLE) { //  put to idle_buffer
    idle_buffer[rx_iter & 0x01] = 0;
  } else { // put to rx_buffer
    rx_buffer[rx_iter] = 0;
  }
  rx_iter++;
}

