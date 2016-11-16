#include "VolcanoPhy.h"

// update sampling_buffer every PHY_SAMPLE_PERIOD
// update tx pin every pulse period
static uint16_t sampling_buffer[PHY_SAMPLE_WINDOW] = {0};
static uint8_t sampling_iter = 0;
static uint8_t pulse_iter = 0;
// safe idle pulse period
static uint8_t idle_counter = 0;
// time of last detected edge
static unsigned long prev_edge = 0;
static uint8_t phy_state = PHY_IDLE;
// no edge counter
static uint8_t no_edge_count = 0;

// adaptive threshold
static int16_t threshold = EDGE_THRESHOLD;
static bool buffer_ready = false;
static bool start_counter_pulse_rx = false;
static uint16_t counter_pulse_rx = 0;

// encoding buffer and iterator
static uint8_t encode_buffer[BIT_LEN] = {0};
static uint8_t encode_iter = 0;
// decoding bit to byte for rx_buffer
static uint8_t decode_buffer[BYTE_LEN] = {0};
static uint8_t decode_iter = 0;

// bit array representation of byte in tx_buffer
static uint8_t tx_bits_buffer[BYTE_LEN] = {0};
static uint8_t tx_bits_iter = 0;

// rx tx buffer and iterator
static uint8_t rx_buffer[MAX_PHY_BUFFER] = {0};
static uint8_t tx_buffer[MAX_PHY_BUFFER] = {0};
static uint8_t preamble_buffer[PREAMBLE_LEN] = {0};
static uint16_t rx_iter = 0;
static uint16_t tx_iter = 0;

// maximum iteration for rx_buffer and tx_buffer
static uint16_t rx_len = 0;
static uint16_t tx_len = 0;
static uint8_t preamble_iter = 0;
static int8_t rx_in_bit = -1;

#ifdef DEBUG_OSC
static uint8_t osc_pin1 = 12;
static uint8_t osc_pin2 = 13;
static bool osc_pin1_state = LOW;
static bool osc_pin2_state = LOW;
#endif

// Public functions
VolcanoPhy::VolcanoPhy() //constructor
{
  rx_pin = A0;
  tx_pin = A1;
  pinMode(rx_pin, INPUT);
  pinMode(tx_pin, OUTPUT);
}

void VolcanoPhy::setTxPin(uint8_t pin)
{
  tx_pin = pin; // user sets the digital pin as output
  pinMode(tx_pin, OUTPUT);
}

void VolcanoPhy::setRxPin(uint8_t pin)
{
  rx_pin = pin; // user sets the digital pin as input
  pinMode(rx_pin, INPUT);
}

void VolcanoPhy::begin(uint8_t TxPin, uint8_t RxPin)
{
  setTxPin(TxPin);
  setRxPin(RxPin);
  _setupTimer();
  _setupADC();
#ifdef DEBUG_OSC
  pinMode(osc_pin1, OUTPUT);
  pinMode(osc_pin2, OUTPUT);
  digitalWrite(osc_pin1, osc_pin1_state);
  digitalWrite(osc_pin2, osc_pin2_state);
  pinMode(LED_BUILTIN, OUTPUT);
#endif
}

// returns true if medium is idle, false otherwise
bool VolcanoPhy::phy_sense() {
  bool is_idle = false;
  if (phy_state == PHY_IDLE) {
    is_idle = true;
  }
  return is_idle;
}

// data will be filled with received frame
// must be read before being overwritten (rx_iter larger than MAX_PHY_BUFFER)
int16_t VolcanoPhy::phy_rx(uint8_t *data) {
  if ((rx_iter != 0) || (rx_len == 0)) { // rx_iter is not finished yet OR rx_len is not updated
    return -1;
  }
  uint16_t rx_byte_len = rx_len;
  memcpy(data, rx_buffer, MAX_PHY_BUFFER);
  rx_iter = 0;
  return rx_byte_len;
}

// data contains frame to send, will be transferred to tx_buffer
void VolcanoPhy::phy_tx(uint8_t *data, uint16_t data_len) {
  memcpy(tx_buffer, data, data_len);
  tx_len = data_len;
  tx_iter = 0;
}

// Private functions
void VolcanoPhy::_setupTimer()
{
#ifdef USE_TIMER_0
  noInterrupts();
  TCCR0A = 0x00;                // Timer0 disable interrupt
  TCCR0B = 0x00;                // Timer0 disable interrupt
  OCR0A = TIMER_COUNT;          // Timer0 compare match value
  TCCR0A |= _BV(WGM01);       // Timer0 Control Reg A: Set to CTC mode
  TCCR0B |= _BV(CS01) | _BV(CS00);           // Timer0 Control Reg B: Timer Prescaler set to 64
  TIMSK0 |= _BV(OCIE0A);      // Timer0 INT Reg: Timer2 CTC Interrupt Enable
  interrupts();
#elif defined USE_TIMER_1
  noInterrupts();
  TCCR1A = 0x00;                // Timer1 disable interrupt
  TCCR1B = 0x00;                // Timer1 disable interrupt
  OCR1A = TIMER_COUNT;          // Timer1 compare match value low byte (16 bits)
  TCCR1B |= _BV(WGM12) | _BV(CS11) | _BV(CS10);           // Timer1 Control Reg B: Timer Prescaler set to 64, CTC Mode
  TIMSK1 |= _BV(OCIE1B);      // Timer1 INT Reg: Timer1 CTC Interrupt Enable
  interrupts();
#else // default TIMER_2
  noInterrupts();
  TCCR2A = 0x00;                // Timer2 disable interrupt
  TCCR2B = 0x00;                // Timer2 disable interrupt
  OCR2A = TIMER_COUNT;          // Timer2 compare match value
  TCCR2A |= _BV(WGM21);         // Timer2 Control Reg A: Set to CTC mode
  TCCR2B |= _BV(CS22);          // Timer2 Control Reg B: Timer Prescaler set to 64
  TIMSK2 |= _BV(OCIE2A);      // Timer2 INT Reg: Timer2 CTC Interrupt Enable
  interrupts();
#endif
} //end of setupReceive

void VolcanoPhy::_setupADC()
{
  noInterrupts();
  ADMUX = _BV(REFS0);                                //use AVcc as reference
  ADCSRA  = _BV(ADEN)  | _BV(ADATE) | _BV(ADIE);     //enable ADC, auto trigger, interrupt when conversion complete
  ADCSRA |= _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);    //ADC prescaler: divide by 128
#ifdef USE_TIMER_0
  ADCSRB = _BV(ADTS1) | _BV(ADTS0);                  //trigger ADC on Timer/Counter0 Compare Match A
#elif defined USE_TIMER_1
  ADCSRB = _BV(ADTS2) | _BV(ADTS0);                  //trigger ADC on Timer/Counter1 Compare Match B
#endif
// default: free running mode
  interrupts();
}

// called every PHY_SAMPLE_PERIOD
void VolcanoPhy::_phy_fsm_control() {
  _phy_update_tx();
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

// update tx pin every PULSE_WINDOW_LEN
void VolcanoPhy::_phy_update_tx() {
  if (pulse_iter == 0) { // update tx_pin
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
void VolcanoPhy::_phy_idle() {
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

void VolcanoPhy::_phy_preamble_rx() {
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

#ifdef DEBUG_OSC

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
void VolcanoPhy::_phy_rx() {
  int8_t in_bit;
  in_bit = _detect_edge();
  if (in_bit > -1) { // check incoming bit
#ifdef DEBUG_RX
    Serial.print("bit: ");Serial.println(in_bit);
#endif
    decode_buffer[decode_iter] = in_bit;
    decode_iter++;
    if (decode_iter == BYTE_LEN) {
      rx_buffer[rx_iter] = _bits_byte(decode_buffer);
#ifdef DEBUG_RX
      Serial.print("OK, 1 byte: ");Serial.println(rx_buffer[rx_iter]);
#endif
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
void VolcanoPhy::_phy_preamble_tx() {
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
void VolcanoPhy::_phy_tx_rx() {
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

// idle
void VolcanoPhy::_encode_idle() {
  encode_buffer[0] = LOW;
  encode_buffer[1] = LOW;
}

// rising edge
void VolcanoPhy::_encode_one() {
  encode_buffer[0] = LOW;
  encode_buffer[1] = HIGH;
}

// falling edge
void VolcanoPhy::_encode_zero() {
  encode_buffer[0] = HIGH;
  encode_buffer[1] = LOW;
}

// detect rising (1) or falling (0) edge using range in the sampling_buffer
// avoid transistional edge between two consecutive identical bit:
//   i.e. 00: HIGH (falling) LOW (rising) HIGH (falling) LOW -> falling, falling
//   ensure that the edge is detected after one PHY_PULSE_WIDTH
int8_t VolcanoPhy::_detect_edge() {
  int8_t in_bit = -1;

  if(buffer_ready)
  {
    int i;
    int sum_left = 0;
    int counter_left = 0;
    int sum_right = 0;
    int counter_right = 0;
    for(i = 0; i < PHY_SAMPLE_WINDOW; i++)
    {
      if(i < MID_BIT) {
        sum_left += sampling_buffer[i];
        counter_left++;
      } else {
        sum_right += sampling_buffer[i];
        counter_right++;
      }
    }
    // Calculate difference of average
    int sample_diff = (sum_right/counter_right) - (sum_left/counter_left);

    if(!start_counter_pulse_rx)
    {
      if (sample_diff < -threshold) { // falling edge
          in_bit = 0;
          no_edge_count = 0;
          counter_pulse_rx = 0;
          start_counter_pulse_rx = true;
      } else if(sample_diff > threshold) { // rising edge
          in_bit = 1;
          no_edge_count = 0;
          counter_pulse_rx = 0;
          start_counter_pulse_rx = true;
      }
    } else {
      counter_pulse_rx++;
      if(counter_pulse_rx >= MIN_EDGE_LEN) {
        counter_pulse_rx = 0;
        start_counter_pulse_rx = false;
      }
    }

    // No edge detected
    if (in_bit == -1) {
      no_edge_count++;
    }
  }

  return in_bit;
}

// transform bit array to byte
// LSB is at index 0
uint8_t VolcanoPhy::_bits_byte(uint8_t *bits) {
  uint8_t sum = 0;
  for (uint8_t i = 0; i < BYTE_LEN; i++) {
    sum += bits[i] << i;
  }
  return sum;
}

void VolcanoPhy::_byte_bits(uint8_t tx_byte, uint8_t *bits) {
  for (uint8_t i = 0; i < BYTE_LEN; i++) {
    bits[i] = (tx_byte >> i) & 0x01;
  }
}

// FIFO buffer push implementation for sampling_buffer
void VolcanoPhy::_push_sampling_buffer(uint16_t data)
{
  if(!buffer_ready)
  {
    sampling_buffer[sampling_iter] = data;  // put sample to buffer
    sampling_iter++;
    if (sampling_iter >= PHY_SAMPLE_WINDOW) {
      buffer_ready = true;
    }
  } else {
    memmove(sampling_buffer, &sampling_buffer[1], sizeof(uint16_t)*(PHY_SAMPLE_WINDOW-1));   // kick earlist sample at index 0 and shift the remaining
    sampling_buffer[PHY_SAMPLE_WINDOW - 1] = data; // add new data to last position in the buffer
  }
}

void VolcanoPhy::_empty_array(uint8_t *buff, uint8_t len) {
  for (uint8_t i = 0; i < len; i++) {
    buff[i] = 0;
  }
}

// Timer interrupt routine
#ifdef USE_TIMER_0
ISR(TIMER0_COMPA_vect)
#elif defined USE_TIMER_1
ISR(TIMER1_COMPB_vect)
#else
ISR(TIMER2_COMPA_vect)
#endif
{
  volcanoPhy._phy_fsm_control();
}

// ADC Interrupt routine
ISR(ADC_vect)
{
  volcanoPhy._push_sampling_buffer(ADC); // sample rx_pin
}

VolcanoPhy* VolcanoPhy::getInstance() {
  return &volcanoPhy;
};

// Preinit class
VolcanoPhy volcanoPhy;
