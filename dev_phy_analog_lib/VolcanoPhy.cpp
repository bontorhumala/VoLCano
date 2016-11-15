#include "VolcanoPhy.h"

// update sampling_buffer every PHY_SAMPLE_PERIOD
// update tx pin every pulse period
static uint16_t sampling_buffer[PULSE_LEN] = {0};
static uint8_t sampling_iter = 0;
static uint8_t pulse_iter = 0;
// safe idle pulse period
static uint8_t idle_counter = 0;
// time of last detected edge
static unsigned long prev_edge = 0;
static uint8_t phy_state = PHY_IDLE;
// no edge counter
static uint8_t no_edge_count = 0;

VolcanoPhy::VolcanoPhy() //constructor
{
  rx_pin = A0;
  tx_pin = A1;
  pinMode(rx_pin, INPUT);
  pinMode(tx_pin, OUTPUT);
}


void VolcanoPhy::setTxPin(uint8_t pin)
{
  ::tx_pin = pin; // user sets the digital pin as output
  pinMode(::tx_pin, OUTPUT);
}

void VolcanoPhy::setRxPin(uint8_t pin)
{
  ::rx_pin = pin; // user sets the digital pin as input
  pinMode(::rx_pin, INPUT);
}

void VolcanoPhy::setupTransmit(uint8_t pin, uint8_t SF)
{
  setTxPin(pin);
}

void VolcanoPhy::setupReceive(uint8_t pin, uint8_t SF)
{
  setRxPin(pin);
}

void VolcanoPhy::setup(uint8_t TxPin, uint8_t RxPin, uint8_t SF)
{
  setupTransmit(TxPin, SF);
  setupReceive(RxPin, SF);
}

void VolcanoPhy::transmit(uint16_t data)
{
  uint8_t byteData[2] = {data >> 8, data & 0xFF};
  transmitArray(2, byteData);
}

//global functions
void VOLCANO_SetupTimer()
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
#elif USE_TIMER_1
  noInterrupts();
  TCCR1A = 0x00;                // Timer1 disable interrupt
  TCCR1B = 0x00;                // Timer1 disable interrupt
  OCR1BL = TIMER_COUNT;         // Timer1 compare match value low byte
  OCR1BH = 0x00;                // Timer1 compare match value high byte
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

void VOLCANO_SetupADC()
{
  noInterrupts();
  ADMUX = _BV(REFS0);                                //use AVcc as reference
  ADCSRA  = _BV(ADEN)  | _BV(ADATE) | _BV(ADIE);     //enable ADC, auto trigger, interrupt when conversion complete
  ADCSRA |= _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);    //ADC prescaler: divide by 128
#ifdef USE_TIMER_0
  ADCSRB = _BV(ADTS1) | _BV(ADTS0);                  //trigger ADC on Timer/Counter0 Compare Match A
#elif USE_TIMER_1
  ADCSRB = _BV(ADTS2) | _BV(ADTS0);                  //trigger ADC on Timer/Counter0 Compare Match A
#endif
// default: free running mode
  interrupts();
}

// called every PHY_SAMPLE_PERIOD
void VOLCANO_phy_fsm_control() {
  VOLCANO_phy_update_tx();
  switch (phy_state) {
    case PHY_IDLE:
      VOLCANO_phy_idle();
      break;
    case PHY_PREAMBLE_RX:
      VOLCANO_phy_preamble_rx();
      break;
    case PHY_RX:
      VOLCANO_phy_rx();
      break;
    case PHY_PREAMBLE_TX:
      VOLCANO_phy_preamble_tx();
      break;
    case PHY_TX_RX:
      VOLCANO_phy_tx_rx();
      break;
  }
  if (pulse_iter == PULSE_LEN) {
    pulse_iter = 0;
  }
}

// update tx pin every PULSE_WINDOW_LEN
void VOLCANO_phy_update_tx() {
  if (pulse_iter == 0) { // update tx_pin
    digitalWrite(tx_pin, encode_buffer[encode_iter & 0x01]);
    encode_iter++;
    if (phy_state == PHY_IDLE) {
      idle_counter++;
    }
  }
  pulse_iter++;
}

// transform bit array to byte
// LSB is at index 0
uint8_t VOLCANO_bits_byte(uint8_t *bits) {
  uint8_t sum = 0;
  for (uint8_t i = 0; i < BYTE_LEN; i++) {
    sum += bits[i] << i;
  }
  return sum;
}

void VOLCANO_byte_bits(uint8_t tx_byte, uint8_t *bits) {
  for (uint8_t i = 0; i < BYTE_LEN; i++) {
    bits[i] = (tx_byte >> i) & 0x01;
  }
}

// FIFO buffer push implementation for sampling_buffer
void VOLCANO_push_sampling_buffer(uint16_t data)
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

void VOLCANO_empty_array(uint8_t *buff, uint8_t len) {
  for (uint8_t i = 0; i < len; i++) {
    buff[i] = 0;
  }
}

// Timer interrupt routine
#ifdef USE_TIMER_0
ISR(TIMER0_COMPA_vect)
#elif USE_TIMER_1
ISR(TIMER1_COMPB_vect)
#else
ISR(TIMER2_COMPA_vect)
#endif
{
  VOLCANO_phy_fsm_control();
}

// ADC Interrupt routine
ISR(ADC_vect)
{
  VOLCANO_push_sampling_buffer(ADC); // sample rx_pin
}

VolcanoPhy volcanoPhy;
