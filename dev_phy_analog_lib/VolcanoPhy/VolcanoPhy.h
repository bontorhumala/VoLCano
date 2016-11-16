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
//   (?) able to receive bits, but really depends on timing. need to check tx output and also adc sampling period

#ifndef VOLCANOPHY_H
#define VOLCANOPHY_H

#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
  #include "pins_arduino.h"
  #include "WConstants.h"
#endif

//#define SERIAL_PLOT
//#define DEBUG
//#define DEBUG_RX
//#define DEBUG_RX_DETECT
//#define DEBUG_TX
#define DEBUG_OSC
//#define RX_NODE
#define TX_NODE

// FSM states
#define PHY_IDLE 0
#define PHY_RX 1
#define PHY_TX_RX 2
#define PHY_PREAMBLE_RX 3
#define PHY_PREAMBLE_TX 4

// Period
// #define USE_TIMER_0
#define USE_TIMER_1
// #define USE_TIMER_2
#define PHY_SAMPLE_PERIOD   500 // phy sensing (sampling) period
#define PHY_SAMPLE_WINDOW   5  // number of samples or window size
#define PHY_PULSE_WIDTH     2000 // pulse width
#define TIMER_COUNT         125 // Timer runs at 16MHz, prescaler 64, (4 us per tick), in order to interrupt every 500us -> count 125 (CTC)

#define BIT_LEN             2
#define PULSE_LEN           (PHY_PULSE_WIDTH/PHY_SAMPLE_PERIOD) // pulse window size, also used to indicate new pulse
#define PERIOD_LEN          (BIT_LEN*PULSE_LEN) // ppm window size (2 consecutive pulse)
#define PHY_SAFE_IDLE       5*PERIOD_LEN // minimum idle pulse period to ensure it is safe to transmit
#define MIN_EDGE_LEN        (3*PULSE_LEN/4)
#define NO_EDGE_PERIOD_LEN  2*PERIOD_LEN // maximum no edge is 2*period

// Rx and Tx Buffer
#define MAX_PHY_BUFFER 263 // 263 (maximum MAC packet), assume no additional PHY bytes
#define ACK_PHY_BUFFER 8 // 8 (ACK in MAC), assume no additional PHY bytes

// Byte packet len
#define PREAMBLE_LEN        2
#define BYTE_LEN            8

// Edge detection threshold
#define EDGE_THRESHOLD        10 // minimum range in PULSE_WINDOW_LEN to be considered an edge
#define MID_BIT               (PHY_SAMPLE_WINDOW>>1)    // Middle point of pulse

// Debugger
#define DEBUG_OSC_P1    ({osc_pin1_state = !osc_pin1_state; digitalWrite(osc_pin1, osc_pin1_state);})
#define DEBUG_OSC_P2    ({osc_pin2_state = !osc_pin2_state; digitalWrite(osc_pin1, osc_pin2_state);})

class VolcanoPhy
{
  public:
    VolcanoPhy(); //the constructor
    void setTxPin(uint8_t pin); //set the arduino digital pin for transmit.
    void setRxPin(uint8_t pin); //set the arduino digital pin for receive.
    void begin(uint8_t TxPin = A1, uint8_t RxPin = A0); //set up transceceiver

    bool phy_sense();
    int16_t phy_rx(uint8_t *data);
    void phy_tx(uint8_t *data, uint16_t data_len);

    // ISR related func, executed inside interrupt
    void _phy_fsm_control();
    void _push_sampling_buffer(uint16_t data);

    // singleton
    static VolcanoPhy* getInstance();

  private:
    void _phy_update_tx();
    void _phy_idle();
    void _phy_rx();
    void _phy_tx_rx();
    void _phy_preamble_rx();
    void _phy_preamble_tx();
    void _encode_zero();
    void _encode_one();
    void _encode_idle();
    int8_t _detect_edge();
    uint8_t _bits_byte(uint8_t *bits);
    void _byte_bits(uint8_t tx_byte, uint8_t *bits);
    void _empty_array(uint8_t *buff, uint8_t len);

    void _setupTimer();
    void _setupADC();

    uint8_t rx_pin;
    uint8_t tx_pin;
};
//end of class Phy

extern VolcanoPhy volcanoPhy;

#endif
