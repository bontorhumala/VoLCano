#include <TimerOne.h>
#include <QueueArray.h>
#include <util/crc16.h>

#define MAX_PACKET 256
#define MAX_FRAME 263 // 5+MAX_PACKET+2
#define ACK_FRAME 8 // 5+1+2
#define HEADER 0xFF
#define MAC_IDLE 0
#define MAC_INIT_WAIT 1
#define MAC_RANDOM_CW 2
#define MAC_WAIT_CW 3
#define MAC_ACCESS 4
#define MAC_WAIT_ACK 6
#define CW_SLOT 16
#define CW_DELAY 500
#define PDU_TX 0
#define PDU_RET 1
#define TX_WAIT 0
#define TX_SENT 1
#define TX_FAIL 2
#define MAX_RET 3
#define MAC_CTRL_FREQ 10000 // mac control every 10 ms
#define MAX_TX_QUEUE 100

uint8_t mac_rx();
bool mac_tx();

struct Tx_State {
  uint8_t state;
  uint8_t msg_id;
};

Tx_State tx_state[MAX_TX_QUEUE]; // unused, will be used for async or event driven notification
uint8_t mac_state;
uint8_t random_cw;
uint8_t i;
uint8_t addr;
bool b_medium_idle;
QueueArray <uint8_t> rx_queue;
QueueArray <uint16_t> rx_queue_len;
QueueArray <uint8_t> rx_ack_queue;
QueueArray <uint8_t> tx_queue;
uint8_t re_tx_buffer[MAX_PACKET]; // retransmission data backup
QueueArray <uint8_t> tx_addr_queue;
QueueArray <uint16_t> tx_queue_len; // array of tx dataframe len
uint16_t re_tx_data_len; // retransmission length backup
uint8_t re_tx_addr; // retransmission address backup
uint8_t re_count;
bool is_ack_received;

uint16_t _calculate_fcs(uint8_t* data, uint8_t count);
uint16_t _mac_create_pdu(uint8_t mac_pdu[], uint8_t tx_type);
void _mac_create_ack(uint8_t mac_ack_pdu[], uint8_t ack_dest_addr, uint8_t first_byte);
void _mac_wait_ack();
void _mac_access();
void _mac_wait_cw();
void _mac_random_cw();
void _mac_init_wait();
void _mac_idle();
void _wait_cw_slot(uint8_t num_cw);
void _wait_ack_slot();
void _mac_read_packet();
void _mac_fsm_control();

// application layer uses this function to read out mac buffer
// there could be multiple PDU in the rx_queue, this function will only return the first on the queue (FIFO)
uint8_t mac_rx(uint8_t *data) {
  uint16_t count = 0;
  uint16_t rx_len = rx_queue_len.dequeue();
  do {
    *data++ = rx_queue.dequeue();
    count++;
  } while ((!rx_queue.isEmpty()) && (rx_len--));
  return count;
}

// application layer uses this function to fill mac buffer
// transmission attempt is non blocking (i.e. can send several times without waiting)
// maximum retries defined by MAX_RET
bool mac_tx(uint8_t *data, uint8_t count, uint8_t dest_addr) {
   if ((data == 0) || (count == 0)) {
     return false;
   } else {
     tx_queue_len.enqueue(count);
     do {
       tx_queue.enqueue((uint8_t)*data++);
     } while (count--);
     tx_addr_queue.enqueue(dest_addr);
     return true;
   }
}

// switch between MAC states
void _mac_fsm_control() {
  // read phy
  _mac_read_packet();
  switch(mac_state){
    case MAC_IDLE:
      _mac_idle();
      break;
    case MAC_INIT_WAIT:
      _mac_init_wait();
      break;
    case MAC_RANDOM_CW:
      _mac_random_cw();
      break;
    case MAC_WAIT_CW:
      _mac_wait_cw();
      break;
    case MAC_ACCESS:
      _mac_access();
      break;
    case MAC_WAIT_ACK:
      _mac_wait_ack();
      break;
  }
}

// read packet from PHY layer, can be ACK or data
void _mac_read_packet() {
  uint8_t mac_pdu[MAX_FRAME];
  uint8_t mac_ack_pdu[ACK_FRAME];
  uint16_t data_len = 0;
  uint16_t fcs = 0;
  uint16_t i=0;
  data_len = phy_rx(mac_pdu);
  data_len = data_len - 7; // remove header and fcs
  fcs = (mac_pdu[data_len-2]<<8) | mac_pdu[data_len-1]; // fcs are located at the end of mac_pdu
  if ((fcs == _calculate_fcs(mac_pdu, 5+data_len)) && (mac_pdu[2] == addr)) {  
    if (data_len == 1) { // only 1 byte, must be ACK
      rx_ack_queue.enqueue(mac_pdu[5]);
    } 
    else {// larger than 1 bytes, must be data
      rx_queue_len.enqueue(data_len);
      do {
        rx_queue.enqueue(mac_pdu[i+5]);
        i++;
      } while(data_len--);
      // send ACK
      _mac_create_ack(mac_ack_pdu, mac_pdu[1], mac_pdu[5]);
      phy_tx(mac_ack_pdu, ACK_FRAME);
    }
  }
}

void _wait_ack_slot() {
  uint8_t slot_counter=0;
  while (slot_counter < 4) {
    delayMicroseconds(CW_DELAY);
    slot_counter++;
  }
}

void _wait_cw_slot(uint8_t num_cw) {
  uint8_t cw_slot_counter=0;
  uint8_t cw_counter=0;
   
  while (cw_counter < num_cw) {
    while (cw_slot_counter < CW_SLOT) {
      delayMicroseconds(CW_DELAY);
      cw_slot_counter++;
    }
    cw_counter++;
  }
}

void _mac_idle() {
  // check if medium is idle
  b_medium_idle = phy_sense();
  if (b_medium_idle && (!tx_queue.isEmpty())) {
    mac_state = MAC_INIT_WAIT;
  } else {
    mac_state = MAC_IDLE;
  }
}

void _mac_init_wait(){
  _wait_cw_slot(1);
  mac_state = MAC_RANDOM_CW;
}

void _mac_random_cw() {
  random_cw = random(0, 16); 
  mac_state = MAC_WAIT_CW;
}

// wait transmission slot according to random_cw
void _mac_wait_cw() {
  i = 0;
  b_medium_idle = true;
  if ((i<=random_cw) && b_medium_idle ){
    b_medium_idle = phy_sense();
    if (b_medium_idle) {
      _wait_cw_slot(1);
      i++;
    } else {
      mac_state = MAC_RANDOM_CW;
    }
  }
  if (i == random_cw) {
    re_count = 0;
    is_ack_received = false;
    mac_state = MAC_ACCESS;
  }
}

// send data to phy layer, either original transmission (PDU_TX) or retransmission (PDU_RET)
void _mac_access() {
  uint8_t mac_pdu[MAX_FRAME];
  bool is_retx = false;
  uint16_t data_len;
  if (re_count == 0) { // first transmission attempt
    _mac_create_pdu(mac_pdu, PDU_TX);
  } else if ((re_count>0) && (re_count < MAX_RET)){ // retransmission
    _mac_create_pdu(mac_pdu, PDU_RET);
  } else { // failed to send, ignore the packet and move on
    mac_state = MAC_IDLE;
  }
  phy_tx(mac_pdu, 5+data_len+2);
  mac_state = MAC_WAIT_ACK;
}

void _mac_wait_ack() {
  if ((mac_state == MAC_WAIT_ACK) && (i<135)) { // wait 134 slots
    _wait_cw_slot(1);

    if ( rx_ack_queue.dequeue() == (addr ^ re_tx_addr ^ re_tx_buffer[0]) ) { // correct ACK is received
      is_ack_received = true;
      mac_state = MAC_IDLE;
      i = 135; // get out of wait_ack loop
      memset(re_tx_buffer, 0, MAX_PACKET);
    }
  }
  if ((i==135) && (!is_ack_received)) { // failed to receive ACK, do retransmission
    re_count++;
    mac_state = MAC_ACCESS;
  }
}

// Create mac ACK pdu dataframe in mac_ack_pdu[] array, other parameters are used to calculate messageID
void _mac_create_ack(uint8_t mac_ack_pdu[], uint8_t ack_dest_addr, uint8_t first_byte) {
  uint16_t fcs;
  uint8_t messageID = addr ^ ack_dest_addr ^ first_byte; // messageID = addr XOR dest_addr XOR first byte
  
  mac_ack_pdu[0] = HEADER;
  mac_ack_pdu[1] = addr;
  mac_ack_pdu[2] = ack_dest_addr;
  mac_ack_pdu[3] = 0x00;
  mac_ack_pdu[4] = ACK_FRAME;
  mac_ack_pdu[5] = messageID;
  fcs = _calculate_fcs(mac_ack_pdu, ACK_FRAME);
  mac_ack_pdu[6] = (fcs >> 8) & 0xFF;
  mac_ack_pdu[7] = fcs & 0xFF;
}

// Create mac pdu dataframe in mac_pdu[] array, tx_type can be PDU_TX or PDU_RET
uint16_t _mac_create_pdu(uint8_t mac_pdu[], uint8_t tx_type) {
  uint16_t fcs;
  uint8_t i;
  uint16_t data_len;
  uint8_t tx_addr;
  
  mac_pdu[0] = HEADER; // header
  mac_pdu[1] = addr;
  mac_pdu[3] = 0x00; // RESERVED
  
  switch(tx_type) {
    case PDU_TX:
      mac_pdu[2] = tx_addr_queue.dequeue();
      re_tx_addr = mac_pdu[2];
      mac_pdu[4] = tx_queue_len.dequeue();
      re_tx_data_len = data_len = mac_pdu[4]; // retransmission backup and iterator
      while (data_len) {
        mac_pdu[i+5] = tx_queue.dequeue();
        re_tx_buffer[i] = mac_pdu[i+5]; // retransmission backup
        i++;
        data_len--;
      }
      break;
    case PDU_RET:
      mac_pdu[2] = re_tx_addr;
      data_len = mac_pdu[4] = re_tx_data_len; // NAV      
      while (data_len) {
        mac_pdu[i+5] = re_tx_buffer[i];
        i++;
        data_len--;
      }
      break;
    default:
      re_tx_data_len = 0;
      break;
  }
 
  fcs = _calculate_fcs(mac_pdu, 5+re_tx_data_len);
  mac_pdu[4+i] = (fcs >> 8) & 0xFF;
  mac_pdu[4+i+1] = fcs & 0xFF;

  return re_tx_data_len;
}

// Calculate CRC
uint16_t _calculate_fcs(uint8_t* data, uint8_t count)
{
  uint16_t result;
  result = 0;

  while ( count > 0 )
  {
    result = _crc16_update ( result, *data );
    ++data;
    --count;
  }
  return result;
}

void setup() {
  addr = 0x77;
  mac_state = MAC_IDLE;
  random_cw = 0;
  
  Timer1.initialize(MAC_CTRL_FREQ);
  Timer1.attachInterrupt(_mac_fsm_control);
}

void loop() {
  // put your main code here, to run repeatedly:

}
