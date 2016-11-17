// By Bontor Humala
// MAC layer of Volcano
// LOG
// 10/31
//   (-) initial version, use QueueArray for rx and tx buffer
// 11/8
//   (-) use https://github.com/JChristensen/Timer library to update FSM
// 11/17
//   (?) add guard condition to queue

#include <Event.h>
#include <Timer.h>
#include <QueueArray.h>
#include <util/crc16.h>

//#define TEST_TX_NODE
#define TEST_RX_NODE

// maximum packets in MAC layer. if 1 frame is 263 bytes, then MAC can only hold that one frame
#define MAX_PACKET 256
#define MAX_FRAME 263 // 5+MAX_PACKET+2
#define ACK_FRAME 8 // 5+1+2
#define MAX_QUEUE 5
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
#define MAC_CTRL_FREQ 50 // mac control every 10 ms
#define MAX_TX_QUEUE 100

// PUBLIC
uint8_t mac_update(); // IMPORTANT: if MAC class is used in application, this needs to be called from loop(). Otherwise, state machine wont run
uint8_t mac_rx();
bool mac_tx();

// dev_phy_analog
bool phy_sense();
int16_t phy_rx(uint8_t *data);
void phy_tx(uint8_t *data, uint16_t data_len);
void phy_initialize();

// PRIVATE
struct Tx_State {
  uint8_t state;
  uint8_t msg_id;
};

Tx_State tx_state[MAX_TX_QUEUE]; // unused, will be used for async or event driven notification
uint8_t mac_state;
uint8_t random_cw;
uint8_t mac_wait_iter;
uint8_t addr;
bool b_medium_idle;
QueueArray <uint8_t> rx_queue;
uint16_t rx_queue_len[MAX_QUEUE]; // array of rx dataframe len
uint8_t rx_queue_len_iter;
uint8_t rx_ack_queue[MAX_QUEUE]; // array of ack
uint8_t rx_ack_queue_iter;
QueueArray <uint8_t> tx_queue;
uint8_t re_tx_buffer[MAX_PACKET]; // retransmission data backup
uint8_t tx_addr_queue[MAX_QUEUE]; // array of tx destination address
uint8_t tx_addr_queue_iter;
uint16_t tx_queue_len[MAX_QUEUE]; // array of tx dataframe len
uint8_t tx_queue_len_iter;
uint16_t re_tx_data_len; // retransmission length backup
uint8_t re_tx_addr; // retransmission address backup
uint8_t re_count;
bool is_ack_received;
Timer t; //instantiate the timer object

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

#if defined(TEST_TX_NODE) || defined(TEST_RX_NODE)
#define BUFFER_SIZE 40
#endif

#ifdef TEST_TX_NODE
#define DEST_ADDR 0x02
#define NODE_ADDR 0x01
long tx_node_len;
uint8_t tx_node_buffer[BUFFER_SIZE];
#endif

#ifdef TEST_RX_NODE
#define SRC_ADDR 0x01
#define NODE_ADDR 0x02
uint16_t rx_node_len;
uint8_t rx_node_buffer[BUFFER_SIZE];
#endif

void setup() {
  Serial.begin(115200);  
#ifdef TEST_TX_NODE
  randomSeed(analogRead(3));
  tx_node_len = random(3, BUFFER_SIZE);
  Serial.print(F("mac tx: "));
  for (uint8_t i=0; i<tx_node_len; i++) { // generate bytes
    tx_node_buffer[i] = random(255);
    Serial.print(tx_node_buffer[i]);Serial.print(F(", "));
  }
  Serial.print(F("\n"));
  mac_tx(tx_node_buffer, tx_node_len, DEST_ADDR);
#endif
#ifdef TEST_RX_NODE
  Serial.print(F("mac rx: "));
#endif
  phy_initialize();
  mac_initialize();
}

void loop() {
  mac_update();
#ifdef TEST_RX_NODE
  rx_node_len = mac_rx(rx_node_buffer);
  if (rx_node_len) {
    for (uint8_t i=0; i<rx_node_len; i++) {
      Serial.print(rx_node_buffer[i]);Serial.print(F(", "));
    }
    Serial.print(F("\n"));
  }
#endif
}

void mac_initialize() {
  addr = NODE_ADDR;
  mac_state = MAC_IDLE;
  random_cw = 0;
  t.every(MAC_CTRL_FREQ, _mac_fsm_control);
}

uint8_t mac_update() {
  t.update();
}

// application layer uses this function to read out mac buffer
// there could be multiple PDU in the rx_queue, this function will only return the first on the queue (FIFO)
uint8_t mac_rx(uint8_t *data) {
  uint16_t count = 0;
  uint16_t rx_len = rx_queue_len[rx_queue_len_iter];
  rx_queue_len_iter--;
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
   } 
   if ((count + tx_queue.count()) > MAX_FRAME) {
     return false;
   } 
   else {
     tx_queue_len[tx_queue_len_iter] = count+7;
     tx_queue_len_iter++;
     do {
       tx_queue.enqueue((uint8_t)*data++);
     } while (count--);
     tx_addr_queue[tx_addr_queue_iter] = dest_addr;
     tx_addr_queue_iter++;
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
// drop new packet if old packet still exists
void _mac_read_packet() {
  uint8_t mac_pdu[MAX_FRAME];
  uint8_t mac_ack_pdu[ACK_FRAME];
  uint16_t data_len = 0;
  uint16_t fcs = 0;
  uint16_t i=0;
  data_len = phy_rx(mac_pdu);
  if (((data_len + rx_queue.count()) < MAX_FRAME) && (data_len > 0)) { // drop if queue is full or data_len is negative
    fcs = (mac_pdu[data_len-2]<<8) | mac_pdu[data_len-1]; // fcs are located at the end of mac_pdu
    data_len = data_len - 7; // remove header and fcs
    if ((fcs == _calculate_fcs(mac_pdu, 5+data_len)) && (mac_pdu[2] == addr)) {  
      if (data_len == 1) { // only 1 byte, must be ACK
        rx_ack_queue[rx_ack_queue_iter] = mac_pdu[5];
        rx_ack_queue_iter++;
      } 
      else {// larger than 1 bytes, must be data
        rx_queue_len[rx_queue_len_iter] = data_len;
        rx_queue_len_iter++;
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
}

// wait for 4 slots
void _wait_ack_slot() {
  uint8_t slot_counter=0;
  while (slot_counter < 4) {
    delayMicroseconds(CW_DELAY);
    slot_counter++;
  }
}

// wait for 16 slots * num_cw
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
  if (b_medium_idle && (!tx_queue.isEmpty())) { // send if tx_queue is not empty
    mac_state = MAC_INIT_WAIT;
  } else {
    mac_state = MAC_IDLE;
  }
}

// SFD or initial wait
void _mac_init_wait(){
  _wait_cw_slot(1);
  mac_state = MAC_RANDOM_CW;
}

// get random contention window
void _mac_random_cw() {
  random_cw = random(0, 16); 
  mac_wait_iter = 0;
  mac_state = MAC_WAIT_CW;
}

// wait transmission slot according to random_cw
void _mac_wait_cw() {
  b_medium_idle = true;
  if ((mac_wait_iter<=random_cw) && b_medium_idle ){ // medium has to be idle to increment cw counter
    b_medium_idle = phy_sense();
    if (b_medium_idle) {
      _wait_cw_slot(1);
      mac_wait_iter++;
    } else { // if medium is not idle, go back to random_cw calculation state
      mac_state = MAC_RANDOM_CW;
    }
  }
  if (mac_wait_iter == random_cw) {  // random_cw is reached
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
  if ((mac_state == MAC_WAIT_ACK) && (mac_wait_iter<135)) { // wait 134 slots
    _wait_cw_slot(1);
    if ( rx_ack_queue[rx_ack_queue_iter] == (addr ^ re_tx_addr ^ re_tx_buffer[0]) ) { // correct ACK is received
      rx_ack_queue_iter--;
      is_ack_received = true;
      mac_state = MAC_IDLE;
      mac_wait_iter = 135; // get out of wait_ack loop
      memset(re_tx_buffer, 0, MAX_PACKET);
    }
  }
  if ((mac_wait_iter==135) && (!is_ack_received)) { // failed to receive ACK, do retransmission
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
  
  switch(tx_type) { // different PDU for transmission and retransmission
    case PDU_TX:
      mac_pdu[2] = tx_addr_queue[tx_addr_queue_iter]; // destination address
      tx_addr_queue_iter--;
      re_tx_addr = mac_pdu[2]; // retransmission address backup
      mac_pdu[4] = tx_queue_len[tx_queue_len_iter]; // NAV or data length
      tx_queue_len_iter--;
      re_tx_data_len = data_len = mac_pdu[4]; // retransmission data length backup and iterator
      while (data_len) { // extract mac_pdu from tx_queue
        mac_pdu[i+5] = tx_queue.dequeue();
        re_tx_buffer[i] = mac_pdu[i+5]; // retransmission backup
        i++;
        data_len--;
      }
      break;
    case PDU_RET:
      mac_pdu[2] = re_tx_addr; // destination address
      data_len = mac_pdu[4] = re_tx_data_len; // NAV or data length 
      while (data_len) { // extract mac_pdu from tx_queue
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
  while ( count > 0 ) {
    result = _crc16_update ( result, *data );
    ++data;
    --count;
  }
  return result;
}
