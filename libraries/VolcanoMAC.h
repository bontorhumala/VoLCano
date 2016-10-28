/*
  VolcanoMAC.h - Library for for MAC in VoLCano: visible light to light communication.
  Created by Bontor Humala, October 28, 2016.
  
*/
#ifndef VolcanoMAC_h
#define VolcanoMAC_h

#include "Arduino.h"
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
#define MAX_RET 3
#define MAC_CTRL_FREQ 10000 // mac control every 10 ms

class VolcanoMAC 
{
  public:
    VolcanoMAC(uint8_t mac_addr);
    uint8_t mac_rx();
    bool mac_tx();
  
  private:
    uint8_t mac_state;
    uint8_t random_cw;
    uint8_t i;
    uint8_t addr;
    bool b_medium_idle;
    QueueArray <uint8_t> rx_queue;
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
}

#endif
