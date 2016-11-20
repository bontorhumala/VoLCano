#include <SoftwareSerial.h>
#define BUFFER_SIZE 10
// SoftwareSerial
#define HEADER_SYMBOL   0x99

// Commands
#define CMD_STATUS_REQUEST    0x01
#define CMD_STATUS_REPLY      0x02
#define CMD_GREETING_REQUEST  0x03
#define CMD_GREETING_REPLY    0x04
#define CMD_BYE_REQUEST       0x05
#define CMD_BYE_REPLY         0x06
#define CMD_SPEEDTEST_REQUEST 0x07
#define CMD_SPEEDTEST_REPLY   0x08

// Commands Length
#define CMD_STATUS_REQUEST_LEN    2
#define CMD_STATUS_REPLY_LEN      2
#define CMD_GREETING_REQUEST_LEN  2
#define CMD_GREETING_REPLY_LEN    2
#define CMD_BYE_REQUEST_LEN       2
#define CMD_BYE_REPLY_LEN         2
#define CMD_SPEEDTEST_REQUEST_LEN 2
#define CMD_SPEEDTEST_REPLY_LEN   2

// FSM read software serial
#define WAIT_HEADER   0
#define WAIT_CMD      1
#define WAIT_DATA     2

// Node ID
#define NODE_GATEWAY_ID   0
#define NODE_1_ID         1
#define NODE_2_ID         2

// Current node ID
//#define THIS_NODE_ID      NODE_GATEWAY_ID
#define THIS_NODE_ID      NODE_1_ID
//#define THIS_NODE_ID      NODE_2_ID

// buffer software serial
#define MAX_DATA_RX_BUFF    20

// timeout
#define TIMEOUT_MS          6000

// timeout
#if (THIS_NODE_ID == NODE_GATEWAY_ID)
#endif  /* if (THIS_NODE_ID == NODE_GATEWAY_ID) */
unsigned long timeout_volcano_time;

#if (THIS_NODE_ID == NODE_GATEWAY_ID)
SoftwareSerial swSer(10, 11); // RX (10), TX (11)
#endif  /* if (THIS_NODE_ID == NODE_GATEWAY_ID) */

// Global variables for software serial communication
#if (THIS_NODE_ID == NODE_GATEWAY_ID)
char buf_in[MAX_DATA_RX_BUFF] = {0};
char cmd_in;
int buf_in_len = 0;
#endif  /* if (THIS_NODE_ID == NODE_GATEWAY_ID) */

// Global variables for VolCano comm
bool waitingReply_volcano = false;
char src_node_id = NODE_GATEWAY_ID;
char dst_node_id = NODE_GATEWAY_ID;

void mac_initialize();
uint8_t mac_update();
uint8_t mac_rx(uint8_t *data);
bool mac_tx(uint8_t *data, uint8_t data_len, uint8_t dest_addr);
uint8_t rx_node_buffer_app[BUFFER_SIZE] = {0};
uint8_t tx_node_buffer_app[BUFFER_SIZE] = {0};
uint8_t tx_node_length;

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

#if (THIS_NODE_ID == NODE_GATEWAY_ID)
  // set the data rate for the SoftwareSerial port
  swSer.begin(9600);
#endif  /* if (THIS_NODE_ID == NODE_GATEWAY_ID) */

#ifdef TEST_RX_NODE
  Serial.print(F("mac rx: "));
#endif
  phy_initialize();
  mac_initialize();
  // Set initial timer
  timeout_volcano_time = millis();
}

void loop() { // run over and over
#if (THIS_NODE_ID == NODE_GATEWAY_ID)
  checkUpdateSoftwareSerial();    // get software serial update
#endif  /* if (THIS_NODE_ID == NODE_GATEWAY_ID) */
  checkCommVolcano();   // get update from VoLCano mac and phy
  mac_update();
}

#if (THIS_NODE_ID == NODE_GATEWAY_ID)
/*------------ Software Serial - communicating with nodeMCU -------------------------*/
void sendToSoftwareSerial(char cmd, char *data)
{
  int i;
  int len = cmdToLen(cmd);
  swSer.write(HEADER_SYMBOL);
  swSer.write(cmd);
  for(i=0; i<len; i++)
  {
    swSer.write(data[i]);
  }
}

// return len, non-blocking mode
int readSoftwareSerial(char *cmd, char *data)
{
  int len = 0;
  int data_len_in = 0;

  if(swSer.available())
  {
    static char swState = WAIT_HEADER;
    static int counter = 0;
    unsigned char c;

    c = swSer.read();
    Serial.println(c, HEX);

    switch(swState)
    {
      case WAIT_HEADER:
        if(c == HEADER_SYMBOL)
          swState = WAIT_CMD;
        break;
      case WAIT_CMD:
        *cmd = c;
        swState = WAIT_DATA;
        break;
      case WAIT_DATA:
        data[counter] = c;
        counter++;
        // Check invalid command
        data_len_in = cmdToLen(*cmd);
        if(data_len_in != -1)
        {
          if(counter == data_len_in) {
            len = counter;
            counter = 0;
            swState = WAIT_HEADER;
          }
        } else {
          len = -1; // error
          counter = 0;
          swState = WAIT_HEADER;
        }
        break;
    }
  }

  return len;
}

#endif  /* if (THIS_NODE_ID == NODE_GATEWAY_ID) */

int cmdToLen(char cmd)
{
  int len = -1;
  switch(cmd)
  {
    case CMD_GREETING_REQUEST:
      len = CMD_GREETING_REQUEST_LEN;
      break;
    case CMD_GREETING_REPLY:
      len = CMD_GREETING_REPLY_LEN;
      break;
    case CMD_BYE_REQUEST:
      len = CMD_BYE_REQUEST_LEN;
      break;
    case CMD_BYE_REPLY:
      len = CMD_BYE_REPLY_LEN;
      break;
    case CMD_STATUS_REQUEST:
      len = CMD_STATUS_REQUEST_LEN;
      break;
    case CMD_STATUS_REPLY:
      len = CMD_STATUS_REPLY_LEN;
      break;
    case CMD_SPEEDTEST_REQUEST:
      len = CMD_SPEEDTEST_REQUEST_LEN;
      break;
    case CMD_SPEEDTEST_REPLY:
      len = CMD_SPEEDTEST_REPLY_LEN;
      break;
  }
  return len;
}

#if (THIS_NODE_ID == NODE_GATEWAY_ID)
void checkUpdateSoftwareSerial()
{
  // Get update from software serial
  if(!waitingReply_volcano)
  {
    buf_in_len = readSoftwareSerial(&cmd_in, buf_in);
    // finished reading data
    if(buf_in_len > 0)
    {
      Serial.println(F("[App] Receiving new message"));
      switch (cmd_in) {
        case CMD_GREETING_REQUEST:
        case CMD_BYE_REQUEST:
        case CMD_STATUS_REQUEST:
        case CMD_SPEEDTEST_REQUEST:
          src_node_id = buf_in[0];
          dst_node_id = buf_in[1];
          // TODO: Send to another node through mac layer
          // Packet length
          tx_node_length = cmdToLen(cmd_in) + 1;
          Serial.print(F("[App] mac tx: "));
          // Command - data[0]
          tx_node_buffer_app[0] = cmd_in;
          // Data
          memcpy((char *)&tx_node_buffer_app[1], buf_in, tx_node_length - 1);
          for (uint8_t i=0; i<tx_node_length; i++) { // generate bytes
              Serial.print(tx_node_buffer_app[i]);Serial.print(F(", "));
          }
          Serial.print(F("\r\n"));
          mac_tx(tx_node_buffer_app, tx_node_length, dst_node_id);
  
          waitingReply_volcano = true;  // waiting for reply from destination node
          timeout_volcano_time = millis();
          break;
        default:
          Serial.println(F("Error invalid command from gateway"));
          break;
      }
    } else if(buf_in_len == -1) {
      Serial.println(F("Error receiving data from gateway"));
    }
  }
}
#endif  /* if (THIS_NODE_ID == NODE_GATEWAY_ID) */

/*------------------------- VoLCano comm -----------------*/
void checkCommVolcano()
{
  // TODO: integrate with MAC
  // .... Whenever data from volcano is avaliable, then process it
  uint16_t rx_node_length = mac_rx(rx_node_buffer_app);
  if (rx_node_length) {
    Serial.println(F("Received from node"));
    for (uint8_t i=0; i<rx_node_length; i++) {
      Serial.print(rx_node_buffer_app[i]);Serial.print(F(", "));
    }
    Serial.print(F("\r\n"));
    handleRxCommand(rx_node_buffer_app, rx_node_length);

#if (THIS_NODE_ID == NODE_GATEWAY_ID)
    timeout_volcano_time = millis();
//    handleRxCommandSS(rx_node_buffer_app, rx_node_length);
#endif  /* if (THIS_NODE_ID == NODE_GATEWAY_ID) */  
  }

#if (THIS_NODE_ID == NODE_GATEWAY_ID)
  // Check for communication timeout on VLC comm
  if(waitingReply_volcano)
  {
    if(millis() - timeout_volcano_time > TIMEOUT_MS)
    {
      Serial.println(F("Error VoLCano communication timeout"));
      waitingReply_volcano = false;
    }
  }
#endif  /* if (THIS_NODE_ID == NODE_GATEWAY_ID) */  
}

void handleRxCommand(char *buff, int len)
{
  // Command message
  unsigned char cmd_in = buff[0];
  int cmd_len;

  Serial.println(cmd_in);
  switch (cmd_in) {
    case CMD_GREETING_REQUEST:
      tx_node_buffer_app[0] = CMD_GREETING_REPLY;
      break;
    case CMD_BYE_REQUEST:
      tx_node_buffer_app[0] = CMD_BYE_REPLY;
      break;
    case CMD_STATUS_REQUEST:
      tx_node_buffer_app[0] = CMD_STATUS_REPLY;
      break;
    case CMD_SPEEDTEST_REQUEST:
      tx_node_buffer_app[0] = CMD_SPEEDTEST_REPLY;
      break;
    default:
      return;   // exit
  }
  cmd_len = cmdToLen(cmd_in);
  
  tx_node_buffer_app[1] = THIS_NODE_ID;   // src
  tx_node_buffer_app[2] = buff[1];        // dst
  // Check data length
  if(len > 1) memcpy((char *)&tx_node_buffer_app[3], (char *)&buff[2], len - 2);  // other data
  
  Serial.print(F("[App] mac tx after rx: "));
  for (uint8_t i=0; i<tx_node_length; i++) { // generate bytes
      Serial.print(tx_node_buffer_app[i]);Serial.print(F(", "));
  }
  Serial.print(F("\r\n"));
  
  // TODO: Send to another node through mac layer
  // send to other nodes
  mac_tx(tx_node_buffer_app, cmd_len, buff[1]);
}

#if (THIS_NODE_ID == NODE_GATEWAY_ID)
void handleRxCommandSS(char *buff, int len)
{
  // Command message
  unsigned char cmd_in = buff[0];
  int cmd_len;
  char buff_out[5];
  
  switch (cmd_in) {
    case CMD_GREETING_REPLY:
    case CMD_BYE_REPLY:
    case CMD_STATUS_REPLY:
    case CMD_SPEEDTEST_REPLY:
      buff_out[0] = buff[1];            // src
      buff_out[1] = NODE_GATEWAY_ID;    // dst
      Serial.println(F("Send back to nodemcu"));      
      sendToSoftwareSerial(cmd_in, buff_out);
      timeout_volcano_time = millis();
      waitingReply_volcano = false;
      break;
  }
}
#endif  /* if (THIS_NODE_ID == NODE_GATEWAY_ID) */

