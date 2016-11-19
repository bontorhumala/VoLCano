#include <SoftwareSerial.h>

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
#define THIS_NODE_ID      1

// buffer software serial
#define MAX_DATA_RX_BUFF    20

// buffer VolCano
#define MAX_RX_BUFF_VOLCANO 20
#define MAX_TX_BUFF_VOLCANO 20

// timeout
#define TIMEOUT_MS          3000

// timeout
unsigned long timeout_softserial_time;
unsigned long timeout_volcano_time;

SoftwareSerial swSer(10, 11); // RX (10), TX (11)

// Global variables for software serial communication
char buf_in[MAX_DATA_RX_BUFF] = {0};
char cmd_in;
int buf_in_len = 0;
bool waitingReply_softserial = false;

// Global variables for VolCano comm
bool waitingReply_volcano = false;
char buff_rx_volcano[MAX_RX_BUFF_VOLCANO];
char buff_tx_volcano[MAX_TX_BUFF_VOLCANO];
char src_node_id = NODE_GATEWAY_ID;
char dst_node_id = NODE_GATEWAY_ID;

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // set the data rate for the SoftwareSerial port
  swSer.begin(9600);

  // Set initial timer
  timeout_softserial_time = millis();
  timeout_volcano_time = millis();
}

void loop() { // run over and over
  checkUpdateSoftwareSerial();    // get software serial update
  checkCommVolcano();   // get update from VoLCano mac and phy
}

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

void checkUpdateSoftwareSerial()
{
  // Get update from software serial
  if(!waitingReply_volcano)
  {
    buf_in_len = readSoftwareSerial(&cmd_in, buf_in);
    // finished reading data
    if(buf_in_len > 0)
    {
      switch (cmd_in) {
        case CMD_GREETING_REQUEST:
        case CMD_BYE_REQUEST:
        case CMD_STATUS_REQUEST:
        case CMD_SPEEDTEST_REQUEST:
          src_node_id = buf_in[0];
          dst_node_id = buf_in[1];
          // TODO: Send to another node through mac layer
          // ...
          waitingReply_volcano = true;  // waiting for reply from destination node
          break;
        default:
          Serial.println("Error invalid command from gateway");
          break;
      }
    } else if(buf_in_len == -1) {
      Serial.println("Error receiving data from gateway");
    }
  }
}

/*------------------------- VoLCano comm -----------------*/
void checkCommVolcano()
{
  // TODO: integrate with MAC
  // .... Whenever data from volcano is avaliable, then process it
    char buff[] = {THIS_NODE_ID, src_node_id};
    sendToSoftwareSerial(CMD_GREETING_REPLY, buff);
    timeout_volcano_time = millis();
    waitingReply_volcano = false;
    counter++;
  }

  // Check for communication timeout on VLC comm
  if(waitingReply_volcano)
  {
    if(millis() - timeout_volcano_time > TIMEOUT_MS)
    {
      Serial.println("Error VoLCano communication timeout");
      waitingReply_volcano = false;
    }
  }
}
