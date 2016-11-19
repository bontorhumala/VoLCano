/*----------------------------------------------------------------
 * VoLCano (Visible Light Communication on Arduino) gateway
 *
 * Handle communication between telegram bot server and
 * node mcu device.
 *
 * Library used:
 * - ESP8266Wifi
 * - Universal Telegram-Bot Library
 *
 * Andri Rahmadhani
 * Oct 2016
 *-------------------------------------------------------------
 */
#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <SoftwareSerial.h>
#include <UniversalTelegramBot.h>

// Timing
#define UPDATE_INTERVAL_MS      3000

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

// FSM read argument telegram
#define WAIT_TELE_CMD      0
#define WAIT_TELE_ARG_1    1
#define WAIT_TELE_ARG_2    2

// buffer
#define MAX_DATA_RX_BUFF    20

// timeout
#define TIMEOUT_MS          3000

// VoLCano telegram Bot token
// Get it on telegram's botfather
#define BOTtoken "299066100:AAGhsHTpkI2jrNQ_24ylOZpsx-RifeDURLc"

// Node ID
#define NODE_GATEWAY_ID   0
#define NODE_1_ID         1
#define NODE_2_ID         2

/*------------- TelegramBot Library ----------------------------*/
const char* host = "api.telegram.org"; //
const int httpsPort = 443; //
int update_id=0; //to store update_id chat //

// Stickers ID
const String stickers[] = {"BQADAgADvwADlIB3BSO3_Hx3PVABAg",   // Meow Nya Sticker - 0
  "BQADBAADhgQAApv7sgAB__9L9LBS7C0C", // Funny smile - 1
  "BQADBAADLgQAApv7sgABEbDOpTv0VEwC", // Axe angry - 2
  "BQADBAADCQQAAhmm3wABh7ovFpyee0oC", // Conan OK - 3
  "BQADBAADWgQAAhmm3wABLOucTa947x4C", // Bye bye conan - 4
  "BQADBAADGAQAAhmm3wABrZ4fZ5QcTSYC", // Hai conan - 5
  "BQADAgAEBQACYyviCQXAY0t2nQWlA", // Bye obama - 6
  "BQADAgAD0wQAAmMr4glSNeHBA4HAMAI", // Hi trump - 7
};

bool ledStatus = false;
/*------------End of Telegram Bot Library-----------------------*/

// Software Serial
char buf_in[MAX_DATA_RX_BUFF] = {0};
char cmd_in;
int buf_in_len = 0;
String pending_chat_id = "";
bool waitingReply = false;

// Initialize WiFi connection to the router
const char* ssid = "Redmi";
const char* passwd = "12345678";
//const char* ssid = "TP-LINK_721458";
//const char* passwd = "51827380";


// Create objects
WiFiClientSecure client;
// Telegram bot object
UniversalTelegramBot bot(BOTtoken, client);

// Software Serial
// GPIO 12 (D6) and 14 (D5)
SoftwareSerial swSer(14, 12, false, 256); // RX (D5), TX (D6)

// timing
unsigned long cur_time;
unsigned long timeout_time;

// FSM state
char telegram_state = WAIT_TELE_CMD;
String telegram_cmd = "";
char src_node_id = NODE_GATEWAY_ID;
char dst_node_id = NODE_GATEWAY_ID;

void setup() {
  Serial.begin(115200);
  swSer.begin(9600);

  // Wait for serial port initialization
  while (!Serial);

  Serial.println();
  Serial.print("Connecting WiFi to router: ");
  Serial.println(ssid);
  WiFi.begin(ssid, passwd);

  // Waiting for WiFi connected
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  // Print WiFi info
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("Local IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println("Subnet mask: ");
  Serial.println(WiFi.subnetMask());
  Serial.println("Gateway IP address: ");
  Serial.println(WiFi.gatewayIP());
  Serial.println("MAC address: ");
  Serial.println(WiFi.macAddress());

  // Set initial time
  cur_time = millis();
}

void loop() {
  // Get update from telegram server only if we are not waiting reply
  if(!waitingReply)
  {
    if(millis() - cur_time >= UPDATE_INTERVAL_MS)
    {
      int numNewMessages = bot.getUpdates(bot.last_message_recived + 1);
      while(numNewMessages) {
        Serial.println("got response");
        handleNewMessages(numNewMessages);
        numNewMessages = bot.getUpdates(bot.last_message_recived + 1);
      }

      // update time
      cur_time = millis();
    }
  }

  // Get update from software serial
  buf_in_len = readSoftwareSerial(&cmd_in, buf_in);
  // finished reading data
  if(buf_in_len > 0)
  {
    // print for debugging
    {
      int i;
      for(i=0; i<buf_in_len; i++)
      {
        Serial.print(buf_in[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
    }
    switch (cmd_in) {
      case CMD_GREETING_REPLY:
        // Check source
        switch(buf_in[0]) {
          case NODE_1_ID:
            bot.sendSticker(pending_chat_id, stickers[7]);
            break;
          case NODE_2_ID:
            bot.sendSticker(pending_chat_id, stickers[1]);
            break;
          default:
            bot.sendSimpleMessage(pending_chat_id, "Receiving from invalid node ID\n", "");
            break;
        }
        break;
      case CMD_BYE_REPLY:
        // Check source
        switch(buf_in[0]) {
          case NODE_1_ID:
            bot.sendSticker(pending_chat_id, stickers[6]);
            break;
          case NODE_2_ID:
            bot.sendSticker(pending_chat_id, stickers[0]);
            break;
          default:
            bot.sendSimpleMessage(pending_chat_id, "Receiving from invalid node ID\n", "");
            break;
        }
        break;
      case CMD_STATUS_REPLY:
        // Check source
        switch(buf_in[0]) {
          case NODE_1_ID:
            bot.sendSimpleMessage(pending_chat_id, "Node " + String(NODE_1_ID) + " -- OK\n", "");
            break;
          case NODE_2_ID:
            bot.sendSimpleMessage(pending_chat_id, "Node " + String(NODE_2_ID) + " -- OK\n", "");
            break;
          default:
            bot.sendSimpleMessage(pending_chat_id, "Receiving from invalid node ID\n", "");
            break;
        }
        break;
      case CMD_SPEEDTEST_REPLY:
        String msg;
        msg = "Node-" + String(buf_in[0]) + " to Node-" + String(buf_in[1]) + " (roundtrip): " + buf_in[2] + " bps\n";
        bot.sendSimpleMessage(pending_chat_id, msg, "");
        break;
      // default:
      //   break;
    }
    sendDefaultMessage(pending_chat_id);
    waitingReply = false;
  } else if(buf_in_len == -1) {
    Serial.println("Error receiving data from node");
    buf_in_len = 0;
    bot.sendSimpleMessage(pending_chat_id, "Internal communication error", "");
    sendDefaultMessage(pending_chat_id);
    waitingReply = false;
  }

  // Check timeout
  if(waitingReply)
  {
    if(millis() - timeout_time > TIMEOUT_MS)
    {
      Serial.println("Error internal timeout");
      bot.sendSimpleMessage(pending_chat_id, "Internal timeout", "");
      sendDefaultMessage(pending_chat_id);
      waitingReply = false;
    }
  }
}

/*------------------- TelegramBot Library ------------------------------------------- */

void handleNewMessages(int numNewMessages) {
  Serial.println("handleNewMessages");
  Serial.println(String(numNewMessages));
  for(int i=0; i<numNewMessages; i++) {
    String chat_id = String(bot.messages[i].chat_id);
    String text = bot.messages[i].text;

    // Process FSM
    switch(telegram_state)
    {
      case WAIT_TELE_CMD:
        // Welcome screen
        if (text == "/start") {
          String keyboardJson = "[[\"/start\"],[\"/greeting\",\"/bye\"],[\"/status\",\"/speedtest\"]]";
          String welcome = "# Welcome to VoLCano bot, Visible Light Communication on Arduino.\n";
          welcome += "/start : view welcome screen and list of commands \n";
          welcome += "/greeting : receive greeting from a node \n";
          welcome += "/bye : say bye to a node \n";
          welcome += "/status : returns current status of a node \n";
          welcome += "/speedtest : to test VoLCano connection speed \n";
          bot.sendMessageWithReplyKeyboard(chat_id, welcome, "Markdown", keyboardJson, true);
        }
        else if (text == "/greeting") {
          sendSelectNode(chat_id);
          telegram_cmd = "/greeting";
          telegram_state = WAIT_TELE_ARG_1;
        }
        else if(text == "/bye")
        {
          sendSelectNode(chat_id);
          telegram_cmd = "/bye";
          telegram_state = WAIT_TELE_ARG_1;
        }
        else if(text == "/status")
        {
          sendSelectNode(chat_id);
          telegram_cmd = "/status";
          telegram_state = WAIT_TELE_ARG_1;
        }
        else if(text == "/speedtest")
        {
          sendSelectNode(chat_id);
          telegram_cmd = "/speedtest";
          telegram_state = WAIT_TELE_ARG_1;
        }
        else  {
          // Invalid command
          bot.sendSticker(chat_id, stickers[2]);
          sendDefaultMessage(chat_id);
        }
        break;

      case WAIT_TELE_ARG_1:
        if (telegram_cmd == "/greeting") {
          if(text == "0") {
            bot.sendSticker(chat_id, stickers[5]);
            sendDefaultMessage(chat_id);
          } else {
            char buff[] = {NODE_GATEWAY_ID, text.toInt()};
            sendToSoftwareSerial(CMD_GREETING_REQUEST, buff);
            pending_chat_id = chat_id;
            timeout_time = millis();
            waitingReply = true;
          }
        }
        else if(telegram_cmd == "/bye")
        {
          if(text == "0") {
            bot.sendSticker(chat_id, stickers[4]);
            sendDefaultMessage(chat_id);
          } else {
            char buff[] = {NODE_GATEWAY_ID, text.toInt()};
            sendToSoftwareSerial(CMD_BYE_REQUEST, buff);
            pending_chat_id = chat_id;
            timeout_time = millis();
            waitingReply = true;
          }
        }
        else if(telegram_cmd == "/status")
        {
          if(text == "0") {
            bot.sendSimpleMessage(chat_id, "Node " + String(NODE_GATEWAY_ID) + " (GATEWAY) -- OK \n", "");
            sendDefaultMessage(chat_id);
          } else {
            char buff[] = {NODE_GATEWAY_ID, text.toInt()};
            sendToSoftwareSerial(CMD_STATUS_REQUEST, buff);
            pending_chat_id = chat_id;
            timeout_time = millis();
            waitingReply = true;
          }
        }
        else if(telegram_cmd == "/speedtest")
        {
          dst_node_id = text.toInt();
          sendSourceNode(chat_id);
          telegram_state = WAIT_TELE_ARG_2;
          break;
        }
        else
        {
          bot.sendSimpleMessage(chat_id, "Invalid command :( \n", "");
          sendDefaultMessage(chat_id);
        }

        telegram_state = WAIT_TELE_CMD;
        break;

      case WAIT_TELE_ARG_2:
        if (telegram_cmd == "/speedtest") {
          src_node_id = text.toInt();
          // Check if dst == src, if yes no need to send
          if(dst_node_id == src_node_id)
          {
            bot.sendSimpleMessage(chat_id, "Source node is equal to destination node", "");
          } else {
            char buff[] = {src_node_id, dst_node_id};
            sendToSoftwareSerial(CMD_SPEEDTEST_REQUEST, buff);
            pending_chat_id = chat_id;
            timeout_time = millis();
            waitingReply = true;
          }
        }

        sendDefaultMessage(chat_id);
        telegram_state = WAIT_TELE_CMD;
        break;
    }
  }
}

void sendDefaultMessage(String chat_id)
{
  String keyboardJson = "[[\"/start\"],[\"/greeting\",\"/bye\"],[\"/status\",\"/speedtest\"]]";
  bot.sendMessageWithReplyKeyboard(chat_id, "Select command:", "", keyboardJson, true, true, false, false);
}

void sendSelectNode(String chat_id)
{
  String keyboardJson = "[[\"0\",\"1\",\"2\"]]";
  String destNode = "Please select destination node: \n";
  destNode += "/0 : send to gateway \n";
  destNode += "/1 : send to node 1 \n";
  destNode += "/2 : send to node 2 \n";
  bot.sendMessageWithReplyKeyboard(chat_id, destNode, "", keyboardJson, true, true, false, true);
}

void sendSourceNode(String chat_id)
{
  String keyboardJson = "[[\"0\",\"1\",\"2\"]]";
  String srcNode = "Please select source node: \n";
  srcNode += "/0 : send from gateway \n";
  srcNode += "/1 : send from node 1 \n";
  srcNode += "/2 : send from node 2 \n";
  bot.sendMessageWithReplyKeyboard(chat_id, srcNode, "", keyboardJson, true, true, false, true);
}

/*-------------- End of Telegram Bot Library ----------------------------------------*/

/*------------ Software Serial - communicating with arduino -------------------------*/
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
