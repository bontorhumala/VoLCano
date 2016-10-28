/*----------------------------------------------------------------
 * VoLCano (Visible Light Communication on Arduino) gateway
 * 
 * Handle communication between telegram bot server and 
 * node mcu device.
 * 
 * Library used: 
 * - ESP8266Wifi
 * - Arduino Json
 * - Telegram-Bot Library
 *  
 * Andri Rahmadhani
 * Oct 2016 
 *-------------------------------------------------------------
 */
#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>

/*------------- TelegramBot Library ----------------------------*/
const char* host = "api.telegram.org"; //
const int httpsPort = 443; //
// Use web browser to view and copy //
// SHA1 fingerprint of the certificate //
//const char* fingerprint = "74 a2 e2 28 db 6a 50 8a 76 cb dd 6f ad 71 dc 34 b2 2c 8c e6"; //
const char* fingerprint = "37 21 36 77 50 57 f3 c9 28 d0 f7 fA 4c 05 35 7f 60 c1 20 44";
int update_id=0; //to store update_id chat //
struct message{ //
 String text; //
 int chat_id; //
 String sender; //
 String date; //
}; //
/*------------End of Telegram Bot Library-----------------------*/

// Initialize WiFi connection to the router
const char* ssid = "xflashA316i";
const char* passwd = "12345678";
 
// VoLCano telegram Bot token
// Get it on telegram's botfather
const char bot_token[] = "299066100:AAGhsHTpkI2jrNQ_24ylOZpsx-RifeDURLc";

// Create objects
WiFiClientSecure client;
//TelegramBot bot(BotToken, BotName, BotUsername, client);
 
void setup() {
  Serial.begin(115200);
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

  // Begin bot
//  bot.begin();
}
 
void loop() {
  delay(1000); // break for a second

  // Read new messages
  message m = getUpdates();

  // Checks if there are some updates
  if ( m.chat_id != 0 ) {
    // command parser
    //when receive /start command
    if (m.text.equals("/start")) { 
      String welcome = "Welcome to VoLCano bot, Visible Light Communication on Arduino.\n";
      welcome = welcome + "/status : to check LED status. \n";
      welcome = welcome + "/ledon : to switch the LED ON. \n";
      welcome = welcome + "/ledoff : to switch the LED OFF \n";
      sendMessage(m.chat_id, welcome);
    } else if ( m.text.equals("/ledon")) {
      Serial.println("message received");
      sendMessage(m.chat_id, "The LED is now ON");
    } else if ( m.text.equals("/ledoff")) {
      Serial.println("message received");
      sendMessage(m.chat_id, "The LED is now OFF");
    } else if(m.text.equals("/status")){
      sendMessage(m.chat_id, "DUMMY");
    } else{
      sendMessage(m.chat_id, "Invalid command");
    }
  } 
  else {
    Serial.println("no new message");
  }
}

/*------------------- TelegramBot Library ------------------------------------------- */ 
 
message getUpdates(){
// Use WiFiClientSecure class to create TLS connection
Serial.print("connecting to ");
Serial.println(host);
if (!client.connect(host, httpsPort)) {
Serial.println("connection failed");
//return;
}

if (client.verify(fingerprint, host)) {
Serial.println("certificate matches");
} else {
Serial.println("certificate doesn't match");
}
 
 // url for getupdates
 String url = "/bot";
 url += bot_token;
 url += "/getupdates?limit=1&offset=";
 url += update_id + 1;
 Serial.print("requesting URL: ");
 Serial.println(url);
  
 client.print(String("GET ") + url + " HTTP/1.1\r\n" +
 "Host: " + host + "\r\n" +
 "User-Agent: ESP8266Bot\r\n" +
 "Connection: close\r\n\r\n");
  
 Serial.println("request sent");
 String payload = readPayload();
 if (payload != "") {
 message m;
 DynamicJsonBuffer jsonBuffer; //json parser
 JsonObject & root = jsonBuffer.parseObject(payload);
 if (!root.success()) {
 Serial.println("parseObject() failed");
 //return;
 }
 update_id = root["result"]["update_id"];
 String sender = root["result"]["message"]["from"]["username"];
 String text = root["result"]["message"]["text"];
 m.chat_id = root["result"]["message"]["chat"]["id"];
 String date = root["result"]["message"]["date"];
  
 m.sender = sender;
 m.text = text;
 m.date = date;
 // Print values. for debugging
 Serial.println();
 Serial.println("=====================");
 Serial.println(sender);
 Serial.println(text);
 Serial.println(date);
 Serial.println(m.chat_id);
 Serial.println("=====================");
 return m;
 }
}
 
//construct url for send message
String sendMessage(int chat_id, String text) {
 String url;
 url = "chat_id=";
 url += chat_id;
 url += "&text=";
 url += text;
 return postMessage(url);
}
 
//get constructed url and set it
String postMessage(String msg) {
  Serial.print("connecting to ");
  Serial.println(host);
  if (!client.connect(host, httpsPort)) {
    Serial.println("connection failed");
  }
  
  if (client.verify(fingerprint, host)) {
    Serial.println("certificate matches");
  } else {
    Serial.println("certificate doesn't match");
  }
  
  String url;
  url = "/bot";
  url += bot_token;
  url += "/sendMessage";
  
  // This will send the request to the server
  client.print("POST ");
  client.print(url);
  client.println(" HTTP/1.1");
  client.print("Host: ");
  client.println(host);
  client.println("Connection: close");
  client.println("Content-Type: application/x-www-form-urlencoded");
  client.print("Content-Length: "); //lenght of content data to send
  client.println(msg.length());
  client.println();
  client.println(msg);//data to POST
  client.println();
  
  Serial.println("request sent");
  
  return readPayload();
}
 
// get output from Telegram 
String readPayload(){
  String line;
  while (client.connected()) {
    line = client.readStringUntil('\n');
    if (line == "\r") {
      Serial.println("headers received");
      break;
    }
  }
  line = client.readStringUntil('\r');
  line.replace("[","");
  line.replace("]","");
  Serial.println("reply was:");
  Serial.println("==========");
  Serial.println(line);
  Serial.println("==========");
  Serial.println("closing connection");
  return line;
}

/*-------------- End of Telegram Bot Library ----------------------------------------*/

