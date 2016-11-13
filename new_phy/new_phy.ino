//By Pranav Mani
//Logs:
//Updated PHY layer implementation, removed sync and unused functions
//Removed unnecesary class based implementation
//Updated updateBitQueue() fn, Added debugging options, phy_tx() working from main loop at 10Hz update rate, not working at higher rates,need update in architecture
//Unable to detect Rx pulses. Must be fixed either in code architecture by decreasing Freq

#include <QueueArray.h>
#include <math.h>

// Definitions
#define NODE_ADDR 0x01 // adress of current Node
#define LED_TX_PIN A1
#define LED_RX_PIN A0
#define DETECTION_THRESHOLD 0
#define PHY_IDLE_STATE 0
#define RX_LOW_THRESHOLD -1
#define RX_HIGH_THRESHOLD 50
#define PHY_TX 5
#define PHY_RX 6
#define TIMER1COUNT 61535 //Freq configuration for 500Hz
//#define TIMER1COUNT 64545    //65535-16Mhz/8/Freq currently configured for 2Khz
#define BYTE_LEN 8
#define TEST_TX

/*------------- global -------------*/
unsigned char currentLedState, currentPhyState;

// prototypes
void runPhyFSM(uint8_t currentState);

/*-----------transmitter function prototypes and variables ----------------------- */
bool transmitBusy; //bit used to indicate whether the phy layer is currently transmitting
bool bitTransmitcheck;
bool txStatus,currentBit,nextBit;
int8_t transmitCount; //bit that keeps count of sequence of true/false bits that are transmotted
QueueArray <bool> phy_tx_bit_queue; //tx bit queue
QueueArray <uint8_t> phy_tx_message_queue; //tx byte queue

// Tx prototype
void phy_tx(uint8_t* message, unsigned char len); //function called to transmit
void beginTransmit(); //starts actual transmission of message
void updateBitQueue(); //function used to convert the bytes to bits


/*-----------receiver function prototypes and variables ----------------------- */
bool fallingEdgeDet,risingEdgeDet, currentRx, prevRx, risingEdge, fallingEdge, startSlot;
int data_to_be_rec;   // starting point af data to be read
int byte_pos=0;   // index of byte in queue
char bitCount=0;  // counter of received bit
int16_t currentAnalogRx,prevAnalogRx; //variable used to store analogVoltage on the Rx Pins
QueueArray <bool> phy_rx_bit_queue; // rx bit queue
QueueArray <uint8_t> phy_rx_message_queue; //rx byte queue
uint8_t tx_message;
// Receiver buffer
uint8_t rx_buffer[263]; //message used to store the received data
uint8_t rx_buff[263]; //test variable used to cpy received data

// Rx prototype
bool phySense(); //function used to sense the physical layer
bool phyDetect(); // function used to detect the data on the analog port
void formRxByte(); //function used to form the byte
int phy_rx(uint8_t* rx_buffer); //function called to copy the data onto the rx_buffer, returns -1 if no data is available

//idle state-ON,OFF,ON
  //bool beginTransmitcheck=false;




bool prevData=true; //returns true if node is idle


bool LedState=true;
long prevTime;
bool data_available=false;
bool wait_true= true;
bool wait_false=true;

/*------------------- Microcontroller setup functions ----------------------*/

// Interrupt service routine
ISR(TIMER1_OVF_vect)
{
  // ISR called every time timer register overflows
  phyDetect(); //function used to check whether any data is available
  // Serial.print("MQC=");
  // Serial.println(phy_tx_message_queue.count());
  // Serial.print("BQC=");
  // Serial.println(phy_tx_bit_queue.count());

  if(phy_tx_bit_queue.count()){ //checks whether any data is available in the buffer. If yes, then transmits it
    currentPhyState = PHY_TX;
  }
  else
  {
    currentPhyState = PHY_IDLE_STATE;
  }
  runPhyFSM(currentPhyState);
  TCNT1 = TIMER1COUNT;
}

// FIXME: used for testing only
#ifdef TEST_TX
unsigned char tx_data[]={0xFF,0x02,0x01,0x00,0x0A,0x01,0x02,0x03,0x04,0x05};
#endif

// Arduino Setup
void setup()
{
  initializeTimerLed();
  Serial.begin(9600);
  currentPhyState = PHY_IDLE_STATE;
}

#ifdef TEST_TX
unsigned long pTime;  // FIXME: used for testing only
#endif
// Arduino Loop
void loop()
{
  #ifdef TEST_TX
  if(millis() - pTime >= 1000){ //Test Case for transmission every one sec
    phy_tx(tx_data,0x0A);
    pTime=millis();
  }
  #endif
}

/*----------------------- Functions ---------------------*/
/*---------------------- Global functions ---------------*/
void printBool(bool state){
  //Fn used to debug
    switch(state){
      case true:
        Serial.println("true");
        break;
      case false:
        Serial.println("false");
        break;
    }
}

/*--------------------- Rx functions ---------------------*/
// Function that is used to form the Byte from the received bits
void formRxByte()
{
  char byte=0x00;
    for(int j=0;j<8;j++){
      if(phy_rx_bit_queue.dequeue()){
             byte = byte|(0x01<<j);
      }
      else{
        byte=byte;
      }
    }
    phy_rx_message_queue.enqueue(byte);
}


bool wait;

// Function used to decode each byte and make a frame
void decodeFrame(uint8_t byte){
  if(byte_pos == 0){
    if(byte == 0xFF){
        byte_pos++;
        rx_buffer[0] = byte;
        data_available = false;
    }
  }
  // header
  else if(byte_pos >= 1 && byte_pos <= 3){
    byte_pos++;
    rx_buffer[byte_pos] = byte;
  }
  // start data
  else if(byte_pos == 4){
    rx_buffer[byte_pos] = byte;
    data_to_be_rec = byte;
    byte_pos++;
  }
  // data
  else if(byte_pos > 4 && byte_pos < (4 + data_to_be_rec)){
    rx_buffer[byte_pos]=byte;
    byte_pos++;
    data_available=true;
  }
  // reaching end of data
  else if(byte_pos == (4+data_to_be_rec)){
    byte_pos=0;
  }
}

// Sense physical layer status
bool phySense()
{
  if(currentPhyState == PHY_IDLE_STATE){
    return true; //returns true if idle
  }
  else{
    return false;
  }
}

// Begin transmitting data
void beginTransmit()
{
  // actual bit trasmission takes place
  if(!transmitBusy)
  {
    if(phy_tx_bit_queue.count()){
      currentBit = phy_tx_bit_queue.dequeue();
    }
    transmitCount=0;
  }
  // send bit 1
  if(currentBit == true)
  {
    if(transmitCount == 0)
      {
       transmitBusy = true;
       transmitCount++;
       digitalWrite(LED_TX_PIN,LOW);
//       Serial.println("LOW t");
      }
      else if(transmitCount == 1)
      {
        digitalWrite(LED_TX_PIN,HIGH);
        transmitBusy = false;
        transmitCount = 0;
//        Serial.println("HIGH");
      }
    }
    // send bit 0
    else if(currentBit==false)
    {
      if(transmitCount==0)
      {
       transmitCount++;
       transmitBusy=true;
       digitalWrite(LED_TX_PIN,HIGH); //2-ppm encoding used to encode and send 0s and 1s
//       Serial.println("HIGH f");
      }
      else if (transmitCount==1)
      {
        digitalWrite(LED_TX_PIN,LOW); //2-ppm encoding used to encode and send 0s and 1s
        transmitBusy=false;
        transmitCount=0;
//        Serial.println("LOW");
      }
    }

}

// Function that updates the bit queue that needs to be sent
void updateBitQueue()
{
  txStatus = true;
  if(phy_tx_message_queue.count()) {
    tx_message = phy_tx_message_queue.dequeue();
    // Serial.println(tx_message,HEX);
    for(int i = 0; i < BYTE_LEN; i++)
    {
      // finding the information in each bit
      if(((tx_message >> i & 0x01)) == 0x01)
      {
        phy_tx_bit_queue.enqueue(true);
        printBool(true);
      }
      else if(((tx_message >> i & 0x01)) == 0x00)
      {
        phy_tx_bit_queue.enqueue(false);
        printBool(false);
      }
    }
  }
}

void phy_tx(uint8_t *message, uint8_t len)
{
//  phy_tx_message_queue.enqueue(0xFF);
  for(int i=0; i<len; i++)
  {
//    Serial.println(message[i],HEX); //uncomment to debug
    phy_tx_message_queue.enqueue(message[i]); //queues the message onto the tx queue
//    Serial.println(phy_tx_message_queue.count());
    updateBitQueue();
//    Serial.println("MQC=");
//    Serial.println(phy_tx_message_queue.count()); //uncomment to debug
//    Serial.print("CQC=");
//    Serial.println(phy_tx_bit_queue.count());
    }
    currentPhyState=PHY_TX;
  //does not transmit of in idle state..waits for idle state to finish transmittinfg
//  if(currentPhyState!= PHY_RX)
//  {
//      currentPhyState=PHY_TX;
//    Serial.println("TX"); //uncomment to debug
//     updateBitQueue();

//  }
}

int phy_rx(uint8_t* rx_data)
{
  //copies the data in the rx_buffer to whatever variable you specify to
  if(data_available){
    if(rx_buffer[2] == NODE_ADDR){
     for(int i=0;i<(rx_buffer[4]+3);i++){
      rx_data[i]=rx_buffer[i];
      data_available=false;
      return rx_buffer[4];
     }
  }}

  else{
    return -1;
  }
}

//bool idleState=false;
int idleVar=0;
// state machine for the PHY layer
void runPhyFSM(uint8_t currentState)
{
  // Serial.println("FSM");
  switch(currentState)
  {
    case PHY_IDLE_STATE:
      // Serial.println("IDLE");  //uncomment to debug
      transmitIdle();
      break;
    case PHY_TX:
      // Serial.println("TD");  //uncomment to debug
      beginTransmit(); //actual bit-by-bit transmission takes place
      break;
  }
}

// transmits zero during idle
void transmitIdle()
{
  digitalWrite(LED_TX_PIN,LOW);
}

// Function that initializes timer and LED pin
void initializeTimerLed()
{
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;

  TCNT1 = TIMER1COUNT;
  TCCR1B |= (1 << CS11);
  TIMSK1 |= (1 << TOIE1);
  interrupts();
  pinMode(LED_TX_PIN,OUTPUT);
  pinMode(LED_RX_PIN,INPUT);
}

bool phyDetect()
{
  currentAnalogRx = analogRead(LED_RX_PIN);

  if(currentRx==prevRx)
  {
    if(wait_true==true){
      wait_true==false;
//      currentPhyState=PHY_IDLE_STATE; //high for more than two slots, then go to idle
    }

    else if(wait_false==true){
      wait_false==false;
//      currentPhyState=PHY_IDLE_STATE; //low for more than two slots, then go to idle
    }
    if(currentRx==false)
    {
      wait_true=true;
      wait_false=false;
    }

    if(currentRx==true)
    {
      wait_false=true;
      wait_true=false;
    }
  }
  if(currentRx>prevRx && prevData!=false) // detects rising edge(true) and prevents misdetection of false
  {
//    Serial.println("RE"); //uncomment to debug
    currentPhyState=PHY_RX; //setting this variable so that phySense() does not improperly detect this as idle State
    risingEdge=true;
    fallingEdge=false;
    phy_rx_bit_queue.enqueue(true);
    prevData=true;
    bitCount++;
  }

  if(currentRx<prevRx && prevData!=true) // detects falling edge(false) and prevents misdetection of true
  {
//    Serial.println("FE"); //uncomment to debug
    currentPhyState=PHY_RX; //setting this variable so that phySense() does not improperly detect this as idle State
    risingEdge=false;
    fallingEdge=true;
    phy_rx_bit_queue.enqueue(false);
    prevData=false;
    bitCount++;
  }

  if(bitCount == 8){ //once 8 bits are received, a byte is formed from it.
//    Serial.println("BF"); //uncomment to debug
//    currentPhyState=PHY_RX; //setting this variable so that phySense() does not improperly detect this as idle State
    bitCount = 0;
    formRxByte();
    unsigned char tmpByte = phy_rx_message_queue.dequeue();
//    Serial.print(tmpByte, HEX);
//    Serial.println();
    decodeFrame(tmpByte);
  }
  prevAnalogRx=currentAnalogRx;
  prevRx=currentRx;
}
