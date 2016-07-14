/* This code must be run on the Arduino connected to XBEE pro 2SB
 * The XBEE-PRO must run in API mode-2
 * This code will send message requesting the RoutingNode to send its adc value.
 * This code uses soft serial library and TimerOne library (16bit timer)
 * 
 *  
 * Precautions/Assumptions:
 * 1. Ensure all Nodes use the same PANID as set for Coordinator's
 * 2. Ensure the Node's destination and Source addresses are defined correctly
 * 3. If using encryption Ensure Encryption is enabled via XCTU and the encryption key is same on all nodes.
 * 4. Configuration of the node is done via XCTU software of Digi.
 * 5. All node names must start with N followed by numeric number of the respective node for this code to work
 * 
 * Date created: 12-JUL-2016
 * Author: Rohit Sivakumar
 * Description: Receive Vibration Sensor values on Coordinator when coordinator node request a specific router node.
 */

 /* XCTU Configuration
  *  PANID: 25
  *  SCAN CHANNEL: 0x7FFF
  *  ZS ZigbeeStack Profile: 2
  *  EE: Encryption Status: Disabled
  *  DH: Destination address (HIGH): 0x0000 0000
  *  DL: Destination address (LOW):  0x0000 FFFF  - broadcast mode
  *  API Output mode (AO) should be Native(0)
  */


#include <XBee.h>
#include <SoftwareSerial.h>
#include <TimerOne.h>
#define NODE_VERSION 1.0                //Do not change this unless you have modified your software as per change requests
#define NODE_TYPE "COORDINATOR"         //Do not change this string. This is for Router/End nodes.


#define DEBUG_NORMAL      1         //Enable this to see normal debug messages
#define DEBUG_FRAME_LEVEL 0        //Enable this to see frame level debug messages
#define DEBUG_INDEPTH     0         //Enable this to see indepth debug messages
//#define NODE_SR '2'             //Change this char to the respective Node's serial number. Like '2', '3', '4',...
#define NODE_NAME "MAIN-Coordinator "        //Node identifier name.  
#define MAX_NODES  3            //Change this value to equate as per the total routing/end nodes you wish to have in your network

XBee xbee = XBee();

volatile static char sendFlag = 0;
static uint8_t count = 1;  //must always begin with 1
uint8_t payload[13]= {0,0,0,0,0,0,0,0,0,0,0,0,0};

//XBeeAddress64 CoordinatorAddr64 = XBeeAddress64(0x0013A200, 0x40C8CA96); //Put the address of the node explicitly.
XBeeAddress64 CoordinatorAddr64 = XBeeAddress64(0x00000000, 0x0000FFFF); //The address DH:DL = 0 for coordinator always
XBeeResponse response = XBeeResponse();
ZBRxResponse rx = ZBRxResponse();
ZBTxRequest zbTx = ZBTxRequest(CoordinatorAddr64, payload, sizeof(payload));
ZBTxStatusResponse txStatus = ZBTxStatusResponse();
ModemStatusResponse msr = ModemStatusResponse();


//Define new Soft-serial Tx/Rx pins - Only digital pins
//We will use pin2 and pin3
#define ssRX 2
#define ssTX 3
SoftwareSerial nss(ssRX,ssTX);

const int STATUS_LED = 12;
const int ALIVE_LED  = 13;
const int VIBRATION_SENSOR = A0;

//String checkStr;  //this variable will be used to check the incoming string from coordinator.

/* This function is used for indicating activity of transmission */
void flashLed(int pin, int times, int wait)
{
  for(int i=0; i< times; i++)
  {
    digitalWrite(pin, HIGH);
    delay(wait);
    digitalWrite(pin, LOW);

    if(i+1 < times)
    {
      delay(wait);  //wait time is in ms
    }
  }
}

void toggleStateFn(void) // timer compare interrupt service routine
{
  digitalWrite(ALIVE_LED, !digitalRead(ALIVE_LED));
  sendFlag = 1;
}

void setup() 
{
  // put your setup code here, to run once:
  pinMode(STATUS_LED, OUTPUT);
  pinMode(ALIVE_LED, OUTPUT);
  
   //initialize timer-1 (16 bit timer) for normal timer overflow mode
  Timer1.initialize(1000000); //defined in value/1000000 toggleStateFn called every 500 milliseconds.
  Timer1.attachInterrupt(toggleStateFn);
  
  Serial.begin(9600);
  nss.begin(9600);
  xbee.setSerial(nss);
  
  Serial.println(("ZB Node-" + String(NODE_TYPE)+ " FW Version:"+ String(NODE_VERSION)));
  Serial.println(("Arduino Setup Finished on " + String(NODE_NAME) + ". Console should show up!"));
  Serial.println("------------------------------------------------------------------------------");
} 

void loop() 
{
  // put your main code here, to run repeatedly
  String sample;          //Our incoming messages from coordinator will be collected in this variable.
  uint16_t senderShortAddress;
  
 
  //We must first read the incoming packet to check if we were asked by coordinator to send data.
  xbee.readPacket();
  if(xbee.getResponse().isAvailable())
  {
    //Process if we have received a valid frame.
#if(DEBUG_FRAME_LEVEL)
     //we have got something
    Serial.print("\nFrame type:0x");
    Serial.println(xbee.getResponse().getApiId(),HEX);
#endif    
    if(xbee.getResponse().getApiId() == ZB_RX_RESPONSE)
    {
      //We got a ZB receive packet
      //Fill it in our ZB rx class
      xbee.getResponse().getZBRxResponse(rx);
#if(DEBUG_FRAME_LEVEL)
      //Get the 64bit address of source node from incoming packet
      Serial.print("Source Node address:0x");
      XBeeAddress64 senderLongAddress = rx.getRemoteAddress64();
      Serial.print(senderLongAddress.getMsb(),HEX);
      Serial.println(senderLongAddress.getLsb(),HEX);

      //Get the 16bit address of source node from incoming packet.
      Serial.print("Source Node short address:0x");
      senderShortAddress = rx.getRemoteAddress16();
      Serial.println(senderShortAddress,HEX);
#endif      

        //Receive only if it has come from a VALID coordinator
        //if(senderShortAddress == 0x0000)
       //{
            for(int i=0; i< rx.getDataLength(); i++)
            {
                if((char)rx.getData(i) != '\0' && rx.getData(i) != 0x0D && rx.getData(i) != 0x0A )
                {
                    sample += (char)rx.getData(i);
                }
            }
            Serial.println(sample);  //print locally
       
                      
            if(rx.getOption() == ZB_PACKET_ACKNOWLEDGED)
            {
              //Sender got an ACK
              //SUCCESS 
              flashLed(STATUS_LED, 10, 10);   
            }
            else
            {
              //We got it but sender didn't get an ACK
              //ERROR
              flashLed(STATUS_LED, 2, 20);
            }
            //Serial.println(rx.getData(0));
        //}//end of if(senderShortAddress == 0x0000)
    } //end of if(xbee.getResponse().getApiId() == ZB_RX_RESPONSE)
    else if(xbee.getResponse().getApiId() == MODEM_STATUS_RESPONSE)
    {
      xbee.getResponse().getModemStatusResponse(msr);
      //Local xbee sends this response on certain events like
      //Association/Disaccociation.
      if(msr.getStatus() == ASSOCIATED)
      {
        //We are doing good. Flash Led
        flashLed(STATUS_LED, 10, 10);
        Serial.println("ASSOCIATED");
      }
      else if(msr.getStatus() == DISASSOCIATED)
      {
        //Not good.
        flashLed(STATUS_LED, 10, 10);  
        Serial.println("DIS-ASSOCIATED");     
      }
      else
      {
        //some other status
        flashLed(STATUS_LED, 5, 10);
      }
    } //end of  else if(xbee.getResponse().getApiId() == MODEM_STATUS_RESPONSE)
    else
    {
      //not something we were expecting
      flashLed(STATUS_LED, 1, 25);
    }
  }
  else if(xbee.getResponse().isError())
  {
    #if(DEBUG_INDEPTH)
      Serial.print("Error reading packet. Error Code:");
      Serial.println(xbee.getResponse().getErrorCode());
    #endif
  }

  //This flag indicates that the coordinator has requested this node to send the payload.
  if(sendFlag)
  {  
    sendFlag = 0;
    //Fill in the payload for sending.
    payload[0] = 'N';
    if(count <= MAX_NODES && count !=0)
    {
      payload[1] =  ('0' + count);  //NODE_SR;  
    }
    count++ ;
    if(count > MAX_NODES)
    {
      count = 1;
    }
    //payload[1] = (uint8_t)count,HEX; 
    payload[2] = '\r'; //always terminate 
    #if(DEBUG_NORMAL)
       //Serial.println((char*)payload);  //for debug purpose
     #endif
   
    //Send the payload from Routing Node to Coordinator node.
    xbee.send(zbTx);
    
    //Flash indicator one time to indicate sending
    flashLed(STATUS_LED, 1, 100);

    //Now check if we got a status response for the sent packet. 
    xbee.readPacket();
    if (xbee.getResponse().isAvailable()) 
    //if(xbee.readPacket(5))
    {
        //We got a response
        if(xbee.getResponse().getApiId() == ZB_TX_STATUS_RESPONSE)
        {
          xbee.getResponse().getZBTxStatusResponse(txStatus);
          //get the delivery status (i.e. 5th byte)
          if(txStatus.getDeliveryStatus() == SUCCESS)
          {
            flashLed(STATUS_LED, 5, 50);
            #if(DEBUG_INDEPTH)
             Serial.println("Remote Xbee receive data");
            #endif
          }
          else
          {
            //the Remote xbee did not receive our packet.
            flashLed(STATUS_LED, 3, 500);
            #if(DEBUG_INDEPTH)
              Serial.println("Remote Xbee did not receive data");
            #endif
          }
        }
        else if(xbee.getResponse().isError())
        {
          #if(DEBUG_INDEPTH)
            Serial.print("Error Reading Packet. Error code:");
            Serial.println(xbee.getResponse().getErrorCode());
          #endif
        }
        else
        {
          //local xbee did not provide a timely tx status response.
          //This should not happen.
          flashLed(STATUS_LED, 2, 50);
          #if(DEBUG_INDEPTH)
            Serial.println("Local Xbee did not send a timely tx status response.");
          #endif
        }
      }//end of  if(xbee.readPacket(500))
  }//end of if(sendFlag)
  delay(100);
}
