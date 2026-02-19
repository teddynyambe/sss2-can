// PlatformIO requires Arduino.h explicitly
#include <Arduino.h>

/*
 * Smart Sensor Simulator 2
 * 
 * Arduino Sketch to enable the functions for the SSS2
 * 
 * Written By Dr. Jeremy S. Daily
 * The University of Tulsa
 * Department of Mechanical Engineering
 * 
 * 22 May 2017
 * 19 May 2018
 * 
 * Released under the MIT License
 *
 * Copyright (c) 2017        Jeremy S. Daily
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 * 
 * 
 * Uses Arduino 1.8.5 and Teensyduino 1.41
*/

#include "SSS2_board_defs_rev_5.h"
#include "SSS2_functions.h"

// J1939 static NAME and address for SSS2
#define J1939_STATIC_NAME 0x81228409E9000001ULL
#define J1939_STATIC_ADDRESS 0x80
#define J1939_PGN_PROPRIETARY_B 0x00EF00

// J1939 Address Claim message (8 bytes)
uint8_t j1939_name_bytes[8] = {
  0x01, 0x00, 0x00, 0xE9, 0x09, 0x84, 0x22, 0x81 // LSB first (NAME = 0x81228409E9000001)
};

// Helper: Compose J1939 29-bit ID for a PDU1 (addressed, PF < 240) message
uint32_t j1939_pgn1_id(uint8_t priority, uint8_t dp, uint8_t pf, uint8_t dest, uint8_t sa) {
  return ((uint32_t)(priority & 0x7) << 26) | ((uint32_t)(dp & 0x1) << 24)
       | ((uint32_t)pf << 16) | ((uint32_t)dest << 8) | sa;
}

// Helper: Send J1939 Address Claim
void j1939_send_address_claim() {
  CAN_message_t msg = {};  // zero-init clears flags.remote (rtr) to 0
  msg.id  = j1939_pgn1_id(6, 0, 0xEE, 0xFF, J1939_STATIC_ADDRESS); // 0x18EEFF80
  msg.ext = 1;
  msg.len = 8;
  for (int i = 0; i < 8; i++) msg.buf[i] = j1939_name_bytes[i];
  Can1.write(msg);
}

// Helper: Check if an incoming Address Claim targets our SA (contention)
bool is_j1939_address_claim_for_our_sa(const CAN_message_t& msg) {
  if (!msg.ext) return false;
  uint8_t pf = (msg.id >> 16) & 0xFF;
  uint8_t sa = msg.id & 0xFF;
  return (pf == 0xEE && sa == J1939_STATIC_ADDRESS);
}

// Helper: Check if incoming message is a Request PGN for our address claim
bool is_j1939_request_for_address_claim(const CAN_message_t& msg) {
  if (!msg.ext) return false;
  uint8_t pf = (msg.id >> 16) & 0xFF;
  uint8_t ps = (msg.id >> 8) & 0xFF;
  if (pf != 0xEA) return false;
  if (ps != J1939_STATIC_ADDRESS && ps != 0xFF) return false;
  if (msg.len < 3) return false;
  return (msg.buf[0] == 0x00 && msg.buf[1] == 0xEE && msg.buf[2] == 0x00);
}

// Helper: Send "Cannot Claim Address" (SA = 0xFE, NAME = ours)
void j1939_send_cannot_claim() {
  CAN_message_t msg = {};  // zero-init clears flags.remote (rtr) to 0
  msg.id  = j1939_pgn1_id(6, 0, 0xEE, 0xFF, 0xFE); // 0x18EEFFFE
  msg.ext = 1;
  msg.len = 8;
  for (int i = 0; i < 8; i++) msg.buf[i] = j1939_name_bytes[i];
  Can1.write(msg);
}

// Helper: Check if CAN message is J1939 PGN 0x00EF00
bool is_j1939_pgn_ef00(const CAN_message_t& msg) {
  // Extract PGN from 29-bit ID
  uint32_t pgn = (msg.id >> 8) & 0xFFFF;
  pgn |= ((msg.id >> 16) & 0xFF) << 16;
  return (pgn == J1939_PGN_PROPRIETARY_B);
}

// Handle incoming J1939 proprietary command (PGN 0x00EF00)
void handle_j1939_command(const CAN_message_t& msg) {
  // Example: Byte 0 = setting index, Byte 1 = value (expand as needed)
  if (msg.len < 2) return;
  uint8_t setting = msg.buf[0];
  int16_t value = msg.buf[1];
  // Optionally, parse more bytes for multi-byte values
  // Call same logic as serial command
  setSetting(setting, value, DEBUG_ON);
  Serial.printf("INFO: Setting %d updated to %d via CAN (PGN 0x00EF00)\n", setting, value);
}

// J1939 address claim state
bool j1939_address_claimed = false;
bool j1939_address_failed  = false;
elapsedMillis j1939_claim_timer;
bool j1939_prev_ignition = false;   // tracks previous ignitionCtlState for edge detection


//softwareVersion
String softwareVersion = "SSS2*REV" + revision + "*1.1*master*feaf4593b427b5ebe96f3d96ef746ec59d722ef9"; //Hash of the previous git commit

void listSoftware(){
  Serial.print("FIRMWARE ");
  Serial.println(softwareVersion);
}

void setup() {
  SPI.begin();
  SPI1.begin();
  //while(!Serial); //Uncomment for testing
  
  if(MCPCAN.begin(MCP_ANY, getBAUD(BAUDRATE_MCP), MCP_16MHZ) == CAN_OK) Serial.println("MCP2515 Initialized Successfully!");
  else Serial.println("Error Initializing MCP2515...");
  MCPCAN.setMode(MCP_NORMAL);   // Change to normal mode to allow messages to be transmitted

  
  LIN.begin(19200);
   
  commandString.reserve(256);
  commandPrefix.reserve(20);
  
  kinetisUID(uid);
  print_uid();
  
  analogWriteResolution(12);
  
  setSyncProvider(getTeensy3Time);
  setSyncInterval(1);
  
  setPinModes();
  
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);
  Wire.setDefaultTimeout(20000); // 20ms
   
  PotExpander.begin(potExpanderAddr);  //U33
  ConfigExpander.begin(configExpanderAddr); //U21
  for (uint8_t i = 0; i<16; i++){
    PotExpander.pinMode(i,OUTPUT);
    ConfigExpander.pinMode(i,OUTPUT);
  }
  PotExpander.writeGPIOAB(0xFFFF);
  ConfigExpander.writeGPIOAB(0xFFFF);
  
  setTerminationSwitches();
  setPWMSwitches();
  
  uint16_t configSwitchSettings = setConfigSwitches();
  
  button.attachClick(myClickFunction);
  button.attachDoubleClick(myDoubleClickFunction);
  button.attachLongPressStart(longPressStart);
  button.attachLongPressStop(longPressStop);
  button.attachDuringLongPress(longPress);
  button.setPressTicks(2000);
  button.setClickTicks(250);
 
  initializeDACs(Vout2address);
    
  for (int i = 1; i < numSettings; i++) {
    currentSetting = setSetting(i, -1,DEBUG_OFF);
    setSetting(i, currentSetting ,DEBUG_ON);
  }
  listInfo();

 
  
  

  Can0.begin(BAUDRATE0);
  Can1.begin(BAUDRATE1);
  
  Can0.startStats();
  Can1.startStats();

  //leave the first 4 mailboxes to use the default filter. Just change the higher ones
  allPassFilter.id=0;
  allPassFilter.ext=1;
  allPassFilter.rtr=0;
  for (uint8_t filterNum = 4; filterNum < 16;filterNum++){
    Can0.setFilter(allPassFilter,filterNum); 
    Can1.setFilter(allPassFilter,filterNum); 
  }

  txmsg.ext = 1;
  txmsg.len = 8;
  
 
  setConfigSwitches();
    
  LIN.begin(19200,SERIAL_8N2);

  J1708.begin(9600);
  J1708.clear();
  
  CANTimer.begin(runCANthreads, 1000); // Run can threads on an interrupt. Produces very little jitter.

  currentSetting = 1;
  knobLowLimit = 1;
  knobHighLimit = numSettings - 1;

  LINfinished = true;
  getCompIdEEPROMdata();
 
  commandString="1";
  setEnableComponentInfo();
  reloadCAN();

  // J1939: address claim deferred until ignition turns on
}

void loop() {
  //Check CAN messages
  if (Can0.available()) {
    Can0.read(rxmsg);
    //parseJ1939(rxmsg);
    RXCount0++;
    RXCAN0timer = 0;
    if (displayCAN0) printFrame(rxmsg, -1, 0, RXCount0);
    redLEDstate = !redLEDstate;
    digitalWrite(redLEDpin, redLEDstate);

    // J1939: only active while ignition is on
    if (ignitionCtlState) {
      // Handle Address Claim contention
      if (is_j1939_address_claim_for_our_sa(rxmsg)) {
        uint64_t their_name = 0;
        for (int i = 7; i >= 0; i--) their_name = (their_name << 8) | rxmsg.buf[i];
        if (their_name < J1939_STATIC_NAME) {
          j1939_address_claimed = false;
          j1939_address_failed  = true;
          j1939_send_cannot_claim();
          Serial.printf("ERROR: J1939 address %d lost to contender (lower NAME).\n", J1939_STATIC_ADDRESS);
        } else {
          // We win; re-assert our claim and restart the 250ms timer
          j1939_send_address_claim();
          j1939_claim_timer = 0;
          j1939_address_claimed = false;
        }
      }

      // Respond to Request PGN for address claim (J1939-81 s4.5.2)
      if (is_j1939_request_for_address_claim(rxmsg)) {
        if (j1939_address_failed) j1939_send_cannot_claim();
        else                      j1939_send_address_claim();
      }
    }

    // J1939: Check for proprietary command PGN 0x00EF00
    if (is_j1939_pgn_ef00(rxmsg)) {
      handle_j1939_command(rxmsg);
    }
  }
  if (Can1.available()) {
    Can1.read(rxmsg);
    RXCount1++;
    RXCAN1orJ1708timer = 0;
    if (displayCAN2) printFrame(rxmsg, -1, 2, RXCount1);
    if (ignitionCtlState){
      greenLEDstate = !greenLEDstate;
      digitalWrite(greenLEDpin, greenLEDstate);
    }

    // J1939: only active while ignition is on
    if (ignitionCtlState) {
      // Handle address claim contention and requests on Can1
      if (is_j1939_address_claim_for_our_sa(rxmsg)) {
        uint64_t their_name = 0;
        for (int i = 7; i >= 0; i--) their_name = (their_name << 8) | rxmsg.buf[i];
        if (their_name < J1939_STATIC_NAME) {
          j1939_address_claimed = false;
          j1939_address_failed  = true;
          j1939_send_cannot_claim();
          Serial.printf("ERROR: J1939 address %d lost to contender (lower NAME).\n", J1939_STATIC_ADDRESS);
        } else {
          j1939_send_address_claim();
          j1939_claim_timer = 0;
          j1939_address_claimed = false;
        }
      }
      if (is_j1939_request_for_address_claim(rxmsg)) {
        if (j1939_address_failed) j1939_send_cannot_claim();
        else                      j1939_send_address_claim();
      }
    }
  }
  //!digitalRead(INTCANPin) &&
  if(displayCAN1)  // If low, read receive buffer
  {
    if(MCPCAN.readMsgBuf(&rxId, &len, rxBuf)==CAN_OK){      // Read data: len = data length, buf = data byte(s)
    RXCount2++;
    rxmsg.id = (rxId & 0x1FFFFFFF);
    rxmsg.len = len;
    for(byte i = 0; i<len; i++) rxmsg.buf[i] = rxBuf[i];
    printFrame(rxmsg, -1, 1, RXCount2);
    }
  
  }
  
  //Check J1708
  if (J1708.available()){
    J1708RXbuffer[J1708_index] = J1708.read();
    J1708RXtimer = 0;
    newJ1708Char = true;
    
    J1708_index++;
    if (J1708_index > sizeof(J1708RXbuffer)) J1708_index = 0;
  }
  if (showJ1708 && newJ1708Char && J1708RXtimer > 1150) { //At least 11 bit times must pass
    //Check to see if this is the first displayed message. If so, discard and start showing subsequent messages.
    if (firstJ1708) firstJ1708 = false; 
    else{
      uint8_t j1708_checksum = 0;
      Serial.printf("J1708 %10lu.%06lu ",now(),uint32_t(microsecondsPerSecond));
      for (int i = 0; i<J1708_index;i++){
        j1708_checksum += J1708RXbuffer[i];
        Serial.printf("%02X ", J1708RXbuffer[i]);
      }
      if (j1708_checksum == 0) Serial.println("OK");
      else Serial.println("Checksum Failed.");
    }
    J1708_index = 0;
    newJ1708Char = false;
  }
    
  if (send_voltage){
    if (analog_tx_timer >= analog_display_period ){
      analog_tx_timer=0;
      Serial.print("ANALOG");
      Serial.printf(" %lu",uint32_t(analogMillis));
      for (uint8_t j = 0; j < numADCs; j++){
        Serial.printf(" %d",analogRead(analogInPins[j]));
      }
      Serial.print("\n");
    }
  }

  if (enableSendComponentInfo){
    if (canComponentIDtimer >5000){
      canComponentIDtimer = 0;
      can_messages[comp_id_index]->enabled = true;
      can_messages[comp_id_index]->transmit_number = 0;
      can_messages[comp_id_index]->ok_to_send = true;
      can_messages[comp_id_index]->loop_cycles = 0; 
      can_messages[comp_id_index]->cycle_count = 0;
      can_messages[comp_id_index]->message_index = 0;
    }
  }

  sendLINResponse();

  // J1939: rising edge of ignition → start address claim procedure
  if (ignitionCtlState && !j1939_prev_ignition) {
    j1939_address_claimed = false;
    j1939_address_failed  = false;
    j1939_send_address_claim();
    j1939_claim_timer = 0;
    Serial.printf("INFO: J1939 address claim sent (ignition ON, Addr=0x%02X)\n",
                  J1939_STATIC_ADDRESS);
  }
  // J1939: falling edge of ignition → release address
  if (!ignitionCtlState && j1939_prev_ignition) {
    j1939_address_claimed = false;
    j1939_address_failed  = false;
    Serial.printf("INFO: J1939 address released (ignition OFF).\n");
  }
  j1939_prev_ignition = ignitionCtlState;

  // J1939: Transition to claimed state after 250ms with no contention
  if (ignitionCtlState && !j1939_address_claimed && !j1939_address_failed
      && j1939_claim_timer >= 250) {
    j1939_address_claimed = true;
    Serial.printf("INFO: J1939 address 0x%02X claimed successfully.\n", J1939_STATIC_ADDRESS);
  }

  /****************************************************************/
  /*            Begin Serial Command Processing                   */
  if (Serial.available() >= 2 && Serial.available() < 256) {
    commandPrefix = Serial.readStringUntil(',');
    commandString = Serial.readStringUntil('\n');

    if      (commandPrefix.toInt() > 0)                   fastSetSetting();  
    else if (commandPrefix.equalsIgnoreCase("AI"))        displayVoltage();
    else if (commandPrefix.equalsIgnoreCase("B0"))        autoBaud0();
    else if (commandPrefix.equalsIgnoreCase("B1"))        autoBaud1();
    else if (commandPrefix.equalsIgnoreCase("BMCP"))      autoBaudMCP();
    else if (commandPrefix.equalsIgnoreCase("DB"))        displayBaud();
    else if (commandPrefix.equalsIgnoreCase("CANCOMP"))   setEnableComponentInfo();
    else if (commandPrefix.equalsIgnoreCase("ID"))        print_uid();
    else if (commandPrefix.equalsIgnoreCase("C0"))        startStopCAN0Streaming();
    else if (commandPrefix.equalsIgnoreCase("C1"))        startStopCAN1Streaming();
    else if (commandPrefix.equalsIgnoreCase("C2"))        startStopCAN2Streaming();
    else if (commandPrefix.equalsIgnoreCase("GO"))        startCAN();
    else if (commandPrefix.equalsIgnoreCase("SP"))        set_shortest_period();
    else if (commandPrefix.equalsIgnoreCase("STOPCAN"))   stopCAN();
    else if (commandPrefix.equalsIgnoreCase("STARTCAN"))  goCAN();
    else if (commandPrefix.equalsIgnoreCase("CLEARCAN"))  clearCAN();
    else if (commandPrefix.equalsIgnoreCase("STATS"))     displayStats();
    else if (commandPrefix.equalsIgnoreCase("CLEARSTATS"))clearStats();
    else if (commandPrefix.equalsIgnoreCase("CI"))        changeComponentID();
    else if (commandPrefix.equalsIgnoreCase("LS"))        listSettings();
    else if (commandPrefix.equalsIgnoreCase("OK"))        checkAgainstUID();
    else if (commandPrefix.equalsIgnoreCase("CANNAME"))   getThreadName();
    else if (commandPrefix.equalsIgnoreCase("CANSIZE"))   getThreadSize();
    else if (commandPrefix.equalsIgnoreCase("THREADS"))   getAllThreadNames();
    else if (commandPrefix.equalsIgnoreCase("SOFT"))      listSoftware();
    else if (commandPrefix.equalsIgnoreCase("J1708"))     displayJ1708();
    else if (commandPrefix.equalsIgnoreCase("SM"))        setupPeriodicCANMessage();
    else if (commandPrefix.equalsIgnoreCase("CANSEND"))   sendMessage();
    else if (commandPrefix.equalsIgnoreCase("RELOAD"))    reloadCAN();
    else if (commandPrefix.equalsIgnoreCase("TIME"))      displayTime();
    else if (commandPrefix.equalsIgnoreCase("GETTIME"))   Serial.printf("INFO Timestamp: %D\n",now());
    else if (commandPrefix.equalsIgnoreCase("LIN"))       displayLIN();
    else if (commandPrefix.equalsIgnoreCase("SENDLIN"))   sendLINselect();    
    else {
      Serial.println(("ERROR Unrecognized Command Characters. Use a comma after the command."));
      Serial.clear();
      //Serial.println(("INFO Known commands are setting numbers, GO, SP, J1708, STOPCAN, STARTCAN, B0, B1, C0, C1, C2, DS, SW, OK, ID, STATS, CLEAR, MK, LI, LS, CI, CS, SA, SS, or SM."));
    }
  }
  /*              End Serial Command Processing                   */
  /****************************************************************/

  /****************************************************************/
  /*            Begin Quadrature Knob Processing                  */
  button.tick(); //check for presses
  int32_t newKnob = knob.read(); //check for turns
  if (newKnob != currentKnob) {
    if (newKnob >= knobHighLimit) {  //note: knob limits are for each input parameter
      knob.write(knobHighLimit);
      currentKnob = knobHighLimit;
    }
    else if (newKnob <= knobLowLimit) {
      knob.write(knobLowLimit);
      currentKnob = knobLowLimit;
    }
    else
    {
      currentKnob = newKnob;
    }
    //Place function calls to execute when the knob turns.
    if (ADJUST_MODE_ON) {
      setSetting(currentSetting, currentKnob,DEBUG_ON);
    }
    else {
      currentSetting = currentKnob;
      setSetting(currentSetting,-1,DEBUG_ON);
    }
  }
  /*            End Quadrature Knob Processing                    */
  /****************************************************************/


  /****************************************************************/
  /*           Begin LED Indicators for messages                  */
  /*
  /*Reset the greenLED after a timeout in case the last CAN message was on the wrong state*/
  if (RXCAN0timer >= 200) { 
    RXCAN0timer = 0;
    redLEDstate = true;
    digitalWrite(redLEDpin, redLEDstate); //Use red because it is the power button.
  }
  
  /*Reset the greenLED after a timeout in case the last CAN message was on the wrong state*/
  if (RXCAN1orJ1708timer >= 200) { 
    RXCAN1orJ1708timer = 0;
    if (ignitionCtlState) greenLEDstate = true;
    else greenLEDstate = false;
    digitalWrite(greenLEDpin, greenLEDstate); 
  }
  /*             End LED Indicators for messages                        */
  /**********************************************************************/
}
