/*
 * File:        mega.ino
 * Author:      Richard Hemphill
 * Class:       ECE 5595 Computer Network Architecture
 * Description: Configures an Arduino Mega for the following:
 *  1. Can Chat - Team Zero sends 8 character messages to each other.
 *  2. Send LED ON/OFF command to Arduino Uno.
 *  3. Turn ON/OFF LED connected to Mega per command over CAN Bus.
 *  4. Display CAN Bus communication via 16x2 LCD.
 * Libraries:
 *  1. https://github.com/ivanseidel/ArduinoThread
 *  2. https://www.arduino.cc/en/Reference/SD
 *  3. https://github.com/Seeed-Studio/CAN_BUS_Shield
 *  4. https://www.arduino.cc/en/Reference/LiquidCrystal
 * CAN Bus Interface:
 * |--------------------------------|
 * |               ID               |
 * |01|02|03|04|05|06|07|08|09|10|11|
 * |Ch|   Source     |  Destination |
 * |at|DC|EH|WM|RH|JZ|DC|EH|WM|RH|JZ|
 * |--------------------------------|
 * |Chat|   Data Field (8 bytes)    |
 * |  0 | char text from src to dst |
 * |  1 | bits 2-11 = 0: cmd device |
 * | .. |   byte[0]: Device         |
 * | .. |     0 = Arduino Uno       |
 * | .. |     1 = Arduino Mega      |
 * | .. |   byte[1]: LED Value      |
 * | .. |     LOW  = LED Off        |
 * | .. |     HIGH = LED On         |
 * |--------------------------------|
 */

// includes
#include <Thread.h>           // thread class
#include <ThreadController.h> // scheduler for threads
#include <SPI.h>              // serial peripheral interface (i.e. CAN Bus port)
#include <mcp_can.h>          // mcp2515 driver function (i.e. CAN shield)
#include <LiquidCrystal.h>    // 16x2 liquid crystal display

// types
enum Device {
  UNO   = 0,    // Arduino Uno
  MEGA  = 1     // Arduino Mega
};

enum Zero {
  NONE  = B00000,   // none of the members
  JZ    = B00001,   // Joel Zamora
  RH    = B00010,   // Richard Hemphill
  WM    = B00100,   // William Mckinnon
  EH    = B01000,   // Eric Hansen
  DC    = B10000,   // Daniel Caballero
};

// global constants
const char ERROR_MSG[]              = "ERROR";        // text to indicate an error
const int SERIAL_SPEED              = 9600;           // baud rate for serial port
const int SERIAL_POLL_RATE          = 10;             // how fast the serial line should be check for user input
const int BUTTON_THREAD_INTERVAL    = 100;            // period for running button function
const int LED_PIN                   = 24;             // light emiting diod pin
const int BUTTON_PIN                = 22;             // push button pin
const int BUTTON_PUSHED             = HIGH;           // button pressed
const int SPI_CS_PIN                = 53;             // SPI chip select pin
const byte CAN_SPEED                = CAN_20KBPS;     // serial peripherial interface baud rate
const byte CAN_CLOCK                = MCP_8MHz;       // serial peripherial interface clock frequency
const char CAN_BUS[]                = "CAN Bus";      // CAN bus system
const char CAN_ID[]                 = "ID:";          // message for CAN ID field
const char CAN_LEN[]                = "LEN:";         // message for CAN data length field
const char CAN_DATA[]               = "DATA:";        // message for CAN data field
const char CAN_INIT[]               = "Initializing"; // CAN shield init state
const char CAN_INIT_RDY[]           = "Init Ready";   // CAN shield finished init
const char CAN_INIT_FAIL[]          = "Init Failed";  // CAN shield couldn't init
const char CAN_READ_FAIL[]          = "Read Failed";  // CAN shield couldn't receive message
const int CAN_SETUP_INTERVAL        = 50;             // period for trying to setup can bus
const int CAN_THREAD_INTERVAL       = 100;            // period for running CAN bus message receiver
const byte CAN_BASE_FRAME_FORMAT    = 0;              // 11 identifier bits (used with CAN.sendMsgBuf)
const byte CAN_EXT_FRAME_FORMAT     = 1;              // 29 identifier bits (used with CAN.sendMsgBuf)
const bool LCD_CONNECTED            = true;           // LCD is connected to board
const int LCD_RS_PIN                = 8;              // arduino pin connected to LCD register select pin
const int LCD_EN_PIN                = 9;              // arduino pin connected to LCD enable pin
const int LCD_D4_PIN                = 4;              // arduino pin connected to LCD D4 pin
const int LCD_D5_PIN                = 5;              // arduino pin connected to LCD D5 pin
const int LCD_D6_PIN                = 6;              // arduino pin connected to LCD D6 pin
const int LCD_D7_PIN                = 7;              // arduino pin connected to LCD D7 pin
const int NUM_LCD_COLS              = 16;             // number of columns that the LCD screen has
const int NUM_LCD_ROWS              = 2;              // number of rows that the LCD screen has
const int DISPLAY_SECTION_LIMIT     = NUM_LCD_COLS+1; // number of characters per display section (include null termination at end)
const unsigned int CAN_ID_CHAT      = 0x3ff;          // mask for can chat
const unsigned int CAN_SRC_SHIFT    = 5;              // number of bits to shift Zero member to set source
const unsigned int CAN_CHAT_MASK    = 0x400;          // mask to extract chat indication bit (b10000000000)
const unsigned int CAN_ZERO_MASK    = B11111;         // mask to extract team bits
const unsigned int CAN_DEVICE_CMD   = 0x400;          // id for commanding device (b10000000000)
const Device MY_DEVICE              = MEGA;           // current device
const char MY_DEV_NAME[]            = "Aduino Mega";  // text to indicate device commanded
const Device DEVICE_TO_COMMAND      = UNO;            // device to send LED On/Off command to
const char DEV_TO_CMD_NAME[]        = "Aduino Uno";   // text to indicate device commanded
const byte DEV_INDEX                = 0;              // index into data for device
const byte VAL_INDEX                = 1;              // index into data for value commanded
const char LED_ON_MSG[]             = "LED On";                 // text to indicate device LED commanded On
const char LED_OFF_MSG[]            = "LED Off";                // text to indicate device LED commanded Off
const char MSG_TOO_LONG[]           = "Msg Len Too Long";       // chat message too long for CAN bus data field
const char DIVIDER[]                = "--------------------";   // divides serial message groupings
const char ZERO_NONE[]              = "None";                   // No Zero Member
const char ZERO_NAME_JZ[]           = "Joel Zamora";            // Zero Member - Joel Zamora
const char ZERO_NAME_RH[]           = "Richard Hemphill";       // Zero Member - Richard Hemphill
const char ZERO_NAME_WM[]           = "William Mckinnon";       // Zero Member - William Mckinnon
const char ZERO_NAME_EH[]           = "Eric Hansen";            // Zero Member - Eric Hansen
const char ZERO_NAME_DC[]           = "Daniel Caballero";       // Zero Member - Daniel Caballero
const char ZERO_ALL[]               = "Everyone";               // All Zero Member
const char NUMBER_STYLE[]           = ". ";                     // seperation for bulleted numbers
const char QUERY_USER[]             = "Which member are you?";  // ask user for member identity
const char QUERY_TARGET[]           = "Which members should message be sent to?"; // ask user who message should be sent to
const char QUERY_ANOTHER[]          = "Another member (n to exit)?";  // allow user another input
const char USER_WELCOME[]           = "Welcome ";               // welcome message for user
const char RECIPIENT_LIST[]         = "List of recipients:";    // inform user of who will receive messages
const char MESSAGING_CAN_START[]    = "You can now send messages."; // inform user they can begin sending messages
const char INVALID_SELECTION[]      = "Input types is out of range!"; // notify user that input was not selectabel
const char CHAT_SENT[]              = "SENT: ";                 // prefix for logging what user sent
const char FIRST_SECOND_SEG[]       = ": ";                     // seperator between first and second

// global variables
ThreadController threadControl  = ThreadController(); // controls all threads
Thread buttonThread             = Thread();           // thread for handling the push-button
Thread canRxThread              = Thread();           // thread for receiving CAN bus messages
Thread canTxThread              = Thread();           // thread for sending CAN bus messages
MCP_CAN mpcCan(SPI_CS_PIN);                           // set Aruduino chip select pin used for CAN shield		
LiquidCrystal lcd(LCD_RS_PIN,LCD_EN_PIN,LCD_D4_PIN,
                  LCD_D5_PIN,LCD_D6_PIN,LCD_D7_PIN);  // initialize the library by providing the nuber of pins to it
Zero me                         = NONE;               // member of zero using device
byte destination                = 0;                  // bit packed specifier for which members should receive message
unsigned int lastId             = 0;                  // evey time id changes, need to send twice (once to set register and other with data)

/////////// Main Functions ///////////

/*
 * Function: setup
 * Discription: configures the board
 */
void setup() {
  setupSerial();    // serial communication to PC
  setupLcd();       // LCD connected to board
  setupGpio();      // general purpose I/O
  setupCan();       // CAN bus
  setupThreads();   // functions as concurrent threads
  setupUser();      // determine member sending messages
  setupTargets();   // determine members messages should be sent to
}

/*
 * Function: loop
 * Discription: main control loop
 */
void loop() {
  threadControl.run();  // process all threads
}

/////////// Utility Functions ///////////

/*
 * Function: byteArrayToHexStr
 * Discription: convert byte array to string of hex characters
 * Parameters:
 *   byteArray - arry of bytes
 *   len - length of array
 *   str - string of hex characters
 */
void byteArrayToStr(const byte *byteArray, byte len, char *str) {
  memset(str,0,len+1);                            // initialize character array
  for (int i=0; i<len; i++) {                     // loop through each byte
    sprintf(&str[i*2], "%02X", byteArray[i]);     // convert byte to hex string equivalent
  }
}

/////////// Display Functions ///////////

/*
 * Function: displayLcd
 * Discription: displays messages to user
 * Parameters:
 *   first: first line of text on LCD
 *   second: second line of text on LCD
 */
void displayLcd(const char* first, const char* second = "") {
  if (LCD_CONNECTED) {    // only for board with LCD connected
    lcd.clear();          // wipe characters from LCD
    lcd.setCursor(0,0);   // set cursor position to start of first line on the LCD
    lcd.print(first);     // display characters, starting at cursor
    lcd.setCursor(0,1);   // set cusor position to start of next line
    lcd.print(second);    // display characters, starting at cursor
  }
}

/*
 * Function: display
 * Discription: displays messages to user via serial console
 * Parameters:
 *   line: line of text
 *   second: second line of text on LCD
 */
void displaySerial(const char* line, const char* second = "") {
  // load first and seconds line as one line into buffer
  char buff[NUM_LCD_COLS*NUM_LCD_ROWS+1]; // display buffer for logging results  
  strcpy(buff, line);                     // prepend first line
  strcat(buff, FIRST_SECOND_SEG);         // seperate with space
  strcat(buff, second);                   // append second line
  
  Serial.println(buff);   // send line of text to serial console
}

/*
 * Function: display
 * Discription: wrapper for displays messages to possible outputs
 * Parameters:
 *   first: first line of text on LCD
 *   second: second line of text on LCD
 */
void display(const char* first, const char* second = "") {
  displaySerial(first, second);           // send buffer to serial console
}

void displayChatMessage(const Zero sender, const byte len, const byte* data) {
  char srcName[DISPLAY_SECTION_LIMIT];    // name of team member sending message

  switch (sender) {                       // set name per sender value
    case JZ:
      strcpy(srcName,ZERO_NAME_JZ);
      break;
    case RH:
      strcpy(srcName,ZERO_NAME_RH);
      break;
    case WM:
      strcpy(srcName,ZERO_NAME_WM);
      break;
    case EH:
      strcpy(srcName,ZERO_NAME_EH);
      break;
    case DC:
      strcpy(srcName,ZERO_NAME_DC);
      break;
    default:
      strcpy(srcName,ZERO_NONE);
  }

  char msg[DISPLAY_SECTION_LIMIT];          // chat message
  memset(msg,0,DISPLAY_SECTION_LIMIT);      // initialize message buffer
  memcpy(msg, data, len);                   // set message with data
  
  display(srcName, msg);                    // display chat message
}

/////////// Sender Functions ///////////

/*
 * Function: sendCanMsg
 * Discription: sends text for chat session
 * Parameters:
 *   msg - text message for chat
 */
void sendCanMsg(char* msg) {
  static bool firstTime = true;             // for some reason, first send is currupted
  
  byte len = strlen(msg);                   // length in bytes of message

  if (len > CAN_MAX_CHAR_IN_MESSAGE) {      // ensure that message will fit
    display(ERROR_MSG, MSG_TOO_LONG);       // report problem
    return;
  }

  unsigned long id =0;                      // id field of packet
  id &= CAN_ID_CHAT;                        // set chat bit
  id |= me << CAN_SRC_SHIFT;                // set source to me
  id |= destination;                        // set recipient

  if(lastId != id) {                        // when id changes, need to send message once with new id before real one
    // init CAN shield with new id field
    mpcCan.sendMsgBuf(id,CAN_BASE_FRAME_FORMAT,0,0);
    lastId = id;                            // save new id
  }

  // send text message over CAN bus
  mpcCan.sendMsgBuf(id, CAN_BASE_FRAME_FORMAT, len, (const byte*) msg);

  Serial.print(CHAT_SENT);                  // identity that you are sending
  Serial.println(msg);                      // message you sent

  // display packet on LCD
  displayCanPacket(id, len, (const byte*) msg);
}

/*
 * Function: canLedCmdSender
 * Discription: send commands to set LED in other device
 * Parameters:
 *   ledVal - value that LED should be set to
 */
void canLedCmdSender(const int ledVal) {
  const byte data[] = {                     // data field of packet
              (byte) DEVICE_TO_COMMAND,     // device (i.e. Mega or Uno) that should set its LED
              (byte) ledVal};               // value that LED should be set to
  byte sz = sizeof(data);                   // size of data field

  unsigned int id = CAN_DEVICE_CMD;         // initialize id field for device command
  
  if(lastId != id) {                        // when id changes, need to send message once with new id before real one
    // init CAN shield with new id field
    mpcCan.sendMsgBuf(id,CAN_BASE_FRAME_FORMAT,0,0);
    lastId = id;                            // save new id
  }

  // publish message over CAN bus
  mpcCan.sendMsgBuf(id, CAN_BASE_FRAME_FORMAT, sz, data);

  // log LED command
  if (ledVal == HIGH) {
    display(DEV_TO_CMD_NAME, LED_ON_MSG);          // LED commanded On
  }
  else {
    display(DEV_TO_CMD_NAME, LED_OFF_MSG);         // LED commanded Off
  }
}

/////////// Logger Functions ///////////

/*
 * Function: displayCanPacket
 * Discription: shows the id and data of the CAN bus packet
 * Parameters:
 *   id - message id for packet
 *   len - length of data in bytes
 *   data - message data for packet
 */
void displayCanPacket(const unsigned long id, const byte len, const byte* data) {
  char dataStr[CAN_MAX_CHAR_IN_MESSAGE*2+1];    // data value in hex string format
  byteArrayToStr(data, len, dataStr);           // convert byte arry to hex string

  char first[DISPLAY_SECTION_LIMIT];            // first line of display buffer 
  sprintf (first, "%s 0x%03X ", CAN_ID, id);    // create first line

  char lenStr[DISPLAY_SECTION_LIMIT];           // string for length results
  sprintf (lenStr, "%s %d", CAN_LEN, len);      // create string with length info
  strcat(first, lenStr);                        // append length info
  
  displayLcd(first, dataStr);                   // diplay data on LCD
}

/////////// Condition Checks ///////////

/*
 * Function: isChat
 * Discription: determines if message is intended for chat
 * Parameters:
 *   id - message id for packet
 *   sender - who is sending message
 *   forMe - am I the an intended receipient
 */
bool isChat(const unsigned long id, Zero* sender, bool* forMe) {
  bool chat = ((~id & CAN_CHAT_MASK) > 0);          // is chat bit set to zero?
  byte src = (id >> CAN_SRC_SHIFT) & CAN_ZERO_MASK; // extract sender bits
  byte dst = (id & CAN_ZERO_MASK);                  // extract receiver bits

  *sender = (Zero) src;                             // cast source to sender
  *forMe = ((dst & me) > 0);                        // is my bit set in desination field

  return (chat && (src > 0));                       // only a chat if there is a sender
}

/*
 * Function: isMyLed
 * Discription: determines if LED specified in CAN message is intended for this board
 * Parameters:
 *   len - length of data in bytes
 *   data - message data for packet
 *   ledVal - value LED should be set to
 */
bool isMyLed(const byte len, const byte* data, byte* ledVal) {
  if(len < (VAL_INDEX+1)) {               // enough bytes for LED data
    return false;                         // hell no
  }
  
  *ledVal = data[VAL_INDEX];              // load LED command value
  return (data[DEV_INDEX] == MY_DEVICE);  // reply with check on if applies to LED on this board
}

/////////// Processing Functions ///////////

/*
 * Function: processLedCmd
 * Discription: set the LED connect to board if commanded
 * Parameters:
 *   len - length of data in bytes
 *   data - message data for packet
 */
void processLedCmd(const byte len, const byte* data) {
  byte ledVal;                              // value LED should be set to
  
  if (isMyLed(len, data, &ledVal)) {        // does command apply to current board
    digitalWrite(LED_PIN, ledVal);          // set LED
  }
}

/*
 * Function: processCanPacket
 * Discription: parses the id and sends data to correct hanlder
 * Parameters:
 *   id - message id for packet
 *   len - length of data in bytes
 *   data - message data for packet
 */
void processCanPacket(const unsigned long id, const byte len, const byte* data) {
  Zero sender = NONE;                         // sender of CAN char message
  bool forMe = false;                         // was message for me
  
  if(isChat(id, &sender, &forMe)) {           // check for chat message
    if(forMe) {                               // was chat for me
      displayChatMessage(sender, len, data);  // show chat
    }
  }
  else {                                      
    processLedCmd(len, data);                 // LED command
  }
  
  displayCanPacket(id, len, data);            // log raw packet results
}

/////////// Threads ///////////

/*
 * Function: buttonHandler
 * Discription: activates LED per button press and logs results
 */
void buttonHandler() {
  static int lastButtonVal = LOW;         // persistent variable for keeping last button press state
  int buttonVal = LOW;                    // variable for reading the push button status

  buttonVal = digitalRead(BUTTON_PIN);    // read push button value
  
  if (buttonVal != lastButtonVal) {       // check if button state has changed
    canLedCmdSender(buttonVal);           // set LED on other device per button state
    lastButtonVal = buttonVal;            // save last button state
  } 
}

/*
 * Function: canMsgReceiver
 * Discription: listens for CAN bus messages and it appropriately
 */
void canMsgReceiver() {
  if(CAN_MSGAVAIL != mpcCan.checkReceive()) {     // check if packet is on the bus
    return;                                       // no packet, stop processing
  }
  
  unsigned long id = mpcCan.getCanId();           // id field of packet
  byte len = 0;                                   // length of data in packet
  byte data[CAN_MAX_CHAR_IN_MESSAGE];             // data field of packet
  memset(data,0,CAN_MAX_CHAR_IN_MESSAGE);         // initialize data buffer
  if (CAN_OK != mpcCan.readMsgBuf(&len, data)) {  // read data
    display(CAN_BUS, CAN_READ_FAIL);              // report failure
    return;                                       // stop processing
  }

  processCanPacket(id, len, data);                // process packet
}

/*
 * Function: canMsgSender
 * Discription: sends message over CAN bus if received over serial
 */
void canMsgSender() {
  String entered;                                 // chat message from serial
  char msg[CAN_MAX_CHAR_IN_MESSAGE];              // buffer for chat

  if (Serial.available() != 0) {                  // check for message from serial
    entered = Serial.readString();                // get message from serial
    entered.trim();                               // remove carrage return at end due to hitting enter to send
    strcpy(msg,entered.c_str());                  // copy string to char buffer
    sendCanMsg(msg);                              // use function to send message over CAN bus
  }
}

/*
 * Function: setupThreads
 * Discription: initializes threads
 */
void setupThreads() {
  // configure button handler thread
  buttonThread.onRun(buttonHandler);                    // function to call every cycle
  buttonThread.setInterval(BUTTON_THREAD_INTERVAL);     // when to call function
  threadControl.add(&buttonThread);                     // button handler

  // configure CAN bus message receiver thread
  canRxThread.onRun(canMsgReceiver);                    // function to call every cycle
  canRxThread.setInterval(CAN_THREAD_INTERVAL);         // when to call function
  threadControl.add(&canRxThread);                      // CAN bus message receiver

  // configure CAN bus message sender thread
  canTxThread.onRun(canMsgSender);                      // function to call every cycle
  canTxThread.setInterval(CAN_THREAD_INTERVAL);         // when to call function
  threadControl.add(&canTxThread);                      // CAN bus message receiver
}

/////////// Setup Functions ///////////

/*
 * Function: setupSerial
 * Discription: initialize serial port
 */
void setupSerial() {
  Serial.begin(SERIAL_SPEED);   // set communication speed
}

/*
 * Function: setupLcd
 * Discription: initialize liquid crystal display
 */
void setupLcd() {
  if (LCD_CONNECTED) {                      // only for board with LCD connected
    lcd.begin(NUM_LCD_COLS,NUM_LCD_ROWS);   // set LCD size
  }
}

/*
 * Function: setupGpio
 * Discription: initialize general purpose I/O pins for specific functionality
 */
void setupGpio() {
  pinMode(BUTTON_PIN,INPUT);     // configure port for push button
  pinMode(LED_PIN,OUTPUT);       // configure port for LED
}

/*
 * Function: setupCan
 * Discription: initialize controller area network port
 */
void setupCan() {
  display(CAN_BUS, CAN_INIT);               // report initializing start
  
  while (CAN_OK != mpcCan.begin(CAN_SPEED, CAN_CLOCK)) {
    display(CAN_BUS, CAN_INIT_FAIL);        // report failure
    delay(CAN_SETUP_INTERVAL);              // try again after delay
  }
  
  display(CAN_BUS, CAN_INIT_RDY);           // CAN Bus is ready to use
}

/*
 * Function: setupUser
 * Discription: queries the user for which team member they are
 */
void setupUser() {
  int member=1;                                       // user selection for member
  int numMembers=0;                                   // number of members
  String reply;                                       // selection from user

  Serial.println(DIVIDER);                            // segregate query from other prompts
  Serial.println(QUERY_USER);                         // tell user to select a member
  // print list of members
  Serial.print(member++);Serial.print(NUMBER_STYLE);Serial.println(ZERO_NAME_JZ);
  Serial.print(member++);Serial.print(NUMBER_STYLE);Serial.println(ZERO_NAME_RH);
  Serial.print(member++);Serial.print(NUMBER_STYLE);Serial.println(ZERO_NAME_WM);
  Serial.print(member++);Serial.print(NUMBER_STYLE);Serial.println(ZERO_NAME_EH);
  Serial.print(member++);Serial.print(NUMBER_STYLE);Serial.println(ZERO_NAME_DC);
  
  numMembers = member-1;                              // save number of members
  member = -1;                                        // init member selected to keep looping
  while ((member <= 0) || (member >= numMembers)) {   // keep looping until member selected
    while (Serial.available() == 0) {                 // poll serial line for input
      delay(SERIAL_POLL_RATE);                        // wait for user input
    }
    reply = Serial.readString();                      // read user input
    member = atoi(reply.c_str());                     // conver string number to integer type
  }

  Serial.println(DIVIDER);                            // segregate welcome message from other prompts
  Serial.print(USER_WELCOME);                         // welcome selected user
  switch(member) {                                    // map number selected to user
    case 1:                                           // Joel Zamora
      me = JZ;                                        // set me identifier
      Serial.println(ZERO_NAME_JZ);                   // let user know selection
      break;                                          // no more selection needed
    case 2:                                           // Richard Hemphill
      me = RH; 
      Serial.println(ZERO_NAME_RH);
      break;
    case 3:                                           // William Mckinnon
      me = WM;
      Serial.println(ZERO_NAME_WM);
      break;
    case 4:                                           // Eric Hansen
      me = EH;
      Serial.println(ZERO_NAME_EH);
      break;
    case 5:                                           // Daniel Caballero
      me = DC;
      Serial.println(ZERO_NAME_DC);
      break;
    default:                                          // Anonymous
      me = NONE;
      Serial.println(ZERO_NONE);
  }
  Serial.println(DIVIDER);                           // segregate from other prompts
}

/*
 * Function: setupTargets
 * Discription: queries user for which member should receive message
 */
void setupTargets() {
  destination = 0;                                  // init destination targets to no-one
  
  int member = 1;                                   // member number
  int allMembers = 0;                               // all members (broadcast) selected
  // print list of members
  Serial.print(member++);Serial.print(NUMBER_STYLE);Serial.println(ZERO_NAME_JZ);
  Serial.print(member++);Serial.print(NUMBER_STYLE);Serial.println(ZERO_NAME_RH);
  Serial.print(member++);Serial.print(NUMBER_STYLE);Serial.println(ZERO_NAME_WM);
  Serial.print(member++);Serial.print(NUMBER_STYLE);Serial.println(ZERO_NAME_EH);
  Serial.print(member++);Serial.print(NUMBER_STYLE);Serial.println(ZERO_NAME_DC);
  Serial.print(member++);Serial.print(NUMBER_STYLE);Serial.println(ZERO_ALL);
  allMembers=member-1;                              // set number of members
  
  Serial.println(QUERY_TARGET);                     // tell user to select member(s) to send messages to

  String reply;                                     // relply from user over serial
  do {                                              // loop until selection is over (use break to exit)
    while (Serial.available() == 0) {               // poll serial line for input
      delay(SERIAL_POLL_RATE);                      // wait for user input
    }
    reply = Serial.readString();                    // read user input

    // user has selected one destination and wants to exit 
    if (reply.startsWith("n") && (destination != 0)) {          
      break;                                        // exit loop
    }
    
    member = atoi(reply.c_str());                   // conver string number to integer type

    if (member == allMembers) {                     // user choose to broadcast
      destination = CAN_ZERO_MASK;                  // set bits for all destination users
      destination &= ~me;                           // don't talk to self
      break;                                        // exit loop
    }
    else if ((member > 0) || (member < allMembers)) { // a member was selected
      destination |= 1U << (member-1);              // add member to destination
    }
    else {                                          // user chose input out of range
      Serial.println(INVALID_SELECTION);
    }

    Serial.println(QUERY_ANOTHER);                  // ask user if they want more members
  } while (true);

  Serial.println(DIVIDER);                          // segregate recipient list from other prompts
  Serial.println(RECIPIENT_LIST);                   // inform user that following will receive messages
  if ((destination & JZ) > 0) {Serial.println(ZERO_NAME_JZ);}
  if ((destination & RH) > 0) {Serial.println(ZERO_NAME_RH);}
  if ((destination & WM) > 0) {Serial.println(ZERO_NAME_WM);}
  if ((destination & EH) > 0) {Serial.println(ZERO_NAME_EH);}
  if ((destination & DC) > 0) {Serial.println(ZERO_NAME_DC);}
  Serial.println(DIVIDER);                          // segregate recipient list from other prompts
  Serial.println(MESSAGING_CAN_START);              // let user know that they can start communcating
  Serial.println(DIVIDER);                          // segregate messaging begin from other prompts
}
