/// @dir RF12demo
/// Configure some values in EEPROM for easy config of the RF12 later on.
// 2009-05-06 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

// this version adds flash memory support, 2009-11-19

// LCD_Jeenode
// 20x4 LCD display
// Michael LeBlanc generaleccentric.net
// September 2014 


#include <JeeLib.h>
#include <util/crc16.h>
#include <util/parity.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>

// LCD stuff
#include <PortsLCD.h>

PortI2C myI2C (4);
LiquidCrystalI2C lcd (myI2C);

#define screen_width 20
#define screen_height 4

byte colonChar[8] = {   // colon
    0b00000,0b00000,0b00000,0b00100,0b00000,0b00000,0b00100,0b00000};

byte prcentChar[8] = { // percent
    0x00,0x00,0x11,0x02,0x04,0x08,0x11,0x00};

byte openChar[8] = {    // open
    0b11111,0b11111,0b10001,0b10001,0b10001,0b10001,0b10001,0b10001};

byte closedChar[8] = {  // closed
    0b00000,0b10001,0b01010,0b00100,0b01010,0b10001,0b00000,0b11111};

void writeNewColon() {
    lcd.write((uint8_t)0);        // write new colon character
}

void writeNewPrcnt() {
    lcd.write((uint8_t)1);        // write new percent character 
}

void writeNewOpen() {
    lcd.write((uint8_t)2);        // write new door up character 
}

void writeNewClosed() {
    lcd.write((uint8_t)3);        // write new door down character 
}

int remote1temp,remote1tempW,remote1tempF,remote1hum,remote1humW,remote1humF;
int remote5temp,remote5tempW,remote5tempF,remote5hum,remote5humW,remote5humF;
int remote10temp,remote10tempW,remote10tempF,remote10hum,remote10humW,remote10humF;
int remote10btemp,remote10btempW,remote10btempF,remote10bhum,remote10bhumW,remote10bhumF;
int remote10CO;
boolean updateOut,updateGar,updateOffice,remote10door,remote1Life;

static unsigned long lastMillis = 0; // holds the last read millis()
static int timer_1 = 0; // a repeating timer max time 32768 mS = 32sec use a long if you need a longer timer
// NOTE timer MUST be a signed number int or long as the code relies on timer_1 being able to be negative
// NOTE timer_1 is a signed int
#define TIMER_INTERVAL_1 1000
// 1S interval

int ledPin = 5;
boolean ledState = 1;
boolean garageOpen = false;

// ATtiny's only support outbound serial @ 38400 baud, and no DataFlash logging

#if defined(__AVR_ATtiny84__) ||defined(__AVR_ATtiny44__)
#define SERIAL_BAUD 38400
#else
#define SERIAL_BAUD 57600

//#define DATAFLASH 1 // check for presence of DataFlash memory on JeeLink
//#define FLASH_MBIT  16  // support for various dataflash sizes: 4/8/16 Mbit

//#define LED_PIN   9 // activity LED, comment out to disable

#endif

#define COLLECT 0x20 // collect mode, i.e. pass incoming without sending acks

static unsigned long now () {
  // FIXME 49-day overflow
  return millis() / 1000;
}

static void activityLed (byte on) {
#ifdef LED_PIN
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, !on);
#endif
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// LED Blinker function

static void ledTimer() {
 // set millisTick at the top of each loop if and only if millis() has changed
  unsigned long deltaMillis = 0; // clear last result
  unsigned long thisMillis = millis(); 
  // do this just once to prevent getting different answers from multiple calls to   millis()
  if (thisMillis != lastMillis) {
  // we have ticked over
  // calculate how many millis have gone past  
  deltaMillis = thisMillis-lastMillis; // note this works even if millis() has rolled over back to 0
  lastMillis = thisMillis;
}
  // handle repeating timer
  // repeat this code for each timer you need to handle
  timer_1 -= deltaMillis;
  if (timer_1 <= 0) {
    // reset timer since this is a repeating timer
    timer_1 += TIMER_INTERVAL_1; // note this prevents the delay accumulating if we miss a mS or two 
    ledState = !ledState;
    if (ledState == 1) {
      digitalWrite(ledPin, HIGH);
    } else {
      digitalWrite(ledPin, LOW);
    }
  } 
}


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// RF12 configuration setup code

typedef struct {
  byte nodeId;
  byte group;
  char msg[RF12_EEPROM_SIZE-4];
  word crc;
} RF12Config;

static RF12Config config;

static char cmd;
static byte value, stack[RF12_MAXDATA+4], top, sendLen, dest, quiet;
static byte testbuf[RF12_MAXDATA], testCounter, useHex;

static void showNibble (byte nibble) {
  char c = '0' + (nibble & 0x0F);
  if (c > '9')
    c += 7;
  Serial.print(c);
}

static void showByte (byte value) {
  if (useHex) {
    showNibble(value >> 4);
    showNibble(value);
  } else
    Serial.print((int) value);
    //lcd.clear();
    //lcd.print((int) value);
}

static void addCh (char* msg, char c) {
  byte n = strlen(msg);
  msg[n] = c;
}

static void addInt (char* msg, word v) {
    if (v >= 10)
        addInt(msg, v / 10);
        addCh(msg, '0' + v % 10);
}

static void saveConfig () {
    // set up a nice config string to be shown on startup
    memset(config.msg, 0, sizeof config.msg);
    strcpy(config.msg, " ");
    
    byte id = config.nodeId & 0x1F;
    addCh(config.msg, '@' + id);
    strcat(config.msg, " i");
    addInt(config.msg, id);
    if (config.nodeId & COLLECT)
        addCh(config.msg, '*');
    
    strcat(config.msg, " g");
    addInt(config.msg, config.group);
    
    strcat(config.msg, " @ ");
    static word bands[4] = { 315, 433, 868, 915 };
    word band = config.nodeId >> 6;
    addInt(config.msg, bands[band]);
    strcat(config.msg, " MHz ");
    
    config.crc = ~0;
    for (byte i = 0; i < sizeof config - 2; ++i)
      config.crc = _crc16_update(config.crc, ((byte*) &config)[i]);
  
    // save to EEPROM
    for (byte i = 0; i < sizeof config; ++i) {
      byte b = ((byte*) &config)[i];
      eeprom_write_byte(RF12_EEPROM_ADDR + i, b);
    }
    
    if (!rf12_config())
      Serial.println("config save failed");
}

static byte bandToFreq (byte band) {
  return band == 8 ? RF12_868MHZ : band == 9 ? RF12_915MHZ : RF12_433MHZ;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// OOK transmit code

 //Turn transmitter on or off, but also apply asymmetric correction and account
 //for 25 us SPI overhead to end up with the proper on-the-air pulse widths.
 //With thanks to JGJ Veken for his help in getting these values right.
static void ookPulse(int on, int off) {
  rf12_onOff(1);
  delayMicroseconds(on + 150);
  rf12_onOff(0);
  delayMicroseconds(off - 200);
}

static void fs20sendBits(word data, byte bits) {
  if (bits == 8) {
    ++bits;
    data = (data << 1) | parity_even_bit(data);
  }
  for (word mask = bit(bits-1); mask != 0; mask >>= 1) {
    int width = data & mask ? 600 : 400;
    ookPulse(width, width);
  }
}

static void fs20cmd(word house, byte addr, byte cmd) {
  byte sum = 6 + (house >> 8) + house + addr + cmd;
  for (byte i = 0; i < 3; ++i) {
    fs20sendBits(1, 13);
    fs20sendBits(house >> 8, 8);
    fs20sendBits(house, 8);
    fs20sendBits(addr, 8);
    fs20sendBits(cmd, 8);
    fs20sendBits(sum, 8);
    fs20sendBits(0, 1);
    delay(10);
  }
}

static void kakuSend(char addr, byte device, byte on) {
  int cmd = 0x600 | ((device - 1) << 4) | ((addr - 1) & 0xF);
  if (on)
    cmd |= 0x800;
  for (byte i = 0; i < 4; ++i) {
    for (byte bit = 0; bit < 12; ++bit) {
      ookPulse(375, 1125);
      int on = bitRead(cmd, bit) ? 1125 : 375;
      ookPulse(on, 1500 - on);
    }
    ookPulse(375, 375);
    delay(11); // approximate
  }
}


#define df_present() 0
#define df_initialize()
#define df_dump()
#define df_replay(x,y)
#define df_erase(x)

//#endif

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

const char helpText1[] PROGMEM = 
  "\n"
  "Available commands:" "\n"
  "  <nn> i     - set node ID (standard node ids are 1..30)" "\n"
  "  <n> b      - set MHz band (4 = 433, 8 = 868, 9 = 915)" "\n"
  "  <nnn> g    - set network group (RFM12 only allows 212, 0 = any)" "\n"
  "  <n> c      - set collect mode (advanced, normally 0)" "\n"
  "  t          - broadcast max-size test packet, request ack" "\n"
  "  ...,<nn> a - send data packet to node <nn>, request ack" "\n"
  "  ...,<nn> s - send data packet to node <nn>, no ack" "\n"
  "  <n> l      - turn activity LED on PB1 on or off" "\n"
  "  <n> q      - set quiet mode (1 = don't report bad packets)" "\n"
  "  <n> x      - set reporting format (0 = decimal, 1 = hex)" "\n"
  "  123 z      - total power down, needs a reset to start up again" "\n"
  "Remote control commands:" "\n"
  "  <hchi>,<hclo>,<addr>,<cmd> f     - FS20 command (868 MHz)" "\n"
  "  <addr>,<dev>,<on> k              - KAKU command (433 MHz)" "\n"
;
//const char helpText2[] PROGMEM = 
//  "Flash storage (JeeLink only):" "\n"
//  "  d                                - dump all log markers" "\n"
//  "  <sh>,<sl>,<t3>,<t2>,<t1>,<t0> r  - replay from specified marker" "\n"
//  "  123,<bhi>,<blo> e                - erase 4K block" "\n"
//  "  12,34 w                          - wipe entire flash memory" "\n"
//;

static void showString (PGM_P s) {
  for (;;) {
    char c = pgm_read_byte(s++);
    if (c == 0)
        break;
    if (c == '\n')
        Serial.print('\r');
        Serial.print(c);
  }
}

static void showHelp () {
  showString(helpText1);
  //if (df_present())
    //showString(helpText2);
  Serial.println("Current configuration:");
  rf12_config();
}

static void handleInput (char c) {
  if ('0' <= c && c <= '9')
    value = 10 * value + c - '0';
  else if (c == ',') {
    if (top < sizeof stack)
      stack[top++] = value;
    value = 0;
  } else if ('a' <= c && c <='z') {
    Serial.print("> ");
    for (byte i = 0; i < top; ++i) {
      Serial.print((int) stack[i]);
      Serial.print(',');
    }
    Serial.print((int) value);
    Serial.println(c);
    switch (c) {
      default:
        showHelp();
        break;
      case 'i': // set node id
        config.nodeId = (config.nodeId & 0xE0) + (value & 0x1F);
        saveConfig();
        break;
      case 'b': // set band: 4 = 433, 8 = 868, 9 = 915
        if (value)
          config.nodeId = (bandToFreq(value) << 6) + (config.nodeId & 0x3F);
        saveConfig();
        break;
      case 'g': // set network group
        config.group = value;
        saveConfig();
        break;
      //case 'c': // set collect mode (off = 0, on = 1)
      //  if (value)
      //    config.nodeId |= COLLECT;
      //  else
      //    config.nodeId &= ~COLLECT;
      //  saveConfig();
      //  break;
      case 't': // broadcast a maximum size test packet, request an ack
        cmd = 'a';
        sendLen = RF12_MAXDATA;
        dest = 0;
        for (byte i = 0; i < RF12_MAXDATA; ++i)
          testbuf[i] = i + testCounter;
        Serial.print("test ");
        Serial.println((int) testCounter); // first byte in test buffer
        ++testCounter;
        break;
      case 'a': // send packet to node ID N, request an ack
      case 's': // send packet to node ID N, no ack
        cmd = c;
        sendLen = top;
        dest = value;
        memcpy(testbuf, stack, top);
        break;
      case 'l': // turn activity LED on or off
        //activityLed(value);
        break;
      case 'f': // send FS20 command: <hchi>,<hclo>,<addr>,<cmd>f
        rf12_initialize(0, RF12_868MHZ);
        //activityLed(1);
        fs20cmd(256 * stack[0] + stack[1], stack[2], value);
        //activityLed(0);
        rf12_config(0); // restore normal packet listening mode
        break;
      case 'k': // send KAKU command: <addr>,<dev>,<on>k
        rf12_initialize(0, RF12_433MHZ);
        //activityLed(1);
        kakuSend(stack[0], stack[1], value);
        //activityLed(0);
        rf12_config(0); // restore normal packet listening mode
        break;
      //case 'd': // dump all log markers
      //  if (df_present())
      //    df_dump();
      //  break;
      //case 'r': // replay from specified seqnum/time marker
      //  if (df_present()) {
      //    word seqnum = (stack[0] << 8) || stack[1];
      //    long asof = (stack[2] << 8) || stack[3];
      //    asof = (asof << 16) | ((stack[4] << 8) || value);
      //    df_replay(seqnum, asof);
      //  }
      //  break;
      //case 'e': // erase specified 4Kb block
      //  if (df_present() && stack[0] == 123) {
      //    word block = (stack[1] << 8) | value;
      //    df_erase(block);
      //  }
      //  break;
      //case 'w': // wipe entire flash memory
      //  if (df_present() && stack[0] == 12 && value == 34) {
      //    df_wipe();
      //    Serial.println("erased");
      //  }
      //  break;
      case 'q': // turn quiet mode on or off (don't report bad packets)
        quiet = value;
        break;
      case 'z': // put the ATmega in ultra-low power mode (reset needed)
        if (value == 123) {
          delay(10);
          rf12_sleep(RF12_SLEEP);
          cli();
          Sleepy::powerDown();
        }
        break;
      case 'x': // set reporting mode to hex (1) or decimal (0)
        useHex = value;
        break;
      case 'v': //display the interpreter version
        displayVersion(1);
        break;
    }
    value = top = 0;
    memset(stack, 0, sizeof stack);
  } else if (c == '>') {
    // special case, send to specific band and group, and don't echo cmd
    // input: band,group,node,header,data...
    stack[top++] = value;
    rf12_initialize(stack[2], bandToFreq(stack[0]), stack[1]);
    rf12_sendNow(stack[3], stack + 4, top - 4);
    rf12_sendWait(2);
    rf12_config(0); // restore original band, etc
    value = top = 0;
    memset(stack, 0, sizeof stack);
  } else if (' ' < c && c < 'A')
    showHelp();
}

void displayVersion(uint8_t newline ) {
  Serial.print("\n[RF12demo.10]");
  if(newline!=0)  Serial.println();

}




void setup() {
    pinMode(ledPin, OUTPUT);            // garage door open LED
    lcd.createChar(0, colonChar);       // new colon
    lcd.createChar(1, prcentChar);      // new percent
    lcd.createChar(2, openChar);        // door open
    lcd.createChar(3, closedChar);      // door closed
    quiet = 1;
    updateOut = false;
    updateGar = false;
    updateOffice = false;
    // set up the LCD's number of rows and columns: 
    lcd.begin(screen_width, screen_height);
    // Print a message to the LCD.
    lcd.setCursor(6,1);
    lcd.print("Weather");
    lcd.setCursor(5,2);
    lcd.print("Telemetry");
    lcd.setCursor(0,3);
    writeNewPrcnt();
    writeNewColon();
    writeNewOpen();
    writeNewClosed();
    
    
    
    Serial.begin(SERIAL_BAUD);
    displayVersion(0);
    //activityLed(0);

  if (rf12_config()) {
    config.nodeId = eeprom_read_byte(RF12_EEPROM_ADDR);
    config.group = eeprom_read_byte(RF12_EEPROM_ADDR + 1);
  } else {
    config.nodeId = 0x41; // 433 MHz, node 1
    config.group = 0xD4;  // default group 212
    saveConfig();
  }
  
  digitalWrite(ledPin, HIGH);  
  df_initialize();
  
  showHelp();
  
  delay(3000);
  lcd.clear();
  digitalWrite(ledPin, LOW);
  
  // For LED blinker: initialize the digital pin as an output.
  pinMode(ledPin, OUTPUT);  
  timer_1 = TIMER_INTERVAL_1; // timeout in first loop 
  lastMillis = millis(); // do this last in setup
}

void loop() {
      
    if (Serial.available())
        handleInput(Serial.read());

    if (rf12_recvDone()) {
        byte n = rf12_len;
        if (rf12_crc == 0) {
            Serial.print("OK");
            //lcd.clear();
            //lcd.print("OK");
        }
        else {
        if (quiet)
            return;
        Serial.print(" ?");
        if (n > 20) // print at most 20 bytes if crc is wrong
            n = 20;
        }
        if (useHex)
            Serial.print('X');
        if (config.group == 0) {
            Serial.print(" G");
            showByte(rf12_grp);
            //lcd.print(rf12_grp);
            //lcd.print(" ");
        }
        
        showByte(rf12_hdr);
        int hdr = int(rf12_hdr);
        
        if (hdr == 1) {  // Gather data from Node 1 (Office)
            remote1temp = (rf12_data[0] + (rf12_data[1] * 256)) - 1;
            remote1tempW = remote1temp / 10;
            remote1tempF = remote1temp % 10;
            
            remote1hum = (rf12_data[2] + (rf12_data[3] * 256)) - 1;
            remote1humW = remote1hum / 10;
            remote1humF = remote1hum % 10;
            updateOffice = true;
        }
        
        if (hdr == 5) {  // Gather data from Node 5 (Shed)
            remote5temp = (rf12_data[0] + (rf12_data[1] * 256)) - 1;
            remote5tempW = remote5temp / 10;
            remote5tempF = remote5temp % 10;
            
            remote5hum = (rf12_data[2] + (rf12_data[3] * 256)) - 1;
            remote5humW = remote5hum / 10;
            remote5humF = remote5hum % 10;
            updateOut = true;
        }
        
        if (hdr == 10) {  // Gather data from Node 10 (Garage)
            remote10temp = (rf12_data[0] + (rf12_data[1] * 256)) - 1;
            remote10tempW = remote10temp / 10;
            remote10tempF = remote10temp % 10;
            remote10hum = (rf12_data[2] + (rf12_data[3] * 256)) - 1;
            remote10humW = remote10hum / 10;
            remote10humF = remote10hum % 10;
            
            remote10btemp = (rf12_data[4] + (rf12_data[5] * 256)) - 1;
            remote10btempW = remote10btemp / 10;
            remote10btempF = remote10btemp % 10;
            remote10bhum = (rf12_data[6] + (rf12_data[7] * 256)) - 1;
            remote10bhumW = remote10bhum / 10;
            remote10bhumF = remote10bhum % 10;
            
            remote10CO = (rf12_data[8] + (rf12_data[9] * 256)) - 1;
            
            remote10door = rf12_data[12];
            
            updateGar = true;
        }
    
        for (byte i = 0; i < n; ++i) {
            if (!useHex)
                Serial.print(' ');
            showByte(rf12_data[i]);
            //lcd.print(rf12_data[i]);
            //lcd.print(" ");
        }
        Serial.println();
        
        if (rf12_crc == 0) {
            //activityLed(1);
          
            //if (df_present())
            //df_append((const char*) rf12_data - 2, rf12_len + 2);
    
        if (RF12_WANTS_ACK && (config.nodeId & COLLECT) == 0) {
          Serial.println(" -> ack");
          rf12_sendStart(RF12_ACK_REPLY, 0, 0);
        }
          
          //activityLed(0);
        }
    }

    if (cmd && rf12_canSend()) {
        //activityLed(1);
    
        Serial.print(" -> ");
        Serial.print((int) sendLen);
        Serial.println(" b");
        byte header = cmd == 'a' ? RF12_HDR_ACK : 0;
        if (dest)
            header |= RF12_HDR_DST | dest;
        rf12_sendStart(header, testbuf, sendLen);
        cmd = 0;
    
        //activityLed(0);
        
      }
        
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // LCD print routines
        if (updateOut == true) {
            // LINE 0
            // Outside Temp
            lcd.setCursor(0,0);
            lcd.print("                    ");  // clear this line
            lcd.setCursor(0,0);
            lcd.print("Outside");
            writeNewColon();
            lcd.print(remote5tempW);
            lcd.print("c ");
            // Outside Humidity
            lcd.print("RH");
            writeNewColon();
            lcd.print(remote5humW);
            //lcd.print("%");
            writeNewPrcnt();
            
            updateOut = false;
        }
        if (updateGar == true) {
            // LINE 1
            // Garage Temp
            lcd.setCursor(0,1);
            lcd.print("                    ");  // clear this line
            lcd.setCursor(0,1);
            lcd.print("Gar");
            writeNewColon();
            lcd.print(remote10tempW);
            lcd.print("c ");
            
            // Garage Humidity
            lcd.print("H");
            writeNewColon();
            lcd.print(remote10humW);
            writeNewPrcnt();
            
            // Garage CO
            lcd.print(" CO");
            writeNewColon();
            lcd.print("000");
            // Adjust for length of number
            if (remote10CO > 99){
              lcd.setCursor(17,1);
            } else if (remote10CO > 9){
              lcd.setCursor(18,1);
            } else {
              lcd.setCursor(19,1);
            }
            lcd.print(remote10CO);
            //lcd.print("/1024");
            
            // LINE 2
            // Wormbox Temperature
            lcd.setCursor(0,2);
            lcd.print("                    ");  // clear this line
            lcd.setCursor(0,2);
            lcd.print("Wrm");
            writeNewColon();
            lcd.print(remote10btempW);
            lcd.print("c ");
            // Wormbox Humidity
            lcd.print("H");
            writeNewColon();
            lcd.print(remote10bhumW);
            writeNewPrcnt();
            
            
            if (remote10door == 0) {
                lcd.setCursor(14,2);
                lcd.print("Door");
                writeNewColon();
                writeNewOpen();
                garageOpen = true;
            }
            else {
                lcd.setCursor(14,2);
                lcd.print("Door");
                writeNewColon();
                writeNewClosed();
                digitalWrite(ledPin, LOW);
                garageOpen = false;
            }
            updateGar = false;
        }
        
        if (garageOpen == true) {
          ledTimer();
        }
        
        
        if (updateOffice == true) {
            // LINE 3
            // Office Temp
            lcd.setCursor(0,3);
            lcd.print("                    ");  // clear this line
            lcd.setCursor(0,3);
            lcd.print("Office");
            writeNewColon();
            lcd.print(remote1tempW);
            lcd.print("c ");
            // Office Humidity
            lcd.print("RH");
            writeNewColon();
            lcd.print(remote1humW);
            writeNewPrcnt();
            
            updateOffice = false;
        }
    }
