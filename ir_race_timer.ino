/*
 * SPI driver based on fs_skyrf_58g-main.c Written by Simon Chambers
 * TVOUT by Myles Metzel 
 * Scanner by Johannes Hermen
 * Inital 2 Button version by Peter (pete1990)
 * Refactored and GUI reworked by Marko Hoepken
 * Universal version my Marko Hoepken
 * Raceband added by Johannes Hermen
 * Marko Hoepken
 * der Frickler

The MIT License (MIT)

Copyright (c) 2015 Martin Rudat - Ingenierurbüro Rudat

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#define DEBUG

#include <TVout.h>
#include <fontALL.h>
#include <avr/pgmspace.h>
#include <EEPROM.h>

#define spiDataPin 10
#define slaveSelectPin 11
#define spiClockPin 12
#define rssiPin A6
// this two are minimum required 
#define buttonSeek 2
#define buttonMode 3
// optional comfort buttons
#define buttonDown 4
#define buttonSave 5
// Buzzer
#define buzzer 6

#define TV_FORMAT PAL

#define KEY_DEBOUNCE 200

#define led 13
// RSSI default raw range
#define RSSI_MIN_VAL 90
#define RSSI_MAX_VAL 300
// 75% threshold, when channel is printed in spectrum
#define RSSI_SEEK_FOUND 75 
// 80% under max value for RSSI 
#define RSSI_SEEK_TRESHOLD 80
// scan loops for setup run
#define RSSI_SETUP_RUN 10

#define STATE_START 0
#define STATE_STOP 1
#define STATE_SET 2
#define STATE_RUN 3
#define STATE_STOPPED 4
#define STATE_SAVE 5
#define STATE_RSSI_SETUP 6
#define STATE_RACE_TIMER 3

#define RSSI_RACE1 82

#define START_STATE STATE_START
#define MAX_STATE STATE_MANUAL

#define CHANNEL_BAND_SIZE 8
#define CHANNEL_MIN_INDEX 0
#define CHANNEL_MAX_INDEX 39

#define CHANNEL_MAX 39
#define CHANNEL_MIN 0

#define TV_COLS 128
#define TV_ROWS 96
#define TV_Y_MAX TV_ROWS-1
#define TV_X_MAX TV_COLS-1
#define TV_SCANNER_OFFSET 14

#define TV_Y_GRID 14
        #define TV_Y_OFFSET 3   

#define SCANNER_BAR_SIZE 52
#define SCANNER_LIST_X_POS 4
#define SCANNER_LIST_Y_POS 16
#define SCANNER_MARKER_SIZE 2

#define EEPROM_ADR_STATE 0
#define EEPROM_ADR_TUNE 1
#define EEPROM_ADR_RSSI_MIN_L 2
#define EEPROM_ADR_RSSI_MIN_H 3
#define EEPROM_ADR_RSSI_MAX_L 4
#define EEPROM_ADR_RSSI_MAX_H 5
//#define DEBUG

// Channels to sent to the SPI registers
const uint16_t channelTable[] PROGMEM = {
  // Channel 1 - 8
 /* 0x2A05,    0x299B,    0x2991,    0x2987,    0x291D,    0x2913,    0x2909,    0x289F,    // Band A
  0x2903,    0x290C,    0x2916,    0x291F,    0x2989,    0x2992,    0x299C,    0x2A05,    // Band B
  0x2895,    0x288B,    0x2881,    0x2817,    0x2A0F,    0x2A19,    0x2A83,    0x2A8D,    // Band E
  0x2906,    0x2910,    0x291A,    0x2984,    0x298E,    0x2998,    0x2A02,    0x2A0C,    // Band F / Airwave*/
  0x281D,    0x288F,    0x2902,    0x2914,    0x2978,    0x2999,    0x2A0C,    0x2A1E     // Band R / Immersion Raceband
};

// Channels with their Mhz Values
const uint16_t channelFreqTable[] PROGMEM = {
  // Channel 1 - 8
 /* 5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725, // Band A
  5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866, // Band B
  5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945, // Band E
  5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880, // Band F / Airwave */
  5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917  // Band R / Immersion Raceband
};

// do coding as simple hex value to save memory.
const uint8_t channelNames[] PROGMEM = {
/*  0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8,
  0xB1, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6, 0xB7, 0xB8,
  0xE1, 0xE2, 0xE3, 0xE4, 0xE5, 0xE6, 0xE7, 0xE8,
  0xF1, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6, 0xF7, 0xF8,*/
  0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8
};

// All Channels of the above List ordered by Mhz
const uint8_t channelList[] PROGMEM = {
  19, 18, 32, 17, 33, 16, 7, 34, 8, 24, 6, 9, 25, 5, 35, 10, 26, 4, 11, 27, 3, 36, 12, 28, 2, 13, 29, 37, 1, 14, 30, 0, 15, 31, 38, 20, 21, 39, 22, 23
};

uint8_t channel = 0;
uint8_t channelIndex = 0;
uint8_t rssi = 0;
uint8_t rssi_scaled = 0;
uint8_t hight = 0;
uint8_t state = START_STATE;
uint8_t state_last_used=START_STATE;
uint8_t last_state= START_STATE+1; // force screen draw
uint8_t writePos = 0;
uint8_t switch_count = 0;
uint8_t man_channel = 0;
uint8_t last_channel_index = 0;
uint8_t force_seek=0;
unsigned long time_of_tune = 0;        // will store last time when tuner was changed
uint8_t last_maker_pos=0;
uint8_t last_active_channel=0;
uint8_t first_channel_marker=1;
uint8_t update_frequency_view=0;
uint8_t seek_found=0;
uint8_t last_dip_channel=255;
uint8_t last_dip_band=255;
uint8_t scan_start=0;
uint8_t first_tune=1;
uint8_t force_menu_redraw=0;
uint16_t rssi_min=0;
uint16_t rssi_max=0;
uint16_t rssi_setup_min=0;
uint16_t rssi_setup_max=0;
uint16_t rssi_seek_found=0;
uint16_t rssi_setup_run=0;


//RACE TIMER

#define COUNTDOWN_LENGTH 8
#define MAX_LAPS  5
#define RSSI_STOP_TIMER 50

long interval = 100;                // blink interval - change to suit
long previousMillis = 0;            // variable to store last time LED was updated
long startTime ;                    // start time for stop watch
long elapsedTime ;
bool race_timing = false;
char buf [4];
uint8_t lap_counter=0;
 long laps[MAX_LAPS]; // = 
//{0,0,0,0,0,0,0,0,0,0,0,0,0};
long best_lap=0;
uint8_t lap_time_offsetx = 0;
uint8_t lap_time_offsety = 0;
bool run_once = false;
bool clr_scr = false;

TVout TV;

void setup() {
  // IO INIT
    // initialize digital pin 13 LED as an output.

    
    pinMode(led, OUTPUT); // status pin for TV mode errors
    // buzzer
    pinMode(buzzer, OUTPUT); // Feedback buzzer (active buzzer, not passive piezo)   
    digitalWrite(buzzer, HIGH);
    // minimum control pins
    pinMode(buttonSeek, INPUT);
    digitalWrite(buttonSeek, INPUT_PULLUP);
    pinMode(buttonMode, INPUT);
    digitalWrite(buttonMode, INPUT_PULLUP);
    // optional control
    pinMode(buttonDown, INPUT);
    digitalWrite(buttonDown, INPUT_PULLUP);
    pinMode(buttonSave, INPUT);
    digitalWrite(buttonSave, INPUT_PULLUP);   
    #ifdef DEBUG
    Serial.begin(115200);
    Serial.println(F("START:")); 
#endif
    // SPI pins for RX control
    pinMode (slaveSelectPin, OUTPUT);
    pinMode (spiDataPin, OUTPUT);
  pinMode (spiClockPin, OUTPUT);
    // tune to first channel

    
    // init TV system
    char retVal = TV.begin(TV_FORMAT, TV_COLS, TV_ROWS);
    // 0 if no error.
    // 1 if x is not divisable by 8.
    // 2 if y is to large (NTSC only cannot fill PAL vertical resolution by 8bit limit)
    // 4 if there is not enough memory for the frame buffer.
    if (retVal > 0) {
        // on Error flicker LED
        while (true) { // stay in ERROR for ever
            digitalWrite(13, !digitalRead(13));
            delay(100);
        }
    }
    TV.select_font(font4x6);
    // Setup Done - LED ON
    digitalWrite(13, HIGH);
    
    // use values only of EEprom is not 255 = unsaved
    uint8_t eeprom_check = EEPROM.read(EEPROM_ADR_STATE);
    if(eeprom_check == 255) // unused
    {
        EEPROM.write(EEPROM_ADR_STATE,START_STATE);
        EEPROM.write(EEPROM_ADR_TUNE,CHANNEL_MIN_INDEX);
        // save 16 bit
        EEPROM.write(EEPROM_ADR_RSSI_MIN_L,lowByte(RSSI_MIN_VAL));        
        EEPROM.write(EEPROM_ADR_RSSI_MIN_H,highByte(RSSI_MIN_VAL));    
        // save 16 bit
        EEPROM.write(EEPROM_ADR_RSSI_MAX_L,lowByte(RSSI_MAX_VAL));
        EEPROM.write(EEPROM_ADR_RSSI_MAX_H,highByte(RSSI_MAX_VAL));
    }
    // debug reset EEPROM
    //EEPROM.write(EEPROM_ADR_STATE,255);    
        
    // read last setting from eeprom
    state=EEPROM.read(EEPROM_ADR_STATE);
    channelIndex=EEPROM.read(EEPROM_ADR_TUNE);
    rssi_min=((EEPROM.read(EEPROM_ADR_RSSI_MIN_H)<<8) | (EEPROM.read(EEPROM_ADR_RSSI_MIN_L)));
    rssi_max=((EEPROM.read(EEPROM_ADR_RSSI_MAX_H)<<8) | (EEPROM.read(EEPROM_ADR_RSSI_MAX_L)));
    force_menu_redraw=1; 
    channel=6;    
    setChannelModule(channel); 
}

void loop() {
  // put your main code here, to run repeatedly:
    state_last_used=state; // save save settings
     if (digitalRead(buttonDown) == LOW) // key pressed 
          {      
            if (channel==0)
              channel=7;
            else
              channel--;
            
          //  TV.clear_screen();
          }
          if (digitalRead(buttonSeek) == LOW) // key pressed 
          {
            if (channel==7)
              channel=0;
            else
              channel++;
            
          //  TV.clear_screen();
          }
        
    if (last_channel_index != channel)
    {
      setChannelModule(channel); 
      last_channel_index =channel; 
    }
    if (digitalRead(buttonMode) == LOW) // key pressed 
    {
      #define MAX_MENU 3
        #define MENU_Y_SIZE 12
        #define MENU_Y_SIZE 12
        
        uint8_t menu_id=0;
        // Show Mode Screen            
      
        uint8_t in_menu=1;
        uint8_t in_menu_time_out=10; // 10x 200ms = 2 seconds
        /*
        Enter Mode menu
        Show current mode
        Change mode by MODE key
        Any Mode will refresh screen
        If not MODE changes in 2 seconds, it uses last selected mode
        */
       //  TV.clear_screen();
            // simple menu
       if (state==STATE_SET || state==STATE_STOPPED)
                TV.clear_screen();
        do
        {
          //Print Menu
            TV.select_font(font8x8);
            TV.draw_rect(0,0,TV_X_MAX,TV_Y_MAX,  WHITE); // outer frame
            TV.printPGM(10, TV_Y_OFFSET,  PSTR("IR RACE TIMER"));                 
            TV.draw_line(0,1*TV_Y_GRID,TV_X_MAX,1*TV_Y_GRID,WHITE);
            TV.printPGM(5,TV_Y_OFFSET+1*TV_Y_GRID,  PSTR("START STOP SET")); 
            TV.select_font(font4x6);               
            TV.draw_line(0,2*TV_Y_GRID,TV_X_MAX,2*TV_Y_GRID,WHITE);             
            TV.draw_line(0,3*TV_Y_GRID,TV_X_MAX,3*TV_Y_GRID,WHITE);  state_last_used=state; // save save settings                            
            // selection by inverted box
            switch (menu_id) 
            {              
               case 0: // Start Timer
                    TV.draw_rect(5,16,39,10,  WHITE, INVERT);
                    state=STATE_START;                   
                break;
                case 1: // Stop Timer
                    TV.draw_rect(53,16,31,10,  WHITE, INVERT);   
                    state=STATE_STOP;                    
                break;
                case 2: // Settings 
                    TV.draw_rect(92,16,26,10,  WHITE, INVERT); 
                    state=STATE_SET;
                    clr_scr = true;
                break;               
            } // end switch            

            
            while(digitalRead(buttonMode) == LOW)
            {
                // wait for MODE release
                in_menu_time_out=10;
            }                
            while(--in_menu_time_out && (digitalRead(buttonMode) == HIGH)) // wait for next mode or time out
            {
                delay(200); // timeout delay
            }    
            if(in_menu_time_out==0) 
            {
                in_menu=0; // EXIT
                beep(KEY_DEBOUNCE/2); // beep & debounce
                delay(50); // debounce 
                beep(KEY_DEBOUNCE/2); // beep & debounce
                delay(50); // debounce 
            }
            else // no timeout, must be keypressed
            {
                in_menu_time_out=10;
                beep(50); // beep & debounce
                delay(KEY_DEBOUNCE); // debounce 
                /*********************/
                /*   Menu handler   */
                /*********************/
                if (menu_id < MAX_MENU)
                {
                    menu_id++; // next state
                } 
                else 
                {
                    menu_id = 0; 
                }                  
            }
        } while(in_menu);
        last_state=255; // force redraw of current screen
        switch_count = 0;       
        // clean line?
        TV.print(TV_COLS/2, (TV_ROWS/2), "             ");                        
    } 
    else // key pressed
    { // reset debounce      
        switch_count = 0;    
    }


    //----------------ACTIONS----------------------
    if(force_menu_redraw || state != last_state)
    {
      force_menu_redraw=0;
      switch (state)
      {
//--------------- CountDown----------------------------------------
      case STATE_START:
           TV.printPGM(5 ,TV_Y_OFFSET-1+3*TV_Y_GRID,  PSTR("LAP TIME"));
           TV.select_font(font8x8);
           //TV.clear_screen();
           //Countdown
           for (uint8_t i=COUNTDOWN_LENGTH;i--;i<1)
           {
            TV.print(5,2*TV_Y_GRID+3,"START");
            TV.print(48,2*TV_Y_GRID+3,i+0);
            if (i==0)
               break;
              delay(1000);
              // TV.clear_screen();              
           }
           //Timer Start
           state = STATE_RUN;
           startTime = millis();       
           break;
//-------------Stop Timer----------------------------------           
      case STATE_STOP:
           /* TV.select_font(font8x8);
           //  TV.clear_screen();
           elapsedTime =   millis() - startTime; 
           // TV.print(16,46,elapsedTime / 1000L);
           printTime(5,2*TV_Y_GRID+3, elapsedTime);*/
           lap_counter=0;
           lap_time_offsetx = 0;
           lap_time_offsety = 0;
           // TV.printPGM(5 ,TV_Y_OFFSET-1+3*TV_Y_GRID,  PSTR("STOP"));
           state = STATE_STOPPED;
          break;
//---------Settings-----------------------------------------
      case STATE_SET:
          TV.clear_screen();
          TV.select_font(font8x8);
            TV.draw_rect(0,0,TV_X_MAX,TV_Y_MAX,  WHITE); // outer frame
            TV.printPGM(10, TV_Y_OFFSET,  PSTR("IR RACE TIMER"));                 
            TV.draw_line(0,1*TV_Y_GRID,TV_X_MAX,1*TV_Y_GRID,WHITE);
            TV.printPGM(5,TV_Y_OFFSET+1*TV_Y_GRID,  PSTR("SETTINGS")); 
            TV.select_font(font4x6);               
            TV.draw_line(0,2*TV_Y_GRID,TV_X_MAX,2*TV_Y_GRID,WHITE);             
            TV.draw_line(0,3*TV_Y_GRID,TV_X_MAX,3*TV_Y_GRID,WHITE);
          
          if (digitalRead(buttonDown) == LOW) // key pressed 
          {      
            if (channel==0)
              channel=7;
            else
              channel--;
            
          //  TV.clear_screen();
          }
          if (digitalRead(buttonSeek) == LOW) // key pressed 
          {
            if (channel==7)
              channel=0;
            else
              channel++;
            
          //  TV.clear_screen();
          }
          if (last_channel_index != channel)
          {
            setChannelModule(channel); 
            last_channel_index =channel; 
          }
          TV.print(50,TV_Y_OFFSET+3*TV_Y_GRID, pgm_read_word_near(channelFreqTable + channel)); 
         // TV.printPGM(5 ,TV_Y_OFFSET-1+3*TV_Y_GRID,  PSTR("DELETE"));         
        // force tune on new scan start to get right RSSI value      
      break;
//-----------Timer Running-------------------------------------
      case STATE_RUN:
          //TV.print(16,46,(millis() - startTime)/10L);          
          // print bar for spectrum
          wait_rssi_ready();
          // value must be ready
          rssi = readRSSI();
          #define RSSI_BAR_SIZE 60
          rssi_scaled=map(rssi, 1, 100, 1, RSSI_BAR_SIZE);        
         
          if (rssi_scaled>RSSI_STOP_TIMER)
          {        
              if (lap_counter>0)
              { 
                laps[lap_counter]=  millis() - startTime;
                elapsedTime =  laps[lap_counter]; 
                for (int lap=0 ;lap< lap_counter; lap++)
                {
                  laps[lap_counter]=   laps[lap_counter]- laps[lap];             
                }
              /*    if (best_lap>laps[lap_counter])
               best_lap= laps[lap_counter];*/
          
              }
              else
              {
                laps[lap_counter]=  millis() - startTime;
                //    best_lap= laps[lap_counter]; 
              }
               if (lap_counter>6)
              {
                  lap_time_offsetx = 42;
                  lap_time_offsety = 42;
              }
              
              TV.select_font(font4x6); //TV_Y_OFFSET-1+3*TV_Y_GRID
              TV.print(5+lap_time_offsetx,TV_Y_OFFSET-1+3*TV_Y_GRID+6+6*lap_counter-lap_time_offsety, lap_counter+1);
              printTimeSmall(15+lap_time_offsetx,TV_Y_OFFSET-1+3*TV_Y_GRID+6+6*lap_counter-lap_time_offsety,laps[lap_counter]);
               
              if (lap_counter==MAX_LAPS-1)
              {
                  state= STATE_STOP;
                  TV.select_font(font8x8);
                  printTime(5,2*TV_Y_GRID+3, elapsedTime);
              }
            
              lap_counter++;
              /*      TV.print(5,6*TV_Y_GRID+3, "Best");
              printTime(25,6*(lap_counter+1)*TV_Y_GRID+3,best_lap);*/
              delay(2000);
        
          } 
          //Print Race Time
          TV.select_font(font8x8);
          printTime(5,2*TV_Y_GRID+3,millis() - startTime);
         
           // clear last bar
          TV.draw_rect(62, TV_Y_OFFSET+2*TV_Y_GRID+2, RSSI_BAR_SIZE,4 , BLACK, BLACK);
          //  draw new bar
          TV.draw_rect(62, TV_Y_OFFSET+2*TV_Y_GRID+2, rssi_scaled, 4 , WHITE, WHITE);    
            
          rssi_scaled=map(rssi, 1, 100, 5, SCANNER_BAR_SIZE);
          TV.print(72,2*TV_Y_GRID+3,rssi_scaled);
          hight = (TV_ROWS - TV_SCANNER_OFFSET - rssi_scaled);
          // clear last bar
          TV.draw_rect((channel * 3), (TV_ROWS - TV_SCANNER_OFFSET - SCANNER_BAR_SIZE), 2, SCANNER_BAR_SIZE , BLACK, BLACK);
          //  draw new bar
          TV.draw_rect((channel * 3), hight, 2, rssi_scaled , WHITE, WHITE);
          delay(10);
          break;
//------------Race Timer Stopped-----------------------
       case STATE_STOPPED:
          //TV.print(16,46,millis() - startTime);
          delay(10);
          break;
      }
   }
}

/*###########################################################################*/
/*******************/
/*   SUB ROUTINES  */
/*******************/    
void beep(uint16_t time)
{
    digitalWrite(buzzer, LOW);
    delay(time);
    digitalWrite(buzzer, HIGH);
}
void printTime(uint8_t dx,  uint8_t dy, long timeMillis)
{
    long minute = 60000; // 60000 milliseconds in a minute
    long second =  1000; // 1000 milliseconds in a second
    long second100 =  10; // 100 100seconds in a second

    int minutes = timeMillis  / minute ;         //and so on...
    int seconds = (timeMillis % minute) / second;
    int seconds100 = ((timeMillis % minute)% second) / second100;
    if (minutes==0)
       TV.print(dx,dy,"0:");
    else 
    {
      TV.print(dx,dy,minutes);
      TV.print(dx+8,dy,':'); 
    }
     if (seconds<10)
     {
      TV.print(dx+16,dy,0);
      TV.print(dx+24,dy,seconds);
     }
     else
       TV.print(dx+16,dy,seconds);
       
     TV.print(dx+32,dy,':');
     if (seconds100<10)
     {
       TV.print(dx+40,dy,0);
       TV.print(dx+48,dy,seconds100);
     }
     else
       TV.print(dx+40,dy,seconds100);  
}

void printTimeSmall(uint8_t dx,  uint8_t dy, long timeMillis)
{
    long minute = 60000; // 60000 milliseconds in a minute
    long second =  1000; // 1000 milliseconds in a second
    long second100 =  10; // 100 100seconds in a second

    int minutes = timeMillis  / minute ;         //and so on...
    int seconds = (timeMillis % minute) / second;
    int seconds100 = ((timeMillis % minute)% second) / second100;
    if (minutes==0)
       TV.print(dx,dy,"0:");
    else 
    {
     TV.print(dx,dy,minutes);
     TV.print(dx+4,dy,':'); 
    }
     if (seconds<10)
     {
      TV.print(dx+8,dy,0);
      TV.print(dx+12,dy,seconds);
     }
     else
       TV.print(dx+8,dy,seconds);
     TV.print(dx+16,dy,':');
     if (seconds100<10)
     {
       TV.print(dx+20,dy,0);
       TV.print(dx+24,dy,seconds100);
     }
     else     
       TV.print(dx+20,dy,seconds100);
}


uint8_t channel_from_index(uint8_t channelIndex)
{
    uint8_t loop=0;
    uint8_t channel=0;
    for (loop=0;loop<=CHANNEL_MAX;loop++)
    {
        if(pgm_read_byte_near(channelList + loop) == channelIndex)
        {
            channel=loop;
            break;
        }
    }
    return (channel);
}    


void wait_rssi_ready()
{
    // CHECK FOR MINIMUM DELAY
    // check if RSSI is stable after tune by checking the time
    uint16_t tune_time = millis()-time_of_tune;
    // module need >20ms to tune.
    // 30 ms will to a 32 channel scan in 1 second.
    #define MIN_TUNE_TIME 30
    if(tune_time < MIN_TUNE_TIME)
    {
        // wait until tune time is full filled
        delay(MIN_TUNE_TIME-tune_time);
    }
}
        

uint16_t readRSSI() 
{
    uint16_t rssi = 0;
    for (uint8_t i = 0; i < 10; i++) 
    {
        rssi += analogRead(rssiPin);
    }
    rssi=rssi/10; // average
    // special case for RSSI setup
    if(state==STATE_RSSI_SETUP)
    { // RSSI setup
        if(rssi < rssi_setup_min)
        {
            rssi_setup_min=rssi;
            TV.print(50, SCANNER_LIST_Y_POS, "   ");
            TV.print(50, SCANNER_LIST_Y_POS, rssi_setup_min , DEC);            
        }
        if(rssi > rssi_setup_max)
        {
            rssi_setup_max=rssi;
        TV.print(110, SCANNER_LIST_Y_POS, "   ");
        TV.print(110, SCANNER_LIST_Y_POS, rssi_setup_max , DEC);                    
        }    
        // dump current values
    }   
    //TV.print(50, SCANNER_LIST_Y_POS-10, rssi_min , DEC);  
    //TV.print(110, SCANNER_LIST_Y_POS-10, rssi_max , DEC); 
    // scale AD RSSI Valaues to 1-100%     
    //#define RSSI_DEBUG 

    // Filter glitches
    #ifdef RSSI_DEBUG
        TV.print(1,20, "RAW:             ");
        TV.print(30,20, rssi, DEC);    
    #endif
    rssi = constrain(rssi, rssi_min, rssi_max);    //original 90---250
    rssi=rssi-rssi_min; // set zero point (value 0...160)
    rssi = map(rssi, 0, rssi_max-rssi_min , 1, 100);   // scale from 1..100%
    #ifdef RSSI_DEBUG
        TV.print(1,40, "SCALED:           ");    
        TV.print(50,40, rssi, DEC);    
    #endif
    
    return (rssi);
}

// Private function: from http://arduino.cc/playground/Code/AvailableMemory
int freeRam () {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

void setChannelModule(uint8_t channel)
{
  uint8_t i;
  uint16_t channelData;

  //channelData = pgm_read_word(&channelTable[channel]);
  //channelData = channelTable[channel];
  channelData = pgm_read_word_near(channelTable + channel);

  // bit bash out 25 bits of data
  // Order: A0-3, !R/W, D0-D19
  // A0=0, A1=0, A2=0, A3=1, RW=0, D0-19=0
  SERIAL_ENABLE_HIGH();
  delayMicroseconds(1);  
  //delay(2);
  SERIAL_ENABLE_LOW();

  SERIAL_SENDBIT0();
  SERIAL_SENDBIT0();
  SERIAL_SENDBIT0();
  SERIAL_SENDBIT1();

  SERIAL_SENDBIT0();

  // remaining zeros
  for (i = 20; i > 0; i--)
    SERIAL_SENDBIT0();

  // Clock the data in
  SERIAL_ENABLE_HIGH();
  //delay(2);
  delayMicroseconds(1);  
  SERIAL_ENABLE_LOW();

  // Second is the channel data from the lookup table
  // 20 bytes of register data are sent, but the MSB 4 bits are zeros
  // register address = 0x1, write, data0-15=channelData data15-19=0x0
  SERIAL_ENABLE_HIGH();
  SERIAL_ENABLE_LOW();

  // Register 0x1
  SERIAL_SENDBIT1();
  SERIAL_SENDBIT0();
  SERIAL_SENDBIT0();
  SERIAL_SENDBIT0();

  // Write to register
  SERIAL_SENDBIT1();

  // D0-D15
  //   note: loop runs backwards as more efficent on AVR
  for (i = 16; i > 0; i--)
  {
    // Is bit high or low?
    if (channelData & 0x1)
    {
      SERIAL_SENDBIT1();
    }
    else
    {
      SERIAL_SENDBIT0();
    }

    // Shift bits along to check the next one
    channelData >>= 1;
  }

  // Remaining D16-D19
  for (i = 4; i > 0; i--)
    SERIAL_SENDBIT0();

  // Finished clocking data in
  SERIAL_ENABLE_HIGH();
  delayMicroseconds(1);
  //delay(2);

  digitalWrite(slaveSelectPin, LOW);
  digitalWrite(spiClockPin, LOW);
  digitalWrite(spiDataPin, LOW);
}


void SERIAL_SENDBIT1()
{
  digitalWrite(spiClockPin, LOW);
  delayMicroseconds(1);

  digitalWrite(spiDataPin, HIGH);
  delayMicroseconds(1);
  digitalWrite(spiClockPin, HIGH);
  delayMicroseconds(1);

  digitalWrite(spiClockPin, LOW);
  delayMicroseconds(1);
}

void SERIAL_SENDBIT0()
{
  digitalWrite(spiClockPin, LOW);
  delayMicroseconds(1);

  digitalWrite(spiDataPin, LOW);
  delayMicroseconds(1);
  digitalWrite(spiClockPin, HIGH);
  delayMicroseconds(1);

  digitalWrite(spiClockPin, LOW);
  delayMicroseconds(1);
}

void SERIAL_ENABLE_LOW()
{
  delayMicroseconds(1);
  digitalWrite(slaveSelectPin, LOW);
  delayMicroseconds(1);
}

void SERIAL_ENABLE_HIGH()
{
  delayMicroseconds(1);
  digitalWrite(slaveSelectPin, HIGH);
  delayMicroseconds(1);
}
