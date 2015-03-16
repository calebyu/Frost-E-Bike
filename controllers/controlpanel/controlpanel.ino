//Pin parameters
#define button_pin 1 
#define rotary_A_pin  18
#define rotary_B_pin  19
#define LED_pin  13

#define CANS 2

#include <FlexCAN.h>
#include <OLED.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <FrostEBike.h>


// Components
FlexCAN CANbus(CAN_BAUD);
CAN_message_t msg,rxmsg;
OLED oled(9,11,10); // sends pin numbers for SCL, SDO, and CS, respectively.

long previousTime = 0;
long interval = 100;  

//Internal state variables
int isDash = 1; // 0 = menu; 1 = dashboard;
int prevRotary = -1; // 0 = 00; 1 = 01; 2 = 11; 3 = 10;
int isPress = 0;
int spin = 0;
int menu_index = 0;

//settings
int spd = 0;
int bat = 0;
int timeout = 0;

//for fun variables
int cnt = 0;
int spinFlag = 0;

// 0 is noting, 1 is CW, 2 is CCW
void isr_rotary()
{
  int rotary_A_in = digitalRead(rotary_A_pin); 
  int rotary_B_in = digitalRead(rotary_B_pin);
  int state;
  
  if (!rotary_A_in && !rotary_B_in)
     state = 0; 
  else if (!rotary_A_in && rotary_B_in)
     state = 1; 
  else if (rotary_A_in && rotary_B_in)
     state = 2; 
  else if (rotary_A_in && !rotary_B_in)
     state = 3; 

  //update state
  if (state == 0 || state == 2)
    prevRotary = state;  
  else // figure out turning
  {
     if (prevRotary == 0)
     {
        if (state == 1) 
          spin++;
        else
          spin--;
     }
     else
     {
        if (state == 1) 
          spin--;
        else
          spin++;
     }
  }
}

void updateDashboard ()
{
  oled.print(1,"      Dashboard     ");
  oled.print(2,"Bat: "+ String(bat) + "% Spd: "+ String(spd) +"km/h");
  oled.print(3,"Mode: Pedal TRAC ABS");
  oled.print(4,"Rotary Test: " + String(cnt));
}

void updateMenu ()
{
  oled.print(1,"        Menu        ");
  oled.print(2,"8: (>'.')>          ");
  oled.print(3,"1: Test the menu    ");
  oled.print(4,"2: <('.'<)          ");
}

void setup() {
  pinMode(CANS, OUTPUT);
  digitalWrite(CANS, HIGH);
  // initializes i/o stuff
  Serial.begin(9600);
  oled.begin();
  
  //Setup CANBus
  CAN_filter_t mask;
  mask.rtr = 0;
  mask.ext = 0;
  mask.id = 0xFF;  
  CANbus.begin(mask);
  //Set filters - specs says to set all the filters 
  CAN_filter_t filter;
  filter.rtr = 0;
  filter.ext = 0;
  filter.id = CONTROL_PANEL_ID;
  for (int i = 0; i<8;i++)
    CANbus.setFilter(filter,i);
  
  //LED
  pinMode(LED_pin,OUTPUT);
  digitalWrite(LED_pin,HIGH); //OTUPUT??
  
  //Setup button
  pinMode(button_pin,INPUT_PULLUP);
  
  //Setup Rotary Encoder 
  pinMode(rotary_A_pin,INPUT);
  pinMode(rotary_B_pin,INPUT);
  attachInterrupt(rotary_A_pin, isr_rotary, CHANGE);
  attachInterrupt(rotary_B_pin, isr_rotary, CHANGE);
  
  //Setup Dash board
  delay(100);
  timeout = 0;  
}

void loop() { unsigned long currentTime = millis();
  //wrap case
  if (currentTime < previousTime) previousTime = currentTime;
  
  //time limited loop
  if(currentTime - previousTime > interval) {
  
  digitalWrite(LED_pin,HIGH);
  previousTime = currentTime;   
  
  //CANBus
  if (CANbus.read(rxmsg))
  {
    switch (rxmsg.buf[0])
    {
      case DASHBOARD_OUTPUT:
      {
        spd = rxmsg.buf[1];
        bat = rxmsg.buf[2];
        break; 
      }
    } 
  }
  
  //OLED 
  //Menu
  if (!isDash)
  {
     //Check menu stuff
     updateMenu();
     //Check exit menu  
     if (digitalRead(button_pin) == 0)
        isPress ++;       
     else if (isPress > 10 || timeout < 1)
     {
       isDash = 1;
       isPress = 0;
	   cnt = 0;
     }
     else timeout--;
     
  } 
  //Dashboard
  else
  {
     cli();
     cnt += spin;
	 sei();
     updateDashboard();
      
     //Check exit dash  
     if (digitalRead(button_pin) == 0)
        isPress = 1;       
     else if (isPress)
     {
       isDash = 0;
       isPress = 0;
       timeout = 50;
     }
  }
  }
  digitalWrite(LED_pin,LOW);
}

