//Pin parameters
#define button_pin 1 
#define rotary_A_pin  7
#define rotary_B_pin  8
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
int menu_mode = 0; // 0 = browse, 1 = enter 
const int MENU_SIZE = 4;
int buttonFlag = 0;
int exitFlag = 0;
//settings
int spd = 0;
int bat = 0;
int timeout = 0;
int ABS_on = 1;
int TRC_on = 1;
int lights_on = 1;
int pedal_ratio = 100;

//for fun variables
int cnt = 0;
int spinFlag = 0;

void isr_button() {
  cli();
  buttonFlag = 1;
  sei();
}
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
     cli();
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
     sei();
  }
}

void updateDashboard ()
{
  oled.print(1,"      Dashboard     ");
  oled.print(2,"Bat: "+ String(bat) + "% Spd: "+ String(spd) +"km/h");
  oled.print(3,"Mode: Pedal TRAC ABS");
  oled.print(4,"Rotary Test: " + String(cnt) + " " +String(prevRotary));
}

void updateMenu ()
{
  String margin = " ";
  if (!menu_mode)
    oled.print(1,"BROWSE  Menu     "+ String(timeout));
  else
    oled.print(1,"ENTER   Menu     "+ String(timeout));
  
  for (int i = 2; i<5; i++){
    if (i == 3) margin = ">";
    else margin = " ";
    switch (menu_index - 3 + i){
      case -1 :{
        oled.print(i,margin + "4: Pedal Ratio: " + String(pedal_ratio));
        break;
      }
      case 0 :{
        if (ABS_on)
          oled.print(i,margin + "1: ABS ON          ");
        else
          oled.print(i,margin + "1: ABS OFF         ");
        break;
      }
      case 1 :{
        if (TRC_on)
          oled.print(i,margin + "2: Tract Ctrl ON   ");
        else 
          oled.print(i,margin + "2: Tract Ctrl OFF  ");
        break;
      }
      case 2 :{
        if (lights_on)
          oled.print(i,margin + "3: Headlights ON   ");
        else 
          oled.print(i,margin + "3: Headlights OFF  ");
        break;
      }
      case 3 :{
        oled.print(i,margin + "4: Pedal Ratio: " + String(pedal_ratio) );
        break;
      }
      case 4 :{
        if (ABS_on)
          oled.print(i,margin + "1: ABS ON          ");
        else
          oled.print(i,margin + "1: ABS OFF         ");
        break;
      }
    }
  }
  
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
  attachInterrupt(button_pin, isr_button, RISING);
  
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
    if (isPress > 10){
      exitFlag = 1;
    }
    if ((exitFlag && buttonFlag == 1)|| timeout < 1)
    {
      exitFlag = 0;
      buttonFlag = 0; 
      isDash = 1;
      isPress = 0;
      cnt = 0;
      menu_mode = 0;      
    }    
    else if ( digitalRead (button_pin) == 0 ) isPress++;   
    else if (buttonFlag == 1 ){ 
      buttonFlag = 0; 
      timeout = 50;
      if (menu_mode) menu_mode = 0;
      else {
        switch (menu_index){
          case 0 :{
            if (ABS_on)
              ABS_on = 0;
            else
              ABS_on = 1;
            break;
          }
          case 1 :{
            if (TRC_on)
              TRC_on = 0;
            else 
              TRC_on = 1;
            break;
          }
          case 2 :{
            if (lights_on)
              lights_on = 0;
            else 
              lights_on = 1;
            break;
          }
          case 3 :{
            menu_mode = 1;
            break;
          }
        }
      }
    }
    else if (spin != 0){
      timeout = 50;
      if (!menu_mode) { // browse mode 
        menu_index += spin;
        if (menu_index < 0) menu_index += MENU_SIZE;
        else if (menu_index >= MENU_SIZE) menu_index -= MENU_SIZE;
      }
      else if (menu_mode) { // entry mode 
        switch (menu_index){
          case 3 :{
            cli();
            pedal_ratio += spin;
            spin = 0;
            sei();
            if (pedal_ratio > 255) pedal_ratio =  255;
            else if (pedal_ratio < 0 ) pedal_ratio =  0;
            break;
          }
        }
      }
      cli();
      spin = 0;
      sei();
    }
    else {
      timeout--;
      isPress = 0;
    }
  } 
  //Dashboard
  else
  {
     cli();
     cnt += spin;
     spin = 0;
     sei();
     updateDashboard();
      
     //Check exit dash  
     if (buttonFlag){
        isPress = 1;  
        buttonFlag = 0;    
     } 
     else if (isPress){
       isDash = 0;
       isPress = 0;
       timeout = 100;
       menu_index = 0;
     }
  }
  }
  digitalWrite(LED_pin,LOW);
}

