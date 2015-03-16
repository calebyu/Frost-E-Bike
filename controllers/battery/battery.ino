#define THERM1 A9
#define THERM2 A8

#define CUR_SEN A4
#define CUR_SEN_REV A3

#define RAIL_SEN A0

#define CHARGE_RESET 12
#define CANS 2

#define BAT_CONN_REV 7
#define BAT_CONN 6
#define HEAT_ON 5

#include <FlexCAN.h>
#include <FrostEBike.h>

//CANBus
FlexCAN CANbus(CAN_BAUD);
CAN_message_t msg,rxmsg;

int ABS, TRC = 1;
int target_current = 0;

void setup() {
  pinMode(THERM1, INPUT);
  pinMode(THERM2, INPUT);
  pinMode(CUR_SEN, INPUT);
  pinMode(CUR_SEN_REV, INPUT);
  pinMode(RAIL_SEN, INPUT);
  pinMode(CHARGE_RESET, INPUT);
  pinMode(CANS, OUTPUT);
  pinMode(BAT_CONN_REV, OUTPUT);
  pinMode(BAT_CONN, OUTPUT);
  pinMode(HEAT_ON, OUTPUT);
  
  digitalWrite(HEAT_ON, LOW);
  digitalWrite(BAT_CONN, LOW);
  digitalWrite(BAT_CONN_REV, LOW);
  digitalWrite(CANS, LOW);
  
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH); //light on, just because

  Serial.begin(9600);
  Serial.println("Hello world!");
  
  CAN_filter_t mask;
  mask.rtr = 0;
  mask.ext = 0;
  mask.id = 0xFF;  
  CANbus.begin(mask);
  //Set filters - specs says to set all the filters 
  CAN_filter_t filter;
  filter.rtr = 0;
  filter.ext = 0;
  filter.id = BATTERY_ID;
  for (int i = 0; i<8;i++)
  CANbus.setFilter(filter,i);
  
    
}

void loop() {
    Serial.println("I am battery");
  int ff1, ff2, curr, curr_r, curr_ref_fb, curr_set;
  
  //Read CANBus
  while (CANbus.read(rxmsg))
  {
    Serial.print("MSG RCV: ");
    switch (rxmsg.buf[0])
    {
      case GENERIC:
      {
        Serial.println(String(rxmsg.buf[1]));
        break; 
      }
      case SET_TORQUE:
      {
        target_current = rxmsg.buf[1];
        ABS = rxmsg.buf[2]; 
        TRC = rxmsg.buf[3];
      }
    } 
  }
  
    Serial.println("...");
  
  analogWrite(BAT_CONN, 250);
  digitalWrite(BAT_CONN_REV, LOW);
  
  int curr, curr_rev, rail;
  curr = analogRead(CUR_SEN);
  curr_rev = analogRead(CUR_SEN_REV);
  rail = analogRead(RAIL_SEN);
  
  Serial.print("CURR: ");
  Serial.println(curr, DEC);
  Serial.print("CURR REV: ");
  Serial.println(curr_rev, DEC);
  /*Serial.print("RAIL: ");
  Serial.println(rail, DEC);*/
  
  delay(1000);

}
