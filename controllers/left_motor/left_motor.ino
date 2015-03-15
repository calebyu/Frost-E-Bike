#define COAST 22
#define PWM_OUT 21
#define VDSTH 20
#define BRAKE 19
#define CUR_REF_FB A2
#define CUR_SEN_R A1
#define CUR_SEN A0
#define MODE 13
#define CUR_REF A14
#define ESF 12
#define TACHO 11
#define RST_OUT 10
#define DIR 6
#define DIRO 5
#define FF1 1
#define FF2 0

#define CANS 2
#define CANTX 3
#define CANRX 4

#define THERM1 A9
#define THERM2 A4

#define HALL3 9
#define HALL2 8
#define HALL1 7

#include <FlexCAN.h>
#include <FrostEBike.h>

//CANBus
FlexCAN CANbus(50000);
CAN_message_t msg,rxmsg;

int ABS, TRC = 1;
int target_current = 0;

void setup() {
  pinMode(COAST, OUTPUT); 
  pinMode(PWM_OUT, OUTPUT);
  pinMode(VDSTH, OUTPUT);
  pinMode(BRAKE, OUTPUT);
  pinMode(CUR_REF_FB, INPUT);
  pinMode(CUR_SEN_R, INPUT);
  pinMode(CUR_SEN, INPUT);
  pinMode(MODE, OUTPUT);
  pinMode(CUR_REF, OUTPUT);
  pinMode(ESF,  OUTPUT);
  pinMode(TACHO, INPUT);
  pinMode(RST_OUT, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(DIRO, INPUT);
  pinMode(FF1, INPUT_PULLUP);
  pinMode(FF2, INPUT_PULLUP);

  pinMode(CANS, OUTPUT);  
  
  pinMode(THERM1, INPUT);
  pinMode(THERM2, INPUT);
  
  pinMode(HALL1, INPUT);
  pinMode(HALL2, INPUT);
  pinMode(HALL3, INPUT);
  
  digitalWrite(COAST, LOW); //Start by coasting
  digitalWrite(PWM_OUT, HIGH);
  digitalWrite(VDSTH, HIGH); //TODO: Set Realisticaly with analog value
  digitalWrite(BRAKE, HIGH); // BRAKE OFF
  digitalWrite(MODE, HIGH);
  //digitalWrite(CUR_REF
  digitalWrite(ESF, LOW);
  digitalWrite(RST_OUT,LOW);
  digitalWrite(DIR, HIGH);
  digitalWrite(CANS, LOW);
  digitalWrite(RST_OUT, HIGH);
  
  
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
  filter.id = LEFT_MOTOR_ID;
  for (int i = 0; i<8;i++)
  CANbus.setFilter(filter,i);
  
    
}

void loop() {
    Serial.println("I am left motor ");
  int ff1, ff2, curr, curr_r, curr_ref_fb, curr_set;
  
  //CANBus
  msg.len = 3;
  msg.id = GENERATOR_ID;
  for( int idx=0; idx< msg.len; ++idx ) {
      msg.buf[idx] = 0;
  }
  msg.buf[0] = GENERIC;
  msg.buf[1] = 'h';
  msg.buf[2] = 'w';
  int err = 0;
  err = CANbus.write(msg);
  Serial.println(String(err));
  
  while (CANbus.read(rxmsg)) {
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
  
  //{ Send CANBus
  //}
  ff1 = digitalRead(FF1);
  ff2 = digitalRead(FF2);
  curr = analogRead(CUR_SEN);
  curr_r = analogRead(CUR_SEN_R);
  curr_ref_fb = analogRead(CUR_REF_FB);
  curr_set = (int)( (130.0/4.0)*(2.5/3.3) );
  
  /*Serial.print("FF1: ");
  Serial.println(ff1, DEC);
  Serial.print("FF2: ");
  Serial.println(ff2, DEC);*/
  Serial.print("CURR: ");
  Serial.println(curr, DEC);
  Serial.print("CURR_R: ");
  Serial.println(curr_r, DEC);
  /*Serial.print("CURR Ref FB: ");
  Serial.println(curr_ref_fb/1024.*2.5, DEC);
  Serial.print("CURR Set: ");
  Serial.println(curr_set/256.*3.3, DEC);*/
  
  //digitalWrite(COAST, HIGH); // turn coast off
  digitalWrite(DIR, LOW);
  analogWrite(CUR_REF, curr_set);
  
  delay(1000);

}
