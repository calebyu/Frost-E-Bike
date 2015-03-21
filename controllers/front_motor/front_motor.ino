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
#define LED 13

#define CANS 2
#define CANTX 3
#define CANRX 4

#define THERM1 A9
#define THERM2 A4

#define HALL3 9
#define HALL2 8
#define HALL1 7

#define TICKS_PER_CYCLE 69
#define CIRCUMFERENCE 2.07

#include <FlexCAN.h>
#include <FrostEBike.h>
#include <math.h>
//CANBus
FlexCAN CANbus(CAN_BAUD);
CAN_message_t msg,rxmsg;

//loop timing
long previousTime = 0;
long interval_set = 10; //ms  
long spd_msg_cnt = 10;

//Settings
int ABS = 1, TRC = 1;

//State
int ff1, ff2, curr, curr_r, curr_ref_fb, curr_set = 0, brake_set = 0;
int ABS_trig, TRC_trig = 0;
int target_current = 0;
int loop_cnt = 0;
int spd_cnt = 0;
//float spd = 0;
float acc = 0;
float prev_spd = 0;
unsigned long prevtime = 0; 
float TRC_weight = 0;
float ABS_weight = 0;
LPF spd = NULL;
int off_cnt = 0;
int brake = 0;

void isr_spdcnt(){
  cli();
  spd_cnt++;
  sei();
}

void setup() {
  //{Pin Mode
  pinMode(COAST, OUTPUT); 
  digitalWrite(COAST, LOW); //Start by coasting
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
  //}
  
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
  
  //Setup Interrupts
  attachInterrupt(TACHO, isr_spdcnt, CHANGE);
  
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
  filter.id = FRONT_MOTOR_ID;
  for (int i = 0; i<8;i++)
  CANbus.setFilter(filter,i);
  
  
  spd = LPF(10);
  delay(100);
  prevtime = millis();
  //digitalWrite(COAST, HIGH); //GET RID OF THIS CODE
  
  digitalWrite(COAST, LOW); 
}

void loop() { unsigned long currentTime = millis();
  //wrap case
  if (currentTime < previousTime) previousTime = currentTime;
   
  //time limited loop
  if(currentTime - previousTime > interval_set) {
  
  digitalWrite(LED,HIGH);
  previousTime = currentTime; 
  
  Serial.println("I am front motor ");
    
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
        target_current = rxmsg.buf[1] + 150;// 150 is measurement offset
        ABS = rxmsg.buf[2]; 
        TRC = rxmsg.buf[3];
      }
    } 
  }
  
  //{ Process speed
  cli();
    spd.addPoint( ((float) spd_cnt / TICKS_PER_CYCLE) * CIRCUMFERENCE / (millis() - prevtime )*3600);
    spd_cnt = 0;
  sei();
  acc = (spd.getValue() - prev_spd)/(millis() - prevtime)*1000;
  prev_spd = spd.getValue();
  prevtime = millis();
  //}
  //{ Traction control
  if (acc > 2)
    TRC_weight += .3;
  else
    TRC_weight -= .9;
  // Limit traction
  if (TRC_weight < 0) TRC_weight = 0;
  if (TRC_weight > 5) TRC_weight = 5;
  
  // traction turn off flag
  if (!TRC) TRC_weight = 0;
  
  //Report to central if traction control is activated
  if (TRC_weight > 0) TRC_trig = 1;
  else TRC_trig = 0;
  //}
  
  //{ ABS control
  if (acc < -0.5)
    ABS_weight += 10;
  else
    ABS_weight -= 5;
  if (ABS_weight < 0) ABS_weight = 0;
  if (ABS_weight > 5) ABS_weight = 5;
  
  if (!ABS) ABS_weight = 0;
  
  if (ABS_weight > 0) ABS_trig = 1;
  else ABS_trig = 0;
  //}
  
  // Serial debug output
  Serial.print("SPD: ");
  Serial.println(prev_spd,DEC);
  Serial.print("ACC: ");
  Serial.println(acc,DEC);
  Serial.print("Weight: ");
  Serial.println(TRC_weight,DEC);
  Serial.print("TAR TOR: ");
  Serial.println(target_current);
  
  //{ Send CANBus - report velocity
  if (spd_msg_cnt > 10){
    spd_msg_cnt = 0;
    msg.id = FRONT_MOTOR_ID << 4 | CENTRAL_ID;
    for( int idx=0; idx<8; ++idx ) {
      msg.buf[idx] = 0;
    } 
    msg.len = 4;
    msg.buf[0] = REPORT_VELOCITY;
    msg.buf[1] = spd.getValue();
    msg.buf[2] = ABS_trig;
    msg.buf[3] = TRC_trig;
    int err = CANbus.write(msg);
    Serial.println (String(err)); 
  }    
  else{
    spd_msg_cnt++;
  }
  //}
  ff1 = digitalRead(FF1); // fault flag
  ff2 = digitalRead(FF2); // fault flag
  curr = analogRead(CUR_SEN); // actual forward current
  curr_r = analogRead(CUR_SEN_R); // actual reverse current
  curr_ref_fb = analogRead(CUR_REF_FB); // sense of current set value
  
  if (target_current > 167) {
    digitalWrite(COAST, HIGH); 
    curr_set += 4*(target_current - curr - TRC_weight); 
  }  
  else if ( brake ){
    digitalWrite(COAST, HIGH);
    brake_set = 200 - ABS_weight;
  }
  else
    off_cnt++
    
  if (off_cnt > 5){  
    digitalWrite(COAST, LOW); 
    off_cnt = 0;
  }
  //curr_set = 35 to overcome mechanical losses
  
  /*Serial.print("FF1: ");
  Serial.println(ff1, DEC);
  Serial.print("FF2: ");
  Serial.println(ff2, DEC);*/
  Serial.print("CURR: ");
  Serial.println(curr, DEC);
  Serial.print("CURR_R: ");
  Serial.println(curr_r, DEC);
  Serial.print("CURR Ref: ");
  Serial.println(curr_set, DEC);
  Serial.print("CURR Ref FB: ");
  Serial.println(curr_ref_fb, DEC);
  //Serial.print("CURR Set: ");
  //Serial.println(curr_set/256.*3.3, DEC);
  
  //digitalWrite(COAST, HIGH); // turn coast off
  digitalWrite(DIR, LOW);
  
  if (curr_set < 0) curr_set = 0;
  if (curr_set > 255) curr_set = 255;
  analogWrite(CUR_REF, curr_set);
  //analogWrite(PWM_OUT,50);
  
  if (brake_set < 0) brake_set = 0;
  if (brake_set > 255) brake_set = 255;
  analogWrite(BRAKE, brake_set);
  
  }
}
