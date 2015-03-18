#define THERM1 A1
#define THERM2 A0

#define RAIL_SEN A9
#define CUR_SEN A8

#define GATES 5
#define HALL1 10
#define HALL2 11
#define HALL3 12

#define CANS 2

#include <FlexCAN.h>
#include <FrostEBike.h>

//State
LPF spd = NULL;
int curr_in = 0;
volatile int spd_cnt = 0;
int prevtime = 0;
int curr, rail;
  
void isr_spdcnt(){
  cli();
  spd_cnt++;
  sei();
} 

//CANBus
FlexCAN CANbus(CAN_BAUD);
CAN_message_t msg,rxmsg;

void setup(){
  pinMode(CANS, OUTPUT);
  pinMode(THERM1, INPUT);
  pinMode(THERM2, INPUT);
  pinMode(RAIL_SEN, INPUT);
  pinMode(CUR_SEN, INPUT);
  pinMode(HALL1, INPUT_PULLUP);
  pinMode(HALL2, INPUT_PULLUP);
  pinMode(HALL3, INPUT_PULLUP);
  attachInterrupt(HALL1, isr_spdcnt, RISING);
  attachInterrupt(HALL2, isr_spdcnt, RISING);
  attachInterrupt(HALL3, isr_spdcnt, RISING);
  
  pinMode(GATES, OUTPUT);
  analogWriteFrequency(GATES, 23437);
  digitalWrite(GATES, LOW);
  
  digitalWrite(CANS, LOW);
  
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  
  Serial.begin(9600);
  Serial.println("Hello World!");
  
  CAN_filter_t mask;
  mask.rtr = 0;
  mask.ext = 0;
  mask.id = 0xFF;  
  CANbus.begin(mask);
  //Set filters - specs says to set all the filters 
  CAN_filter_t filter;
  filter.rtr = 0;
  filter.ext = 0;
  filter.id = GENERATOR_ID;
  for (int i = 0; i<8;i++)
  CANbus.setFilter(filter,i);
  
  //speed calculation
  prevtime = millis();
  spd = LPF(5);
  delay(100);
}

void loop(){
  Serial.println("I am generator ");
    
  //{ CANBus Read
  
  //{ Process Inputs
  //{ Speed
  cli();
    spd.addPoint( (float) spd_cnt/ (millis() - prevtime ));
    
      Serial.print("SPD_CNT: ");
  Serial.println((String)spd_cnt);
  
    spd_cnt = 0;
  sei();
  prevtime = millis();
  
  Serial.print("SPD: ");
  Serial.println((String)spd.getValue());
  //}
  //}
  if (CANbus.read(rxmsg))
  {
    Serial.print("MSG RCV: ");
    switch (rxmsg.buf[0])
    {
      case GENERIC:
      {
        Serial.println(String(rxmsg.buf[1]));
        break; 
      }
    } 
  }
  //}
  //{ CANBus Write
  //{ current processing to amps
  curr = analogRead(CUR_SEN);
  
  msg.len = 8;
  msg.id = msg.id = GENERATOR_ID << 4 + CENTRAL_ID;
  for( int idx=0; idx< msg.len; ++idx ) {
      msg.buf[idx] = 0;
  }
  msg.buf[0] = REPORT_PEDAL;
  msg.buf[1] = curr;
  int err = 0;
  err = CANbus.write(msg);
  Serial.print("ERR: ");
  Serial.println(err);
  //}
  //Serial.println(String(err)); 
  //}
  Serial.print("CURR: ");
  Serial.println((String)curr);
  rail = analogRead(RAIL_SEN);
  
  Serial.print("RAIL: ");
  Serial.println(rail, DEC);
  
  analogWrite(GATES, 200);
  
  delay(100);
}
