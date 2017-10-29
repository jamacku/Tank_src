#include <Adafruit_MCP23017.h>

#include <Wire.h>
#include <Adafruit_RGBLCDShield.h>
#include <TimerThree.h>                                     // Header file for TimerOne library

Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

/* Sensors */
#define trigPin_L 6                                       // Pin 6 trigger output
#define echoPin_L 7                                       // Pin 7 Echo input
#define echo_int_L 4                                      // Interrupt id for echo pulse Int 4

#define trigPin_F 5                                       // Pin 5 trigger output
#define echoPin_F 1                                       // Pin 1 Echo input
#define echo_int_F 3                                      // Interrupt id for echo pulse Int 3

#define trigPin_R 4                                       // Pin 4 trigger output
#define echoPin_R 0                                       // Pin 0 Echo input
#define echo_int_R 2                                      // Interrupt id for echo pulse Int 2

/* Timer */
#define TIMER_US 50                                       // 50 uS timer duration 
#define TICK_COUNTS 4000                                  // 200 mS worth of timer ticks
#define TICK_SENSOR_L 3000                                // 150 mS worth of timer ticks
#define TICK_SENSOR_F 2000                                // 100 mS worth of timer ticks
#define TICK_SENSOR_R 1000                                // 50 mS worth of timer ticks

/* Timer Stale */
#define STATE_BEG 0                                      // Begin
#define STATE_TRIG_L_UP 1                                // Trigr L HIGH
#define STATE_TRIG_L_DOWN 2                              // Trigr L DOWN
#define STATE_TRIG_F_UP 3                                // Trigr F HIGH
#define STATE_TRIG_F_DOWN 4                              // Trigr F DOWN
#define STATE_TRIG_R_UP 5                                // Trigr R HIGH
#define STATE_TRIG_R_DOWN 6                              // Trigr R DOWN

#define STATE_FORW 1
#define STATE_REG 2
#define STATE_TURN_RIGHT 3
#define STATE_TURN_LEFT 4
#define STATE_STOP 5
#define STATE_OTOC 6
#define STATE_TURN_AROUND 7
#define STATE_ROV 8
#define STATE_WAIT 9
#define STATE_WAIT_2 10

#define R_SET 8 
#define L_SET 8

#define MIN_DELKA_F 15
#define MIN_DELKA_S 15

#define K 3

#define VOLNO 25

#define LEFT 0
#define RIGHT 1

#define NORMAL_SPEED_R 100
#define NORMAL_SPEED_L 100

#define DELKA_POLE 3

#define TYPE_CYCLIC 0
#define TYPE_ONDEMAND 1
#define STATE_RUNNING 0
#define STATE_SLEEPING 1

volatile long echo_start_L = 0;                         // Records start of echo pulse L 
volatile long echo_end_L = 0;                           // Records end of echo pulse L
volatile long echo_duration_L = 0;                      // Duration - difference between end and start L

volatile long echo_start_F = 0;                         // Records start of echo pulse F 
volatile long echo_end_F = 0;                           // Records end of echo pulse F
volatile long echo_duration_F = 0;                      // Duration - difference between end and start F

volatile long echo_start_R = 0;                         // Records start of echo pulse F 
volatile long echo_end_R = 0;                           // Records end of echo pulse F
volatile long echo_duration_R = 0;                      // Duration - difference between end and start F

volatile int trigger_time_count = 0;                    // Count down counter to trigger pulse time

boolean static poprve = true;

int speed_motor_r = NORMAL_SPEED_R;
int speed_motor_l = NORMAL_SPEED_L;

int pwm_a = 10;   //PWM control for motor outputs 1 and 2 is on digital pin 3
int pwm_b = 11;  //PWM control for motor outputs 3 and 4 is on digital pin 11
int dir_a = 12;  //direction control for motor outputs 1 and 2 is on digital pin 12
int dir_b = 13;  //direction control for motor outputs 3 and 4 is on digital pin 13

int pole_r[DELKA_POLE];
int pole_l[DELKA_POLE];

int iErr1;
int iErr2;

int iSide;
int iOtoc;

long lL = 0;
long lF = 0;
long lR = 0;

char sL[3];
char sF[3];
char sR[3];

int state_dis = STATE_BEG;

struct Task {
  int iTaskID;                                // task identification
  unsigned long lPeriod;                      // task's period in ms
  unsigned long lLastRun;
  unsigned long lNextRun;                     // last run time stamp in ms
  unsigned long lDuration;                    // time consumed by this task
  byte bType;
  byte bState;
  void (*fnCallBack)(int);   
};

enum {
  TASK_MER,
  TASK_DISPECER,
  TASK_LCD,
  TASK_REG,
  TASK_STOP,
  //TASK_STOP_2,
  TASK_OTOC,
  TASK_ROV,
  TASKS_NUM
};


struct Task stTasks[TASKS_NUM];

// ------------------------------------------------------------------------------------------------------
// Inicializace
// Nastavení přerušení Int2 Int3 a Int4  
// Nastavení displaye
// ------------------------------------------------------------------------------------------------------
void setup() 
{
  pinMode(trigPin_L, OUTPUT);                           // Trigger pin L set to output
  pinMode(echoPin_L, INPUT);                            // Echo pin set L to input

  pinMode(trigPin_F, OUTPUT);                           // Trigger pin F set to output
  pinMode(echoPin_F, INPUT);                            // Echo pin set F to input  

  pinMode(trigPin_R, OUTPUT);                           // Trigger pin set R to output
  pinMode(echoPin_R, INPUT);                            // Echo pin set to R input

  Timer3.initialize(TIMER_US);                        // Initialise timer 3
  Timer3.attachInterrupt( timerIsr );                 // Attach interrupt to the timer service routine 

  attachInterrupt(echo_int_L, echo_interrupt_L, CHANGE);  // Attach interrupt to the sensor echo input

  attachInterrupt(echo_int_F, echo_interrupt_F, CHANGE);  // Attach interrupt to the sensor echo input

  attachInterrupt(echo_int_R, echo_interrupt_R, CHANGE);  // Attach interrupt to the sensor echo input

  lcd.begin(16, 2);
  //lcd.print("Sensors value:");

  pinMode(pwm_a, OUTPUT);  //Set control pins to be outputs
  pinMode(pwm_b, OUTPUT);
  pinMode(dir_a, OUTPUT);
  pinMode(dir_b, OUTPUT);

  digitalWrite(dir_a, HIGH);  
  digitalWrite(dir_b, HIGH);

  stTasks[TASK_MER].iTaskID = TASK_MER;
  stTasks[TASK_MER].lPeriod = 400;
  stTasks[TASK_MER].lNextRun = 50;
  stTasks[TASK_MER].lDuration = 0;
  stTasks[TASK_MER].bType = TYPE_CYCLIC;
  stTasks[TASK_MER].bState = STATE_RUNNING;
  stTasks[TASK_MER].fnCallBack = &fnTask_MER;

  stTasks[TASK_DISPECER].iTaskID = TASK_DISPECER;
  stTasks[TASK_DISPECER].lPeriod = 100;
  stTasks[TASK_DISPECER].lNextRun = 5300;
  stTasks[TASK_DISPECER].lDuration = 0;
  stTasks[TASK_DISPECER].bType = TYPE_CYCLIC;
  stTasks[TASK_DISPECER].bState = STATE_RUNNING;
  stTasks[TASK_DISPECER].fnCallBack = &fnTask_DISPECER;

  stTasks[TASK_LCD].iTaskID = TASK_LCD;
  stTasks[TASK_LCD].lPeriod = 1000;
  stTasks[TASK_LCD].lNextRun = 100;
  stTasks[TASK_LCD].lDuration = 0;
  stTasks[TASK_LCD].bType = TYPE_CYCLIC;
  stTasks[TASK_LCD].bState = STATE_RUNNING;
  stTasks[TASK_LCD].fnCallBack = &fnTask_LCD;

  stTasks[TASK_REG].iTaskID = TASK_REG;
  stTasks[TASK_REG].lPeriod = 400;
  stTasks[TASK_REG].lNextRun = 5300;
  stTasks[TASK_REG].lDuration = 0;
  stTasks[TASK_REG].bType = TYPE_CYCLIC;
  stTasks[TASK_REG].bState = STATE_SLEEPING;
  stTasks[TASK_REG].fnCallBack = &fnTask_REG;

  stTasks[TASK_STOP].iTaskID = TASK_STOP;
  stTasks[TASK_STOP].lPeriod = 1000;
  stTasks[TASK_STOP].lNextRun = 0;
  stTasks[TASK_STOP].lDuration = 0;
  stTasks[TASK_STOP].bType = TYPE_ONDEMAND;
  stTasks[TASK_STOP].bState = STATE_SLEEPING;
  stTasks[TASK_STOP].fnCallBack = &fnTask_STOP;
  /*
  stTasks[TASK_STOP_2].iTaskID = TASK_STOP_2;
  stTasks[TASK_STOP_2].lPeriod = 1000;
  stTasks[TASK_STOP_2].lNextRun = 0;
  stTasks[TASK_STOP_2].lDuration = 0;
  stTasks[TASK_STOP_2].bType = TYPE_ONDEMAND;
  stTasks[TASK_STOP_2].bState = STATE_SLEEPING;
  stTasks[TASK_STOP_2].fnCallBack = &fnTask_STOP_2;
  */
  stTasks[TASK_OTOC].iTaskID = TASK_OTOC;
  stTasks[TASK_OTOC].lPeriod = 1000;
  stTasks[TASK_OTOC].lNextRun = 0;
  stTasks[TASK_OTOC].lDuration = 0;
  stTasks[TASK_OTOC].bType = TYPE_ONDEMAND;
  stTasks[TASK_OTOC].bState = STATE_SLEEPING;
  stTasks[TASK_OTOC].fnCallBack = &fnTask_OTOC;

  stTasks[TASK_ROV].iTaskID = TASK_ROV;
  stTasks[TASK_ROV].lPeriod = 1000;
  stTasks[TASK_ROV].lNextRun = 0;
  stTasks[TASK_ROV].lDuration = 0;
  stTasks[TASK_ROV].bType = TYPE_ONDEMAND;
  stTasks[TASK_ROV].bState = STATE_SLEEPING;
  stTasks[TASK_ROV].fnCallBack = &fnTask_ROV;
}

// ----------------------------------
// loop() Runs continuously in a loop.
// This is the background routine where most of the processing usualy takes place.
// Non time critical tasks should be run from here.
// ----------------------------------
void loop()
{
  //GetSensorValues();
  //zobraz();
  //delay(1000);                                       
  fnScheduler();
}

// --------------------------
// timerIsr() 50uS second interrupt ISR()
// Called every time the hardware timer 1 times out.
// --------------------------
void timerIsr()
{
  trigger_pulse();                                 // Schedule the trigger pulses
}

// --------------------------
// trigger_pulse() called every 50 uS to schedule trigger pulses.
// Generates a pulse one timer tick long.
// Minimum trigger pulse width for the HC-SR04 is 10 us. This system
// delivers a 50 uS pulse.
// --------------------------
void trigger_pulse()
{
  static volatile int state = STATE_BEG;                 // State machine variable

  if (!(--trigger_time_count)){                   // Count to 200mS
    trigger_time_count = TICK_COUNTS;           //  Time out - Initiate trigger pulse and Reload
  }

  if(trigger_time_count==TICK_SENSOR_L){
    state = STATE_TRIG_L_UP;
  }

  if(trigger_time_count==TICK_SENSOR_F){
    state = STATE_TRIG_F_UP;
  }

  if(trigger_time_count==TICK_SENSOR_R){
    state = STATE_TRIG_R_UP;
  }

  switch(state)                                  // State machine handles delivery of trigger pulse
  {
  case STATE_BEG:                                      // Normal state does nothing
    break;

  case STATE_TRIG_L_UP:                                      // Initiate pulse
    digitalWrite(trigPin_L, HIGH);              // Set the trigger output high
    state = STATE_TRIG_L_DOWN;                                // and set state to 2
    break;

  case STATE_TRIG_L_DOWN:
    digitalWrite(trigPin_L, LOW);              // Set the trigger output high
    state = STATE_BEG;                                // and set state to 2
    break;

  case STATE_TRIG_F_UP:                                      // Initiate pulse
    digitalWrite(trigPin_F, HIGH);              // Set the trigger output high
    state = STATE_TRIG_F_DOWN;                                // and set state to 2
    break;

  case STATE_TRIG_F_DOWN:
    digitalWrite(trigPin_F, LOW);              // Set the trigger output high
    state = STATE_BEG;                                // and set state to 2
    break;

  case STATE_TRIG_R_UP:                                      // Initiate pulse
    digitalWrite(trigPin_R, HIGH);              // Set the trigger output high
    state = STATE_TRIG_R_DOWN;                                // and set state to 2
    break;

  case STATE_TRIG_R_DOWN:
    digitalWrite(trigPin_R, LOW);              // Set the trigger output high
    state = STATE_BEG;                                // and set state to 2
    break;

  default:      
    digitalWrite(trigPin_L, LOW);               // Set the trigger output low
    digitalWrite(trigPin_F, LOW);               // Set the trigger output low           
    digitalWrite(trigPin_R, LOW);               // Set the trigger output low           
    state = STATE_BEG;                                // and return state to normal 0
    break;
  }
}

// --------------------------
// echo_interrupt() External interrupt from HC-SR04 echo signal. 
// Called every time the echo signal changes state.
//
// Note: this routine does not handle the case where the timer
//       counter overflows which will result in the occassional error.
// --------------------------
void echo_interrupt_L()
{
  switch (digitalRead(echoPin_L))                     // Test to see if the signal is high or low
  {
  case HIGH:                                      // High so must be the start of the echo pulse
    echo_end_L = 0;                                 // Clear the end time
    echo_start_L = micros();                        // Save the start time
    break;

  case LOW:                                       // Low so must be the end of hte echo pulse
    echo_end_L = micros();                          // Save the end time
    echo_duration_L = echo_end_L - echo_start_L;        // Calculate the pulse duration
    break;
  }
}

void echo_interrupt_F()
{
  switch (digitalRead(echoPin_F))                     // Test to see if the signal is high or low
  {
  case HIGH:                                      // High so must be the start of the echo pulse
    echo_end_F = 0;                                 // Clear the end time
    echo_start_F = micros();                        // Save the start time
    break;

  case LOW:                                       // Low so must be the end of hte echo pulse
    echo_end_F = micros();                          // Save the end time
    echo_duration_F = echo_end_F - echo_start_F;        // Calculate the pulse duration
    break;
  }
}

void echo_interrupt_R()
{
  switch (digitalRead(echoPin_R))                     // Test to see if the signal is high or low
  {
  case HIGH:                                      // High so must be the start of the echo pulse
    echo_end_R = 0;                                 // Clear the end time
    echo_start_R = micros();                        // Save the start time
    break;

  case LOW:                                       // Low so must be the end of hte echo pulse
    echo_end_R = micros();                          // Save the end time
    echo_duration_R = echo_end_R - echo_start_R;        // Calculate the pulse duration
    break;
  }
}

void GetSensorValues()
{
  noInterrupts();
  lL = echo_duration_L;
  interrupts();
  lL = lL/58;

  noInterrupts();
  lF = echo_duration_F;
  interrupts();
  lF = lF/58;

  noInterrupts();
  lR = echo_duration_R;
  interrupts();
  lR = lR/58;
}

void getCharNum(long lNum, char *cNum)
{
  if(lNum < 10){
    cNum[0] = ' ';
    cNum[1] = lNum + int('0'); 
  }
  else{
    cNum[0] = lNum / 10 + int('0');
    cNum[1] = lNum % 10 + int('0');  
  }
  if(lNum > 99){
    cNum[0] = int('*');
    cNum[1] = int('*');
  }
  cNum[3] = '\0';
}

void zobraz()
{
  lcd.setCursor(0, 0);
  lcd.print("S:");
  lcd.print(state_dis);
  lcd.setCursor(0, 1);
  lcd.print("L:");
  getCharNum(lL, sL);
  lcd.print(sL);
  lcd.setCursor(6,1);
  lcd.print("F:");
  getCharNum(lF, sF);
  lcd.print(sF);
  lcd.setCursor(12, 1);
  lcd.print("R:");
  getCharNum(lR, sR);
  lcd.print(sR);
}

void fnRegulace(int iSide)
{
  boolean static poprve = true;

  if(poprve){
    for(int i=0; i<DELKA_POLE; i++){
      pole_r[i]=lR;
      pole_l[i]=lL;
    }
    poprve=false;
  }
  else{
    for(int i=0; i<DELKA_POLE-1; i++){
      pole_r[i]=pole_r[i+1];
      pole_l[i]=pole_l[i+1];
    }
    pole_r[DELKA_POLE-1]=lR;
    pole_l[DELKA_POLE-1]=lL;
  }

  if(iSide == LEFT){
    iErr1 = L_SET - lL;  
    iErr2 = fnUhel(pole_l);

    if(iErr1 < 0) {                                 // je moc daleko 
      speed_motor_l = NORMAL_SPEED_L + iErr1 * K;   // zpomal levy -> zatoc doleva
      speed_motor_r = NORMAL_SPEED_R;
    }
    else if (iErr1 > 0) {                           // je moc blizko
      speed_motor_l = NORMAL_SPEED_L;               // 
      speed_motor_r = NORMAL_SPEED_R - iErr1 * K;   // zpomal pravy -> zatoc doprava
    } 
    else {
      speed_motor_r = NORMAL_SPEED_R;               // je akorat - oba motory na normal
      speed_motor_l = NORMAL_SPEED_L;
    } 

    if(iErr2<0){                                    // priblizuje se
      speed_motor_r += iErr2;                       // pravy motor zpomalit - zatocit doprava
    }
    else{                                           // vzdaluje se 
      speed_motor_l -= iErr2;                       // levy zpomalit -> zatocit doleva
    }
  }

  else if(iSide == RIGHT){
    iErr1 = R_SET - lR;
    iErr2 = fnUhel(pole_r);

    if(iErr1 < 0) {                                 // je moc daleko 
      speed_motor_l = NORMAL_SPEED_L;               
      speed_motor_r = NORMAL_SPEED_R + iErr1 * K;   // zpomal pravy -> zatoc doprava
    }
    else if (iErr1 > 0) {                           // je moc blizko
      speed_motor_l = NORMAL_SPEED_L - iErr1 * K;   // zpomal levy -> zatoc doleva
      speed_motor_r = NORMAL_SPEED_R;               // 
    } 
    else {
      speed_motor_r = NORMAL_SPEED_R;               // je akorat - oba motory na normal
      speed_motor_l = NORMAL_SPEED_L;
    } 

    iErr2 = fnUhel(pole_r);                         // odchylka uhlu

    if(iErr2<0){                                    // priblizuje se
      speed_motor_l += iErr2;                       // levy motor zpomalit - zatocit doleva
    }
    else{                                           // vzdaluje se 
      speed_motor_r -= iErr2;                       // pravy zpomalit -> zatocit doprava
    }
  }

  analogWrite(pwm_a, speed_motor_r);
  analogWrite(pwm_b, speed_motor_l);
}

int fnUhel(int pole[3]) 
{
  float fVys = 0;
  for(int i=0; i<DELKA_POLE; i++)
    fVys += pole[i];
  fVys = (fVys/DELKA_POLE) - pole[0];
  return(round(fVys));
}

void fnStop()
{  
  analogWrite(pwm_a, 0);
  analogWrite(pwm_b, 0);
}

void fnOtoc(int iOtoc)
{
  if(iOtoc == RIGHT){
    analogWrite(pwm_a, 0);
    analogWrite(pwm_b, 100);
  }
  else{
    analogWrite(pwm_a, 100);
    analogWrite(pwm_b, 0);
  }
  //state_dis = STATE_STOP;
}

/*==================================================================================*/
/*                                                                                  */
/*                                                                                  */
/*                                                                                  */
/*==================================================================================*/

void fnScheduler()
{
  unsigned long lNow = millis();
  unsigned long lTmp;

  for(int i=0; i<TASKS_NUM; i++) {
    if(stTasks[i].bState == STATE_RUNNING){
      if(stTasks[i].lNextRun <= lNow) {
        stTasks[i].lLastRun = lNow; 
        lTmp = micros();
        stTasks[i].fnCallBack(stTasks[i].iTaskID);
        stTasks[i].lDuration = micros()-lTmp;
        if(stTasks[i].bType == TYPE_CYCLIC){  
          stTasks[i].lNextRun += stTasks[i].lPeriod;
        }
        else{
          stTasks[i].bState = STATE_SLEEPING;
        }
      }
    }  
  }
}

void fnTask_MER(int iID)
{
  GetSensorValues();
}

void fnTask_DISPECER(int iID)
{
  // State machine variable

  switch(state_dis)                                  // State machine handles delivery of trigger pulse
  {
  case STATE_BEG:                                      // Normal state_dis does nothing
    if(lF>MIN_DELKA_F) {
      state_dis = STATE_FORW;
    }
    else{
      if(lR>MIN_DELKA_S)
        state_dis = STATE_TURN_RIGHT;      
      if(lL>MIN_DELKA_S)
        state_dis = STATE_TURN_LEFT;
      if(lR<=MIN_DELKA_S && lL<=MIN_DELKA_S)
        state_dis = STATE_STOP;
    }    
    //stTasks[TASK_STOP].bState = STATE_SLEEPING;
    //stTasks[TASK_OTOC].bState = STATE_SLEEPING;
    break;

  case STATE_FORW:
    if(lL<lR)
      iSide = LEFT;
    else
      iSide = RIGHT;
    stTasks[TASK_REG].bState = STATE_RUNNING;
    state_dis = STATE_REG;
    break;

  case STATE_REG:                                      // Initiate pulse
    if(lF<MIN_DELKA_F){
      stTasks[TASK_REG].bState = STATE_SLEEPING;
      state_dis = STATE_STOP;
    }                              // and set state_dis to 2
    break;

  case STATE_TURN_RIGHT:
    iOtoc = RIGHT;
    state_dis = STATE_OTOC;
    break;

  case STATE_TURN_LEFT:
    iOtoc = LEFT;
    state_dis = STATE_OTOC;
    break;

  case STATE_STOP:
    stTasks[TASK_STOP].bState = STATE_RUNNING;
    //state_dis = STATE_BEG;                                // and set state_dis to 2
    break;

  case STATE_OTOC:                                      // Initiate pulse
    //stTasks[TASK_OTOC].lNextRun = millis()+1500;
    stTasks[TASK_OTOC].bState = STATE_RUNNING;
    state_dis = STATE_WAIT;                                // and set state_dis to 2
    break;

  case STATE_TURN_AROUND:
    //TODO: turn around
    break;

  case STATE_ROV:                                      // Initiate pulse
    stTasks[TASK_ROV].bState = STATE_RUNNING;
    state_dis = STATE_BEG;                                // and set state_dis to 2
    break;

  case STATE_WAIT:
    stTasks[TASK_STOP].lNextRun = millis()+4500;
    break;

  case STATE_WAIT_2:
    stTasks[TASK_STOP_2].lNextRun = millis()+1500;
    stTasks[TASK_DISPECER].bState = STATE_SLEEPING;
    break;

  default:      
    state_dis = STATE_BEG;                                // and return state_dis to normal 0
    break;
  }
}

void fnTask_LCD(int iID)
{
  zobraz();
}

void fnTask_REG(int iID)
{
  fnRegulace(iSide);
}

void fnTask_STOP(int iID)
{
  fnStop();
  state_dis = STATE_BEG;
  //state_dis = STATE_WAIT_2;
}
/*
void fnTask_STOP_2(int iID)
{
  fnStop();
  stTasks[TASK_DISPECER].bState = STATE_RUNNING;
  state_dis = STATE_BEG;
}
*/
void fnTask_OTOC(int iID)
{
  fnOtoc(iOtoc);
}

void fnTask_ROV(int iID)
{
  digitalWrite(dir_a, HIGH);  
  digitalWrite(dir_b, HIGH);
  analogWrite(pwm_a, NORMAL_SPEED_R);
  analogWrite(pwm_b, NORMAL_SPEED_L);
}

