
/*function prototypes */

void pwm_init(void);             /*initialize pwm*/
void direction_init(void);       /*initialize direction pins*/

void pwm_set_duty(int channel, int duty);
void set_direction(int channel, int dir);

void motor_init(void);          /* call pwm_init and direction_init */
void set_motor_duty(int channel, int duty);

//void setup_timer(void); //Setup timer for Arduino Mega
void get_current_status(void);
void sensor_read(void);
void sensor_init(void);

//Variables for sensor_read and sensor_init
float RC; //LPF time constant fc=1/(2*pi*RC)
int sensor[4], prev_sensor[4], sensor_in[4];
int theta; //difference between sensor[1] and sensor[0]
int theta_prev; //previous difference between sensor[1] and sensor[0]
int fsensor; //average of sensor[1] and sensor[0]
int fsensor_prev; //previous average of sensor[1] and sensor[0]


/****** Motor Function ******/


void motor_init(void)
{
  pwm_init();
  direction_init();
}

void set_motor_speed(int channel, int duty)
{
  pwm_set_duty(channel, duty);
}

void direction_init(void) {
  pinMode(34, OUTPUT); //motor 0 dir
  pinMode(36, OUTPUT); //motor 1 dir
  pinMode(38, OUTPUT); //motor 2 dir
  pinMode(40, OUTPUT); //motor 3 dir

} /*end of direction_init*/


void pwm_init(void) { //pwm set up
  //PWM Setup for Due

  PMC->PMC_PCER1 |= PMC_PCER1_PID36;                   // PWM on

  //PWM Motor 0 Pin 35
  REG_PIOC_ABSR |= PIO_ABSR_P3;                        // Set PWM pin perhipheral type B
  REG_PIOC_PDR |= PIO_PDR_P3;                          // Set PWM pin to an output
  REG_PWM_ENA = PWM_ENA_CHID0;                         // Enable the PWM channel 0 (see datasheet page 973)
  REG_PWM_CLK = PWM_CLK_PREA(0) | PWM_CLK_DIVA(42);    // Set the PWM clock rate to 2MHz (84MHz/42). Adjust DIVA for the resolution you are looking for
  REG_PWM_CMR0 = PWM_CMR_CPOL | PWM_CMR_CALG | PWM_CMR_CPRE_CLKA;     // Duty cycle polarity is positve ,period is left aligned, clock source as CLKA on channel 0
  REG_PWM_CPRD0 = 500;                                 // Channel 0: Set the PWM frequency 2MHz/(2 * CPRD) = F; F = 2kHz
  REG_PWM_CDTY0 = 0;                                   // Channel 2: Set the PWM duty cycle to x%= (CDTY/ CPRD)  * 100 %

  //PWM Motor 1 Pin 37
  REG_PIOC_ABSR |= PIO_ABSR_P5;                        // Set PWM pin perhipheral type B
  REG_PIOC_PDR |= PIO_PDR_P5;                          // Set PWM pin to an output
  REG_PWM_ENA = PWM_ENA_CHID1;                         // Enable the PWM channel 1 (see datasheet page 973)
  REG_PWM_CLK = PWM_CLK_PREA(0) | PWM_CLK_DIVA(42);    // Set the PWM clock rate to 2MHz (84MHz/42). Adjust DIVA for the resolution you are looking for
  REG_PWM_CMR1 = PWM_CMR_CPOL | PWM_CMR_CALG | PWM_CMR_CPRE_CLKA;     // Duty cycle polarity is positve ,period is left aligned, clock source as CLKA on channel 1
  REG_PWM_CPRD1 = 500;                                 // Channel 1: Set the PWM frequency 2MHz/(2 * CPRD) = F; F = 2kHz
  REG_PWM_CDTY1 = 0;                                   // Channel 2: Set the PWM duty cycle to x%= (CDTY/ CPRD)  * 100 %

  //PWM Motor 2 Pin 39
  REG_PIOC_ABSR |= PIO_ABSR_P7;                        // Set PWM pin perhipheral type B
  REG_PIOC_PDR |= PIO_PDR_P7;                          // Set PWM pin to an output
  REG_PWM_ENA = PWM_ENA_CHID2;                         // Enable the PWM channel 2 (see datasheet page 973)
  REG_PWM_CLK = PWM_CLK_PREA(0) | PWM_CLK_DIVA(42);    // Set the PWM clock rate to 2MHz (84MHz/42). Adjust DIVA for the resolution you are looking for
  REG_PWM_CMR2 = PWM_CMR_CPOL | PWM_CMR_CALG | PWM_CMR_CPRE_CLKA;     // Duty cycle polarity is positve ,period is left aligned, clock source as CLKA on channel 2
  REG_PWM_CPRD2 = 500;                                 // Channel 2: Set the PWM frequency 2MHz/(2 * CPRD) = F ; F = 2kHz
  REG_PWM_CDTY2 = 0;                                   // Channel 2: Set the PWM duty cycle to x%= (CDTY/ CPRD)  * 100 %

  //PWM Motor 3 Pin 41
  REG_PIOC_ABSR |= PIO_ABSR_P9;                        // Set PWM pin perhipheral type B
  REG_PIOC_PDR |= PIO_PDR_P9;                          // Set PWM pin to an output
  REG_PWM_ENA = PWM_ENA_CHID3;                         // Enable the PWM channel 3 (see datasheet page 973)
  REG_PWM_CLK = PWM_CLK_PREA(0) | PWM_CLK_DIVA(42);    // Set the PWM clock rate to 2MHz (84MHz/42). Adjust DIVA for the resolution you are looking for
  REG_PWM_CMR3 = PWM_CMR_CPOL | PWM_CMR_CALG | PWM_CMR_CPRE_CLKA;     // Duty cycle polarity is positve ,period is left aligned, clock source as CLKA on channel 3
  REG_PWM_CPRD3 = 500;                                 // Channel 3: Set the PWM frequency 2MHz/(2 * CPRD) = F ; F = 2kHz
  REG_PWM_CDTY3 = 0;                                   // Channel 3: Set the PWM duty cycle to x%= (CDTY/ CPRD)  * 100 %
} /*end of pwm_init*/



void pwm_set_duty(int channel, int duty)
{

  if (duty > 500) {
    duty = 500;
  }
  else if (duty < -500) {
    duty = -500;
  }


  if (duty > 0) {
    set_direction(channel, 1);   // forward
  }
  else if (duty < 0) {
    set_direction(channel, 0);   //reverse
    duty = -duty;
  }

  if (channel == 0) //motor 0
    REG_PWM_CDTY0 = duty; //pin35

  else if (channel == 1) //motor 1
    REG_PWM_CDTY1 = duty; //pin37

  else if (channel == 2) //motor 2
    REG_PWM_CDTY2 = duty; //pin39

  else if (channel == 3) //motor 3
    REG_PWM_CDTY3 = duty; //pin41
}
/*end of pwn_set_duty*/


void set_direction(int channel, int dir)//set motor direction
{
  if (channel == 3)//motor 3
  {
    if (dir == 0) //reverse
    {
      digitalWrite(40, LOW); //motor 3 dir
    }
    else if (dir == 1) //forward
    {
      digitalWrite(40, HIGH); //motor 3 dir
    }

  } /*end if channel ==3 */

  if (channel == 2)//motor 2(right)
  {
    if (dir == 0) //reverse
    {
      digitalWrite(38, LOW); //motor 2 dir
    }
    else if (dir == 1) //forward
    {
      digitalWrite(38, HIGH); //motor 2 dir
    }

  } /*end if channel ==2 */

  if (channel == 1)//motor 1(right)
  {
    if (dir == 0) //reverse
    {
      digitalWrite(36, LOW); //motor 1 dir
    }
    else if (dir == 1) //forward
    {
      digitalWrite(36, HIGH); //motor 1 dir
    }

  } /*end if channel ==1 */

  else if (channel == 0) //motor 1(left)
  {
    if (dir == 0) //reverse
    {
      digitalWrite(34, LOW); //motor 0 dir
    }
    else if (dir == 1) //forward
    {
      digitalWrite(34, HIGH); //motor 0 dir
    }
  }  /*end else if */
} /*end set_direction*/


/****************************  Encoder Functions  *********************************/




void encoder_init(void);
void encoder0CHB(void);
void encoder0CHA(void);
void encoder1CHB(void);
void encoder1CHA(void);
void encoder2CHB(void);
void encoder2CHA(void);
void encoder3CHB(void);
void encoder3CHA(void);

const byte interruptPIN7 = 21;
const byte interruptPIN6 = 20;
const byte interruptPIN5 = 19;
const byte interruptPIN4 = 18;
const byte interruptPIN3 = 9;
const byte interruptPIN2 = 8;
const byte interruptPIN1 = 7;
const byte interruptPIN0 = 6;

byte state0 = 0;
byte state1 = 0;
byte state2 = 0;
byte state3 = 0;

long int encoder0_val = 0;
long int encoder1_val = 0;
long int encoder2_val = 0;
long int encoder3_val = 0;

void encoder_init(void) {

  pinMode(interruptPIN6, INPUT_PULLUP);
  pinMode(interruptPIN7, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPIN6), encoder3CHA, CHANGE);//set GPIO pin for encoder
  attachInterrupt(digitalPinToInterrupt(interruptPIN7), encoder3CHB, CHANGE);//set GPIO pin for encoder


  pinMode(interruptPIN4, INPUT_PULLUP);
  pinMode(interruptPIN5, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPIN4), encoder2CHA, CHANGE);//set GPIO pin for encoder
  attachInterrupt(digitalPinToInterrupt(interruptPIN5), encoder2CHB, CHANGE);//set GPIO pin for encoder


  pinMode(interruptPIN2, INPUT_PULLUP);
  pinMode(interruptPIN3, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPIN2), encoder1CHA, CHANGE);//set GPIO pin for encoder
  attachInterrupt(digitalPinToInterrupt(interruptPIN3), encoder1CHB, CHANGE);//set GPIO pin for encoder


  pinMode(interruptPIN0, INPUT_PULLUP);
  pinMode(interruptPIN1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPIN0), encoder0CHA, CHANGE);//set  GPIO pin for encoder
  attachInterrupt(digitalPinToInterrupt(interruptPIN1), encoder0CHB, CHANGE);//set  GPIO pin for encoder
}


/******* Encoder 0 for Motor 0 ************************/

void encoder0CHA(void) //function to read and determine state of encoder 0-A
{
  if ((state0 == 0) && (digitalRead(interruptPIN0) == 1))
  {
    state0 = 1;
    encoder0_val += 1;
  }
  else if ((state0 == 1) && (digitalRead(interruptPIN0) == 0))
  {
    state0 = 0;
    encoder0_val -= 1;
  }
  else if ((state0 == 2) && (digitalRead(interruptPIN0) == 1))
  {
    state0 = 3;
    encoder0_val -= 1;
  }
  else if ((state0 == 3) && (digitalRead(interruptPIN0) == 0))
  {
    state0 = 2;
    encoder0_val += 1;
  }
}


void encoder0CHB(void) //function to read and determien state of encoder 0-B
{
  if ((state0 == 0) && (digitalRead(interruptPIN1) == 1))
  {
    state0 = 2;
    encoder0_val -= 1;
  }
  else if ((state0 == 1) && (digitalRead(interruptPIN1) == 1))
  {
    state0 = 3;
    encoder0_val += 1;
  }
  else if ((state0 == 2) && (digitalRead(interruptPIN1) == 0))
  {
    state0 = 0;
    encoder0_val += 1;
  }
  else if ((state0 == 3) && (digitalRead(interruptPIN1) == 0))
  {
    state0 = 1;
    encoder0_val -= 1;
  }
}


/***************** Encoder 1 for Motor 1 ********************/

void encoder1CHA(void)
{
  if ((state1 == 0) && (digitalRead(interruptPIN2) == 1))
  {
    state1 = 1;
    encoder1_val += 1;
  }
  else if ((state1 == 1) && (digitalRead(interruptPIN2) == 0))
  {
    state1 =    0;
    encoder1_val -= 1;
  }
  else if ((state1 == 2) && (digitalRead(interruptPIN2) == 1))
  {
    state1 = 3;
    encoder1_val -= 1;
  }
  else if ((state1 == 3) && (digitalRead(interruptPIN2) == 0))
  {
    state1 = 2;
    encoder1_val += 1;
  }
}

void encoder1CHB(void)
{
  if ((state1 == 0) && (digitalRead(interruptPIN3) == 1))
  {
    state1 = 2;
    encoder1_val -= 1;
  }
  else if ((state1 == 1) && (digitalRead(interruptPIN3) == 1))
  {
    state1 = 3;
    encoder1_val += 1;
  }
  else if ((state1 == 2) && (digitalRead(interruptPIN3) == 0))
  {
    state1 = 0;
    encoder1_val += 1;
  }
  else if ((state1 == 3) && (digitalRead(interruptPIN3) == 0))
  {
    state1 = 1;
    encoder1_val -= 1;
  }
}


/******* Encoder 2 for Motor 2 ************************/

void encoder2CHA(void) //function to read and determine state of encoder 2-A
{
  if ((state2 == 0) && (digitalRead(interruptPIN4) == 1))
  {
    state2 = 1;
    encoder2_val += 1;
  }
  else if ((state2 == 1) && (digitalRead(interruptPIN4) == 0))
  {
    state2 = 0;
    encoder2_val -= 1;
  }
  else if ((state2 == 2) && (digitalRead(interruptPIN4) == 1))
  {
    state2 = 3;
    encoder2_val -= 1;
  }
  else if ((state2 == 3) && (digitalRead(interruptPIN4) == 0))
  {
    state2 = 2;
    encoder2_val += 1;
  }
}


void encoder2CHB(void) //function to read and determine state of encoder 2-B
{
  if ((state2 == 0) && (digitalRead(interruptPIN5) == 1))
  {
    state2 = 2;
    encoder2_val -= 1;
  }
  else if ((state2 == 1) && (digitalRead(interruptPIN5) == 1))
  {
    state2 = 3;
    encoder2_val += 1;
  }
  else if ((state2 == 2) && (digitalRead(interruptPIN5) == 0))
  {
    state2 = 0;
    encoder2_val += 1;
  }
  else if ((state2 == 3) && (digitalRead(interruptPIN5) == 0))
  {
    state2 = 1;
    encoder2_val -= 1;
  }
}

/******* Encoder 3 for Motor 3 ************************/

void encoder3CHA(void) //function to read and determine state of encoder 3-A
{
  if ((state3 == 0) && (digitalRead(interruptPIN6) == 1))
  {
    state3 = 1;
    encoder3_val += 1;
  }
  else if ((state3 == 1) && (digitalRead(interruptPIN6) == 0))
  {
    state3 = 0;
    encoder3_val -= 1;
  }
  else if ((state3 == 2) && (digitalRead(interruptPIN6) == 1))
  {
    state3 = 3;
    encoder3_val -= 1;
  }
  else if ((state3 == 3) && (digitalRead(interruptPIN6) == 0))
  {
    state3 = 2;
    encoder3_val += 1;
  }
}


void encoder3CHB(void) //function to read and determien state of encoder 3-B
{
  if ((state3 == 0) && (digitalRead(interruptPIN7) == 1))
  {
    state3 = 2;
    encoder3_val -= 1;
  }
  else if ((state3 == 1) && (digitalRead(interruptPIN7) == 1))
  {
    state3 = 3;
    encoder3_val += 1;
  }
  else if ((state3 == 2) && (digitalRead(interruptPIN7) == 0))
  {
    state3 = 0;
    encoder3_val += 1;
  }
  else if ((state3 == 3) && (digitalRead(interruptPIN7) == 0))
  {
    state3 = 1;
    encoder3_val -= 1;
  }
}

// Following code is from mobile robot code for Arduino Mega, not compatiable with Due
/***********  Timer Function for creating deterministic clock *******************/

/*
  void setup_timer(void)
  {
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = 625;   //compare match register 16MHz/256/2Hz
  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B |= (1 << CS12);    // 256 prescaler
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
  interrupts();
  }
*/


/*** This function determines the wheel velocities of the mobile robot **/


float cur_wpos[4]  = {0., 0., 0., 0.}; // current wheel position 0 - fwd lt  1 - aft 1t 2 - aft rt 3 - fwd rt
float cur_wvel[4]  = {0., 0., 0., 0.}; // current wheel velocity
float des_wpos[4]  = {0., 0., 0., 0.}; // desired wheel position
float des_wvel[4]  = {0., 0., 0., 0.}; // desired wheel velocity
float vel_offset[4] = {0., 0., 0., 0.};
float wvel;
float prev_wpos[4] = {0., 0., 0., 0.}; // previous wheel position
float prev_des_wpos[4] = {0., 0., 0., 0.}; // previous desired wheel position




#define END_CPR     16
#define Gear_Ratio  270
#define T           0.005  // 5 msec

void get_current_status(void)
{
  int i;

  /*determine current wheel position in rad*/


  cur_wpos[0] = encoder0_val / (4.0 * END_CPR * Gear_Ratio);
  cur_wpos[1] = encoder1_val / (4.0 * END_CPR * Gear_Ratio);
  cur_wpos[2] = encoder2_val / (4.0 * END_CPR * Gear_Ratio);
  cur_wpos[3] = encoder3_val / (4.0 * END_CPR * Gear_Ratio);


  for (i = 0; i < 4; i++)
  {
    cur_wvel[i]  = (cur_wpos[i] - prev_wpos[i]) / T;
    prev_wpos[i] = cur_wpos[i];
  }

  /*determine des_wheel position*/
  for (i = 0; i < 4; i++)
  {
    des_wpos[i] = prev_des_wpos[i] + des_wvel[i] * T;
    prev_des_wpos[i] = des_wpos[i];
  }

}

void sensor_init(void) 
{
  int i;
  for(i = 0; i < 4; i++)
  {
    sensor_in[i] = analogRead(i);
    sensor[i] = sensor_in[i];
    prev_sensor[i] = 0;
    fsensor_prev = 0;
  }
  theta_prev = 0;
  
}

void sensor_read (void)
{
  int i;
  theta_prev = theta;
  fsensor_prev = fsensor;
  for(i = 0; i < 4; i++)
  {
    sensor_in[i] = analogRead(i);
    sensor[i] = (sensor_in[i]*T + prev_sensor[i]*RC)/(RC + T); //IR sensor LPF
    prev_sensor[i] = sensor[i];
  }
  theta = sensor[1] - sensor[0];
  fsensor = (sensor[1] + sensor[0])/2;
}


void low_level_control(void)
{
  float voltage[4] = {0., 0., 0., 0.};
  float Kp = 1000.0;
  float Kd = 25.0;
  int   duty = 0;
  float Vcc  = 12; /* Battery Voltage*/


  /*PD Control*/


  voltage[0] = Kp * (des_wpos[0] - cur_wpos[0]) + Kd * (des_wvel[0] - cur_wvel[0]);
  duty      = voltage[0] / Vcc * 500;
  
  set_motor_speed(0, duty);



  voltage[1] = Kp * (des_wpos[1] - cur_wpos[1]) + Kd * (des_wvel[1] - cur_wvel[1]);
  duty       = voltage[1] / Vcc * 500;

  set_motor_speed(1, duty);

  voltage[2] = Kp * (des_wpos[2] - cur_wpos[2]) + Kd * (des_wvel[2] - cur_wvel[2]);
  duty      = voltage[2] / Vcc * 500;

  set_motor_speed(2, duty);

  voltage[3] = Kp * (des_wpos[3] - cur_wpos[3]) + Kd * (des_wvel[3] - cur_wvel[3]);
  duty      = voltage[3] / Vcc * 500;

  set_motor_speed(3, duty);

}




















