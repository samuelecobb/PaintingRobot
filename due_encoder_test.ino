#include "mrobot.h"
#include "my_func.h"
#include "DueTimer.h"
#include "navigation.h"

int LEDGpin = 11; //Green LED
int LEDYpin = 12; //Yellow LED
int LEDRpin = 13; //Red LED

int init_delay = 0;

//long duration; //Ultrasonic sensor duration
//float distance; //Ultrasonic sensor distance

void control(void);
void standby(void);

void setup() {
  sensor_init();
  encoder_init();
  motor_init();
  init_delay = 0;
  pinMode(10, INPUT_PULLUP);
  pinMode(LEDGpin, OUTPUT);
  pinMode(LEDYpin, OUTPUT);
  pinMode(LEDRpin, OUTPUT);
  Serial.begin(9600);
  Timer3.attachInterrupt(control).start(5000); //Timer interrupt for low level control
  attachInterrupt(digitalPinToInterrupt(10), standby, HIGH);
  digitalWrite(LEDYpin, LOW);
  digitalWrite(LEDRpin, LOW);
  digitalWrite(LEDGpin, LOW);
  wvel = 0.3; //initial wheel velocity in rad/s
  RC = 0.1; //initial RC time constant for LPF
}

void loop() {
//    Serial.println(sensor[1]);
  while(init_delay < 10) //initial delay only to occur on first loop of program
  {
    if (digitalRead(LEDYpin)==LOW)
    {
      digitalWrite(LEDYpin, HIGH);
      digitalWrite(LEDGpin, LOW);
    }
    else
    {
    digitalWrite(LEDYpin, LOW);
    digitalWrite(LEDGpin, HIGH);
    }
    delay(200);
    init_delay++;
  }

  digitalWrite(LEDYpin, HIGH); //turns on caution light
  digitalWrite(LEDGpin, LOW); //turns off standby light

  //Serial.println(theta);

//    Serial.print("EN0: ");
//  Serial.print(encoder0_val);
//  Serial.print(" EN1: ");
//  Serial.print(encoder1_val);
//  Serial.print(" EN2: ");
//  Serial.print(encoder2_val);
//  Serial.print(" EN3: ");
//  Serial.println(encoder3_val);
//
//Serial.print("Sen0: ");
//Serial.print(sensor[0]);
//Serial.print(" Sen1: ");
//Serial.print(sensor[1]);
//Serial.print(" Sen2: ");
//Serial.print(sensor[2]);
//Serial.print(" Sen3: ");
//Serial.println(sensor[3]);
  

  find_wall();
  parallel_pid_adjust();
  maintain_distance_pid();
  right_move_parallel();
  turn_corner();
  right_move_parallel_measure();

// right_PID_move_parallel();
Serial.print("Distance: ");
Serial.print(wall_dist);
Serial.println("m");
Serial.print("Encoder Start: ");
Serial.print(encoder_dist[0]);
Serial.print(" Encoder End: ");
Serial.println(encoder_dist[2]);

  while (1) //infinite WHILE loop to stop the motors and blink the yellow LED to indicate program has ended
  {
    set_motor_speed(0, 0);
    set_motor_speed(1, 0);
    set_motor_speed(2, 0);
    set_motor_speed(3, 0);
    digitalWrite(LEDYpin, HIGH);
    delay(100);
    digitalWrite(LEDYpin, LOW);
    delay(100);
  }



  // Ultrasound Sensor Code
  /*
    // Clears the trigPin
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(echoPin, HIGH);
    // Calculating the distance
    distance= duration/148.0;
    // Prints the distance on the Serial Monitor
    Serial.print("Distance: ");
    Serial.println(distance);*/

}

void control(void) //ISR that executes every 10ms
{

  get_current_status(); //get status of the wheels/motors
  sensor_read(); //get sensor readings
  /******************************/
  global_ticks++;
  move_mr(move_command);
  /********************************/
  low_level_control();

}

void standby(void) //ISR will stop all the motors when the toggle switch is in STANDBY
{
  set_motor_speed(0, 0);
  set_motor_speed(1, 0);
  set_motor_speed(2, 0);
  set_motor_speed(3, 0);
  digitalWrite(LEDYpin, LOW);
  while (digitalRead(10) == HIGH)
  {
    digitalWrite(LEDGpin, HIGH);
  }
  digitalWrite(LEDGpin, LOW);
  digitalWrite(LEDYpin, HIGH);
}


