#include <Arduino.h>
#include <EEPROM.h>
#include "PCA9685.h"

#define SENSOR_DUMPING_STATION 2
#define SENSOR_FOLDING_STATION 3

#define STEP_ENA_PIN 5
#define STEP_DIR_PIN 6
#define STEP_PUL_PIN 7

#define CHANNEL_DUMPING_STATION 0
#define CHANNEL_LEFT_FOLDING_STATION 1
#define CHANNEL_CENTER_FOLDING_STATION 2
#define CHANNEL_RIGHT_FOLDING_STATION 3

#define STATIONS 6

volatile int check_Sensor2 = 0, check_Sensor3 = 0;
volatile int times_Counter = 0;

const int FOLD_STATION[3] = {CHANNEL_LEFT_FOLDING_STATION,
                             CHANNEL_CENTER_FOLDING_STATION,
                             CHANNEL_RIGHT_FOLDING_STATION};

PCA9685 pwmController(Wire, PCA9685_PhaseBalancer_Weaved);
PCA9685_ServoEvaluator pwmServo(102, 307, 512);

void setup()
{

  Init(); //initialize inputs and outputs

  cli(); //stop interrupts

  external_Interrupt_Init(); //initialize external interrupts
  timer1_Init();             //initialize timer1 for interrupt

  sei(); //allow interrupts
}

void Init()
{
  Serial.begin(115000);

  Wire.begin();
  Wire.setClock(400000);

  //set input_pullup for sensor input
  pinMode(SENSOR_DUMPING_STATION, INPUT_PULLUP);
  pinMode(SENSOR_FOLDING_STATION, INPUT_PULLUP);

  //initialize for PCA9685 i2C
  pwmController.resetDevices();
  pwmController.init(B00000000);
  pwmController.setPWMFrequency(50); //50Hz ---> 20ms length phases

  //set pins as outputs
  pinMode(STEP_ENA_PIN, OUTPUT);
  pinMode(STEP_DIR_PIN, OUTPUT);
  pinMode(STEP_PUL_PIN, OUTPUT);
}

void timer1_Init()
{
  //set timer1 interrupt at 1Hz
  TCCR1A = 0; // set entire TCCR1A register to 0
  TCCR1B = 0; // same for TCCR1B
  TCNT1 = 0;  //initialize counter value to 0
  // set compare match register for 1/3 Hz increments
  OCR1A = 46874; // = (16*10^6) / (f*1024) - 1 (must be <65536)  --- 3s
  // turn on CTC mode 4
  TCCR1B |= (1 << WGM12);
  // Set CS12 and CS10 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
}

ISR(TIMER1_COMPA_vect)
{
  const float pulse_theory = (float)96000 / 11;
  const int pulse_stations_1_5 = 1455;
  static float pulse_station_6 = 1455;
  static float delta = 0;
  static int station = 1;
  int pulse_change;

  times_Counter += 1;
  if (times_Counter == 10)
  {
    switch (station)
    {
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
      pulse_supply(pulse_stations_1_5);
      station += 1;
      break;
    case 6:

      delta = delta + ((float)pulse_stations_1_5 * 5 + pulse_station_6 - pulse_theory);
      pulse_change = round(delta);
      delta = delta - pulse_change;
      pulse_station_6 = pulse_station_6 - pulse_change;

      pulse_supply(pulse_station_6);
      station = 1;
      break;
    default:
      break;
    }
    times_Counter = 0;
  }
}

void pulse_supply(int numpusles)
{
  for (int i = 0; i < numpusles; i++)
  {
    digitalWrite(STEP_ENA_PIN, HIGH);
    digitalWrite(STEP_DIR_PIN, HIGH);
    digitalWrite(STEP_PUL_PIN, HIGH);
    delayMicroseconds(500);
    digitalWrite(STEP_PUL_PIN, LOW);
    delayMicroseconds(500);
  }
}

void external_Interrupt_Init()
{
  attachInterrupt(SENSOR_DUMPING_STATION, DUMPING_Interrupt, RISING);
  attachInterrupt(SENSOR_FOLDING_STATION, FOLDING_Interrupt, RISING);
}

void DUMPING_Interrupt()
{
  check_Sensor2 = 2;
}

void FOLDING_Interrupt()
{
  check_Sensor3 = 3;
}

void control_RCservo(int check_Sensor2, int check_Sensor3)
{
  int checkSum = check_Sensor2 + check_Sensor3;

  switch (checkSum)
  {
  case 2:

    delay(2000);
    pwmController.setChannelPWM(CHANNEL_DUMPING_STATION, pwmServo.pwmForAngle(0));
    delay(2000);
    pwmController.setChannelPWM(CHANNEL_DUMPING_STATION, pwmServo.pwmForAngle(30));
    delay(500);
    pwmController.setChannelPWM(CHANNEL_DUMPING_STATION, pwmServo.pwmForAngle(0));
    delay(2000);
    pwmController.setChannelPWM(CHANNEL_DUMPING_STATION, pwmServo.pwmForAngle(130));
    delay(2000);
    pwmController.setChannelPWM(CHANNEL_DUMPING_STATION, pwmServo.pwmForAngle(180));
    delay(2000);

    check_Sensor2 = 0;

    break;
  case 3:
    delay(1000);
    FOLDING_STATION_CONTROL(FOLD_STATION, 90, -80, -90);
    delay(1000);
    FOLDING_STATION_CONTROL(FOLD_STATION, -90, 80, 90);
    delay(1000);
    FOLDING_STATION_CONTROL(FOLD_STATION, 90, -80, -90);
    delay(1000);
    FOLDING_STATION_CONTROL(FOLD_STATION, -90, 80, 90);
    delay(1000);
    FOLDING_STATION_CONTROL(FOLD_STATION, 90, -80, -90);
    delay(1000);

    check_Sensor3 = 0;

    break;

  case 5:
    delay(1000);
    FOLDING_STATION_CONTROL(FOLD_STATION, 90, -80, -90);
    delay(1000);
    pwmController.setChannelPWM(CHANNEL_DUMPING_STATION, pwmServo.pwmForAngle(0));
    FOLDING_STATION_CONTROL(FOLD_STATION, -90, 80, 90);
    delay(1000);
    FOLDING_STATION_CONTROL(FOLD_STATION, 90, -80, -90);
    delay(1000);
    pwmController.setChannelPWM(CHANNEL_DUMPING_STATION, pwmServo.pwmForAngle(30));
    delay(500);
    pwmController.setChannelPWM(CHANNEL_DUMPING_STATION, pwmServo.pwmForAngle(0));
    delay(500);
    FOLDING_STATION_CONTROL(FOLD_STATION, -90, 80, 90);
    delay(1000);
    pwmController.setChannelPWM(CHANNEL_DUMPING_STATION, pwmServo.pwmForAngle(130));
    FOLDING_STATION_CONTROL(FOLD_STATION, 90, -80, -90);
    delay(1000);
    pwmController.setChannelPWM(CHANNEL_DUMPING_STATION, pwmServo.pwmForAngle(180));
    delay(2000);

    check_Sensor2 = 0;
    check_Sensor3 = 0;

    break;

  default:
    break;
  }
}

void FOLDING_STATION_CONTROL(const int channel[], int angles_left, int angles_center, int angles_right)
{
  pwmController.setChannelPWM(channel[0], pwmServo.pwmForAngle(angles_left));
  pwmController.setChannelPWM(channel[1], pwmServo.pwmForAngle(angles_center));
  pwmController.setChannelPWM(channel[2], pwmServo.pwmForAngle(angles_right));
}

void loop()
{
  //do other things here
  control_RCservo(check_Sensor2, check_Sensor3);
}