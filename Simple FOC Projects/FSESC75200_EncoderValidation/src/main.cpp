#include <Arduino.h>
#include <SimpleFOC.h>

//VESC 6.7 Encoder
#define ENCODER_PWM_PIN PC_6
#define ENCODER_PWN_MIN 4
#define ENCODER_PWM_MAX 1028

//Motor 
#define BLDC_POLES              20
#define BLDC_PHASE_RESISTANCE   0.0608 //Ohms
#define BLDC_KV_RATING          16.6995      // RPM/V
#define BLDC_INDUCTANCE         0.0002174    //Henries 

//Current Sense
#define CURRENT_SENSE_RVALUE    .0005  //Ohms
#define CURRENT_SENSE_GAIN      20.0   //VpV
#define CURRENT_SENSE_A_ADC     PC_0
#define CURRENT_SENSE_B_ADC     PC_1
#define CURRENT_SENSE_C_ADC     PC_2

//Setup encoder
MagneticSensorPWM sensor = MagneticSensorPWM(ENCODER_PWM_PIN, ENCODER_PWN_MIN, ENCODER_PWM_MAX);
void doPWM(){sensor.handlePWM();}

float value = 0.0;

void setup() {
  // monitoring port
  //Serial.begin(115200);

  // initialise magnetic sensor hardware
  sensor.init();
  // comment out to use sensor in blocking (non-interrupt) way
  sensor.enableInterrupt(doPWM);

  //Serial.println("Sensor ready");
  _delay(1000);
}

void loop() {
  // iterative function updating the sensor internal variables
  // it is usually called in motor.loopFOC()
  // this function reads the sensor hardware and 
  // has to be called before getAngle nad getVelocity
  sensor.update();
  // display the angle and the angular velocity to the terminal
  value = sensor.getMechanicalAngle();
  // Serial.print("Angle:");
  // Serial.println(value, 4);
  _delay(500);
}
