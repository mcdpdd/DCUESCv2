// #include <Arduino.h>


// void setup() {
//   // put your setup code here, to run once:
//   pinMode(LED_GREEN, OUTPUT);
//   pinMode(LED_RED, OUTPUT);
// }

// void loop() {
//   // put your main code here, to run repeatedly:
//   digitalWrite(LED_GREEN, !digitalRead(LED_GREEN));
//   digitalWrite(LED_RED, !digitalRead(LED_RED));
//   delay(500); 
// }

// #include <Arduino.h>

// char read_byte = '0';

// HardwareSerial Serial1(PC11,PC10);

// void setup() {
//   Serial1.begin(115200);
// }

// void loop() {

//   Serial1.println("Hello!");

//   if (Serial1.available() > 0) {
//     read_byte = Serial1.read();
//     Serial1.println(read_byte);
//   }
//   delay(1000);  
// }

// #include <SimpleFOC.h>
// #include <SimpleFOCDrivers.h>
// #include "encoders/stm32pwmsensor/STM32MagneticSensorPWM.h"

// //VESC 6.7 Encoder
// #define ENCODER_PWM_PIN PWM_INPUT
// #define ENCODER_PWN_MIN 165
// #define ENCODER_PWM_MAX 43350


// //MagneticSensorPWM sensor = MagneticSensorPWM(PWM_INPUT, ENCODER_PWN_MIN, ENCODER_PWM_MAX);
// STM32MagneticSensorPWM sensor = STM32MagneticSensorPWM(PWM_INPUT, ENCODER_PWN_MIN, ENCODER_PWM_MAX);
// //void doPWM(){sensor.handlePWM();}

// int max_tick;
// int min_tick;

// void setup() {
//   // monitoring port
//   Serial.begin(115200);
//   max_tick = 1000;
//   min_tick = 1000;
//   // initialise encoder hardware
//   sensor.init();
//   // hardware interrupt enable
//   //sensor.enableInterrupt(doPWM);

//   Serial.println("Encoder ready");
//   _delay(1000);
//   int last = _micros();
// }

// void loop() {
//   sensor.update();
//   //display the angle and the angular velocity to the terminal
//   // Serial.print(sensor.getMechanicalAngle(),4);
//   // Serial.print("\t");
//   // Serial.println(sensor.getVelocity());
 
//  int tick = sensor.getDutyCycleTicks();
//  if(tick > max_tick)
//  {
//    max_tick = tick;
//  }

//  if(tick < min_tick)
//  {
//    min_tick = tick;
//  }
//   Serial.print(tick);
//   Serial.print("\t");
//   Serial.print(min_tick);
//   Serial.print("\t");
//   Serial.println(max_tick);
// }

#include <Arduino.h>
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include "encoders/stm32pwmsensor/STM32MagneticSensorPWM.h"

//VESC 6.7 Encoder
#define ENCODER_PWM_PIN PWM_INPUT
#define ENCODER_PWN_MIN 165
#define ENCODER_PWM_MAX 43360
#define ENCODER_FREQ            971
#define ENCODER_HEADER_CLOCKS   16
#define ENCODER_FOOTER_CLOCKS   8
#define ENCODER_PWM_CLOCKS      ENCODER_HEADER_CLOCKS + ENCODER_FOOTER_CLOCKS + 4096

//Motor 
#define BLDC_POLES              40//8  //40
#define BLDC_POLE_PAIRS         20//4   //20
#define BLDC_PHASE_RESISTANCE   0.0608       //Ohms
#define BLDC_KV_RATING          16.6995      // RPM/V
#define BLDC_INDUCTANCE         0.0002174    //Henries 

//Current Sense
#define CURRENT_SENSE_RVALUE    .0005  //Ohms
#define CURRENT_SENSE_GAIN      20.0   //VpV

//Voltage Sense
#define VOLTAGE_SENSE_R1 39000.0
#define VOLTAGE_SENSE_R2 1000.0

BLDCMotor motor = BLDCMotor(BLDC_POLE_PAIRS,BLDC_PHASE_RESISTANCE,BLDC_KV_RATING,BLDC_INDUCTANCE);
BLDCDriver6PWM driver(H1, L1, H2, L2, H3, L3, EN_GATE);
LowsideCurrentSense current_sense = LowsideCurrentSense(CURRENT_SENSE_RVALUE, CURRENT_SENSE_GAIN, ADC_CURRENT_SENSE1, ADC_CURRENT_SENSE2, ADC_CURRENT_SENSE3);

//Setup encoder
STM32MagneticSensorPWM sensor = STM32MagneticSensorPWM(ENCODER_PWM_PIN, ENCODER_PWN_MIN, ENCODER_PWM_MAX);

//Temp Sense
// #define MOTOR_BETA  3589.0f
// #define RT_25C      5440.0f
// #define R1_RESISTOR 10000.0f
// #define TEMP_LT_SIZE 4095
// #define FULL_SCALE_VOLT 3.3f

// uint16_t temp_lt[4096];
// void calcLookupTable(){
    
//     for (int i = 0; i < TEMP_LT_SIZE; i++) {
//         float voltage_rt = (((float)i / 4095.0) * FULL_SCALE_VOLT);
//         float current_rt = ((FULL_SCALE_VOLT - voltage_rt) / R1_RESISTOR);
//         float NTC_resistance_motor = voltage_rt/current_rt;
//         float temp_recip = (1.0 / 298.15) - (log(RT_25C/NTC_resistance_motor)/MOTOR_BETA);
//         temp_lt[i] = static_cast<uint16_t>(((1/temp_recip) - 273.15f) * 100);
//     }
// }

float target = 0;                       // angle set point variable
Commander command = Commander(Serial);        // instantiate the commander

// void getTemp(char* cmd) {
//   Serial.println((float)(temp_lt[analogRead(TEMP_MOTOR)])/100.0);
// }

void onMotor(char* cmd) {
  command.motor(&motor, cmd);
}

void setup() {

  
  analogReadResolution(12);

  // calcLookupTable();

  sensor.init();                     
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = 48;          
  driver.init();

  motor.linkDriver(&driver);
  current_sense.linkDriver(&driver);

  // current sense init hardware
  current_sense.init();
  // link the current sense to the motor
  motor.linkCurrentSense(&current_sense);

  //aligning voltage [V]
  motor.voltage_sensor_align = 2;
  // index search velocity [rad/s]
  motor.velocity_index_search = 1;

  motor.torque_controller = TorqueControlType::foc_current;
  motor.controller = MotionControlType::torque;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  

  //Initial FOC current parameters
  motor.PID_current_q.P = .1;
  motor.PID_current_q.I= 10;
  motor.PID_current_d.P= .1;
  motor.PID_current_d.I = 10;
  motor.LPF_current_q.Tf = 0.002f; // 1ms default
  motor.LPF_current_d.Tf = 0.002f; // 1ms default

  // use monitoring with serial
  Serial.begin(115200);
  motor.useMonitoring(Serial);
  // motor.monitor_variables = _MON_CURR_Q | _MON_CURR_D ; // default _MON_TARGET | _MON_VOLT_Q | _MON_VEL | _MON_ANGLE
  // motor.monitor_downsample = 100;

  Serial.println(F("Motor init."));
  motor.init();

  Serial.println(F("Init FOC"));
  motor.initFOC();

  // add target command T
  command.add('M', onMotor, "my motor");
  //command.verbose = VerboseMode::nothing;
  //command.add('B', getTemp, "Motor Temp");

  Serial.println(F("Motor ready."));
  _delay(1000);
}

void loop() {
  motor.loopFOC();
  motor.move();
  motor.monitor();
  command.run();
}

