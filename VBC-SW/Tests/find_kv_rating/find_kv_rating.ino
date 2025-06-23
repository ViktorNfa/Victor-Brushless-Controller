/**
 * Find KV rating for motor with magnetic sensors
 *
 * Motor KV rating is defiend as the increase of the motor velocity expressed in rotations per minute [rpm] per each 1 Volt int voltage control mode.
 * 
 * This example will set your motor in the torque control mode using voltage and set 1 volt to the motor. By reading the velocity it will calculat the motors KV rating.
 * - To make this esimation more credible you can try increasing the target voltage (or decrease in some cases) 
 * - The KV rating should be realatively static number - it should not change considerably with the increase in the voltage
 */
#include <SimpleFOC.h>

// magnetic sensor instance - SPI
MagneticSensorSPI sensor = MagneticSensorSPI(AS5147_SPI, 16);

// BLDC motor & driver instance
const int pp = 20;
BLDCMotor motor = BLDCMotor(pp);
#define INHA 27 // PWM input signal for bridge A high side
#define INHB 26 // PWM input signal for bridge B high side
#define INHC 25 // PWM input signal for bridge C high side
BLDCDriver3PWM driver = BLDCDriver3PWM(INHC, INHB, INHA);

// voltage set point variable
float target_voltage = 0.3;

// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_voltage, cmd); }
void calcKV(char* cmd) { 
  // calculate the KV
  Serial.println(motor.shaft_velocity/motor.target/_SQRT3*30.0f/_PI);

}

// DRV8305 bring-up (3-PWM, 80× gain, clamp to 3 V3)
constexpr int PIN_CS_DRV = 5;
constexpr int PIN_EN_GATE = 2;
void drv8305_init() {
  SPI.begin();
  digitalWrite(PIN_CS_DRV, HIGH);
  pinMode(PIN_CS_DRV, OUTPUT);
  pinMode(PIN_EN_GATE, OUTPUT);
  digitalWrite(PIN_EN_GATE, LOW);

  auto wr = [&](uint16_t reg) {
    digitalWrite(PIN_CS_DRV, LOW);
    SPI.transfer16(reg);
    digitalWrite(PIN_CS_DRV, HIGH);
  };
  wr(0x3A96);        // CTRL2 – 3-PWM mode
  wr(0x4A80);        // CSA – clamp amp outputs to 3.3 V
  wr(0x5000);        // CSA – gain = 80×
  digitalWrite(PIN_EN_GATE, HIGH);
  delay(2);
}

void setup() { 
  
  // use monitoring with serial 
  Serial.begin(115200);
  // enable more verbose output for debugging
  // comment out if not needed
  SimpleFOCDebug::enable(&Serial);

  drv8305_init();

  // initialize encoder sensor hardware
  sensor.init();
  // link the motor to the sensor
  motor.linkSensor(&sensor);

  // driver config
  // IMPORTANT!
  // make sure to set the correct power supply voltage [V]
  driver.voltage_power_supply = 22.0;
  driver.init();
  // link driver
  motor.linkDriver(&driver);

  motor.voltage_limit = 0.7;
  // aligning voltage
  motor.voltage_sensor_align = 1.0;

  // set motion control loop to be used
  motor.controller = MotionControlType::torque;

  // comment out if not needed
  motor.useMonitoring(Serial);

  // initialize motor
  motor.init();
  // align sensor and start FOC
  motor.initFOC();

  // add target command T
  command.add('T', doTarget, "target voltage");
  command.add('K', calcKV, "calculate KV rating");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target voltage : - commnad T"));
  Serial.println(F("Calculate the motor KV : - command K"));
  _delay(1000);
}


void loop() {

  // main FOC algorithm function
  // the faster you run this function the better
  // Arduino UNO loop  ~1kHz
  // Bluepill loop ~10kHz 
  motor.loopFOC();

  // Motion control function
  // velocity, position or voltage (defined in motor.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code
  motor.move(target_voltage);

  // user communication
  command.run();
}