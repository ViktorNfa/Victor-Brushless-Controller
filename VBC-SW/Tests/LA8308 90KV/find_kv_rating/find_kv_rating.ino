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


/* ------------ Specific setup for Dagor board ------------ */
// Dagor-specific pin map (same names h_DRV8305.ino uses)
#define enGate   2    // DRV8305 EN_GATE
#define nFault   4    // DRV8305 nFAULT  (not really used here)
#define cs       5    // DRV8305 SPI-CS
#define SO1      36   // phase-A current (unused)
#define SO2      35   // phase-B current (unused)
#define SO3      34   // phase-C current (unused)

// Two dummy symbols the Dagor file references – keep it happy
enum { LIFE_IS_GOOD = 0, DRV_ERROR = 1 };
int  state_machine = LIFE_IS_GOOD;

// Prototypes from h_DRV8305.ino
int drv_init(bool resp);
void drv_enable(bool on);

void spi_init(){
  //SPI start up
  pinMode(cs, OUTPUT);
  digitalWrite(cs, HIGH);
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE1);
}

void gpio_init(){
  //Pinmodes assignment
  pinMode(15,OUTPUT);
  digitalWrite(15,HIGH);
  pinMode(SO1, INPUT);
  pinMode(SO2, INPUT);
  pinMode(SO3, INPUT);
  pinMode(nFault, INPUT);
  pinMode(enGate, OUTPUT);
  digitalWrite(enGate, LOW);
}
/* -------------------------------------------------------- */


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
float target_voltage = 3;

// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_voltage, cmd); }
void calcKV(char* cmd) { 
  // calculate the KV
  Serial.println(motor.shaft_velocity/motor.target/_SQRT3*30.0f/_PI);

}

void setup() { 
  
  // use monitoring with serial 
  Serial.begin(115200);
  // enable more verbose output for debugging
  // comment out if not needed
  SimpleFOCDebug::enable(&Serial);

  gpio_init();  // sets pin modes and pulls EN_GATE low
  spi_init();   // SPI MODE1, MSB first
  drv_init(false);

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

  motor.voltage_limit = 5.0;
  // aligning voltage
  motor.voltage_sensor_align = 3.0;

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