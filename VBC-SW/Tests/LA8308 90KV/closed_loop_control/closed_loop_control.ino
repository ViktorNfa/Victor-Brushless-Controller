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


/* ------------ PID Tuning ------------ */
const float vp = 0.1;                     // Velocity control loop PROPORTIONAL gain value     - VP
const float vi = 1.0;                     // Velocity control loop INTEGRAL gain value         - VI
const float vd = 0.0;                     // Velocity control loop DERIVATIVE gain value       - VD
const float lpVelFilter = 0.01;           // Velocity measurement low-pass filter              - VF
const float velocity_limit = 150;         // Velocity limit [rad/s]                            - LV

const float ap = 5.0;                     // Position control loop PROPORTIONAL gain value     - AP
const float ai = 0.0;                     // Position control loop INTEGRAL gain value         - AI
const float ad = 0.3;                     // Position control loop DERIVATIVE gain value       - AD
const float lpPosFilter = 0.000;          // Position measurement low-pass filter              - AF
const float voltageRamp = 2000;           // Change in voltage allowed [Volts per sec]         - VR
/* ------------------------------------ */

// BLDCMotor(pole pair number, phase resistance (optional) );
const int pp = 20;
const float phaseRes = 0.186;
BLDCMotor motor = BLDCMotor(pp);
// BLDCDriver3PWM(pwmA, pwmB, pwmC, Enable(optional));
#define INHA 27 // PWM input signal for bridge A high side
#define INHB 26 // PWM input signal for bridge B high side
#define INHC 25 // PWM input signal for bridge C high side
BLDCDriver3PWM driver = BLDCDriver3PWM(INHC, INHB, INHA);

// Magnetic rotor sensor
MagneticSensorSPI sensor = MagneticSensorSPI(AS5147_SPI, 16);

// instantiate the commander
Commander command = Commander(Serial);
void doMotor(char* cmd) { command.motor(&motor, cmd); }

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
  // power supply voltage [V]
  driver.voltage_power_supply = 22;
  // driver init
  if(!driver.init()){
    Serial.println("Driver init failed!");
    return;
  }
  // link driver
  motor.linkDriver(&driver);

  // aligning voltage
  motor.voltage_sensor_align = 5;

  // set motion control loop to be used
  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::torque;

  /* ------------ PID Tuning ------------ */
  // velocity PI controller parameterstorque
  motor.PID_velocity.P = vp;
  motor.PID_velocity.I = vi;
  motor.PID_velocity.D = vd;
  motor.PID_velocity.output_ramp = voltageRamp;
  motor.LPF_velocity.Tf = lpVelFilter;
  motor.velocity_limit = velocity_limit;       // maximal velocity of the position control
  
  // angle P controller
  motor.P_angle.P = ap;
  motor.P_angle.I = ai;
  motor.P_angle.D = ad;
  motor.LPF_angle.Tf = lpPosFilter;
  /* ------------------------------------ */

  // comment out if not needed
  motor.useMonitoring(Serial);

  // initialize motor
  motor.init();
  // align sensor and start FOC
  if(!motor.initFOC()){
    Serial.println("FOC init failed!");
    return;
  }

  // set the initial motor target
  motor.target = 0.5; // Volts 

  // add target command M
  command.add('M', doMotor, "Motor");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target using serial terminal and command M:"));
  _delay(1000);
}

void loop() {
  // main FOC algorithm function
  motor.loopFOC();

  // Motion control function
  motor.move();

  // user communication
  command.run();
}