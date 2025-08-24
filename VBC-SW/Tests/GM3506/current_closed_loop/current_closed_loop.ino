#include <SimpleFOC.h>


/* ------------ Specific setup for Dagor board ------------ */
// Dagor-specific pin map (same names h_DRV8305.ino uses)
#define enGate   2    // DRV8305 EN_GATE
#define nFault   4    // DRV8305 nFAULT  (not really used here)
#define cs       5    // DRV8305 SPI-CS
#define SO1      36   // phase-A current
#define SO2      35   // phase-B current
#define SO3      34   // phase-C current

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


/* ------------ PID Tuning & others ------------ */
const float amp_limit = 1.0;              // IQ current limit [amps] - requires trueTorque mode
const int motionDownSample = 4;           // Downsample the motion control loops with respect to the torque control loop [amount of loops]

const float cp = 0.015;                     // QD current loops PROPORTONAL gain value           - MQP & MDP
const float ci = 40.0;                   // QD current loops INTEGRAL gain value              - MQI & MDI
const float cd = 0.0;                     // QD current loops DERIVATIVE gain value            - MQD & MDD
const float lpQDFilter = 0.001;           // QD current loops measurement low-pass filter      - QF & DF

const float vp = 0.1;                     // Velocity control loop PROPORTIONAL gain value     - VP
const float vi = 1.0;                     // Velocity control loop INTEGRAL gain value         - VI
const float vd = 0.0;                     // Velocity control loop DERIVATIVE gain value       - VD
const float lpVelFilter = 0.01;           // Velocity measurement low-pass filter              - VF
const float velocity_limit = 100;         // Velocity limit [rpm]                              - LV

const float ap = 15.0;                     // Position control loop PROPORTIONAL gain value     - AP
const float ai = 0.0;                     // Position control loop INTEGRAL gain value         - AI
const float ad = 0.0;                     // Position control loop DERIVATIVE gain value       - AD
const float lpPosFilter = 0.000;          // Position measurment low-pass filter               - AF
const float voltageRamp = 2000;           // Change in voltage allowed [Volts per sec]         - VR
/* --------------------------------------------- */

// BLDCMotor(pole pair number, phase resistance (optional) );
const int pp = 11;
const float phaseRes = 5.6/2.0;
BLDCMotor motor = BLDCMotor(pp);
// BLDCDriver3PWM(pwmA, pwmB, pwmC, Enable(optional));
#define INHA 27 // PWM input signal for bridge A high side
#define INHB 26 // PWM input signal for bridge B high side
#define INHC 25 // PWM input signal for bridge C high side
BLDCDriver3PWM driver = BLDCDriver3PWM(INHC, INHB, INHA);

// Magnetic rotor sensor
MagneticSensorSPI sensor = MagneticSensorSPI(AS5147_SPI, 16);

// current sensor
LowsideCurrentSense current_sense = LowsideCurrentSense(0.002f, 80.0f, SO1, SO2);

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
  
  // Set SPI clock freq. to 10MHz, default is 1MHz
  sensor.clock_speed = 10000000;
  // initialize encoder sensor hardware
  sensor.init();
  // link the motor to the sensor
  motor.linkSensor(&sensor);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  // driver init
  if(!driver.init()){
    Serial.println("Driver init failed!");
    return;
  }
  // link driver
  motor.linkDriver(&driver);
  // link driver to cs
  current_sense.linkDriver(&driver);

  // current sense init hardware
  if(!current_sense.init()){
    Serial.println("Current sense init failed!");
    return;
  }
  // link the current sense to the motor
  motor.linkCurrentSense(&current_sense);
  // skip alignment procedure
  current_sense.skip_align = true;

  // aligning voltage
  motor.voltage_sensor_align = 2.5;

  // set motion control loop to be used
  motor.torque_controller = TorqueControlType::foc_current;
  motor.controller = MotionControlType::torque;
  motor.voltage_limit = amp_limit*phaseRes;
  motor.current_limit = amp_limit;

  /* ------------ PID Tuning ------------ */
  // Current PI controller parameters - q_axis
  motor.PID_current_q.P = cp;
  motor.PID_current_q.I = ci;
  motor.PID_current_q.D = cd;
  motor.PID_current_q.limit = amp_limit*phaseRes;
  motor.PID_current_q.output_ramp = voltageRamp;
  motor.LPF_current_q.Tf = lpQDFilter;

  // Current PI controller parameters - d_axis
  motor.PID_current_d.P = cp;
  motor.PID_current_d.I = ci;
  motor.PID_current_d.D = cd;
  motor.PID_current_d.limit = amp_limit*phaseRes;
  motor.PID_current_d.output_ramp = voltageRamp;
  motor.LPF_current_d.Tf = lpQDFilter;

  // // velocity PI controller parameters
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

  // // Downsampling value of the motion control loops with respect to torque loops
  // motor.motion_downsample = motionDownSample; // - times (default 0 - disabled)

  // use monitoring with serial
  // comment out if not needed
  // motor.useMonitoring(Serial);
  // motor.monitor_downsample = 100; // set downsampling can be even more > 100
  // motor.monitor_variables = _MON_CURR_Q | _MON_CURR_D; // set monitoring of Q and D currents

  // initialize motor
  motor.init();
  // align sensor and start FOC
  if(!motor.initFOC()){
    Serial.println("FOC init failed!");
    return;
  }

  // set the initial motor target
  motor.target = 0.0; // Amps

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

  // real-time monitoring calls
  // motor.monitor();

  // Display Q & D currents in a Serial Plotter friendly way
  static uint16_t cnt = 0;
  if (++cnt >= 50) {        // same down-sampling used before
    cnt = 0;

    // get D-Q currents *now*, with the latest electrical angle
    DQCurrent_s iq = current_sense.getFOCCurrents(motor.electrical_angle);

    Serial.print("Iq:"); 
    Serial.print(iq.q, 3);  // [Amps] 3 decimals
    Serial.print('\t');
    Serial.print("Id:");
    Serial.println(iq.d, 3); // [Amps] 3 decimals
  }

  // user communication
  command.run();
}