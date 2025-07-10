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


// BLDCMotor(pole pair number, phase resistance (optional) );
const int pp = 11;
const float phaseRes = 5.5/2.0;
BLDCMotor motor = BLDCMotor(pp, phaseRes);
// BLDCDriver3PWM(pwmA, pwmB, pwmC, Enable(optional));
#define INHA 27 // PWM input signal for bridge A high side
#define INHB 26 // PWM input signal for bridge B high side
#define INHC 25 // PWM input signal for bridge C high side
BLDCDriver3PWM driver = BLDCDriver3PWM(INHC, INHB, INHA);

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
  driver.voltage_power_supply = 12;
  // driver init
  if(!driver.init()){
    Serial.println("Driver init failed!");
    return;
  }
  // link driver
  motor.linkDriver(&driver);

  // aligning voltage
  motor.voltage_sensor_align = 2.5;

  // set motion control loop to be used
  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::torque;

  // comment out if not needed
  motor.useMonitoring(Serial);

  // initialize motor
  if(!motor.init()){
    Serial.println("Motor init failed!");
    return;
  }
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