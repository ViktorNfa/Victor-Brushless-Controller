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

// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&motor.target, cmd); }
void doLimit(char* cmd) { command.scalar(&motor.voltage_limit, cmd); }

void setup() {

  // use monitoring with serial 
  Serial.begin(115200);
  // enable more verbose output for debugging
  // comment out if not needed
  SimpleFOCDebug::enable(&Serial);

  gpio_init();  // sets pin modes and pulls EN_GATE low
  spi_init();   // SPI MODE1, MSB first
  drv_init(false);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12.0;
  // limit the maximal dc voltage the driver can set
  // as a protection measure for the low-resistance motors
  // this value is fixed on startup
  driver.voltage_limit = 12.0;
  if(!driver.init()){
    Serial.println("Driver init failed!");
    return;
  }
  // link the motor and the driver
  motor.linkDriver(&driver);

  // limiting motor movements
  // limit the voltage to be set to the motor
  // start very low for high resistance motors
  // current = voltage / resistance, so try to be well under 1Amp
  motor.voltage_limit = 3.0;   // [V]
 
  // open loop control config
  motor.controller = MotionControlType::velocity_openloop;

  // init motor hardware
  if(!motor.init()){
    Serial.println("Motor init failed!");
    return;
  }

  // set the target velocity [rad/s]
  motor.target = 3.0; // one rotation per second

  // add target command T
  command.add('T', doTarget, "target velocity");
  command.add('L', doLimit, "voltage limit");

  Serial.println("Motor ready!");
  Serial.println("Set target velocity [rad/s]");
  _delay(1000);
}

void loop() {

  // open loop velocity movement
  motor.move();

  // user communication
  command.run();
}