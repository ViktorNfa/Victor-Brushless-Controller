#include <SimpleFOC.h>

// BLDCMotor(pole pair number, phase resistance (optional) );
const int pp = 11;
const float phaseRes = 5.6/2.0;
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

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12.0;
  // limit the maximal dc voltage the driver can set
  // as a protection measure for the low-resistance motors
  // this value is fixed on startup
  driver.voltage_limit = 12;
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
  motor.voltage_limit = 12;   // [V]
 
  // open loop control config
  motor.controller = MotionControlType::velocity_openloop;

  // init motor hardware
  if(!motor.init()){
    Serial.println("Motor init failed!");
    return;
  }

  // set the target velocity [rad/s]
  motor.target = 3; // one rotation per second

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