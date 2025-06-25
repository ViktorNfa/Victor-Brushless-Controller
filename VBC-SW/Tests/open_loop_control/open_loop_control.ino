#include <SimpleFOC.h>


/*********  Dagor board bring-up for DRV8305  *********/
#include <SPI.h>

constexpr int PIN_CS_DRV   = 5;   // DRV8305 SPI-CS
constexpr int PIN_EN_GATE  = 2;   // DRV8305 nSLEEP / EN_GATE
constexpr int PIN_NFAULT   = 4;   // optional, just for diagnostics

static inline void drv_write(uint8_t addr, uint16_t data)
{
  // 11-bit payload + 3-bit address, see DRV8305 DS p.30
  uint16_t frame = ((addr & 0x07u) << 11) | (data & 0x7FFu);

  digitalWrite(PIN_CS_DRV, LOW);
  SPI.transfer16(frame);
  digitalWrite(PIN_CS_DRV, HIGH);
}

void dagor_begin()
{
  /* --- GPIO and SPI basics ------------------------------------------ */
  pinMode(PIN_CS_DRV,   OUTPUT);   digitalWrite(PIN_CS_DRV, HIGH);
  pinMode(PIN_EN_GATE,  OUTPUT);   digitalWrite(PIN_EN_GATE, LOW);
  pinMode(PIN_NFAULT,   INPUT);            // optional

  SPI.begin();                       // default SCK=18, MISO=19, MOSI=23
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE1);        // DRV8305 uses CPOL=0, CPHA=1

  /* --- Program the driver ------------------------------------------- */
  // CTRL2 (addr 0x02) : 3-PWM mode, no over-current shut-down, etc.
  drv_write(0x02, 0b011_101_00110);    // 0x3A96 in the Dagor FW
  // CSA (addr 0x03)  : clamp amp outputs to 3.3 V, 80× gain
  drv_write(0x03, 0b100_1010_00000);   // 0x4A80
  // CSA2 (addr 0x04) : 80× gain selected
  drv_write(0x04, 0b101_0000_00000);   // 0x5000

  /* --- Let the power stage start switching -------------------------- */
  delay(2);
  digitalWrite(PIN_EN_GATE, HIGH);     // enable half-bridges
  delay(2);
}
/******************************************************/


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

  dagor_begin();

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
  motor.voltage_limit = 3;   // [V]
 
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