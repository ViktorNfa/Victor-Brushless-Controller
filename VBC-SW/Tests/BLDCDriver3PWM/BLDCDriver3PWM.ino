#include <SimpleFOC.h>

#define INHA 27             // PWM input signal for bridge A high side
#define INHB 26             // PWM input signal for bridge B high side
#define INHC 25             // PWM input signal for bridge C high side

// BLDCDriver3PWM(pwmA, pwmB, pwmC, (en optional))
BLDCDriver3PWM driver = BLDCDriver3PWM(INHC, INHB, INHA);

void setup() {
  
  // use monitoring with serial 
  Serial.begin(115200);
  // enable more verbose output for debugging
  // comment out if not needed
  SimpleFOCDebug::enable(&Serial);
  
  // power supply voltage [V]
  driver.voltage_power_supply = 22;
  // Max DC voltage allowed - default voltage_power_supply
  driver.voltage_limit = 22;

  // driver init
  if (!driver.init()){
    Serial.println("Driver init failed!");
    return;
  }

  // enable driver
  driver.enable();
  Serial.println("Driver ready!");
  _delay(1000);
}

void loop() {
    // setting pwm
    // phase A: 3V
    // phase B: 6V
    // phase C: 5V
    driver.setPwm(3,6,5);
}