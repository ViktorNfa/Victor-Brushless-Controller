//#################################################
//                     GLOBAL
//#################################################

// Libraries, pin number assignment and instance initialization

//SimpleFOC Version 2.5
//#include <SimpleFOC.h>
#include <SPI.h>

#ifdef CALIBRATED_SENSOR
  #include <SimpleFOCDrivers.h>
  #include <encoders/calibrated/CalibratedSensor.h>
#endif

enum Dagor_state {
  LIFE_IS_GOOD,
  ACTIVE_COMP,
  DRV_WARNING,
  FETS_TEMP_WARNING,
  MOTOR_TEMP_WARNING,
  SIMPLEFOC_WARNING,
  DRV_ERROR,
  FETS_TEMP_ERROR,
  MOTOR_TEMP_ERROR,
  SIMPLEFOC_ERROR,
  OUT_BOUNDS_ERROR
} state_machine = LIFE_IS_GOOD;


//#############_THREE PHASE DRIVER - DRV8305_########
// Datasheet: www.ti.com/lit/ds/symlink/drv8305.pdf
#define enGate 2            // Chip Enable
#define nFault 4            // Fault reading
#define cs 5                // DRV8305 SPI Chip-select
#define SO1 36              // Current sense phase A
#define SO2 35              // Current sense phase B
#define SO3 34              // Current sense phase C
#define INHA 27             // PWM input signal for bridge A high side
#define INHB 26             // PWM input signal for bridge B high side
#define INHC 25             // PWM input signal for bridge C high side

//#############_MAGNETIC SENSOR - AS5147_############
// Datasheet: https://ams.com/documents/20143/36005/AS5147_DS000307_2-00.pdf
#define sensorCS 16             //AS5147 SPI Chip-select

//#############_TEMPERATURE SENSOR - STLM20_#########
// Datasheet: https://datasheet.lcsc.com/szlcsc/1810010411_STMicroelectronics-STLM20W87F_C129796.pdf
#define vTemp 39

//#############_Voltage Monitor_#####################
#define vMonitor 33

//#############_FUNCTIONS DECLARATION_################
//void drv_enable();
void spi_init();
void gpio_init();
void current_dc_calib(bool activate);
void calibratePhaseZeroOffset();
//unsigned long timeManagement();
void gravityComp();

void findRotorLimits();
void autoSweepUpdate();

bool rotor_limits_found = false;

float sweep_low = -10.0f;
float sweep_high = 10.0f;
int sweep_dir = +1;   // +1 => increasing angle, -1 => decreasing
