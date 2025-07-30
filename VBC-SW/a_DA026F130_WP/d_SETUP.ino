//###########################################
//                  SETUP
//###########################################

// ─── expose the fast ADC api that SimpleFOC compiles ──────────────
extern bool     adcInit(uint8_t pin);   // ← add this
extern uint16_t adcRead(uint8_t pin);

// --------------- self-test (drop this anywhere in your code) -----
void adc_self_test()
{
  Serial.println("\n=== ADC/Gain self-test ===");

  // be sure CSA is in DC-CAL so the outputs sit at Vref
  drv_enable(true);
  current_dc_calib(true);   // short shunts → 0 A
  delayMicroseconds(400);

  // one-time initialisation of the three channels
  adcInit(SO1);
  adcInit(SO2);
  adcInit(SO3);

  const int N = 2048;
  uint32_t sum1 = 0, sum2 = 0, sum3 = 0;
  for (int i = 0; i < N; i++) {
    sum1 += adcRead(SO1);
    sum2 += adcRead(SO2);
    sum3 += adcRead(SO3);
  }

  current_dc_calib(false);  // restore normal CSA inputs

  const float c1 = (float)sum1 / N;
  const float c2 = (float)sum2 / N;
  const float c3 = (float)sum3 / N;

  Serial.println("ADC offset counts  (expected ~2048 @ 12-bit):");
  Serial.printf("  IA: %.1f  |  IB: %.1f  |  IC: %.1f\n", c1, c2, c3);
  Serial.println("=== end self-test ===\n");
}


void setup() {
  Serial.begin(115200);

  state_machine = LIFE_IS_GOOD;

  Serial.println("DAGOR: INIT");
  Serial.print("DAGOR: FW Version -> ");
  Serial.println(FW_VERSION);
  Serial.print("DAGOR: ACTUATOR ID -> ");
  Serial.println(ACT_ID);

  #ifdef ESP_NOW
    Serial.println("DAGOR: ESP-NOW init");
    espNowInit();
    _delay(500);
  #endif
 
  gpio_init();
  spi_init();

  _delay(400);
  state_machine = (Dagor_state)drv_init(false); 
  _delay(100);
  calibratePhaseZeroOffset();

  if (state_machine == LIFE_IS_GOOD){
    Serial.println("DAGOR: Init SimpleFOC");

    #ifdef MONITOR_BUS_VOLTAGE
      float bus_v = busVoltage();
      int sfoc = SimpleFOCinit(bus_v);
    #else
      int sfoc = SimpleFOCinit(voltageOverride);
    #endif

    if (sfoc){
      adc_self_test();          // <-- here
      Serial.println("DAGOR: Ready PMSM.");
      state_machine = LIFE_IS_GOOD;

      //#ifdef CURRENT_SENSE
      //  calibratePhaseZeroOffset();
      //  drv_init();
      //#endif
    } else { 
      Serial.println("DAGOR: Could not initialize SimpleFOC.");
      state_machine = SIMPLEFOC_ERROR;
      drv_fault_status();
      drv_enable(false);
    }

  } else{
    Serial.println("Dagor: ERROR - DRV could not init.");
  }

  taskDRVfault();

  #ifdef MONITOR_TEMP
    taskTempSensor();
  #endif

  #ifdef MONITOR_BUS_VOLTAGE
    taskBusVoltage();
  #endif

  #ifdef MONITOR_ROTOR
    Serial.println("DAGOR: Rotor Monitor Task Enabled.");
    taskRotorMonitor();
  #endif

  #ifdef RS485
    Serial.println("DAGOR: RS485 Task Enabled.");
    taskRS485();
  #endif

  if (print_currents) taskPrintCurrents();
  if (print_voltages) taskPrintVoltages(); 

  /* ------ VBC code to ensure correct gain has ben set ------ */
  // Read current sense gain and HW DRV one
  // Make sure they are the same
  Serial.print("Gain A = ");
  Serial.println(1/(current_sense.gain_a*0.002), 2);
  Serial.print("Gain B = ");
  Serial.println(1/(current_sense.gain_b*0.002), 2);
  Serial.print("Gain C = ");
  Serial.println(1/(current_sense.gain_c*0.002), 2);

  uint16_t csa = drvRead(0x05);
  uint8_t  gcode = (csa >> 9) & 0x03;     // bits10:9
  uint8_t  gain  = (gcode == 0) ? 10 :
                  (gcode == 1) ? 20 :
                  (gcode == 2) ? 40 : 80;

  Serial.printf("CSA_CONTROL1 = 0x%03X  (gain = %dx)\n", csa, gain);
  _delay(1000);
  /* ----------------------------------------------------------*/

}
