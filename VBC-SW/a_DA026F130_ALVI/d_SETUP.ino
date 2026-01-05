//###########################################
//                  SETUP
//###########################################

void setup() {
  Serial.begin(115200);

  state_machine = LIFE_IS_GOOD;

  Serial.println("DAGOR: INIT");
  Serial.print("DAGOR: FW Version -> ");
  Serial.println(FW_VERSION);
 
  gpio_init();
  spi_init();

  _delay(400);
  state_machine = (Dagor_state)drv_init(false); 
  _delay(100);

  if (state_machine == LIFE_IS_GOOD){
    Serial.println("DAGOR: Init SimpleFOC");

    #ifdef MONITOR_BUS_VOLTAGE
      float bus_v = busVoltage();
      int sfoc = SimpleFOCinit(bus_v);
    #else
      int sfoc = SimpleFOCinit(voltageOverride);
    #endif

    if (sfoc){
      Serial.println("DAGOR: Ready PMSM.");
      state_machine = LIFE_IS_GOOD;

      // --- Find mechanical limits using Iq threshold ---
      findRotorLimits();

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

  if (print_currents) taskPrintCurrents();
  if (print_voltages) taskPrintVoltages(); 

}
