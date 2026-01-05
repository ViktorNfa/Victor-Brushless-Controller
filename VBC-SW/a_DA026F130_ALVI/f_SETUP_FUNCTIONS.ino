//###########################################
//       SETUP FUNCTIONS DEFINITIONS
//###########################################


float busVoltage(){

  float bus_v;

  int loops = 100;

  for(int i = 0; i<=loops; i++){
    //Serial.println(analogRead(vMonitor));
    bus_v += analogRead(vMonitor) * 28.7/4095; //36.3
  }

  bus_v = bus_v/loops;

  Serial.print("Dagor: Bus Voltage ->");
  Serial.println(bus_v);
  
  return bus_v;

}

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

void calibratePhaseZeroOffset(){
  float offset_ia = 0, offset_ib = 0, offset_ic = 0;

  const float calibration_rounds = 2000;
  float adc_voltage_conv = (3.3f)/(4095.0f);

  drv_enable(true);
  current_dc_calib(true);

  for (int i = 0; i < calibration_rounds; i++) {
    offset_ia += analogRead(SO1) * adc_voltage_conv;
    offset_ib += analogRead(SO2) * adc_voltage_conv;
    offset_ic += analogRead(SO3) * adc_voltage_conv;
    _delay(1);
  }

  current_dc_calib(false);
  drv_enable(false);
  
  // calculate the mean offsets
  offset_ia = offset_ia / calibration_rounds;
  offset_ib = offset_ib / calibration_rounds;
  offset_ic = offset_ic / calibration_rounds;

  Serial.print("Offset ia: ");
  Serial.println(offset_ia);
  Serial.print("Offset ib: ");
  Serial.println(offset_ib);
  Serial.print("Offset ic: ");
  Serial.println(offset_ic);

  current_sense.offset_ia = offset_ia;
  current_sense.offset_ib = offset_ib;
  current_sense.offset_ic = offset_ic;

}

void taskTempSensor(){
  xTaskCreatePinnedToCore(
    tempStatus, /* Function to implement the task */
    "Temperature Sensor Monitor", /* Name of the task */
    2048,  /* Stack size in words */
    (void*)&print_temp,
    5,  /* Priority of the task */
    NULL,  /* Task handle. */
    0); /* Core where the task should run */
}

void taskBusVoltage(){
  xTaskCreatePinnedToCore(
    busVoltageMonitor, /* Function to implement the task */
    "Bus Voltage Monitor", /* Name of the task */
    2048,  /* Stack size in words */
    (void*)&print_bus_voltage,
    3,  /* Priority of the task */
    NULL,  /* Task handle. */
    0); /* Core where the task should run */
}

void taskPrintCurrents(){
  xTaskCreatePinnedToCore(
    printCurrents, /* Function to implement the task */
    "Monitor Currents", /* Name of the task */
    2048,  /* Stack size in words */
    (void*)&print_dq_currents,  /* Task input parameter */
    0,  /* Priority of the task */
    NULL,  /* Task handle. */
    0); /* Core where the task should run */
}

void taskPrintVoltages(){
  xTaskCreatePinnedToCore(
    printVoltages, /* Function to implement the task */
    "Monitor Voltages", /* Name of the task */
    2048,  /* Stack size in words */
    (void*)&print_dq_voltages,  /* Task input parameter */
    0,  /* Priority of the task */
    NULL,  /* Task handle. */
    0); /* Core where the task should run */
}

void taskRotorMonitor(){
  xTaskCreatePinnedToCore(
    rotorMonitor, /* Function to implement the task */
    "Rotor Position and Veloity", /* Name of the task */
    2048,  /* Stack size in words */
    (void*)&print_rotor_data,
    2,  /* Priority of the task */
    NULL,  /* Task handle. */
    1); /* Core where the task should run */
}

void taskDRVfault(){
  xTaskCreatePinnedToCore(
    drv_task, /* Function to implement the task */
    "DRV Fault Status", /* Name of the task */
    2048,  /* Stack size in words */
    NULL,
    4,  /* Priority of the task */
    NULL,  /* Task handle. */
    0); /* Core where the task should run */
}


void findRotorLimits() {

#ifndef CURRENT_SENSE
  Serial.println("Homing skipped: CURRENT_SENSE not enabled.");
  rotor_limits_found = false;
  return;
#else
  // Ensure the motor is running in velocity mode for the search
  MotionControlType prev = motor.controller;
  motor.controller = MotionControlType::velocity;

  // Small helper for “trip detect” with debounce
  auto tripped = []() -> bool {
    static int over = 0;
    const float iq = motor.current.q;
    if (fabsf(iq) >= iq_trip) over++;
    else over = 0;
    return (over >= 20); // ~20 consecutive loops to ignore spikes
  };

  Serial.println("Homing: searching for MIN limit...");
  motor.enable();

  // Search negative direction first (define this as “min”)
  motor.target = -homing_velocity;

  const unsigned long t0 = millis();
  const unsigned long timeout_ms = 15000;

  while (!tripped()) {
    motor.loopFOC();
    motor.move();

    if (millis() - t0 > timeout_ms) {
      Serial.println("Homing MIN: timeout. Keeping default limits.");
      motor.target = 0;
      motor.move();
      motor.disable();
      motor.controller = prev;
      rotor_limits_found = false;
      return;
    }
    _delay(1);
  }

  motor.target = 0;
  motor.move();
  motor.disable();

  min_rotor_position = motor.shaft_angle;
  Serial.print("Homing: MIN found at ");
  Serial.println(min_rotor_position, 6);

  _delay(300);

  Serial.println("Homing: searching for MAX limit...");
  motor.enable();
  motor.target = +homing_velocity;

  const unsigned long t1 = millis();
  while (!tripped()) {
    motor.loopFOC();
    motor.move();

    if (millis() - t1 > timeout_ms) {
      Serial.println("Homing MAX: timeout. Keeping default limits.");
      motor.target = 0;
      motor.move();
      motor.disable();
      motor.controller = prev;
      rotor_limits_found = false;
      return;
    }
    _delay(1);
  }

  motor.target = 0;
  motor.move();
  motor.disable();

  max_rotor_position = motor.shaft_angle;
  Serial.print("Homing: MAX found at ");
  Serial.println(max_rotor_position, 6);

  // Restore previous control mode
  motor.controller = prev;

  // Compute 80% span (centered): keep 10% margin on each side
  const float range = (max_rotor_position - min_rotor_position);
  const float margin = 0.5f * (1.0f - sweep_span_ratio) * range; // for 0.80 => 0.10*range each side
  sweep_low = min_rotor_position + margin;
  sweep_high = max_rotor_position - margin;

  rotor_limits_found = true;

  Serial.print("Sweep window: [");
  Serial.print(sweep_low, 6);
  Serial.print(", ");
  Serial.print(sweep_high, 6);
  Serial.println("]");

#endif
}