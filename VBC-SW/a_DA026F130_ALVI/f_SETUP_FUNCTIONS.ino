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

  // Ensure velocity mode for the search
  MotionControlType prev = motor.controller;
  motor.controller = MotionControlType::velocity;

  auto stopMotor = [](){
    motor.target = 0;
    // run a bit to actually apply 0 target
    for (int i = 0; i < 200; i++) {
      motor.loopFOC();
      motor.move();
      _delay(1);
    }
  };

  auto tripDetected = [&](int required_samples) -> bool {
    int over = 0;  // <-- NOT static, resets each call-site use
    while (1) {
      motor.loopFOC();
      motor.move();

      if (fabsf(motor.current.q) >= iq_trip) over++;
      else over = 0;

      if (over >= required_samples) return true;
      return false;
    }
  };

  // A “continuous” version used inside the while loops (also non-static)
  auto waitForTrip = [&](unsigned long timeout_ms, int required_samples) -> bool {
    unsigned long t0 = millis();
    int over = 0;

    while (1) {
      motor.loopFOC();
      motor.move();

      if (fabsf(motor.current.q) >= iq_trip) over++;
      else over = 0;

      if (over >= required_samples) return true;

      if (millis() - t0 > timeout_ms) return false;

      _delay(1);
    }
  };

  const unsigned long timeout_ms = 15000;

  // ---- MIN search (negative velocity) ----
  Serial.println("Homing: searching for MIN limit...");
  motor.enable();
  motor.target = -homing_velocity;

  if (!waitForTrip(timeout_ms, 30)) {   // 30 samples debounce
    Serial.println("Homing MIN: timeout. Keeping default limits.");
    stopMotor();
    motor.disable();
    motor.controller = prev;
    rotor_limits_found = false;
    return;
  }

  stopMotor();
  min_rotor_position = motor.shaft_angle;
  Serial.print("Homing: MIN found at ");
  Serial.println(min_rotor_position, 6);

  // ---- Back off from MIN so MAX doesn't instantly re-trip ----
  motor.target = +homing_velocity;  // move away from the stop
  unsigned long t_back = millis();
  while (millis() - t_back < 400) { // 0.4s backoff (adjust if needed)
    motor.loopFOC();
    motor.move();
    _delay(1);
  }
  stopMotor();

  _delay(200);

  // ---- MAX search (positive velocity) ----
  Serial.println("Homing: searching for MAX limit...");
  motor.target = +homing_velocity;

  if (!waitForTrip(timeout_ms, 30)) {
    Serial.println("Homing MAX: timeout. Keeping default limits.");
    stopMotor();
    motor.disable();
    motor.controller = prev;
    rotor_limits_found = false;
    return;
  }

  stopMotor();
  max_rotor_position = motor.shaft_angle;
  Serial.print("Homing: MAX found at ");
  Serial.println(max_rotor_position, 6);

  motor.disable();

  // ---- Enforce ordering (critical for your out-of-bounds check) ----
  if (max_rotor_position < min_rotor_position) {
    float tmp = max_rotor_position;
    max_rotor_position = min_rotor_position;
    min_rotor_position = tmp;
  }

  // Compute 80% span centered
  const float range = (max_rotor_position - min_rotor_position);
  const float margin = 0.5f * (1.0f - sweep_span_ratio) * range;
  sweep_low  = min_rotor_position + margin;
  sweep_high = max_rotor_position - margin;

  rotor_limits_found = true;

  Serial.print("Sweep window: [");
  Serial.print(sweep_low, 6);
  Serial.print(", ");
  Serial.print(sweep_high, 6);
  Serial.println("]");

  // Start sweeping immediately after homing
  motor.controller = MotionControlType::velocity;
  sweep_dir = +1;
  motor.enable();
  motor.target = (float)sweep_dir * sweep_velocity;

#endif
}