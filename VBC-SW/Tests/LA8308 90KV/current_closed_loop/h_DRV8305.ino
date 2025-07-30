//###########################################
//                 DRV8305
//###########################################

// (byte in which to find the 1s, position querying for a 1, binary result of query)
#define FIND_BIT(X, Y, Z)\        
  Z = (X>>Y) & 1;

#define BYTE_CONCATENATE(X, Y, Z)\
  Z = ((X<<8)+Y);


uint16_t drvRead(uint8_t addr) {
  // bit15 = 1 → READ,  bits14:11 = register address
  uint16_t tx = (1u << 15) | (addr << 11);

  digitalWrite(cs, LOW);
  uint16_t rx = SPI.transfer16(tx); // two-byte SPI transfer
  digitalWrite(cs, HIGH);

  return rx & 0x07FF; // strip RW + address bits
}

//Configure DRV8305 to desired operation mode
int drv_init(bool resp){
  Serial.println("DAGOR: DRV8305 INIT");

  //Set to three PWM inputs mode
  digitalWrite(cs, LOW);
  byte resp1 = SPI.transfer(B00111010);
  byte resp2 = SPI.transfer(B10010110);
  digitalWrite(cs, HIGH);
  //Serial.println(resp1, BIN);
  //Serial.println(resp2, BIN);

  if(resp){
    if (resp1 != 0b000010 || resp2 != 0b10010110){
      Serial.println(resp1, BIN);
      Serial.println(resp2, BIN);
      Serial.println("Dagor: DRV cannot init.");
      return DRV_ERROR;
    }
  }
  
  //Clamp sense amplifier output to 3.3V - protect ESP32 adc
  digitalWrite(cs, LOW);
  byte resp3 = SPI.transfer16(0x48A0); // W=0, Addr=0x09, Data=0x0A0
  digitalWrite(cs, HIGH);
  //Serial.println(resp3, BIN);

  if(resp){
    if ( (resp3 & 0x07FF) != 0x0A0){
      Serial.print("REG9 bad = 0x"); 
      Serial.println(resp3 & 0x07FF, HEX);
      Serial.println("Dagor: DRV cannot init.");
      return DRV_ERROR;
    }
  }

  //Set DRV83045's amplifier gain to 80x
  digitalWrite(cs, LOW);
  byte resp7 = SPI.transfer16(0x503F); // W=0, Addr=0xA, Data=0x03F  → 80× gain
  digitalWrite(cs, HIGH);
  //Serial.println(resp7, BIN);

  // //Set DRV83045's amplifier gain to 40x
  // digitalWrite(cs, LOW);
  // byte resp7 = SPI.transfer16(0x502A); // W=0, Addr=0xA, Data=0x02A  → 40× gain 
  // digitalWrite(cs, HIGH);
  // //Serial.println(resp7, BIN);

  // //Set DRV83045's amplifier gain to 20x
  // digitalWrite(cs, LOW);
  // byte resp7 = SPI.transfer16(0x5015); // W=0, Addr=0xA, Data=0x015  → 20× gain
  // digitalWrite(cs, HIGH);
  // //Serial.println(resp7, BIN);

  // Checking for 20x
  if(resp){
    if ( (resp7 & 0x07FF) != 0x015){
      Serial.print  ("REGA bad = 0x"); 
      Serial.println(resp7 & 0x07FF, HEX);
      Serial.println("Dagor: DRV cannot init.");
      return DRV_ERROR;
    }
  }

  drv_enable(true);

  return LIFE_IS_GOOD;
  
}

// Set or reset the enable gate of the DRV8305
void drv_enable(bool enabled){

  if (enabled){
    digitalWrite(enGate, HIGH);
    Serial.println("DRV8305: enGate Enabled");
    _delay(500);
  } else{
    digitalWrite(enGate, LOW);
    Serial.println("DRV8305: enGate Disabled");
    _delay(500);
  }
}

// Use the DRV8305 DC calibration mode, shorts the current control inputs to register 0 amps through the phase shunt resistor
void current_dc_calib(bool activate)
{
  uint16_t regA = drvRead(0x0A);        // keep the current gain & blanking
  if (activate)
      regA |= 0x700;                    // set bits 10-8  (DC_CAL_CH3/2/1)
  else
      regA &= ~0x700;                   // clear them

  digitalWrite(cs, LOW);
  SPI.transfer16(0x5000 | (regA & 0x07FF));   // write back to 0x0A
  digitalWrite(cs, HIGH);
}

// Fault status and manager for the DRV8305 -> Datahseet pages 37 and 38
void drv_fault_status(){
  static bool faultTrig = false;
  //Read nFault pin from DRV8305 - LOW == error / HIGH == normal operation
  int fault = digitalRead(nFault);
  
  if(fault == LOW && faultTrig == false){
    state_machine = DRV_ERROR; // TODO: select which faults are errors and which are warnings.
    int is_error = 0;
    unsigned int faults = 0;

    Serial.println("DRV8305: Fault detected ->");
    faultTrig = true;

    //Check warning and watchdog reset (Address = 0x1)
    digitalWrite(cs, LOW);
    byte ft1 = SPI.transfer(0b10001000);
    byte ft2 = SPI.transfer(0b00000000);
    digitalWrite(cs, HIGH);
    //Serial.println("Address = 0x1");
    //Serial.println(ft1,BIN);
    //Serial.println(ft2,BIN);
    
    BYTE_CONCATENATE(ft2, (ft1 & 0b0000001111111111), faults);

    for(int i = 0; i<=10; i++){

      FIND_BIT(faults, i, is_error);

      if (is_error){
        switch(i){
          case 0:
            Serial.println("- Overtemperature warning");
            break;
          case 1:
            Serial.println("- Temperature flag setting for approximately 135°C");
            break;
          case 2:
            Serial.println("- Temperature flag setting for approximately 125°C");
            break;
          case 3:
            Serial.println("- Temperature flag setting for approximately 105°C");
            break;
          case 4:
            Serial.println("- Charge pump undervoltage flag warning");
            break;
          case 5:
            Serial.println("- Real time OR of all VDS overcurrent monitors");
            break;
          case 6:
            Serial.println("- PVDD overvoltage flag warning");
            break;
          case 7:
            Serial.println("- PVDD undervoltage flag warning");
            break;
          case 8:
            Serial.println("- Temperature flag setting for approximately 175°C");
            break;
          case 9:
            Serial.println("-");
            break;
          case 10:
            Serial.println("- Fault indication");
            break;
        }
      }
    }

    //Check OV/VDS Faults (Address = 0x2)
    digitalWrite(cs, LOW);
    byte ft3 = SPI.transfer(0b10010000);
    byte ft4 = SPI.transfer(0b00000000);
    digitalWrite(cs, HIGH);
    //Serial.println("Address = 0x2");
    //Serial.println(ft3,BIN);
    //Serial.println(ft4,BIN);

    BYTE_CONCATENATE(ft4, (ft3 & 0b0000001111111111), faults);

    for(int i = 0; i<=10; i++){

      FIND_BIT(faults, i, is_error);

      if (is_error){
        switch(i){
          case 0:
            Serial.println("- Sense A overcurrent fault");
            break;
          case 1:
            Serial.println("- Sense B overcurrent fault");
            break;
          case 2:
            Serial.println("- Sense C overcurrent fault");
            break;
          case 3:
            Serial.println("-");
            break;
          case 4:
            Serial.println("-");
            break;
          case 5:
            Serial.println("- VDS overcurrent fault for low-side MOSFET C");
            break;
          case 6:
            Serial.println("- VDS overcurrent fault for high-side MOSFET C");
            break;
          case 7:
            Serial.println("- VDS overcurrent fault for low-side MOSFET B");
            break;
          case 8:
            Serial.println("- VDS overcurrent fault for high-side MOSFET B");
            break;
          case 9:
            Serial.println("- VDS overcurrent fault for low-side MOSFET A");
            break;
          case 10:
            Serial.println("- VDS overcurrent fault for high-side MOSFET A");
            break;
        }
      }
    }

    //Check IC Faults (Address = 0x3)
    digitalWrite(cs, LOW);
    byte ft5 = SPI.transfer(0b10011000);
    byte ft6 = SPI.transfer(0b00000000);
    digitalWrite(cs, HIGH);
    //Serial.println("Address = 0x3");
    //Serial.println(ft5,BIN);
    //Serial.println(ft6,BIN);

    BYTE_CONCATENATE(ft6, (ft5 & 0b0000001111111111), faults);

    for(int i = 0; i<=10; i++){

      FIND_BIT(faults, i, is_error);

      if (is_error){
        switch(i){
          case 0:
            Serial.println("- High-side charge pump overvoltage ABS fault");
            break;
          case 1:
            Serial.println("- High-side charge pump overvoltage fault");
            break;
          case 2:
            Serial.println("- High-side charge pump undervoltage 2 fault");
            break;
          case 3:
            Serial.println("-");
            break;
          case 4:
            Serial.println("- Low-side gate supply fault");
            break;
          case 5:
            Serial.println("- AVDD undervoltage fault");
            break;
          case 6:
            Serial.println("- VREG undervoltage fault");
            break;
          case 7:
            Serial.println("-");
            break;
          case 8:
            Serial.println("- Overtemperature fault");
            break;
          case 9:
            Serial.println("- Watchdog fault");
            break;
          case 10:
            Serial.println("- PVDD undervoltage 2 fault");
            break;
        }
      }
    }

    //Check VGS Faults (Address = 0x4)
    digitalWrite(cs, LOW);
    byte ft7 = SPI.transfer(0b10100000);
    byte ft8 = SPI.transfer(0b00000000);
    digitalWrite(cs, HIGH);
    //Serial.println("Address = 0x4");
    //Serial.println(ft7,BIN);
    //Serial.println(ft8,BIN);

    BYTE_CONCATENATE(ft8, (ft7 & 0b0000001111111111), faults);

    for(int i = 0; i<=10; i++){

      FIND_BIT(faults, i, is_error);

      if (is_error){
        switch(i){
          case 0:
            Serial.println("-");
            break;
          case 1:
            Serial.println("-");
            break;
          case 2:
            Serial.println("-");
            break;
          case 3:
            Serial.println("-");
            break;
          case 4:
            Serial.println("-");
            break;
          case 5:
            Serial.println("- VGS gate drive fault for low-side MOSFET C");
            break;
          case 6:
            Serial.println("- VGS gate drive fault for high-side MOSFET C");
            break;
          case 7:
            Serial.println("- VGS gate drive fault for low-side MOSFET B");
            break;
          case 8:
            Serial.println("- VGS gate drive fault for high-side MOSFET B");
            break;
          case 9:
            Serial.println("- VGS gate drive fault for low-side MOSFET A");
            break;
          case 10:
            Serial.println("- VGS gate drive fault for high-side MOSFET A");
            break;
        }
      }
    }

    if((ft1 & ft2 & ft3 & ft4 & ft5 & ft6 & ft7 & ft8) == 0) Serial.println("- Not responding.");

  }
}
