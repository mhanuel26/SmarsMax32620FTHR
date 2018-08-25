#include <MAX77650-Arduino-Library.h>
#include <MAX17055.h>

//Definitions
#define DEBUG
#ifdef DEBUG
#define MAX77650_debug true
#else
#define MAX77650_debug false
#endif
#define MAX77650_PHLD P2_2   //Pin 18 -> connected to MAX77650 power hold input pin (A1)
#define MAX77650_IRQpin P2_3 //Pin 19 -> connected to MAX77650 IRQ output pin (C2)

#define FTHR_BUTTON   P2_7

#define LEFT_MOTOR_PWM P5_6     // D10
#define RIGHT_MOTOR_PWM P5_5    // D11
#define LEFT_MOTOR_DIR  P5_4    // D12
#define RIGHT_MOTOR_DIR P5_3    // D13

const int trigPin = P3_2;
const int echoPin = P3_3;

long duration;
int distancecm;
boolean interrupt = false;
boolean nEN_Int = false;
volatile boolean btn_int = false;
volatile boolean CHGIN_Flag = false;

const long interval = 50;
unsigned long previousMillis = 0;
// max17055 battery
const long BattMeasInterval = 5000;     // measure battery every 5 seconds
unsigned long SensPrevMillis = 0;

const long CheckChargerStateInterval = 500;
unsigned long CheckChargerState = 0;
boolean CHG_IN_OK = false;

MAX17055 sensor;

void setup() {
  Serial.begin(9600);
  Serial2.begin(9600);
  //Configure the Power-Management (Power-Hold)

  pinMode(MAX77650_PHLD, OUTPUT);          //configure pin as output
  digitalWrite(MAX77650_PHLD, HIGH);       //set output to HIGH to hold the power-on state

  MAX77650_init();
  Init_Max77650();

  pinMode(LEFT_MOTOR_PWM, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);
  pinMode(LEFT_MOTOR_DIR, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR, OUTPUT);
  pinMode(trigPin, OUTPUT);     // Sets the trigPin as an Output
  pinMode(echoPin, INPUT);      // Sets the echoPin as an Input

  pinMode(FTHR_BUTTON, INPUT_PULLUP);
  attachInterrupt(FTHR_BUTTON, BUTTON_IRQ, FALLING);

  if(MAX77650_getCHGIN_DTLS() == 0x3){
    CHG_IN_OK = true;
  }else{
    CHG_IN_OK = false;
  }

  //  sensor.setResistSensor(0.05);
  while (!initSensor()) {
    #ifdef DEBUG
    Serial.println("Problem Init MAX17055 - attempting ");
    #endif
    delay(1000);
  }
  delay(500);
  uint16_t VEmptyValue = sensor.getVEmptyReg();
  #ifdef DEBUG
  Serial.print("VEmpty Register:"); Serial.println(VEmptyValue, HEX); Serial.println((VEmptyValue >> 7)&0x1FF, DEC); 
  #endif
  
  Serial2.println("Setup Done");

#ifdef DEBUG
  Serial.println("Setup Done");
#endif
}

boolean initSensor() {
  if (sensor.getStatusPOR() == 0) {
    float capacity = sensor.getReportedCapacity();
    Serial.print("Capacity of plugged in battery is: ");
    Serial.print(capacity, 4);
    Serial.println(" mAH \n\n");
    float SOC = sensor.getSOC();
    Serial.print("State of Charge value is: %");
    Serial.println(SOC, 4);
    Serial.println("\n\n");
    return true;
  } else {
    while (sensor.DataNotReady()) {
      delay(10);            // do not continue until FSTAT.DNR == 0
    }
    sensor.EzConfig(2000, 300 * 0.15);
    sensor.clearPORbit();
    if (sensor.getStatusPOR() == 1) {
      return false;
    }
    return true;
  }
}

void loop() {
  if(!CHG_IN_OK){         // If charger is NOT connected we can control the motors
    distancecm = mdistance();
    analogWrite(LEFT_MOTOR_PWM, 230);   // 191 for 75% Duty Cycle
    analogWrite(RIGHT_MOTOR_PWM, 230);
  
    if(distancecm < 5){
      digitalWrite(RIGHT_MOTOR_DIR,HIGH);
      digitalWrite(LEFT_MOTOR_DIR,HIGH);
      delay(1000);
      digitalWrite(RIGHT_MOTOR_DIR,HIGH);
      digitalWrite(LEFT_MOTOR_DIR,LOW);
      delay(1000);
    }
    else{
      digitalWrite(RIGHT_MOTOR_DIR,LOW);
      digitalWrite(LEFT_MOTOR_DIR,LOW);
    }
  }

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;
    MAX77650_setBRT_LED0(random(0, 32));
    MAX77650_setBRT_LED1(random(0, 32));
    MAX77650_setBRT_LED2(random(0, 32));
    if (interrupt) check_IRQ_src();
    if (nEN_Int) {
      nEN_Int = false;
      PowerOff();
      while (1);
    }
    if (btn_int) {
      btn_int = false;
#ifdef DEBUG
      Serial.println("Button Press");
#endif
    }
  }
  if (currentMillis - SensPrevMillis >= BattMeasInterval) {
    SensPrevMillis = currentMillis;
    ReadSensorValues();
  }
  if ((currentMillis - CheckChargerState >= CheckChargerStateInterval) && CHGIN_Flag) {  
    CHGIN_Flag = false;
    if(MAX77650_getCHGIN_DTLS() == 0x3){
      CHG_IN_OK = true;
      if (MAX77650_debug) Serial.println("Charger Input is OK");
    }else{
      CHG_IN_OK = false;
      if (MAX77650_debug) Serial.println("Charger Input Not OK");
    }
  }
}

void ReadSensorValues() {
#ifdef DEBUG
  Serial.println("****** Battery Telemetry ******");
  Serial.println("Get Reported Capacity");
  float capacity = sensor.getReportedCapacity();
  Serial.print("Capacity of plugged in battery is: ");
  Serial.print(capacity, 4);
  Serial.println(" mAH \n\n");

  Serial.println("Get Resist Sensor");
  float resistSensorValue = sensor.getResistSensor();
  Serial.print("Resist sensor value is: ");
  Serial.print(resistSensorValue, 4);
  Serial.println(" ohm \n\n");

  Serial.println("Get State of Charge");
  float SOC = sensor.getSOC();
  Serial.print("State of Charge value is: %");
  Serial.println(SOC, 4);
  Serial.println("\n\n");

  Serial.println("Get instantaneous voltage");
  float voltage = sensor.getInstantaneousVoltage();
  Serial.print("Instantaneous Voltage is: ");
  Serial.print(voltage, 4);
  Serial.println(" V \n\n");

  Serial.println("Get instantaneous current");
  float current = sensor.getInstantaneousCurrent();
  Serial.print("Instantaneous Current is: ");
  Serial.print(current, 4);
  Serial.println(" mA \n\n");

  Serial.println("Get Average current");
  float AvgCurrent = sensor.getAverageCurrent();
  Serial.print("Average Current is: ");
  Serial.print(AvgCurrent, 4);
  Serial.println(" mA \n\n");

  Serial.println("Get time to empty");
  float TTE = sensor.getTimeToEmpty();
  Serial.print("Time to empty is: ");
  Serial.print(TTE, 4);
  Serial.println(" Hours \n\n");
#endif
}

void PowerOff() {
  // stop motors
  analogWrite(LEFT_MOTOR_PWM, 0);   // 0% Duty Cycle
  analogWrite(RIGHT_MOTOR_PWM, 0);
#ifdef DEBUG
  Serial.println("shuting down");
#endif
  delay(500);
  digitalWrite(MAX77650_PHLD, LOW);       //set output to LOW to shutdown
}

int mdistance()
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  int duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  int distance = duration * 0.034 / 2;
  return distance;
}

void Init_Max77650() {

  //Baseline Initialization following rules printed in MAX77650 Programmres Guide Chapter 4 Page 5
  if (MAX77650_debug) Serial.print("Set Main Bias to normal Mode: ");
    if (MAX77650_setSBIA_LPM(false)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed");    //Set Main Bias to normal Mode
  if (MAX77650_debug) Serial.print("Set On/Off-Button to push-button-mode: ");
    if (MAX77650_setnEN_MODE(false)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed");    //set on/off-button
  if (MAX77650_debug) Serial.print("Set nEN input debounce time to 30ms: ");
    if (MAX77650_setDBEN_nEN(true)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed");    //set on/off-button to push-button
  if (MAX77650_debug) Serial.print("Comparing part-numbers: ");
    if (MAX77650_getDIDM() == PMIC_partnumber) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed");     //checking partnumbers
  if (MAX77650_debug) Serial.print("Checking OTP options: ");
    if (MAX77650_getCID() != MAX77650_CID) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed");     //checking OTP options
  //Values for NTC beta=3800K; Battery-values are for 1s 303759 with 600mAh
  if (MAX77650_debug) Serial.print("Set the VCOLD JEITA Temperature Threshold to 0°C: ");
    if (MAX77650_setTHM_COLD(2)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed");   //0°C
  if (MAX77650_debug) Serial.print("Set the VCOOL JEITA Temperature Threshold to 15°C: ");
    if (MAX77650_setTHM_COOL(3)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed");   //15°C
  if (MAX77650_debug) Serial.print("Set the VWARM JEITA Temperature Threshold to 45°C: ");
    if (MAX77650_setTHM_WARM(2)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed");   //45°C
  if (MAX77650_debug) Serial.print("Set the VHOT JEITA Temperature Threshold to 60°C: ");
    if (MAX77650_setTHM_HOT(3)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed");    //60°C
  if (MAX77650_debug) Serial.print("Set CHGIN regulation voltage to 4.00V: ");
    if (MAX77650_setVCHGIN_MIN(0)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed"); //
  if (MAX77650_debug) Serial.print("Set CHGIN Input Current Limit to 300mA: ");
    if (MAX77650_setICHGIN_LIM(0)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed"); //475mA
  if (MAX77650_debug) Serial.print("Set the prequalification charge current to 10%: ");
    if (MAX77650_setI_PQ(false)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed");   //10%
  if (MAX77650_debug) Serial.print("Set Battery prequalification voltage threshold to 3.0V: ");
    if (MAX77650_setCHG_PQ(7)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed");        //3.0V
  if (MAX77650_debug) Serial.print("Set Charger Termination Current to 15% of of fast charge current: ");
    if (MAX77650_setI_TERM(3)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed");        //15%
  if (MAX77650_debug) Serial.print("Set Topoff timer value to 0 minutes: ");
    if (MAX77650_setT_TOPOFF(0)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed");      //0 minutes
  if (MAX77650_debug) Serial.print("Set the die junction temperature regulation point to 60°C: ");
    if (MAX77650_setTJ_REG(0)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed");        //60°C Tj
  if (MAX77650_debug) Serial.print("Set System voltage regulation to 4.50V: ");
    if (MAX77650_setVSYS_REG(0x10)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed");   //Vsys=4.5V
  if (MAX77650_debug) Serial.print("Set the fast-charge constant current value to 300mA: ");
    if (MAX77650_setCHG_CC(0x3f)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed");     //300mA
  if (MAX77650_debug) Serial.print("Set the fast-charge safety timer to 3h: ");
    if (MAX77650_setT_FAST_CHG(2)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed");    //5h
  if (MAX77650_debug) Serial.print("Set IFAST-CHG_JEITA to 300mA: ");
    if (MAX77650_setCHG_CC_JEITA(0x3f)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed"); //300mA
  if (MAX77650_debug) Serial.print("Set Thermistor enable bit: ");
    if (MAX77650_setTHM_EN(true)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed");     //enable the thermistor monitoring
  if (MAX77650_debug) Serial.print("Set fast-charge battery regulation voltage to 4.20V: ");
    if (MAX77650_setCHG_CV(0x18)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed");     //4.20V
  if (MAX77650_debug) Serial.print("Set USB not in power down: ");
    if (MAX77650_setUSBS(false)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed");      //USBS not suspended
  if (MAX77650_debug) Serial.print("Set the modified VFAST-CHG to 4.00V: ");
    if (MAX77650_setCHG_CV_JEITA(0x10)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed"); //4.0V
  if (MAX77650_debug) Serial.print("Selects the battery discharge current full-scale current value to 300mA: ");
    if (MAX77650_setIMON_DISCHG_SCALE(0x0A)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed"); //300mA
  if (MAX77650_debug) Serial.print("Disable the analog MUX output: ");
    if (MAX77650_setMUX_SEL(0)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed");    //AMUX=off
  if (MAX77650_debug) Serial.print("Set the Charger to Enable: ");
    if (MAX77650_setCHG_EN(true)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed");  //enable the Charger
  if (MAX77650_debug) Serial.print("Disable SIMO Buck-Boost Channel 0 Active-Discharge: ");
    if (MAX77650_setADE_SBB0(false)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed");
  if (MAX77650_debug) Serial.print("Disable SIMO Buck-Boost Channel 1 Active-Discharge: ");
    if (MAX77650_setADE_SBB1(false)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed");
  if (MAX77650_debug) Serial.print("Disable SIMO Buck-Boost Channel 2 Active-Discharge: ");
    if (MAX77650_setADE_SBB1(false)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed");
  if (MAX77650_debug) Serial.print("Set SIMO Buck-Boost to maximum drive strength: ");
    if (MAX77650_setDRV_SBB(0b00)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed");
  if (MAX77650_debug) Serial.print("Set SIMO Buck-Boost Channel 0 Peak Current Limit to 500mA: ");
    if (MAX77650_setIP_SBB0(0b00)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed");
  if (MAX77650_debug) Serial.print("Set SIMO Buck-Boost Channel 1 Peak Current Limit to 500mA: ");
    if (MAX77650_setIP_SBB1(0b00)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed");
  if (MAX77650_debug) Serial.print("Set SIMO Buck-Boost Channel 2 Peak Current Limit to 500mA: ");
    if (MAX77650_setIP_SBB2(0b00)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed");
  if (MAX77650_debug) Serial.print("Set SIMO Buck-Boost Channel 2 to on while in stand-by-mode: ");
    if (MAX77650_setEN_SBB2(0b110)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed");
  if (MAX77650_debug) Serial.print("Initialize Global Interrupt Mask Register: ");
    if (MAX77650_setINT_M_GLBL(0x0)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed");
  if (MAX77650_debug) Serial.print("Initialize Charger Interrupt Mask Register: ");
    if (MAX77650_setINT_M_CHG(0x0)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed");

  //Initialization of PMIC-LEDs
  MAX77650_setLED_FS0(0b01);
  MAX77650_setINV_LED0(false);    //LED red: phase operation
  MAX77650_setBRT_LED0(0b00000);  //LED red: brightness
  MAX77650_setP_LED0(0b1111);     //LED red: LED period
  MAX77650_setD_LED0(0b1111);     //LED red: LED duty-cycle
  MAX77650_setLED_FS1(0b01);
  MAX77650_setINV_LED1(false);    //LED green: phase operation
  MAX77650_setBRT_LED1(0b00000);  //LED green: brightness
  MAX77650_setP_LED1(0b1111);     //LED green: LED period
  MAX77650_setD_LED1(0b1111);     //LED green: LED duty-cycle
  MAX77650_setLED_FS2(0b01);
  MAX77650_setINV_LED2(false);    //LED blue: phase operation
  MAX77650_setBRT_LED2(0b00000);  //LED blue: brightness
  MAX77650_setP_LED2(0b1111);     //LED blue: LED period
  MAX77650_setD_LED2(0b1111);     //LED blue: LED duty-cycle
  MAX77650_setEN_LED_MSTR(true);  //LEDs Master enable

  //MAX77650 Interrupt wiring
  pinMode(MAX77650_IRQpin, INPUT_PULLUP);
  attachInterrupt(MAX77650_IRQpin, MAX77650_IRQ, FALLING);

  //Read and clear Interrupt Registers
  MAX77650_getINT_GLBL();
  MAX77650_getINT_CHG();
  MAX77650_getERCFLAG();

  if (MAX77650_debug) Serial.println("End Initialisation of MAX77650");
}

void check_IRQ_src(void) {
  byte INT_GLBL = MAX77650_getINT_GLBL();
  byte INT_CHG = MAX77650_getINT_CHG();
  byte ERCFLAG = MAX77650_getERCFLAG();
  if (MAX77650_debug) Serial.println("MAX77650 Interrupts: ");
  //ERCFLAG related Interrupts 0x04
  if (((INT_GLBL) >> 6) & 0b00000001) Serial.println("LDO Dropout Detector Rising Interrupt");    //Returns whether Reset Source was PWR_HLD; Return Value: 0=no event; 1=RESET Source: PWR_HLD
  if (((INT_GLBL) >> 5) & 0b00000001) Serial.println("Thermal Alarm 2 Rising Interrupt");     //Returns whether Reset Source was CRST_F; Return Value: 0=no event; 1=RESET Source: Software Cold Reset
  if (((INT_GLBL) >> 4) & 0b00000001) Serial.println("Thermal Alarm 1 Rising Interrupt");    //Returns whether Reset Source was OFF_F; Return Value: 0=no event; 1=RESET Source: Software Off Flag
  if (((INT_GLBL) >> 3) & 0b00000001) Serial.println("nEN Rising Interrupt");    //Returns whether Reset Source was MRST; Return Value: 0=no event; 1=RESET Source: Manual Reset Timer
  if (((INT_GLBL) >> 2) & 0b00000001) {
    nEN_Int = true;
    if (MAX77650_debug) Serial.println("nEN Falling Interrupt");   //Returns whether Reset Source was SYSUVLO; Return Value: 0=no event; 1=RESET Source: System Undervoltage Lockout
  }
  if (((INT_GLBL) >> 1) & 0b00000001) if (MAX77650_debug) Serial.println("GPI Rising Interrupt");   //Returns whether Reset Source was SYSOVLO; Return Value: 0=no event; 1=RESET Source: System Overvoltage Lockout
  if ((INT_GLBL)  & 0b00000001) if (MAX77650_debug) Serial.println("GPI Falling Interrupt");      //Returns whether Reset Source was TOVLD; Return Value: 0=no event; 1=RESET Source: Thermal Overload
  //Charger related Interrupts 0x01
  if (((INT_CHG) >> 6) & 0b00000001) if (MAX77650_debug) Serial.println("System voltage configuration error interrupt");    //Returns whether Reset Source was PWR_HLD; Return Value: 0=no event; 1=RESET Source: PWR_HLD
  if (((INT_CHG) >> 5) & 0b00000001) if (MAX77650_debug) Serial.println("Minimum System Voltage Regulation-loop related interrup");     //Returns whether Reset Source was CRST_F; Return Value: 0=no event; 1=RESET Source: Software Cold Reset
  if (((INT_CHG) >> 4) & 0b00000001) if (MAX77650_debug) Serial.println("CHGIN control-loop related interrupt");    //Returns whether Reset Source was OFF_F; Return Value: 0=no event; 1=RESET Source: Software Off Flag
  if (((INT_CHG) >> 3) & 0b00000001) if (MAX77650_debug) Serial.println("Die junction temperature regulation interrupt");    //Returns whether Reset Source was MRST; Return Value: 0=no event; 1=RESET Source: Manual Reset Timer
  if (((INT_CHG) >> 2) & 0b00000001) {
    if (MAX77650_debug) Serial.println("CHGIN related interrupt");   //Returns whether Reset Source was SYSUVLO; Return Value: 0=no event; 1=RESET Source: System Undervoltage Lockout
    CHGIN_Flag = true;
    CheckChargerState =  millis();
  }
  if (((INT_CHG) >> 1) & 0b00000001) {
    if (MAX77650_debug) Serial.println("Charger related interrupt");   //Returns whether Reset Source was SYSOVLO; Return Value: 0=no event; 1=RESET Source: System Overvoltage Lockout
  }
  if ((INT_CHG)  & 0b00000001) if (MAX77650_debug) Serial.println("Thermistor related interrupt");      //Returns whether Reset Source was TOVLD; Return Value: 0=no event; 1=RESET Source: Thermal Overload
  //ERCFLAG related Interrupts 0x04
  if (((ERCFLAG) >> 6) & 0b00000001) if (MAX77650_debug) Serial.println("PWR_HLD Reset");    //Returns whether Reset Source was PWR_HLD; Return Value: 0=no event; 1=RESET Source: PWR_HLD
  if (((ERCFLAG) >> 5) & 0b00000001) if (MAX77650_debug) Serial.println("Software Cold Reset");     //Returns whether Reset Source was CRST_F; Return Value: 0=no event; 1=RESET Source: Software Cold Reset
  if (((ERCFLAG) >> 4) & 0b00000001) if (MAX77650_debug) Serial.println("Software Off Flag");    //Returns whether Reset Source was OFF_F; Return Value: 0=no event; 1=RESET Source: Software Off Flag
  if (((ERCFLAG) >> 3) & 0b00000001) if (MAX77650_debug) Serial.println("Manual Reset Timer");    //Returns whether Reset Source was MRST; Return Value: 0=no event; 1=RESET Source: Manual Reset Timer
  if (((ERCFLAG) >> 2) & 0b00000001) if (MAX77650_debug) Serial.println("SYS Domain Undervoltage Lockout");   //Returns whether Reset Source was SYSUVLO; Return Value: 0=no event; 1=RESET Source: System Undervoltage Lockout
  if (((ERCFLAG) >> 1) & 0b00000001) if (MAX77650_debug) Serial.println("SYS Domain Overvoltage Lockout");   //Returns whether Reset Source was SYSOVLO; Return Value: 0=no event; 1=RESET Source: System Overvoltage Lockout
  if ((ERCFLAG)  & 0b00000001) if (MAX77650_debug) Serial.println("Thermal Overload");      //Returns whether Reset Source was TOVLD; Return Value: 0=no event; 1=RESET Source: Thermal Overload
  interrupt = false;
}

void MAX77650_IRQ(void) {
  interrupt = true;
}

void BUTTON_IRQ(void) {
  btn_int = true;
}

