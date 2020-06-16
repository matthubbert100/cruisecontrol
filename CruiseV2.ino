#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>
#include <CircularBuffer.h>
#include <EEPROM.h>

// Adjustables
const int rpmPin = 3;             // Pin for measuring RPM
int minimumSpeed = 30;             // Must be travelling above ? mph to enable cruise control
float tyreCirc = 2.08 * 3.14;     // Wheel size, diameter in feet * Pi
int target = 59;                   // Default target speed
int accelInc = 1;                 // Amount the accelerator is increased by
int accelDec = 1;                 // Amount the accelerator is decreased by

// LCD display, special characters, mode names
LiquidCrystal_I2C lcd(0x27,2,1,0,4,5,6,7,3,POSITIVE);
byte up[8] = {0b00100,0b01110,0b11111,0b00100,0b00100,0b00100,0b00100,0b00000};
byte down[8] = {0b00000,0b00100,0b00100,0b00100,0b00100,0b11111,0b01110,0b00100};
byte topLeft[8] = {0b00000,0b00000,0b00000,0b00000,0b00001,0b00010,0b00100,0b00100};
byte topRight[8] = {0b00000,0b00000,0b00000,0b00000,0b10000,0b01000,0b00100,0b00100};
byte btmRight[8] = {0b00100,0b01000,0b10000,0b00000,0b00000,0b00000,0b00000,0b00000};
byte btmLeft[8] = {0b00100,0b00010,0b00001,0b00000,0b00000,0b00000,0b00000,0b00000};
byte pipe[8] = {0b00100,0b00100,0b00100,0b00100,0b00100,0b00100,0b00100,0b00100};
String nameCruise[] = {"Off", "On"};
String nameMode[] = {"Ready", "Creep", "Go", "Main", "Fall", "Stat"};

// Buttons 9:Control  10:Accel  11:Map
int buttonPin[] = {9, 10, 11};
int buttonState[] = {0, 0, 0};
String buttonName[] = {"Control", "Mode", "Map"};
int encoderPinA = 12;
int encoderPinB = 13;
int encoderPinALast = LOW;
int n = LOW;

int encoderPinC = 7; // for manually changing speed
int encoderPinD = 8; // for manually changing speed
int encoderPinCLast = LOW; // for manually changing speed
int k = LOW; // for manually changing speed

// Main variables
int cruise = 0;                   // Flag: Cruise control ON/OFF
int cruisePrev = 0;               // Previous cruise control state
int mode = 5;                     // Default mode is stationary
int modePrev = 5;                 // Previous mode

int targetPrev = 0;               // Previous target speed (to detect changes)
int targetNew = 0;                // Flag: Target has been changed
int targetNewPrev = 0;            // Flag: Previous new target state (to detect changes)

int accel = 0;                    // Accelerator value %
int accelMap = 0;                 // A stored accel value from EEPROM
int accelMapLow = 0;              // Flag: Mapped accel value is for a speed slower than we're currently moving
int accelManual = 0;              // Flag: Manual acceleration mode
int accelFlash = 0;               // Timed switch: Flash 'accel' text on and off, switches twice per second
int accelPrev = 0;                // Previous accelerator value, used to display an increase or decrease

int actual = 0;                   // Actual speed in MPH
int actualPrev = 0;               // Previous speed
int bufferSpeed = 0;              // Speed about to go into buffer
int bufferTotal = 0;              // Total of all speed values in buffer
CircularBuffer<int, 20> buffer;   // Declare speed circular buffer

// RPM
bool passing = false;             // Flag: Magnet is passing sensor
int rpmRaw = 0;                   // Actual value of RPM pin
int rpm = 0;                      // RPM of wheels
unsigned long rpmNow = 0;         // Time at which magnet passed
unsigned long rpmPrev = 0;        // Time at which magnet last passed
unsigned long rpmDiff = 0;        // Time difference between magnet passes

// Timing
unsigned long loopNow = 0;        // Start of loop()
unsigned long loopPrev = 0;       // Time of previous loop
unsigned long flashPrev = 0;      // Time of previous flash (display)

void setup() {
  Serial.begin(9600);
    
  // Set up display
  lcd.begin(20, 4);
  lcd.createChar(1, up);
  lcd.createChar(2, down);
  lcd.createChar(3, topLeft);
  lcd.createChar(4, topRight);
  lcd.createChar(5, btmLeft);
  lcd.createChar(6, btmRight);
  lcd.createChar(7, pipe);
  displayMessage("CRUISE  MASTER", "3OOO");
  displayReset();

  // Button pin pullup
  pinMode(buttonPin[0], INPUT_PULLUP);
  pinMode(buttonPin[1], INPUT_PULLUP);
  pinMode(buttonPin[2], INPUT_PULLUP);
  
  pinMode(3, INPUT_PULLUP);
}

void loop() {
  loopNow = millis();
/*
  // Read RPM, calculate MPH, get average using buffer
  rpmRaw = digitalRead(rpmPin);
  if(rpmRaw == LOW and passing == false){
    // Magnet passing
    //Serial.println("Passing");
    passing = true;
    rpmDiff = loopNow - rpmPrev;
    if(rpmDiff > 10){
      // If difference is >10ms, should be a reliable value
      rpm = (60000/rpmDiff)/2;
      bufferSpeed = ((tyreCirc * rpm) * 60) / 5280;
      buffer.push(bufferSpeed);
      bufferTotal = 0;
      for (int i = 0; i < buffer.size(); i++) {
        bufferTotal += buffer[i];
      }
      actual = bufferTotal / buffer.size(); // Average
      rpmPrev = loopNow;
    }
  }
  if(rpmRaw == HIGH and passing == true){
    // Magnet has passed, reset switch
    passing = false;
  }*/

  // Rotary encoder, by default changes target speed
  n = digitalRead(encoderPinA);
  if((encoderPinALast == LOW) && (n == HIGH)){
    if(digitalRead(encoderPinB) == LOW){
      // If manual acceleration is enabled, change accel value instead
      if(accelManual == 1){
        if(accel > 0) accel--;
      }else{
        if(target > minimumSpeed) target--;
        if(mapRead() != 0 and cruise == 0){
          displayMap(1);
        }else{
          displayMap(0);
        }
      }
    }else{
      // If manual acceleration is enabled, change accel value instead
      if(accelManual == 1){
        if(accel < 99) accel++;
      }else{
        if(target < 80) target++;
        if(mapRead() != 0 and cruise == 0){
          displayMap(1);
        }else{
          displayMap(0);
        }
      }
    }  

    // Whenever target is changed while cruise control is ON, choose appropriate mode
    if(actual > 0 and cruise == 1){
      if(accelManual == 0 and mapRead() != 0){
        // Target speed has a mapped accel value in memory, set mode to Go
        mode = 2;
        if(actual > target) accelMapLow = 1;
        displayIcon(3);
      }else{
        if(target < actual-5 and cruise == 1){
          // If target speed is 6mph lower than actual speed, drop acceleration completely, mode: FALL
          mode = 4;
        }else{
          // Target speed is close or higher, set mode to Maintain
          mode = 3;
        }
      }
    }
    
    // If cruise control is enabled, and target speed is changed, set flag
    if(cruise > 0 and accelManual == 0) targetNew = 1;
    
  }
  encoderPinALast = n;  

  // Second encoder for testing - manually change speed
  k = digitalRead(encoderPinC);
  if((encoderPinCLast == LOW) && (k  == HIGH)){
    if(digitalRead(encoderPinD) == LOW){
      actual--;
    }else{
      actual++;
    }
  }
  encoderPinCLast = k;  

  // Safety checks
  if(loopNow - rpmPrev > 5000 and actual > 0){
    // Are we even moving? Reset speed if no magnet passes for 5 seconds
    //actual = 0; //uncomment after testing
  }
  if(actual == 0 and mode != 5){
    // If speed is 0 and mode is not stationary, set mode to Stationary
    if(cruise == 1){
      displayMessage("OFF", "Stationary");
      cruise = 0;
      resetData();
    }
    mode = 5;
  }
  if(actual > 0 and actual < minimumSpeed-2 and mode != 0){
    // If we're moving but below minimum speed, and mode isn't Ready, set mode to Ready
    if(cruise == 1){
      displayMessage("OFF", "< Minimum Speed");
      cruise = 0;
      resetData();
    }
    mode = 0;
  }
  if(actual > 0 and mode == 5){
    // If speed >0 and mode is stationary, set mode to Ready
    mode = 0;
  }

  // Detect any button presses, set their state so we only detect one press
  for (int i = 0; i < 3; i++){
    if(digitalRead(buttonPin[i]) == LOW and buttonState[i] == 0){
      buttonState[i] = 1;
      buttonPress(i);
    }else if(digitalRead(buttonPin[i]) == HIGH and buttonState[i] == 1){
      buttonState[i] = 0;
    }
  }

  // Timed events below

  // Flash display item twice per second
  if(loopNow - flashPrev > 500){
    if(accelManual == 1){
      if(accelFlash == 0){
        accelFlash = 1;
        displayAccelFlash();
      }else{
        accelFlash = 0;
        displayAccelFlash();
      }
    }
    flashPrev = loopNow;
  }

  // Check speed every 2 seconds, set appropriate mode
    
  if(loopNow - loopPrev > 2000){
    //Serial.println("loop");
    
    if(mode == 1){ // CREEP - Slowly increase accelerator until target speed met
      if(actual >= target-1){
        // If we've met the target speed, set mode to Maintain
        mode = 3;
        targetNew = 0;
      }else if(accel < 100){
        // Target speed not met, increase accel value
        accel = accel+accelInc;
        displayIcon(1);
      }
    }
    
    if(mode == 2){ // GO (Accelerator mapped) - Set accelerator to mapped value, wait for speed to match target
      accel = accelMap;
      if((accelMapLow == 0 and actual >= target-1) or (accelMapLow == 1 and actual <= target+1)){
        // If we've met the target speed, set mode to Maintain
        mode = 3;
        targetNew = 0;
        accelMapLow = 0;
      }
    }
    
    if(mode == 3 and accelManual == 0){ //  MAINTAIN - Adjust accelerator based on actual speed
      if(actual < target-1 and accel < 100){
        // Target speed not met, increase accel value
        accel = accel+accelInc;
        displayIcon(1);
      }else if(actual > target+1 and accel > 0){
        // Target speed exceeded, decrease accel value
        accel = accel-accelDec;
        displayIcon(2);
      }else{
        // Target speed matches actual speed, do nothing, clear accel direction icon
        targetNew = 0;
        displayIcon(3);
      }
    }
    
    if(mode == 4){ // FALL - Remove accelerator completely
      accel = 0;
      if (actual <= target+1){
        // If actual speed has fallen to target speed, set mode to Maintain
        mode = 3;
        targetNew = 0;
      }
    }
    
  loopPrev = loopNow;
  }

  // Before we end the loop, update display if something changed
  if(cruise != cruisePrev){
    cruisePrev = cruise;
    displayCruise();
  }
  if(mode != modePrev){
    modePrev = mode;
    displayMode();
  }
  if(actual != actualPrev){
    actualPrev = actual;
    displayActual();
  }
  if(target != targetPrev){
    targetPrev = target;
    displayTarget();
  }
  if(targetNew != targetNewPrev){
    targetNewPrev = targetNew;
    displayTarget();
  }
  if(accel != accelPrev){
    accelPrev = accel;
    displayAccel();
  }
}

// FUNCTIONS ####################################################################################################################

void resetData(){
  // Used when cruise control gets turned off, sets all values to default
  bufferSpeed = 0;
  bufferTotal = 0;
  for (int i = 0; i < buffer.size(); i++) {
    buffer.push(0); // Empty speed buffer
  }
  actual = 0;
  accel = 0;
  accelMap = 0;
  accelManual = 0;
  accelFlash = 0;
  rpm = 0;
  rpmRaw = 0;
  passing = false;
  rpmNow = 0;
  rpmPrev = 0;
  rpmDiff = 0;
  targetPrev = 0;
  targetNew = 0;
  targetNewPrev = 0;
  displayReset();
}

void control(){
  // Control button was pressed
  if(mode != 5){
      if(cruise == 0){
        // Cruise is off
        if(actual >= minimumSpeed){
          // If we're moving at minimum speed, enable cruise control
          cruise = 1;
          chooseMode();
        }else{
          // Not fast enough, so don't enable cruise control
          displayMessage("OFF", "< Minimum Speed");
        }
      }else{
        // Disable cruise control
        cruise = 0;
        resetData();
        displayMessage("OFF", "");
      }
  }else{
    // Stationary, so don't enable cruise control
    displayMessage("OFF", "Stationary");
  }
  // Any time the control button is pressed, disable manual acceleration mode
  accelManual = 0;
  accelFlash = 0;
}

void chooseMode(){
  // When cruise control is enabled by the button, choose the mode based on current speed and target speed
  if(mapRead() > 0){
    // An accel value was found in memory for the target speed, set mode to Go
    mode = 2;
    displayMessage("ON", "Go!");
    if(actual > target) accelMapLow = 1;
    displayMap(0);
  }else if(actual >= target-1 and actual <= target+5){
    // Actual speed is pretty close to target speed, set mode to Maintain
    targetNew = 0;
    mode = 3;
    displayMessage("ON", "Maintaining...");
  }else if(actual > target+5){
    // Actual speed was more than 5mph below actual speed, set mode to Fall
    mode = 4;
    displayMessage("ON", "Falling...");
  }else{
    // Actual speed is lower than target speed, but there isn't an accel value mapped, set mode to Creep
    mode = 1;
    displayMessage("ON", "Creeping...");
  }
}

void accelSwitch(){
  // Manual accelerator button was pressed, switch between two states
  if(accelManual == 0){
    accelManual = 1;
    accelFlash = 1;
    displayMessage("CAUTION!", "Accelerator");
  }else{
    accelManual = 0;
    accelFlash = 0;
    displayReset();
    delay(1000);
  }
}

int mapRead(){
  // Read memory for target speed
  int readVal = EEPROM.read(target);
  if(readVal > 0 and readVal <= 80){
    // An accel value was found, return it
    accelMap = readVal;
    return readVal;
  }else{
    // No accel value found, return zero
    accelMap = 0;
    return 0;
  }
}

void mapWrite(){
  // Write current accel value to current speed memory
  if(actual <= 80 and actual > 0 and accel > 0 and accel < 100){
    int readVal = EEPROM.read(actual);
    EEPROM.write(actual, accel);
    String messageString = String(actual) + "mph > " + String(accel) + "%";
    if(cruise == 0){
      accel = 0;
    }
    if(readVal > 0 and readVal <= 80){
      // This speed already had an accel value mapped, so we're overwriting
      displayMessage("REMAPPED", messageString);
    }else{
      // No accel value mapped for this speed, so it's a new map
      displayMessage("MAPPED", messageString);
    }
  }
}

// BUTTONS ######################################################################################################################

void buttonPress(int button){
  //Serial.println(buttonName[button]);
  if(button == 0){
    // Control button pressed
    control();
  }
  if(button == 1){
    // Accel button pressed
    accelSwitch();
  }
  if(button == 2){
    // Map button pressed
    mapWrite();
  }
}

// LCD SCREEN ###################################################################################################################

void displayReset(){
  // Create overall display
  lcd.setCursor(0,0);
  lcd.print("CRUISE  SPEED  ACCEL");
  lcd.setCursor(0,1);
  lcd.print("                    ");
  lcd.setCursor(0,2);
  lcd.print("MODE    TARGET      ");
  lcd.setCursor(0,3);
  lcd.print("                    ");
  displayCruise();
  displayActual();
  displayAccel();
  displayMode();
  displayTarget();
  displayIcon(3);
  if(mapRead() != 0){
    displayMap(1);
  }else{
    displayMap(0);
  }
}

void displayCruise(){
  // Update cruise state
  lcd.setCursor(1,1);
  lcd.print(nameCruise[cruise]);
  lcd.print(" ");
}

void displayMode(){
  // Update mode
  lcd.setCursor(1,3);
  lcd.print(nameMode[mode]);
  lcd.print("   ");
}

void displayActual(){
  // Update actual speed
  lcd.setCursor(9,1);
  lcd.print(actual);
  lcd.print("mph  ");
}

void displayTarget(){
  // Update target speed
  lcd.setCursor(9,3);
  lcd.print(target);
  if(targetNew == 0){
    lcd.print("mph ");
  }else{
    lcd.print("mph*");
  }
}

void displayAccel(){
  // Update accel value
  lcd.setCursor(16,1);
  lcd.print(accel);
  lcd.print("% ");
}

void displayAccelFlash(){
  // Flash 'ACCEL' display item twice per second - called by loop()
  lcd.setCursor(15,0);
  if (accelFlash == 0){
    lcd.print("ACCEL");
  }else{
    lcd.print("     ");
  }
}

void displayIcon(int icon){
  // Update accel increase/decrease icon
  lcd.setCursor(17,2);
  if(icon == 3){
    lcd.write(" ");
  }else{
    lcd.write(icon);
  }
}

void displayMap(int x){
  lcd.setCursor(15,3);
  if(x == 1){
    int mV = EEPROM.read(target);
    Serial.println(mV);
    lcd.print("> ");
    lcd.print(mV);
    lcd.print("%");
  }else{
    lcd.write("     ");
  }
}

void displayMessage(String line1, String line2){
  int line1After = (18 - line1.length()) / 2;
  int line1Before = (18 - line1.length()) / 2;
  if(line1Before + line1.length() + line1After < 18){
    line1After++;
  }
  int line2After = (18 - line2.length()) / 2;
  int line2Before = (18 - line2.length()) / 2;
  if(line2Before + line2.length() + line2After < 18){
    line2After++;
  }
  // Display message with a border for 2 seconds
  lcd.setCursor(0,0);
  lcd.write(3);
  lcd.print("------------------");
  lcd.write(4);
  lcd.setCursor(0,1);
  lcd.write(7);
  
  for(int i=0; i<line1Before; i++){
    lcd.print(" ");
  }
  lcd.print(line1);
  for(int i=0; i<line1After; i++){
    lcd.print(" ");
  }
  
  lcd.write(7);
  lcd.setCursor(0,2);
  lcd.write(7);

  for(int i=0; i<line2Before; i++){
    lcd.print(" ");
  }
  lcd.print(line2);
  for(int i=0; i<line2After; i++){
    lcd.print(" ");
  }

  lcd.write(7);
  lcd.setCursor(0,3);
  lcd.write(5);
  lcd.print("------------------");
  lcd.write(6);
  delay(2000);
  displayReset();
}
