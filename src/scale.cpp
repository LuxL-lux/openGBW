#include "scale.hpp"
#include <MathBuffer.h>
#include <AiEsp32RotaryEncoder.h>
#include <Preferences.h>

HX711 loadcell;
SimpleKalmanFilter kalmanFilter(0.02, 0.02, 0.01);

Preferences preferences;


AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_BUTTON_PIN, ROTARY_ENCODER_VCC_PIN, ROTARY_ENCODER_STEPS);

#define ABS(a) (((a) > 0) ? (a) : ((a) * -1))

TaskHandle_t ScaleTask;
TaskHandle_t ScaleStatusTask;

double scaleWeight = 0; //current weight
double setWeight = 0; //desired amount of coffee
double setCupWeight = 0; //cup weight set by user
double offset = 0; //stop x grams prios to set weight
bool scaleMode = false; //use as regular scale with timer if true
bool grindMode = false;  //false for impulse to start/stop grinding, true for continuous on while grinding
bool grinderActive = false; //needed for continuous mode
MathBuffer<double, 100> weightHistory;

unsigned long scaleLastUpdatedAt = 0;
unsigned long lastSignificantWeightChangeAt = 0;
unsigned long lastTareAt = 0; // if 0, should tare load cell, else represent when it was last tared
bool scaleReady = false;
int scaleStatus = STATUS_EMPTY;
double cupWeightEmpty = 0; //measured actual cup weight
unsigned long startedGrindingAt = 0;
unsigned long finishedGrindingAt = 0;
int encoderDir = 1;
bool greset = false;

bool newOffset = false;
unsigned long lastWeightStableAt = 0;
double lastStableWeight = 0;

int currentMenuItem = 0;
int currentSetting;
int encoderValue = 0;
int menuItemsCount = 7;
MenuItem menuItems[7] = {
    {1, false, "Cup weight", 1, &setCupWeight},
    {2, false, "Calibrate", 0},
    {3, false, "Offset", 0.1, &offset},
    {4, false, "Scale Mode", 0},
    {5, false, "Grinding Mode", 0},
    {6, false, "Exit", 0},
    {7, false, "Reset", 0}}; // structure is mostly useless for now, plan on making menu easier to customize later

// Helper functions for safe parameter storage/retrieval
uint16_t calculateChecksum(const ScaleSettings& settings) {
  return settings.offsetTenths + settings.cupWeightTenths + settings.setWeightTenths + 
         (settings.calibrationHundredths & 0xFFFF) + ((settings.calibrationHundredths >> 16) & 0xFFFF) +
         settings.scaleMode + settings.grindMode + 0xABCD;
}

void saveOffset(double newOffset) {
  // Validate reasonable range
  if (newOffset < MIN_OFFSET || newOffset > MAX_OFFSET) {
    Serial.print("Invalid offset ");
    Serial.print(newOffset);
    Serial.println(", using default");
    newOffset = COFFEE_DOSE_OFFSET;
  }
  
  preferences.begin("scale", false);
  preferences.putShort("offsetTenths", (int16_t)(newOffset * 10));
  preferences.end();
  
  Serial.print("Saved offset: ");
  Serial.println(newOffset);
}

void saveCupWeight(double newCupWeight) {
  if (newCupWeight < MIN_CUP_WEIGHT || newCupWeight > MAX_CUP_WEIGHT) {
    Serial.print("Invalid cup weight ");
    Serial.print(newCupWeight);
    Serial.println(", using default");
    newCupWeight = CUP_WEIGHT;
  }
  
  preferences.begin("scale", false);
  preferences.putShort("cupWeightTenths", (int16_t)(newCupWeight * 10));
  preferences.end();
  
  Serial.print("Saved cup weight: ");
  Serial.println(newCupWeight);
}

void saveSetWeight(double newSetWeight) {
  if (newSetWeight < MIN_SET_WEIGHT || newSetWeight > MAX_SET_WEIGHT) {
    Serial.print("Invalid set weight ");
    Serial.print(newSetWeight);
    Serial.println(", using default");
    newSetWeight = COFFEE_DOSE_WEIGHT;
  }
  
  preferences.begin("scale", false);
  preferences.putShort("setWeightTenths", (int16_t)(newSetWeight * 10));
  preferences.end();
  
  Serial.print("Saved set weight: ");
  Serial.println(newSetWeight);
}

void saveCalibration(double newCalibration) {
  if (newCalibration < 1000 || newCalibration > 50000) {
    Serial.print("Invalid calibration ");
    Serial.print(newCalibration);
    Serial.println(", using default");
    newCalibration = LOADCELL_SCALE_FACTOR;
  }
  
  preferences.begin("scale", false);
  preferences.putInt("calibrationHundredths", (int32_t)(newCalibration * 100));
  preferences.end();
  
  Serial.print("Saved calibration: ");
  Serial.println(newCalibration);
}

void saveScaleMode(bool mode) {
  preferences.begin("scale", false);
  preferences.putBool("scaleMode", mode);
  preferences.end();
}

void saveGrindMode(bool mode) {
  preferences.begin("scale", false);
  preferences.putBool("grindMode", mode);
  preferences.end();
}

double loadOffset() {
  preferences.begin("scale", true);
  int16_t offsetTenths = preferences.getShort("offsetTenths", (int16_t)(COFFEE_DOSE_OFFSET * 10));
  preferences.end();
  
  double offset = offsetTenths / 10.0;
  
  // Validate loaded value
  if (offset < MIN_OFFSET || offset > MAX_OFFSET) {
    Serial.println("Loaded offset out of range, using default");
    offset = COFFEE_DOSE_OFFSET;
    saveOffset(offset); // Save corrected value
  }
  
  return offset;
}

double loadCupWeight() {
  preferences.begin("scale", true);
  int16_t cupWeightTenths = preferences.getShort("cupWeightTenths", (int16_t)(CUP_WEIGHT * 10));
  preferences.end();
  
  double cupWeight = cupWeightTenths / 10.0;
  
  if (cupWeight < MIN_CUP_WEIGHT || cupWeight > MAX_CUP_WEIGHT) {
    Serial.println("Loaded cup weight out of range, using default");
    cupWeight = CUP_WEIGHT;
    saveCupWeight(cupWeight);
  }
  
  return cupWeight;
}

double loadSetWeight() {
  preferences.begin("scale", true);
  int16_t setWeightTenths = preferences.getShort("setWeightTenths", (int16_t)(COFFEE_DOSE_WEIGHT * 10));
  preferences.end();
  
  double setWeight = setWeightTenths / 10.0;
  
  if (setWeight < MIN_SET_WEIGHT || setWeight > MAX_SET_WEIGHT) {
    Serial.println("Loaded set weight out of range, using default");
    setWeight = COFFEE_DOSE_WEIGHT;
    saveSetWeight(setWeight);
  }
  
  return setWeight;
}

double loadCalibration() {
  preferences.begin("scale", true);
  int32_t calibrationHundredths = preferences.getInt("calibrationHundredths", (int32_t)(LOADCELL_SCALE_FACTOR * 100));
  preferences.end();
  
  double calibration = calibrationHundredths / 100.0;
  
  if (calibration < 1000 || calibration > 50000) {
    Serial.println("Loaded calibration out of range, using default");
    calibration = LOADCELL_SCALE_FACTOR;
    saveCalibration(calibration);
  }
  
  return calibration;
}

bool loadScaleMode() {
  preferences.begin("scale", true);
  bool mode = preferences.getBool("scaleMode", false);
  preferences.end();
  return mode;
}

bool loadGrindMode() {
  preferences.begin("scale", true);
  bool mode = preferences.getBool("grindMode", false);
  preferences.end();
  return mode;
}

void resetToDefaults() {
  Serial.println("Resetting all parameters to defaults");
  saveOffset(COFFEE_DOSE_OFFSET);
  saveCupWeight(CUP_WEIGHT);
  saveSetWeight(COFFEE_DOSE_WEIGHT);
  saveCalibration(LOADCELL_SCALE_FACTOR);
  saveScaleMode(false);
  saveGrindMode(false);
}

bool validateAndLoadSettings() {
  ScaleSettings settings;
  
  preferences.begin("scale", true);
  
  // Try to load settings structure
  size_t schLen = preferences.getBytesLength("settings");
  if (schLen == sizeof(ScaleSettings)) {
    preferences.getBytes("settings", &settings, sizeof(ScaleSettings));
    
    // Verify checksum
    uint16_t expectedChecksum = calculateChecksum(settings);
    if (settings.checksum == expectedChecksum) {
      Serial.println("Settings loaded and validated successfully");
      preferences.end();
      return true;
    } else {
      Serial.println("Settings checksum mismatch, using individual parameters");
    }
  } else {
    Serial.println("Settings structure not found or wrong size, using individual parameters");
  }
  
  preferences.end();
  return false;
}

void saveSettingsStructure() {
  ScaleSettings settings;
  
  settings.offsetTenths = (int16_t)(offset * 10);
  settings.cupWeightTenths = (int16_t)(setCupWeight * 10);
  settings.setWeightTenths = (int16_t)(setWeight * 10);
  settings.calibrationHundredths = (int32_t)(loadCalibration() * 100);
  settings.scaleMode = scaleMode ? 1 : 0;
  settings.grindMode = grindMode ? 1 : 0;
  settings.checksum = calculateChecksum(settings);
  
  preferences.begin("scale", false);
  preferences.putBytes("settings", &settings, sizeof(ScaleSettings));
  preferences.end();
  
  Serial.println("Settings structure saved with checksum");
}

void rotary_onButtonClick()
{
  static unsigned long lastTimePressed = 0;
  // ignore multiple press in that time milliseconds
  if (millis() - lastTimePressed < 500)
  {
    return;
  }
  if(scaleStatus == STATUS_EMPTY){
    scaleStatus = STATUS_IN_MENU;
    currentMenuItem = 0;
    rotaryEncoder.setAcceleration(0);
  }
  else if(scaleStatus == STATUS_IN_MENU){
    if(currentMenuItem == 5){
      scaleStatus = STATUS_EMPTY;
      rotaryEncoder.setAcceleration(150);
      Serial.println("Exited Menu");
    }
    else if (currentMenuItem == 2){
      scaleStatus = STATUS_IN_SUBMENU;
      currentSetting = 2;
      Serial.println("Offset Menu");
    }
    else if (currentMenuItem == 0)
    {
      scaleStatus = STATUS_IN_SUBMENU;
      currentSetting = 0;
      Serial.println("Cup Menu");
    }
    else if (currentMenuItem == 1)
    {
      scaleStatus = STATUS_IN_SUBMENU;
      currentSetting = 1;
      Serial.println("Calibration Menu");
    }
    else if (currentMenuItem == 3)
    {
      scaleStatus = STATUS_IN_SUBMENU;
      currentSetting = 3;
      Serial.println("Scale Mode Menu");
    }
    else if (currentMenuItem == 4)
    {
      scaleStatus = STATUS_IN_SUBMENU;
      currentSetting = 4;
      Serial.println("Grind Mode Menu");
    }
    else if (currentMenuItem == 6)
    {
      scaleStatus = STATUS_IN_SUBMENU;
      currentSetting = 6;
      greset = false;
      Serial.println("Reset Menu");
    }
  }
  else if(scaleStatus == STATUS_IN_SUBMENU){
    if(currentSetting == 2){
      saveOffset(offset);
      scaleStatus = STATUS_IN_MENU;
      currentSetting = -1;
    }
    else if (currentSetting == 0)
    {
      if(scaleWeight > 15){       //prevent accidental setting with no cup
        setCupWeight = scaleWeight;
        saveCupWeight(setCupWeight);
        scaleStatus = STATUS_IN_MENU;
        currentSetting = -1;
      }
    }
    else if (currentSetting == 1)
    {
      double currentCalibration = loadCalibration();
      double newCalibrationValue = currentCalibration * (scaleWeight / 100);
      saveCalibration(newCalibrationValue);
      loadcell.set_scale(newCalibrationValue);
      scaleStatus = STATUS_IN_MENU;
      currentSetting = -1;
    }
    else if (currentSetting == 3)
    {
      saveScaleMode(scaleMode);
      scaleStatus = STATUS_IN_MENU;
      currentSetting = -1;
    }
    else if (currentSetting == 4)
    {
      saveGrindMode(grindMode);
      scaleStatus = STATUS_IN_MENU;
      currentSetting = -1;
    }
    else if (currentSetting == 6)
    {
      if(greset){
        resetToDefaults();
        setWeight = COFFEE_DOSE_WEIGHT;
        offset = COFFEE_DOSE_OFFSET;
        setCupWeight = CUP_WEIGHT;
        scaleMode = false;
        grindMode = false;
        loadcell.set_scale(LOADCELL_SCALE_FACTOR);
      }
      
      scaleStatus = STATUS_IN_MENU;
      currentSetting = -1;
    }
  }
}



void rotary_loop()
{
  if (rotaryEncoder.encoderChanged())
  {
    if(scaleStatus == STATUS_EMPTY){
        int newValue = rotaryEncoder.readEncoder();
        Serial.print("Value: ");

        setWeight += ((float)newValue - (float)encoderValue) / 10 * encoderDir;

        encoderValue = newValue;
        Serial.println(newValue);
        saveSetWeight(setWeight);
      }
    else if(scaleStatus == STATUS_IN_MENU){
      int newValue = rotaryEncoder.readEncoder();
      int encoderDiff = newValue - encoderValue;
      
      // Only move one menu item at a time to prevent skipping
      if (encoderDiff > 0) {
        currentMenuItem = (currentMenuItem + 1 * encoderDir) % menuItemsCount;
      } else if (encoderDiff < 0) {
        currentMenuItem = (currentMenuItem - 1 * encoderDir) % menuItemsCount;
      }
      
      currentMenuItem = currentMenuItem < 0 ? menuItemsCount + currentMenuItem : currentMenuItem;
      encoderValue = newValue;
      Serial.println(currentMenuItem);
    }
    else if(scaleStatus == STATUS_IN_SUBMENU){
      if(currentSetting == 2){ //offset menu
        int newValue = rotaryEncoder.readEncoder();
        Serial.print("Value: ");

        offset += ((float)newValue - (float)encoderValue) * encoderDir / 100;
        encoderValue = newValue;

        if(abs(offset) >= setWeight){
          offset = setWeight;     //prevent nonsensical offsets
        }
      }
      else if(currentSetting == 3){
        scaleMode = !scaleMode;
      }
      else if (currentSetting == 4)
      {
        grindMode = !grindMode;
      }
      else if (currentSetting == 6)
      {
        greset = !greset;
      }
    }
  }
  if (rotaryEncoder.isEncoderButtonClicked())
  {
    rotary_onButtonClick();
  }
}

void readEncoderISR()
{
  rotaryEncoder.readEncoder_ISR();
}

void tareScale() {
  Serial.println("Taring scale");
  loadcell.tare(TARE_MEASURES);
  lastTareAt = millis();
}

void updateScale( void * parameter) {
  float lastEstimate;


  for (;;) {
    if (lastTareAt == 0) {
      Serial.println("retaring scale");
      Serial.println("current offset");
      Serial.println(offset);
      tareScale();
    }
    if (loadcell.wait_ready_timeout(300)) {
      lastEstimate = kalmanFilter.updateEstimate(loadcell.get_units(1));
      scaleWeight = lastEstimate;
      scaleLastUpdatedAt = millis();
      weightHistory.push(scaleWeight);
      scaleReady = true;
    } else {
      Serial.println("HX711 not found.");
      scaleReady = false;
    }
  }
}

void grinderToggle()
{
  if(!scaleMode){
    if(grindMode){
      grinderActive = !grinderActive;
      digitalWrite(GRINDER_ACTIVE_PIN, grinderActive);
    }
    else{
      digitalWrite(GRINDER_ACTIVE_PIN, 1);
      delay(100);
      digitalWrite(GRINDER_ACTIVE_PIN, 0);
    }
  }
}

void scaleStatusLoop(void *p) {
  double tenSecAvg;
  for (;;) {
    tenSecAvg = weightHistory.averageSince((int64_t)millis() - 10000);
    

    if (ABS(tenSecAvg - scaleWeight) > SIGNIFICANT_WEIGHT_CHANGE) {
      
      lastSignificantWeightChangeAt = millis();
    }

    if (scaleStatus == STATUS_EMPTY) {
      if (millis() - lastTareAt > TARE_MIN_INTERVAL && ABS(tenSecAvg) > 0.2 && tenSecAvg < 3 && scaleWeight < 3) {
        // tare if: not tared recently, more than 0.2 away from 0, less than 3 grams total (also works for negative weight)
        lastTareAt = 0;
      }

      if (ABS(weightHistory.minSince((int64_t)millis() - 1000) - setCupWeight) < CUP_DETECTION_TOLERANCE &&
          ABS(weightHistory.maxSince((int64_t)millis() - 1000) - setCupWeight) < CUP_DETECTION_TOLERANCE)
      {
        // using average over last 500ms as empty cup weight
        Serial.println("Starting grinding");
        cupWeightEmpty = weightHistory.averageSince((int64_t)millis() - 500);
        scaleStatus = STATUS_GRINDING_IN_PROGRESS;
        
        if(!scaleMode){
          newOffset = true;
          startedGrindingAt = millis();
        }
        
        grinderToggle();
        continue;
      }
    } else if (scaleStatus == STATUS_GRINDING_IN_PROGRESS) {
      if (!scaleReady) {
        
        grinderToggle();
        scaleStatus = STATUS_GRINDING_FAILED;
      }
      //Serial.printf("Scale mode: %d\n", scaleMode);
      //Serial.printf("Started grinding at: %d\n", startedGrindingAt);
      //Serial.printf("Weight: %f\n", cupWeightEmpty - scaleWeight);
      if (scaleMode && startedGrindingAt == 0 && scaleWeight - cupWeightEmpty >= 0.1)
      {
        Serial.printf("Started grinding at: %d\n", millis());
        startedGrindingAt = millis();
        continue;
      }

      if (millis() - startedGrindingAt > MAX_GRINDING_TIME && !scaleMode) {
        Serial.println("Failed because grinding took too long");
        
        grinderToggle();
        scaleStatus = STATUS_GRINDING_FAILED;
        continue;
      }

      if (
          millis() - startedGrindingAt > 2000 &&                                  // started grinding at least 2s ago
          scaleWeight - weightHistory.firstValueOlderThan(millis() - 2000) < 1 && // less than a gram has been grinded in the last 2 second
          !scaleMode)
      {
        Serial.println("Failed because no change in weight was detected");
        
        grinderToggle();
        scaleStatus = STATUS_GRINDING_FAILED;
        continue;
      }

      if (weightHistory.minSince((int64_t)millis() - 200) < cupWeightEmpty - CUP_DETECTION_TOLERANCE && !scaleMode) {
        Serial.printf("Failed because weight too low, min: %f, min value: %f\n", weightHistory.minSince((int64_t)millis() - 200), CUP_WEIGHT + CUP_DETECTION_TOLERANCE);
        
        grinderToggle();
        scaleStatus = STATUS_GRINDING_FAILED;
        continue;
      }
      double currentOffset = offset;
      if(scaleMode){
        currentOffset = 0;
      }
      if (weightHistory.maxSince((int64_t)millis() - 200) >= cupWeightEmpty + setWeight + currentOffset) {
        Serial.println("Finished grinding");
        finishedGrindingAt = millis();
        
        grinderToggle();
        scaleStatus = STATUS_GRINDING_FINISHED;
        continue;
      }
    } else if (scaleStatus == STATUS_GRINDING_FINISHED) {
      double currentWeight = weightHistory.averageSince((int64_t)millis() - 500);
      if (scaleWeight < 5) {
        Serial.println("Going back to empty");
        startedGrindingAt = 0;
        scaleStatus = STATUS_EMPTY;
        // Save settings structure periodically for data integrity
        saveSettingsStructure();
        continue;
      }
      else if (currentWeight != setWeight + cupWeightEmpty && millis() - finishedGrindingAt > 1500 && newOffset)
      {
        // Check if weight has been stable for at least 1 second
        if (ABS(currentWeight - lastStableWeight) < 0.2) {
          if (lastWeightStableAt == 0) {
            lastWeightStableAt = millis();
            lastStableWeight = currentWeight;
          } else if (millis() - lastWeightStableAt > 1000) {
            // Weight has been stable for 1+ seconds, safe to adjust offset
            double weightError = setWeight + cupWeightEmpty - currentWeight;
            
            // Only adjust if error is reasonable (not due to sensor noise or cup removal)
            if (ABS(weightError) <= MAX_AUTO_OFFSET_CHANGE && ABS(weightError) >= 0.2) {
              double proposedOffset = offset + weightError;
              
              // Clamp to reasonable bounds
              if (proposedOffset >= MIN_OFFSET && proposedOffset <= MAX_OFFSET) {
                offset = proposedOffset;
                saveOffset(offset);
                Serial.print("Auto-adjusted offset by ");
                Serial.print(weightError);
                Serial.print("g, new offset: ");
                Serial.println(offset);
              } else {
                Serial.println("Proposed offset out of bounds, skipping auto-adjustment");
              }
            } else {
              Serial.print("Weight error too large for auto-adjustment: ");
              Serial.println(weightError);
            }
            
            newOffset = false;
            lastWeightStableAt = 0;
          }
        } else {
          // Weight not stable, reset stability timer
          lastWeightStableAt = 0;
          lastStableWeight = currentWeight;
        }
      }
    } else if (scaleStatus == STATUS_GRINDING_FAILED) {
      if (scaleWeight >= GRINDING_FAILED_WEIGHT_TO_RESET) {
        Serial.println("Going back to empty");
        scaleStatus = STATUS_EMPTY;
        continue;
      }
    }
    rotary_loop();
    delay(50);
  }
}



void setupScale() {
  rotaryEncoder.begin();
  rotaryEncoder.setup(readEncoderISR);
  // set boundaries and if values should cycle or not
  // in this example we will set possible values between 0 and 1000;
  bool circleValues = true;
  rotaryEncoder.setBoundaries(-10000, 10000, circleValues); // minValue, maxValue, circleValues true|false (when max go to min and vice versa)

  /*Rotary acceleration introduced 25.2.2021.
   * in case range to select is huge, for example - select a value between 0 and 1000 and we want 785
   * without accelerateion you need long time to get to that number
   * Using acceleration, faster you turn, faster will the value raise.
   * For fine tuning slow down.
   */
  // rotaryEncoder.disableAcceleration(); //acceleration is now enabled by default - disable if you dont need it
  rotaryEncoder.setAcceleration(150); // or set the value - larger number = more accelearation; 0 or 1 means disabled acceleration


  loadcell.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

  pinMode(GRINDER_ACTIVE_PIN, OUTPUT);
  digitalWrite(GRINDER_ACTIVE_PIN, 0);

  // Load all parameters using safe helper functions
  double scaleFactor = loadCalibration();
  setWeight = loadSetWeight();
  offset = loadOffset();
  setCupWeight = loadCupWeight();
  scaleMode = loadScaleMode();
  grindMode = loadGrindMode();
  
  Serial.println("Loaded parameters:");
  Serial.print("Calibration: "); Serial.println(scaleFactor);
  Serial.print("Set weight: "); Serial.println(setWeight);
  Serial.print("Offset: "); Serial.println(offset);
  Serial.print("Cup weight: "); Serial.println(setCupWeight);
  Serial.print("Scale mode: "); Serial.println(scaleMode);
  Serial.print("Grind mode: "); Serial.println(grindMode);
  
  loadcell.set_scale(scaleFactor);

  xTaskCreatePinnedToCore(
      updateScale, /* Function to implement the task */
      "Scale",     /* Name of the task */
      10000,       /* Stack size in words */
      NULL,        /* Task input parameter */
      0,           /* Priority of the task */
      &ScaleTask,  /* Task handle. */
      1);          /* Core where the task should run */

  xTaskCreatePinnedToCore(
      scaleStatusLoop, /* Function to implement the task */
      "ScaleStatus", /* Name of the task */
      10000,  /* Stack size in words */
      NULL,  /* Task input parameter */
      0,  /* Priority of the task */
      &ScaleStatusTask,  /* Task handle. */
      1);            /* Core where the task should run */
}
