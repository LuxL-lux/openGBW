#pragma once

#include <SimpleKalmanFilter.h>
#include "HX711.h"

class MenuItem
{
    public:
        int id;
        bool selected;
        char menuName[16];
        double increment;
        double *value;
};

#define STATUS_EMPTY 0
#define STATUS_GRINDING_IN_PROGRESS 1
#define STATUS_GRINDING_FINISHED 2
#define STATUS_GRINDING_FAILED 3
#define STATUS_IN_MENU 4
#define STATUS_IN_SUBMENU 5

#define CUP_WEIGHT 70
#define CUP_DETECTION_TOLERANCE 5 // 5 grams tolerance above or bellow cup weight to detect it

#define LOADCELL_DOUT_PIN 19
#define LOADCELL_SCK_PIN 18

#define LOADCELL_SCALE_FACTOR 7351

#define TARE_MEASURES 20 // use the average of measure for taring
#define SIGNIFICANT_WEIGHT_CHANGE 5 // 5 grams changes are used to detect a significant change
#define COFFEE_DOSE_WEIGHT 18
#define COFFEE_DOSE_OFFSET -2.5
#define MAX_GRINDING_TIME 20000 // 20 seconds diff
#define GRINDING_FAILED_WEIGHT_TO_RESET 150 // force on balance need to be measured to reset grinding

// Parameter validation limits
#define MAX_OFFSET 10.0
#define MIN_OFFSET -10.0
#define MAX_CUP_WEIGHT 200.0
#define MIN_CUP_WEIGHT 10.0
#define MAX_SET_WEIGHT 100.0
#define MIN_SET_WEIGHT 5.0
#define MAX_AUTO_OFFSET_CHANGE 5.0

// Storage settings structure for data integrity
struct ScaleSettings {
  int16_t offsetTenths;
  int16_t cupWeightTenths;
  int16_t setWeightTenths;
  int32_t calibrationHundredths;
  uint8_t scaleMode;
  uint8_t grindMode;
  uint16_t checksum;
};

#define GRINDER_ACTIVE_PIN 33

#define TARE_MIN_INTERVAL 10 * 1000 // auto-tare at most once every 10 seconds

#define ROTARY_ENCODER_A_PIN 32
#define ROTARY_ENCODER_B_PIN 23
#define ROTARY_ENCODER_BUTTON_PIN 34
#define ROTARY_ENCODER_VCC_PIN -1
#define ROTARY_ENCODER_STEPS 4

extern double scaleWeight;
extern unsigned long scaleLastUpdatedAt;
extern unsigned long lastSignificantWeightChangeAt;
extern unsigned long lastTareAt;
extern bool scaleReady;
extern int scaleStatus;
extern double cupWeightEmpty;
extern unsigned long startedGrindingAt;
extern unsigned long finishedGrindingAt;
extern double setWeight;
extern double offset;
extern bool scaleMode;
extern bool grindMode;
extern bool greset;
extern int menuItemsCount;

extern MenuItem menuItems[];
extern int currentMenuItem;
extern int currentSetting;

// Helper functions for safe parameter storage/retrieval
void saveOffset(double newOffset);
void saveCupWeight(double newCupWeight);
void saveSetWeight(double newSetWeight);
void saveCalibration(double newCalibration);
void saveScaleMode(bool mode);
void saveGrindMode(bool mode);
double loadOffset();
double loadCupWeight();
double loadSetWeight();
double loadCalibration();
bool loadScaleMode();
bool loadGrindMode();
bool validateAndLoadSettings();
void saveSettingsStructure();
void resetToDefaults();
uint16_t calculateChecksum(const ScaleSettings& settings);

void setupScale();
