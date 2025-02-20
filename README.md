 
#include <Wire.h>
 
#include <LiquidCrystal_I2C.h>  // Updated LCD library
 
#include <EEPROM.h>
 
#include "driver/mcpwm.h"
 
#include "soc/mcpwm_reg.h"
 
#include "soc/mcpwm_struct.h"
 
#include "esp_timer.h"
 
#include <math.h>
 
 
// Pin Definitions
 
#define PWM_UH 25
 
#define PWM_UL 26
 
#define PWM_VH 27
 
#define PWM_VL 14
 
#define PWM_WH 12
 
#define PWM_WL 13
 
#define POT_PIN 34            // Potentiometer for frequency control
 
#define ON_OFF_PIN 35         // External ON/OFF switch
 
#define FAULT_PIN 33          // Fault input pin
 
#define DC_VOLTAGE_PIN 36     // DC voltage sensor pin
 
#define CURRENT_SENSOR_PIN 39 // ACS712 current sensor (for MPPT)
 
 
// Button Pin Definitions
 
#define BTN_UP    32
 
#define BTN_DOWN   4
 
#define BTN_BACK  16
 
#define BTN_ENTER 17
 
#define BTN_OK    18
 
 
// System Parameters
 
#define MIN_FREQUENCY    1.0    // Hz
 
#define MAX_FREQUENCY   50.0    // Hz (at 600V DC)
 
#define MIN_DC_VOLTAGE 100.0    // V
 
#define MAX_DC_VOLTAGE 600.0    // V
 
#define DEAD_TIME_US    4.0     // microseconds (4µs)
 
#define CARRIER_FREQ    3000    // Carrier frequency: 3kHz
 
#define TORQUE_BOOST_FREQ 10.0  // Hz – below this, boost duty cycle
 
#define TORQUE_BOOST_FACTOR 1.5
 
#define RAMP_RATE       1.0    // Hz per second (ramp up/down)
 
#define ACS712_SENSITIVITY 0.185 // V/A for ACS712 20A model
 
#define MPPT_STEP 0.5         // Step change in frequency for MPPT
 
 
// 8-bit Sine Lookup Table (256 values)
 
const uint8_t sineTable[256] = {
 
  128, 131, 134, 137, 140, 143, 146, 149, 152, 155, 158, 161, 164, 167, 170, 173,
 
  176, 179, 182, 185, 188, 191, 194, 197, 200, 203, 206, 209, 212, 215, 218, 221,
 
  224, 227, 230, 233, 236, 239, 242, 245, 248, 251, 254, 255, 254, 251, 248, 245,
 
  242, 239, 236, 233, 230, 227, 224, 221, 218, 215, 212, 209, 206, 203, 200, 197,
 
  194, 191, 188, 185, 182, 179, 176, 173, 170, 167, 164, 161, 158, 155, 152, 149,
 
  146, 143, 140, 137, 134, 131, 128, 125, 122, 119, 116, 113, 110, 107, 104, 101,
 
  98, 95, 92, 89, 86, 83, 80, 77, 74, 71, 68, 65, 62, 59, 56, 53,
 
  50, 47, 44, 41, 38, 35, 32, 29, 26, 23, 20, 17, 14, 11, 8, 5,
 
  2, 0, 2, 5, 8, 11, 14, 17, 20, 23, 26, 29, 32, 35, 38, 41,
 
  44, 47, 50, 53, 56, 59, 62, 65, 68, 71, 74, 77, 80, 83, 86, 89,
 
  92, 95, 98, 101, 104, 107, 110, 113, 116, 119, 122, 125, 128, 131, 134, 137,
 
  140, 143, 146, 149, 152, 155, 158, 161, 164, 167, 170, 173, 176, 179, 182, 185,
 
  188, 191, 194, 197, 200, 203, 206, 209, 212, 215, 218, 221, 224, 227, 230, 233,
 
  236, 239, 242, 245, 248, 251, 254, 255, 254, 251, 248, 245, 242, 239, 236, 233,
 
  230, 227, 224, 221, 218, 215, 212, 209, 206, 203, 200, 197, 194, 191, 188, 185,
 
  182, 179, 176, 173, 170, 167, 164, 161, 158, 155, 152, 149, 146, 143, 140, 137
 
};
 
 
// Menu Structure (each row now has 8 elements)
 
String menuItems[8][8] = {
 
  {"1. Parameters", "2. Motor Ctrl", "3. Display", "4. Comm", "5. Alarms", "6. Settings", "7. Advanced", "8. Exit"},
 
  {"1.1 Freq Set", "1.2 Voltage", "1.3 Accel", "1.4 Decel", "1.5 DeadTime", "1.6 Save", "1.7 Load", "Back"},
 
  {"2.1 Start", "2.2 Stop", "2.3 Mode", "2.4 Status", "2.5 Reset", "Back", "", ""},
 
  {"3.1 LCD Bright", "3.2 Contrast", "Back", "", "", "", "", ""},
 
  {"4.1 Modbus Addr", "4.2 Baud Rate", "Back", "", "", "", "", ""},
 
  {"5.1 Last Alarm", "5.2 Reset", "Back", "", "", "", "", ""},
 
  {"6.1 Factory Reset", "6.2 Language", "Back", "", "", "", "", ""},
 
  {"Exit", "", "", "", "", "", "", ""}
 
};
 
 
// LCD Initialization
 
LiquidCrystal_I2C lcd(0x27, 16, 2);
 
 
// Global Variables
 
int menuLevel = 0;  // Current menu depth level
 
int menuIndex = 0;  // Current item index within the current menu level
 
float target_frequency = 0.0;
 
float current_frequency = 0.0;
 
float dc_voltage = 0.0;
 
float prev_power = 0.0;
 
float prev_voltage = 0.0;
 
float solar_power = 0.0;
 
float solar_current = 0.0;
 
bool fault_latched = false;
 
bool motorRunning = false;  // Controlled by menu and/or external ON/OFF switch
 
bool mpptMode = true;       // Enable MPPT mode if true
 
unsigned long last_ramp_time = 0;
 
 
// For parameter adjustment via menu
 
bool adjustingParam = false;
 
float parameterValues[8][8] = {
 
  {0}, // Main menu (unused)
 
  {50.0, 400.0, 5.0, 5.0, 4.0}, // Parameters (1.1 - 1.5); remaining cells default to 0
 
  {0}, // Motor Ctrl
 
  {128, 50}, // Display (brightness, contrast)
 
  {1, 9600}, // Communication (Modbus address, baud rate)
 
  {0}, // Alarms
 
  {0}, // Settings
 
  {0}  // Advanced
 
};
 
 
// Function Prototypes
 
void setupMCPWM();
 
void updateSPWM(float angle, float frequency);
 
void stopPWM();
 
void handleFault();
 
void updateDisplay();
 
void handleMenu();
 
void handleMPPT();
 
void handleMenuAction();
 
void handleParameterAdjust();
 
float getAdjustmentStep(int level, int index);
 
void saveParametersToEEPROM();
 
void loadParametersFromEEPROM();
 
void factoryReset();
 
 
//-------------------------------------------------------
 
// Menu Handling Functions
 
//-------------------------------------------------------
 
 
// Main menu handler with debouncing and parameter adjustment support.
 
void handleMenu() {
 
  static unsigned long lastButtonPress = 0;
 
  if (millis() - lastButtonPress < 200) return; // Debounce
 
 
  if (adjustingParam) {
 
    handleParameterAdjust();
 
    return;
 
  }
 
 
  // Determine max valid index for current menu level
 
  int maxIndex = 7;
 
  while (maxIndex >= 0 && menuItems[menuLevel][maxIndex] == "") maxIndex--;
 
 
  // Display current menu item
 
  lcd.setCursor(0, 0);
 
  lcd.print("                ");  // Clear line
 
  lcd.setCursor(0, 0);
 
  lcd.print(menuItems[menuLevel][menuIndex]);
 
 
  // Navigation logic with bounds checking
 
  if (digitalRead(BTN_UP) == LOW) {
 
    menuIndex = (menuIndex > 0) ? menuIndex - 1 : maxIndex;
 
    lastButtonPress = millis();
 
  }
 
  if (digitalRead(BTN_DOWN) == LOW) {
 
    menuIndex = (menuIndex < maxIndex) ? menuIndex + 1 : 0;
 
    lastButtonPress = millis();
 
  }
 
  if (digitalRead(BTN_ENTER) == LOW) {
 
    // Go deeper into menu if valid item
 
    if (menuLevel < 7 && menuItems[menuLevel][menuIndex] != "") {
 
      menuLevel++;
 
      menuIndex = 0;
 
    }
 
    lastButtonPress = millis();
 
  }
 
  if (digitalRead(BTN_BACK) == LOW) {
 
    if (menuLevel > 0) menuLevel--;
 
    menuIndex = 0;
 
    lastButtonPress = millis();
 
  }
 
  if (digitalRead(BTN_OK) == LOW) {
 
    handleMenuAction();
 
    lastButtonPress = millis();
 
  }
 
}
 
 
// Action based on the current menu selection.
 
void handleMenuAction() {
 
  switch(menuLevel) {
 
    case 0: // Main menu
 
      if (menuIndex == 7) { // "Exit"
 
        menuLevel = 0;
 
        menuIndex = 0;
 
      }
 
      break;
 
    case 1: // Parameters menu
 
      switch(menuIndex) {
 
        case 5: saveParametersToEEPROM(); break;
 
        case 6: loadParametersFromEEPROM(); break;
 
        case 7: menuLevel--; break; // Back
 
        default:
 
          // Enter parameter adjustment mode for indices 0-4
 
          adjustingParam = true;
 
          break;
 
      }
 
      break;
 
    case 2: // Motor Control
 
      switch(menuIndex) {
 
        case 0: motorRunning = true; break;  // "2.1 Start"
 
        case 1: motorRunning = false; break; // "2.2 Stop"
 
        case 2: mpptMode = !mpptMode; break;   // Toggle MPPT mode
 
        case 5: menuLevel--; break; // Back
 
      }
 
      break;
 
    case 3: // Display menu
 
      if (menuIndex == 2) { // "Back"
 
        menuLevel--;
 
      } else {
 
        // For display parameters, enter adjustment mode
 
        adjustingParam = true;
 
      }
 
      break;
 
    case 6: // Settings menu
 
      if (menuIndex == 0) { // "6.1 Factory Reset"
 
        factoryReset();
 
      }
 
      break;
 
    // Additional cases for other menus can be added here.
 
    default:
 
      break;
 
  }
 
}
 
 
// Parameter adjustment handler using non-blocking debounce.
 
void handleParameterAdjust() {
 
  static float currentValue = 0;
 
  static int currentMenuLevel = 0;
 
  static int currentMenuIndex = 0;
 
  static unsigned long lastAdjustTime = 0;
 
 
  // On entry to adjustment mode, initialize values.
 
  if (adjustingParam && currentValue == 0) {
 
    currentMenuLevel = menuLevel;
 
    currentMenuIndex = menuIndex;
 
    currentValue = parameterValues[menuLevel][menuIndex];
 
    lcd.clear();
 
  }
 
 
  lcd.setCursor(0, 0);
 
  lcd.print("Adjust: ");
 
  lcd.print(menuItems[currentMenuLevel][currentMenuIndex]);
 
  lcd.setCursor(0, 1);
 
  lcd.print("Value: ");
 
  lcd.print(currentValue);
 
  lcd.print("    ");  // Clear residual characters
 
 
  // Use non-blocking debounce (150 ms)
 
  if (millis() - lastAdjustTime > 150) {
 
    if (digitalRead(BTN_UP) == LOW) {
 
      currentValue += getAdjustmentStep(currentMenuLevel, currentMenuIndex);
 
      parameterValues[currentMenuLevel][currentMenuIndex] = currentValue;
 
      lastAdjustTime = millis();
 
    }
 
    if (digitalRead(BTN_DOWN) == LOW) {
 
      currentValue -= getAdjustmentStep(currentMenuLevel, currentMenuIndex);
 
      parameterValues[currentMenuLevel][currentMenuIndex] = currentValue;
 
      lastAdjustTime = millis();
 
    }
 
    if (digitalRead(BTN_BACK) == LOW || digitalRead(BTN_ENTER) == LOW) {
 
      // Exit adjustment mode and update system parameters as needed.
 
      adjustingParam = false;
 
      parameterValues[currentMenuLevel][currentMenuIndex] = currentValue;
 
      lcd.clear();
 
      // Update system parameters based on menu location.
 
      if (currentMenuLevel == 1) { // Parameters menu
 
        switch(currentMenuIndex) {
 
          case 0: target_frequency = currentValue; break;
 
          case 1: /* Update voltage limit if needed */ break;
 
          case 2: /* Update acceleration */ break;
 
          case 3: /* Update deceleration */ break;
 
          case 4: /* Update dead time */ break;
 
        }
 
      } else if (currentMenuLevel == 3) { // Display menu
 
        if(currentMenuIndex == 0) {
 
          // Example: update LCD backlight brightness if supported
 
          lcd.setBacklight((uint8_t)currentValue);
 
        }
 
      }
 
      lastAdjustTime = millis();
 
      // Reset currentValue for next adjustment.
 
      currentValue = 0;
 
    }
 
  }
 
}
 
 
// Returns the adjustment step size based on the menu context.
 
float getAdjustmentStep(int level, int index) {
 
  if (level == 1) { // Parameters menu
 
    switch(index) {
 
      case 0: return 0.1; // Frequency
 
      case 1: return 1.0; // Voltage
 
      case 2: return 0.1; // Acceleration
 
      case 3: return 0.1; // Deceleration
 
      case 4: return 0.1; // Dead time
 
    }
 
  }
 
  if (level == 3) { // Display menu
 
    switch(index) {
 
      case 0: return 5.0; // Brightness
 
      case 1: return 1.0; // Contrast
 
    }
 
  }
 
  return 1.0; // Default step
 
}
 
 
//-------------------------------------------------------
 
// EEPROM Functions
 
//-------------------------------------------------------
 
 
void saveParametersToEEPROM() {
 
  int address = 0;
 
  for (int i = 0; i < 8; i++) {
 
    for (int j = 0; j < 8; j++) {
 
      EEPROM.put(address, parameterValues[i][j]);
 
      address += sizeof(float);
 
    }
 
  }
 
  EEPROM.commit();
 
  lcd.clear();
 
  lcd.print("Settings Saved!");
 
  // Briefly show the message (blocking delay acceptable here for user feedback)
 
  delay(1000);
 
}
 
 
void loadParametersFromEEPROM() {
 
  int address = 0;
 
  for (int i = 0; i < 8; i++) {
 
    for (int j = 0; j < 8; j++) {
 
      EEPROM.get(address, parameterValues[i][j]);
 
      address += sizeof(float);
 
    }
 
  }
 
  lcd.clear();
 
  lcd.print("Settings Loaded!");
 
  delay(1000);
 
}
 
 
void factoryReset() {
 
  // Reset parameters to default values.
 
  parameterValues[1][0] = 50.0; // Frequency
 
  parameterValues[1][1] = 400.0; // Voltage
 
  parameterValues[1][2] = 5.0;   // Acceleration
 
  parameterValues[1][3] = 5.0;   // Deceleration
 
  parameterValues[1][4] = 4.0;   // Dead time
 
  parameterValues[3][0] = 128;   // Brightness
 
  parameterValues[3][1] = 50;    // Contrast
 
  saveParametersToEEPROM();
 
}
 
 
//-------------------------------------------------------
 
// MCPWM and SPWM Functions
 
//-------------------------------------------------------
 
 
void setupMCPWM() {
 
  // Initialize MCPWM GPIO pins
 
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PWM_UH);
 
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, PWM_UL);
 
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, PWM_VH);
 
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, PWM_VL);
 
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2A, PWM_WH);
 
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2B, PWM_WL);
 
 
  // Configure MCPWM timers
 
  mcpwm_config_t pwm_config;
 
  pwm_config.frequency = CARRIER_FREQ; // Carrier frequency in Hz
 
  pwm_config.cmpr_a = 0;               // Initial duty cycle (0%)
 
  pwm_config.cmpr_b = 0;               // Initial duty cycle (0%)
 
  pwm_config.counter_mode = MCPWM_UP_DOWN_COUNTER;
 
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
 
 
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
 
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);
 
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &pwm_config);
 
 
  // Configure dead time for each phase
 
  uint32_t dead_time_ticks = (uint32_t)((DEAD_TIME_US / 1000000.0) * 160000000);
 
  mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, dead_time_ticks, dead_time_ticks);
 
  mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, dead_time_ticks, dead_time_ticks);
 
  mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, dead_time_ticks, dead_time_ticks);
 
}
 
 
void updateSPWM(float angle, float frequency) {
 
  // Calculate indexes for the three phases.
 
  angle = fmod(angle, 360.0);  // Ensure angle is within 0-360
 
  uint8_t indexU = (uint8_t)(angle / 360.0 * 255.0);
 
  uint8_t indexV = (uint8_t)(fmod(angle + 120.0, 360.0) / 360.0 * 255.0);
 
  uint8_t indexW = (uint8_t)(fmod(angle + 240.0, 360.0) / 360.0 * 255.0);
 
 
  // Calculate duty cycles for each phase
 
  float dutyU = (sineTable[indexU] / 255.0) * 100.0;
 
  float dutyV = (sineTable[indexV] / 255.0) * 100.0;
 
  float dutyW = (sineTable[indexW] / 255.0) * 100.0;
 
 
  // Apply torque boost at low frequencies.
 
  if (frequency < TORQUE_BOOST_FREQ) {
 
    dutyU = constrain(dutyU * TORQUE_BOOST_FACTOR, 0.0, 100.0);
 
    dutyV = constrain(dutyV * TORQUE_BOOST_FACTOR, 0.0, 100.0);
 
    dutyW = constrain(dutyW * TORQUE_BOOST_FACTOR, 0.0, 100.0);
 
  }
 
 
  // Set PWM duty cycles for each phase
 
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, dutyU);
 
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 100 - dutyU);
 
 
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, dutyV);
 
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, 100 - dutyV);
 
 
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, dutyW);
 
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B, 100 - dutyW);
 
}
 
 
void stopPWM() {
 
  // Set all PWM outputs to 0% duty cycle
 
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);
 
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 0);
 
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 0);
 
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, 0);
 
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, 0);
 
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B, 0);
 
}
 
 
//-------------------------------------------------------
 
// Fault Handling & Display Update
 
//-------------------------------------------------------
 
 
void handleFault() {
 
  if (digitalRead(FAULT_PIN) == LOW) {
 
    fault_latched = true;
 
  }
 
  if (fault_latched) {
 
    lcd.setCursor(0, 1);
 
    lcd.print("FAULT DETECTED!");
 
    stopPWM();
 
  }
 
}
 
 
void updateDisplay() {
 
  // In this example the display is updated every loop iteration.
 
  // (If you need a fixed refresh rate, you could check millis() here too.)
 
  lcd.setCursor(0, 0);
 
  lcd.print("Freq: ");
 
  lcd.print(current_frequency, 1);
 
  lcd.print("Hz   ");
 
  lcd.setCursor(0, 1);
 
  lcd.print("DC: ");
 
  lcd.print(dc_voltage, 1);
 
  lcd.print("V   ");
 
}
 
 
//-------------------------------------------------------
 
// MPPT Algorithm (Perturb & Observe)
 
//-------------------------------------------------------
 
 
void handleMPPT() {
 
  if (solar_power > prev_power) {
 
    if (dc_voltage > prev_voltage) {
 
      target_frequency += MPPT_STEP;
 
    } else {
 
      target_frequency -= MPPT_STEP;
 
    }
 
  } else {
 
    if (dc_voltage > prev_voltage) {
 
      target_frequency -= MPPT_STEP;
 
    } else {
 
      target_frequency += MPPT_STEP;
 
    }
 
  }
 
  prev_power = solar_power;
 
  prev_voltage = dc_voltage;
 
  target_frequency = constrain(target_frequency, MIN_FREQUENCY, MAX_FREQUENCY);
 
}
 
 
//-------------------------------------------------------
 
// Setup Function
 
//-------------------------------------------------------
 
 
void setup() {
 
  Serial.begin(115200);
 
  EEPROM.begin(512);
 
 
  // Initialize LCD
 
  lcd.init();
 
  lcd.backlight();
 
  lcd.setCursor(0, 0);
 
  lcd.print("VFD SYSTEM");
 
  delay(2000);  // Start-up message delay can remain
 
  lcd.clear();
 
 
  // Set sensor/control pins
 
  pinMode(POT_PIN, INPUT);
 
  pinMode(ON_OFF_PIN, INPUT_PULLUP);
 
  pinMode(FAULT_PIN, INPUT_PULLUP);
 
  pinMode(DC_VOLTAGE_PIN, INPUT);
 
  pinMode(CURRENT_SENSOR_PIN, INPUT);
 
  
 
  // Set button pins
 
  pinMode(BTN_UP,    INPUT_PULLUP);
 
  pinMode(BTN_DOWN,  INPUT_PULLUP);
 
  pinMode(BTN_BACK,  INPUT_PULLUP);
 
  pinMode(BTN_ENTER, INPUT_PULLUP);
 
  pinMode(BTN_OK,    INPUT_PULLUP);
 
 
  // Initialize MCPWM
 
  setupMCPWM();
 
}
 
 
//-------------------------------------------------------
 
// Main Loop (Non-blocking style)
 
//-------------------------------------------------------
 
 
void loop() {
 
  // Handle menu navigation and actions
 
  handleMenu();
 
  
 
  // Read DC voltage (assuming a proper scaling circuit is used)
 
  int dc_adc_value = analogRead(DC_VOLTAGE_PIN);
 
  dc_voltage = ((float)dc_adc_value / 4095.0) * MAX_DC_VOLTAGE;
 
  
 
  // Read current sensor for MPPT calculations
 
  int current_adc_value = analogRead(CURRENT_SENSOR_PIN);
 
  float current_voltage = ((float)current_adc_value / 4095.0) * 3.3;  // ESP32 ADC reference voltage
 
  solar_current = (current_voltage - 1.65) / ACS712_SENSITIVITY;
 
  solar_power = dc_voltage * solar_current;
 
  
 
  // Determine target frequency
 
  if (!mpptMode) {
 
    // Manual frequency control using potentiometer
 
    int pot_value = analogRead(POT_PIN);
 
    float max_allowed_frequency = (dc_voltage / MAX_DC_VOLTAGE) * MAX_FREQUENCY;
 
    target_frequency = map(pot_value, 0, 4095, (int)(MIN_FREQUENCY * 100), (int)(max_allowed_frequency * 100)) / 100.0;
 
    target_frequency = constrain(target_frequency, MIN_FREQUENCY, max_allowed_frequency);
 
  } else {
 
    // Automatic frequency control using MPPT
 
    handleMPPT();
 
  }
 
  
 
  // Check for fault or external OFF state
 
  if (dc_voltage < MIN_DC_VOLTAGE || fault_latched || digitalRead(ON_OFF_PIN) == HIGH) {
 
    stopPWM();
 
    motorRunning = false;
 
    current_frequency = 0.0;
 
    updateDisplay();
 
  } else {
 
    // Frequency ramping logic (update every second)
 
    unsigned long now = millis();
 
    if (now - last_ramp_time >= 1000) {
 
      float ramp_increment = RAMP_RATE * ((now - last_ramp_time) / 1000.0);
 
      if (current_frequency < target_frequency) {
 
        current_frequency = min(current_frequency + ramp_increment, target_frequency);
 
      } else if (current_frequency > target_frequency) {
 
        current_frequency = max(current_frequency - ramp_increment, target_frequency);
 
      }
 
      last_ramp_time = now;
 
    }
 
 
    // SPWM angle calculation using non-blocking time-difference
 
    static unsigned long last_angle_update = micros();
 
    float delta_t = (micros() - last_angle_update) / 1e6;
 
    static float angle = 0.0;
 
    angle += 360.0 * current_frequency * delta_t;
 
    if (angle >= 360.0) angle -= 360.0;
 
    last_angle_update = micros();
 
 
    if (motorRunning) {
 
      updateSPWM(angle, current_frequency);
 
    } else {
 
      stopPWM();
 
    }
 
  }
 
 
  handleFault();   // Check for faults
 
  updateDisplay(); // Update LCD with current values
 
  
 
  // No blocking delay here; the loop runs continuously.
 
  // You may call yield() if desired to allow background tasks to run.
 
  yield();
 
}
