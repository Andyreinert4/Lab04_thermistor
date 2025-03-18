// *************************************************************************
// Title        : Lab 04 Thermistor
// File Name    : 'main.cpp'
// Target MCU   : Espressif ESP32 (Doit DevKit Version 1)
//
// EGRT 390     : Thermistor Temperature Sensor
//
// Revision History:
// When         Who         Description of change
// -----------  ----------- -----------------------
// 17-MAR-2025  [A.Reinert] initial commit
//
// *************************************************************************

// Include Files
// *************************************************************************
#include <Arduino.h>
#include <Wire.h>
#include <ADS1X15.h>
#include <SSD1306Wire.h>
#include "fonts/Open_Sans_Light_12.h"
#include <stdint.h>
#include <math.h>

// Globals
// *************************************************************************

// Heartbeat LED
const uint8_t LED = 15;               // Pin number connected to LED
const uint16_t BLINK_INTERVAL = 1000; // Blink on/off time (milliseconds)
unsigned long ledBlinkTime = 0;       // Time of last LED blink
bool ledState = false;                // State false=LOW, true=HIGH

// ADS1115 Sensor Update
const uint16_t UPDATE_TIME = 500;  // Update time (milliseconds)
unsigned long lastUpdateTime = 0;  // Time of last update
unsigned long runTimeSeconds = 0;  // Total accumulated runtime (seconds)
unsigned long lastTimeSeconds = 0; // Last getRunTime() call (seconds)

// OLED Display
SSD1306Wire display(0x3C, SDA, SCL); // Initialize display with I2C address and pins

// ADS1115 ADC
ADS1115 ADS(0x48); // 0x48 = 1001000 (ADDR = GND)

// Average voltage over NUM_SAMPLES
const uint8_t NUM_SAMPLES = 30; // Number of samples to average
float samples[NUM_SAMPLES];     // Array to store samples
float thermistorSamples[NUM_SAMPLES]; // Array to store thermistor samples
uint8_t sampleIndex = 0;        // Current sample index

// Thermistor parameters
const float SUPPLY_VOLTAGE = 3.3;           // Supply voltage for the voltage divider
const float FIXED_RESISTOR = 2700;     // Resistance of the fixed resistor in the voltage divider
const float A = 1.125308852E-03;            // Thermistor parameter A
const float B = 2.347657E-04;            // Thermistor parameter B
const float C = 8.5663516E-08;            // Thermistor parameter C

// LM35 temperature sensor scale factor
//const float LM35_SCALE = 0.01;  // 10mV per degree Celsius

// Sensor data as formatted strings
const uint8_t MAX_STR_LEN = 32; // Max string length
char temperatureC[MAX_STR_LEN]; // Temperature (°C) as formatted string
char temperatureF[MAX_STR_LEN]; // Temperature (°F) as formatted string
char voltageStr[MAX_STR_LEN];   // Voltage (V) as formatted string
char runTimeStr[MAX_STR_LEN];   // Run time as formatted string

// Function Prototypes
// *************************************************************************
void getValues();  // Read sensor values and format for display
void updateOLED(); // Update OLED display with current sensor data
void getRunTime(); // Calculate run time of the program
void printValues(); // Print values to serial monitor (if needed)
float calculateTemperature(float voltage);         // Calculate temperature from resistance using Steinhart-Hart equation
float calculateResistance (float voltage);         // Calculate temperature from resistance using Steinhart-Hart equation
float getSmoothedTemperature(); // Calculate smoothed temperature

// Begin Code
// *************************************************************************
void setup()
{
    Serial.begin(115200);                      // Initialize serial comm
    Serial.println();                          // Print blank line
    Serial.println("Thermistor Temperature Sensor"); // Print message for user
    pinMode(LED, OUTPUT);                      // Set pin as digital output

    // Initialize ADS1115
    Wire.begin();       // Start I2C bus
    ADS.begin();        // Start ADS1115
    ADS.setGain(4);     // Sets voltage range to +/- 1.024V
    ADS.setDataRate(0); // 0 = 128 SPS
    ADS.setMode(0);     // Continuous conversion mode
    ADS.getValue();     // Dummy read to set pointer

    // Initialize average samples array, set all to 0
    for (int i = 0; i < NUM_SAMPLES; i++)
    {
      samples[i] = 0;
    }

    // Initialize OLED display
    display.init();                               // Initialize the display
    display.flipScreenVertically();               // Flip display if needed
    display.setFont(Open_Sans_Light_12);            // Set default font
    display.setTextAlignment(TEXT_ALIGN_LEFT);    // Set text alignment
    display.clear();                              // Clear the display
    display.drawString(0, 0, "Thermistor Sensor");      // Display title
    display.drawString(0, 20, "Initializing..."); // Display message
    display.display();                            // Display the content
}

// Main program
// *************************************************************************
void loop()
{
    // Heartbeat LED is on for BLINK_INTERVAL and off BLINK_INTERVAL
    if (millis() - ledBlinkTime >= BLINK_INTERVAL)
    {
        ledBlinkTime = millis();     // Update time of last blink
        ledState = !ledState;        // Toggle LED state
        digitalWrite(LED, ledState); // Set LED state
    }

    // Read sensor values every UPDATE_TIME
    if (millis() - lastUpdateTime >= UPDATE_TIME)
    {
        lastUpdateTime = millis(); // Reset time delay
        getValues();               // Get values & format sensor data
        getRunTime();              // Get run time
        updateOLED();              // Update OLED display
        printValues();             // Print values to serial monitor
    }
}

// Functions
// *************************************************************************
void getValues()
{
    // Read voltage from ADS1115 channel 2 (Thermistor)
    int16_t adc2 = ADS.readADC(2);          //  Raw ADC count value
    float voltageFactor = ADS.toVoltage(1); // Get conversion factor
    float voltage = adc2 * voltageFactor;   // Calculate actual voltage

    // Store voltage in samples circular buffer (array)
    samples[sampleIndex] = voltage;                // Store current sample
    //thermistorSamples[sampleIndex] = voltage;      // Store current thermistor sample
    sampleIndex = (sampleIndex + 1) % NUM_SAMPLES; // Wrap around index

    // Calculate average after collecting enough samples
    float avgVoltage = 0;
    for (int i = 0; i < NUM_SAMPLES; i++)
    {
        avgVoltage += samples[i]; // Sum all samples
    }
    avgVoltage /= NUM_SAMPLES; // Calculate average

    // Calculate temperature from resistance
    float tempC = calculateTemperature(avgVoltage);
    float tempF = tempC * 1.8 + 32.0; // Convert Celsius to Fahrenheit

    // Format data for display
    sprintf(temperatureC, "Temp: %.1f C", tempC);
    sprintf(temperatureF, "Temp: %.1f F", tempF);
    sprintf(voltageStr, "Voltage: %.3f V", avgVoltage);
}

// Function: calculateResistance
// *************************************************************************
float calculateResistance(float voltage)
{
    if (voltage <= 0.0 || voltage >= SUPPLY_VOLTAGE) 
    {
        return -1.0; // Error indicator
    }
    // Voltage divider formula to calculate thermistor resistance
    float resistance = FIXED_RESISTOR * (SUPPLY_VOLTAGE - voltage) / voltage;
    return resistance;
}

// Function: calculateTemperature
// *************************************************************************
float calculateTemperature(float voltage)
{
    float resistance = calculateResistance(voltage); // Calculate resistance from voltage)
    float logR = log(resistance);
    float TempK = 1.0 / (A + B * logR + C * pow(logR, 3)); // Steinhart-Hart equation
    float TempC = TempK - 273.15; // Convert Kelvin to Celsius
    return TempC; // Return temperature in Celsius
}


// Function: updateOLED
// *************************************************************************
void updateOLED()
{
    display.clear();
    display.drawString(0, 0, runTimeStr);    // Display run time
    display.drawString(0, 16, temperatureC); // Display temperature in °C
    display.drawString(0, 32, temperatureF); // Display temperature in °F
    display.drawString(0, 48, voltageStr);   // Display voltage
    display.display();                       // Update the display
}

// Function: getRunTime
// *************************************************************************
void getRunTime()
{
    unsigned long currentTimeSeconds = millis() / 1000; // Current time seconds
    unsigned long elapsedSeconds;                       // Elapsed time seconds
    if (currentTimeSeconds < lastTimeSeconds)           // Handle millis() overflow
    {
        elapsedSeconds = (0xFFFFFFFF / 1000) - lastTimeSeconds + currentTimeSeconds;
    }
    else
    {
        elapsedSeconds = currentTimeSeconds - lastTimeSeconds;
    }
    runTimeSeconds += elapsedSeconds;     // Add elapsed time to total runtime
    lastTimeSeconds = currentTimeSeconds; // Update for next call

    // Calculate run time in hours, minutes, seconds
    unsigned long hours = runTimeSeconds / 3600;
    unsigned long minutes = (runTimeSeconds % 3600) / 60;
    unsigned long seconds = runTimeSeconds % 60;
    sprintf(runTimeStr, "Run Time: %02lu:%02lu:%02lu", hours, minutes, seconds);
}

// Function: print Values
// *************************************************************************
void printValues()
{
    // Print run time and all sensor values to the serial monitor
    Serial.print(runTimeStr);     // Print run time
    Serial.print(" | ");          // Print separator
    Serial.print(temperatureC);   // Print temperature in °C
    Serial.print(" | ");          // Print separator
    Serial.print(temperatureF);   // Print temperature in °F
    Serial.print(" | ");          // Print separator
    Serial.println(voltageStr);   // Print voltage in V
}


