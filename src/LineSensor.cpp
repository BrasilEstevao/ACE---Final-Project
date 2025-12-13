#include "LineSensor.h"
#include "config.h"
#include <Arduino.h>

// ============================================================================
// CONSTRUCTOR
// ============================================================================

LineSensor::LineSensor()
{
    _threshold = IR_THRESHOLD;
    _linePosition = 0;
    _calibrated = false;
    
    // Initialize arrays
    for (int i = 0; i < NUM_ANALOG_SENSORS; i++) {
        _analogSensors[i] = 0;
        _minValues[i] = 1023;
        _maxValues[i] = 0;
    }
    
    for (int i = 0; i < NUM_DIGITAL_SENSORS; i++) {
        _digitalSensors[i] = false;
    }
    
    // Configure analog pins (center 3 sensors)
    pinMode(IR_CENTER_LEFT, INPUT);
    pinMode(IR_CENTER, INPUT);
    pinMode(IR_CENTER_RIGHT, INPUT);
    
    // Configure DIGITAL pins (edge 2 sensors) with pull-up
    // Most IR sensors output LOW when detecting black line
    pinMode(IR_LEFT_PIN, INPUT_PULLUP);
    pinMode(IR_RIGHT_PIN, INPUT_PULLUP);
}

// ============================================================================
// SENSOR READING
// ============================================================================

void LineSensor::read()
{
    // Read analog sensors (center 3) - inverted so black = high value
    _analogSensors[0] = 1023 - analogRead(IR_CENTER_LEFT);
    _analogSensors[1] = 1023 - analogRead(IR_CENTER);
    _analogSensors[2] = 1023 - analogRead(IR_CENTER_RIGHT);
    
    // Read digital sensors (edges)
    // Typical IR sensor: LOW = black detected, HIGH = white detected
    // So we invert: true = black, false = white
    _digitalSensors[0] = !digitalRead(IR_LEFT_PIN);   // Left edge
    _digitalSensors[1] = !digitalRead(IR_RIGHT_PIN);  // Right edge
}

void LineSensor::calibrate()
{
    const int samples = 100;
    
    // Reset calibration values
    for (int i = 0; i < NUM_ANALOG_SENSORS; i++) {
        _minValues[i] = 1023;
        _maxValues[i] = 0;
    }
    
    Serial.println("========================================");
    Serial.println("CALIBRATING ANALOG SENSORS");
    Serial.println("========================================");
    Serial.println("Move robot slowly over BLACK and WHITE");
    Serial.println("Sampling 100 readings...");
    Serial.println();
    
    for (int i = 0; i < samples; i++) {
        read();
        
        for (int j = 0; j < NUM_ANALOG_SENSORS; j++) {
            if (_analogSensors[j] > _maxValues[j]) {
                _maxValues[j] = _analogSensors[j];
            }
            if (_analogSensors[j] < _minValues[j]) {
                _minValues[j] = _analogSensors[j];
            }
        }
        
        // Progress indicator
        if (i % 10 == 0) {
            Serial.print("Progress: ");
            Serial.print(i);
            Serial.println("/100");
        }
        
        delay(20);
    }
    
    // Calculate average threshold
    int avgThreshold = 0;
    for (int i = 0; i < NUM_ANALOG_SENSORS; i++) {
        avgThreshold += (_maxValues[i] + _minValues[i]) / 2;
    }
    _threshold = avgThreshold / NUM_ANALOG_SENSORS;
    _calibrated = true;
    
    Serial.println();
    Serial.println("========================================");
    Serial.println("CALIBRATION COMPLETE");
    Serial.println("========================================");
    Serial.print("Threshold: ");
    Serial.println(_threshold);
    Serial.println("Min values: ");
    for (int i = 0; i < NUM_ANALOG_SENSORS; i++) {
        Serial.print("  Sensor ");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(_minValues[i]);
    }
    Serial.println("Max values: ");
    for (int i = 0; i < NUM_ANALOG_SENSORS; i++) {
        Serial.print("  Sensor ");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(_maxValues[i]);
    }
    Serial.println("========================================");
    Serial.println();
}

// ============================================================================
// LINE POSITION CALCULATION 
// ============================================================================

int LineSensor::getPosition()
{
    // Use weighted average of 3 center sensors
    // Position: -1000 (left) to +1000 (right)
    
    long weighted_sum = 0;
    long sum = 0;
    
    // Weights: -1 (left), 0 (center), +1 (right)
    int weights[3] = {-1, 0, 1};
    
    for (int i = 0; i < NUM_ANALOG_SENSORS; i++) {
        if (_analogSensors[i] > _threshold) {
            weighted_sum += (long)(_analogSensors[i]) * weights[i] * 1000;
            sum += _analogSensors[i];
        }
    }
    
    if (sum > 0) {
        _linePosition = weighted_sum / sum;
    }
    // If no line detected, keep last position
    
    return _linePosition;
}

// ============================================================================
// JUNCTION DETECTION
// ============================================================================

JunctionType LineSensor::detectJunction()
{
    // Count black detections
    int analogBlackCount = 0;
    int totalBlackCount = 0;
    
    // Check analog sensors
    for (int i = 0; i < NUM_ANALOG_SENSORS; i++) {
        if (_analogSensors[i] > _threshold) {
            analogBlackCount++;
            totalBlackCount++;
        }
    }
    
    // Check digital sensors
    bool leftEdge = _digitalSensors[0];
    bool rightEdge = _digitalSensors[1];
    
    if (leftEdge) totalBlackCount++;
    if (rightEdge) totalBlackCount++;
    
    // Decision logic
    if (totalBlackCount == 0) {
        return JUNCTION_LOST;
    }
    else if (leftEdge && rightEdge && analogBlackCount > 0) {
        // Both edges + center = T junction or cross
        return JUNCTION_T;
    }
    else if (leftEdge && !rightEdge && analogBlackCount  > 2) {
        // Left edge detects + some center = left turn
        return JUNCTION_LEFT;
    }
    else if (rightEdge && !leftEdge && analogBlackCount > 2) {
        // Right edge detects + some center = right turn
        return JUNCTION_RIGHT;
    }
    else {
        // Normal line following
        return JUNCTION_NONE;
    }
}

// ============================================================================
// UTILITY METHODS
// ============================================================================

int LineSensor::getAnalogSensorValue(int index)
{
    if (index >= 0 && index < NUM_ANALOG_SENSORS) {
        return _analogSensors[index];
    }
    return 0;
}

bool LineSensor::getDigitalSensorValue(int index)
{
    if (index >= 0 && index < NUM_DIGITAL_SENSORS) {
        return _digitalSensors[index];
    }
    return false;
}

void LineSensor::printValues()
{
    Serial.print("D:[");
    Serial.print(_digitalSensors[0] ? "□" : "■");
    Serial.print("] A:[");
    
    for (int i = 0; i < NUM_ANALOG_SENSORS; i++) {
        if (i > 0) Serial.print(",");
        if (_analogSensors[i] < 100) Serial.print(" ");
        if (_analogSensors[i] < 10) Serial.print(" ");
        Serial.print(_analogSensors[i]);
    }
    
    Serial.print("] D:[");
    Serial.print(_digitalSensors[1] ? "□" : "■");
    Serial.print("] Pos:");
    if (_linePosition >= 0) Serial.print(" ");
    if (abs(_linePosition) < 1000) Serial.print(" ");
    if (abs(_linePosition) < 100) Serial.print(" ");
    Serial.print(_linePosition);
    Serial.print(" | ");
    
    JunctionType j = detectJunction();
    switch(j) {
        case JUNCTION_NONE:  Serial.println("LINE  "); break;
        case JUNCTION_LEFT:  Serial.println("◄ LEFT"); break;
        case JUNCTION_RIGHT: Serial.println("RIGHT►"); break;
        case JUNCTION_T:     Serial.println("◄ T ► "); break;
        case JUNCTION_CROSS: Serial.println("CROSS+"); break;
        case JUNCTION_LOST:  Serial.println("LOST ✗"); break;
    }
}

void LineSensor::setThreshold(int threshold)
{
    _threshold = threshold;
}

int LineSensor::getThreshold()
{
    return _threshold;
}