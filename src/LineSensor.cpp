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
    _waterLevel = 0;
    _posLeft = 0;
    _posRight = 0;
    _irMax = 0;
    _total = 0;
    
    for (int i = 0; i < IR_SENSOR_COUNT; i++) {
        _analogSensors[i] = 0;
        _minValues[i] = 1023;
        _maxValues[i] = 0;
    }
    
    // Configure MUX control pins
    pinMode(MUXA_PIN, OUTPUT);
    pinMode(MUXB_PIN, OUTPUT);
    pinMode(MUXC_PIN, OUTPUT);
    
    // Configure ADC input pin
    pinMode(ADC_IN_PIN, INPUT);
}

// ============================================================================
// MULTIPLEXER CONTROL
// ============================================================================

void LineSensor::setMuxChannel(int channel)
{
    // Set MUX control pins (A, B, C) to select channel
    digitalWrite(MUXA_PIN, channel & 1);
    digitalWrite(MUXB_PIN, (channel >> 1) & 1);
    digitalWrite(MUXC_PIN, (channel >> 2) & 1);
}

uint16_t LineSensor::readMuxChannel(int channel)
{
    setMuxChannel(channel);
    delayMicroseconds(100);  // Wait for MUX to settle
    return analogRead(ADC_IN_PIN);
}

// ============================================================================
// SENSOR READING
// ============================================================================

void LineSensor::read()
{
    // Read all 5 IR sensors through multiplexer
    // Channels 3-7 on the MUX
    for (int i = 0; i < IR_SENSOR_COUNT; i++) {
        // Read and invert (1023 - value) so black = high
        _analogSensors[IR_SENSOR_COUNT - 1 - i] = 1023 - readMuxChannel(IR_MUX_START_CH + i);
    }
}

void LineSensor::calibrate()
{
    const int samples = 100;
    
    for (int i = 0; i < IR_SENSOR_COUNT; i++) {
        _minValues[i] = 1023;
        _maxValues[i] = 0;
    }
    
    Serial.println("========================================");
    Serial.println("CALIBRATING IR SENSORS");
    Serial.println("========================================");
    Serial.println("Move robot over BLACK and WHITE");
    Serial.println("Sampling 100 readings...");
    Serial.println();
    
    for (int i = 0; i < samples; i++) {
        read();
        
        for (int j = 0; j < IR_SENSOR_COUNT; j++) {
            if (_analogSensors[j] > _maxValues[j]) {
                _maxValues[j] = _analogSensors[j];
            }
            if (_analogSensors[j] < _minValues[j]) {
                _minValues[j] = _analogSensors[j];
            }
        }
        
        if (i % 10 == 0) {
            Serial.print("Progress: ");
            Serial.print(i);
            Serial.println("/100");
        }
        
        delay(20);
    }
    
    // Calculate threshold
    int avgThreshold = 0;
    for (int i = 0; i < IR_SENSOR_COUNT; i++) {
        avgThreshold += (_maxValues[i] + _minValues[i]) / 2;
    }
    _threshold = avgThreshold / IR_SENSOR_COUNT;
    _calibrated = true;
    
    Serial.println();
    Serial.println("========================================");
    Serial.println("CALIBRATION COMPLETE");
    Serial.println("========================================");
    Serial.print("Threshold: ");
    Serial.println(_threshold);
    for (int i = 0; i < IR_SENSOR_COUNT; i++) {
        Serial.print("Sensor ");
        Serial.print(i);
        Serial.print(": min=");
        Serial.print(_minValues[i]);
        Serial.print(" max=");
        Serial.println(_maxValues[i]);
    }
    Serial.println("========================================");
}

// ============================================================================
// LINE POSITION - Using IRLine algorithm
// ============================================================================

void LineSensor::calcLineEdgeLeft()
{
    bool found = false;
    _irMax = 0;
    _posLeft = 2 * 16.0;
    _total = 0;
    int lastV = 0;
    
    for (int c = 0; c < IR_SENSOR_COUNT; c++) {
        int v = _analogSensors[c] - _waterLevel;
        if (v < 0) v = 0;
        if (v > _irMax) _irMax = v;
        
        _total += v;
        
        if (!found && lastV < _threshold && v > _threshold) {
            _posLeft = -12 + 16.0 * (c - 2) + 16.0 * (_threshold - lastV) / (v - lastV);
            found = true;
        }
        lastV = v;
    }
}

void LineSensor::calcLineEdgeRight()
{
    bool found = false;
    _irMax = 0;
    _posRight = -2 * 16.0;
    _total = 0;
    int lastV = 0;
    
    for (int c = 0; c < IR_SENSOR_COUNT; c++) {
        int v = _analogSensors[IR_SENSOR_COUNT - 1 - c] - _waterLevel;
        if (v < 0) v = 0;
        if (v > _irMax) _irMax = v;
        
        _total += v;
        
        if (!found && lastV < _threshold && v > _threshold) {
            _posRight = -(-12 + 16.0 * (c - 2) + 16.0 * (_threshold - lastV) / (v - lastV));
            found = true;
        }
        lastV = v;
    }
}

int LineSensor::getPosition()
{
    // Calculate using weighted average
    calcLineEdgeLeft();
    
    long weighted_sum = 0;
    long sum = 0;
    int weights[5] = {-2, -1, 0, 1, 2};
    
    for (int i = 0; i < IR_SENSOR_COUNT; i++) {
        int v = _analogSensors[i] - _waterLevel;
        if (v < 0) v = 0;
        
        if (v > _threshold) {
            weighted_sum += (long)v * weights[i] * 500;
            sum += v;
        }
    }
    
    if (sum > 0) {
        _linePosition = weighted_sum / sum;
    }
    
    return _linePosition;
}

// ============================================================================
// NODE/JUNCTION DETECTION - IRLine style
// ============================================================================

char LineSensor::detectNode()
{
    static char last_node = 'E';
    static int stability_count = 0;
    static int required_stability = 40;
    
    char current_node;
    
    // XXXOO - Left
    if (_analogSensors[0] > _threshold && 
        _analogSensors[1] > _threshold && 
        _analogSensors[2] > _threshold && 
        _analogSensors[3] < _threshold && 
        _analogSensors[4] < _threshold) {
        current_node = 'L';
    }
    // OOXXX - Right
    else if (_analogSensors[0] < _threshold && 
             _analogSensors[1] < _threshold && 
             _analogSensors[2] > _threshold && 
             _analogSensors[3] > _threshold && 
             _analogSensors[4] > _threshold) {
        current_node = 'R';
    }
    // XXXXX - T/Cross/End
    else if (_analogSensors[0] > _threshold && 
             _analogSensors[1] > _threshold && 
             _analogSensors[2] > _threshold && 
             _analogSensors[3] > _threshold && 
             _analogSensors[4] > _threshold) {
        current_node = 'B';
    }
    // OOOOO - Lost/White
    else if (_analogSensors[0] < _threshold && 
             _analogSensors[1] < _threshold && 
             _analogSensors[2] < _threshold && 
             _analogSensors[3] < _threshold && 
             _analogSensors[4] < _threshold) {
        current_node = 'W';
    }
    // Normal line patterns
    else if ((_analogSensors[1] > _threshold) || 
             (_analogSensors[2] > _threshold) || 
             (_analogSensors[3] > _threshold)) {
        current_node = 'N';
    }
    else {
        current_node = 'E';
    }
    
    // Stability check
    if (current_node == last_node) {
        stability_count++;
    } else {
        stability_count = 0;
        last_node = current_node;
    }
    
    if (stability_count >= required_stability) {
        return current_node;
    }
    
    return 'E';
}

JunctionType LineSensor::detectJunction()
{
    char node = detectNode();
    
    switch (node) {
        case 'L': return JUNCTION_LEFT;
        case 'R': return JUNCTION_RIGHT;
        case 'B': return JUNCTION_T;
        case 'W': return JUNCTION_LOST;
        case 'N': return JUNCTION_NONE;
        default:  return JUNCTION_NONE;
    }
}

// ============================================================================
// UTILITY
// ============================================================================

uint32_t LineSensor::encodeIRSensors()
{
    uint32_t result = _analogSensors[0] >> 4;
    for (int c = 1; c < IR_SENSOR_COUNT; c++) {
        result = (result << 6) | (_analogSensors[c] >> 4);
    }
    return result;
}

int LineSensor::getAnalogSensorValue(int index)
{
    if (index >= 0 && index < IR_SENSOR_COUNT) {
        return _analogSensors[index];
    }
    return 0;
}

void LineSensor::printValues()
{
    Serial.print("IR:[");
    for (int i = 0; i < IR_SENSOR_COUNT; i++) {
        if (i > 0) Serial.print(",");
        if (_analogSensors[i] < 100) Serial.print(" ");
        if (_analogSensors[i] < 10) Serial.print(" ");
        Serial.print(_analogSensors[i]);
    }
    Serial.print("] Pos:");
    Serial.print(_linePosition);
    Serial.print(" | Node:");
    Serial.println(detectNode());
}

void LineSensor::setThreshold(int threshold)
{
    _threshold = threshold;
}

int LineSensor::getThreshold()
{
    return _threshold;
}