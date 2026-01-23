#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include <Arduino.h>
#include "config.h"

enum JunctionType {
    JUNCTION_NONE,   // Normal line
    JUNCTION_LEFT,   // Left turn available
    JUNCTION_RIGHT,  // Right turn available
    JUNCTION_T,      // T-junction (left and right)
    JUNCTION_CROSS,  // Cross (all sensors black)
    JUNCTION_LOST    // Lost line (all sensors white)
};

class LineSensor {
public:
    LineSensor();
    
    void read();
    void calibrate();
    
    int getPosition();
    JunctionType detectJunction();
    
    int getAnalogSensorValue(int index);
    
    void printValues();
    void setThreshold(int threshold);
    int getThreshold();
    int IR_sum();
    
    // New methods for IRLine compatibility
    uint32_t encodeIRSensors();
    char detectNode();
    
private:
    // Sensor values (all from multiplexer)
    int _analogSensors[IR_SENSOR_COUNT];
    
    int _threshold;
    int _linePosition;
    bool _calibrated;
    
    // Calibration data
    int _minValues[IR_SENSOR_COUNT];
    int _maxValues[IR_SENSOR_COUNT];
    
    // Water level (baseline offset)
    int _waterLevel;
    
    // Helper methods
    void setMuxChannel(int channel);
    uint16_t readMuxChannel(int channel);
    void calcLineEdgeLeft();
    void calcLineEdgeRight();
    
    // Edge positions
    float _posLeft;
    float _posRight;
    int _irMax;
    int _total;

    double error;         // Error
    double prevError = 0; // Previous Error for each sensor
    double integral = 0; // Integral for each sensor
    double derivative = 0; // Derivative for each sensor
};

#endif