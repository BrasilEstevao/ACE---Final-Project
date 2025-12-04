#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include <Arduino.h>
#include "config.h"

/**
 * @brief Junction detection types
 */
enum JunctionType {
    JUNCTION_NONE,   // Normal line
    JUNCTION_LEFT,   // Left turn available
    JUNCTION_RIGHT,  // Right turn available
    JUNCTION_T,      // T-junction (left and right)
    JUNCTION_CROSS,  // Cross (all sensors black)
    JUNCTION_LOST    // Lost line (all sensors white)
};

/**
 * @class LineSensor
 * @brief Manages IR sensor array for line detection
 * 
 * Uses 3 analog sensors (center) for PID line following
 * and 2 digital sensors (edges) for junction detection.
 */
class LineSensor {
public:
    /**
     * @brief Constructor
     */
    LineSensor();
    
    /**
     * @brief Read all sensors
     */
    void read();
    
    /**
     * @brief Calibrate analog sensors
     */
    void calibrate();
    
    /**
     * @brief Get line position from center 3 sensors
     * @return Position from -1000 (left) to +1000 (right), 0 = center
     */
    int getPosition();
    
    /**
     * @brief Detect junction type using all 5 sensors
     * @return JunctionType enum value
     */
    JunctionType detectJunction();
    
    /**
     * @brief Get individual analog sensor value (0-2)
     * @param index Sensor index (0=left, 1=center, 2=right)
     * @return Sensor reading (0-1023)
     */
    int getAnalogSensorValue(int index);
    
    /**
     * @brief Get digital sensor state
     * @param index 0=left edge, 1=right edge
     * @return true if black detected, false if white
     */
    bool getDigitalSensorValue(int index);
    
    /**
     * @brief Print sensor values to Serial
     */
    void printValues();
    
    /**
     * @brief Set black/white threshold for analog sensors
     * @param threshold New threshold value
     */
    void setThreshold(int threshold);
    
    /**
     * @brief Get current analog threshold
     * @return Threshold value
     */
    int getThreshold();
    
private:
    // Analog sensor values (center 3 for PID)
    int _analogSensors[NUM_ANALOG_SENSORS];
    
    // Digital sensor states (edge 2 for junctions)
    bool _digitalSensors[NUM_DIGITAL_SENSORS];
    
    int _threshold;        // Black/white threshold for analog
    int _linePosition;     // Calculated line position
    
    // Calibration data for analog sensors
    int _minValues[NUM_ANALOG_SENSORS];
    int _maxValues[NUM_ANALOG_SENSORS];
    bool _calibrated;
};

#endif // LINE_SENSOR_H