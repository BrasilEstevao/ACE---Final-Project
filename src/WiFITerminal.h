
#ifndef WIFI_TERMINAL_H
#define WIFI_TERMINAL_H

#include <WiFi.h>

/**
 * @class WiFiTerminal
 * @brief Wireless terminal for robot debugging and control
 * 
 * Creates a WiFi Access Point and Telnet server to allow wireless
 * serial-like communication with the robot.
 * 
 * Usage:
 * 1. Robot creates WiFi AP (e.g., "PicoRobot")
 * 2. Connect to AP with laptop/phone
 * 3. Use telnet to connect: telnet 192.168.4.1
 * 4. Send/receive commands and debug data
 */
class WiFiTerminal {
public:
    /**
     * @brief Constructor
     */
    WiFiTerminal();
    
    /**
     * @brief Initialize WiFi AP and Telnet server
     * @param ssid Access Point name
     * @param password Access Point password (min 8 chars)
     * @return true if successful
     */
    bool begin(const char* ssid, const char* password);
    
    /**
     * @brief Update connection (call in main loop)
     * 
     * Checks for new clients and handles disconnections
     */
    void update();
    
    /**
     * @brief Check if client is connected
     * @return true if client connected
     */
    bool isConnected();
    
    /**
     * @brief Send string to client
     * @param str String to send
     */
    void print(const char* str);
    
    /**
     * @brief Send string with newline
     * @param str String to send
     */
    void println(const char* str);
    
    /**
     * @brief Send string (overload)
     * @param str String to send
     */
    void print(const String& str);
    
    /**
     * @brief Send string with newline (overload)
     * @param str String to send
     */
    void println(const String& str);
    
    /**
     * @brief Send integer
     * @param val Integer to send
     */
    void print(int val);
    
    /**
     * @brief Send integer with newline
     * @param val Integer to send
     */
    void println(int val);
    
    /**
     * @brief Check if data available to read
     * @return Number of bytes available
     */
    int available();
    
    /**
     * @brief Read single byte
     * @return Byte value, or -1 if none available
     */
    int read();
    
    /**
     * @brief Read line of text
     * @return String containing line (without newline)
     */
    String readLine();
    
    /**
     * @brief Get AP IP address
     * @return IP address
     */
    IPAddress getIP();
    
    /**
     * @brief Get number of connected clients
     * @return Client count
     */
    int getClientCount();
    
private:
    WiFiServer* _server;     // Telnet server
    WiFiClient _client;      // Connected client
    bool _isConnected;       // Connection status
};

#endif // WIFI_TERMINAL_H