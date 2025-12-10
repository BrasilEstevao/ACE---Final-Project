#ifndef WIFI_TERMINAL_H
#define WIFI_TERMINAL_H

#include <WiFi.h>

/**
 * @class WiFiTerminal
 * @brief WiFi-based terminal for robot control
 * 
 * Creates an Access Point and Telnet server for remote control.
 * Handles line-buffered input (waits for Enter before processing).
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
     * Checks for new clients and handles disconnections
     */
    void update();
    
    /**
     * @brief Check if client is connected
     * @return true if client connected
     */
    bool isConnected();
    
    // ====================================================================
    // DATA TRANSMISSION
    // ====================================================================
    
    void print(const char* str);
    void println(const char* str);
    void print(const String& str);
    void println(const String& str);
    void print(int val);
    void println(int val);
    void print(float val, int decimalPlaces = 2);
    void println(float val, int decimalPlaces = 2);
    void println();
    
    // ====================================================================
    // DATA RECEPTION
    // ====================================================================
    
    /**
     * @brief Check how many bytes available
     * @return Number of bytes available to read
     */
    int available();
    
    /**
     * @brief Read single byte
     * @return Byte read or -1 if none available
     */
    int read();
    
    /**
     * @brief Read complete command line
     * 
     * IMPORTANT: This function buffers characters until newline (\n)
     * Only returns complete lines - empty string if line not ready yet
     * 
     * @return Complete command line (without \n) or empty string
     */
    String readLine();
    
    // ====================================================================
    // UTILITY
    // ====================================================================
    
    /**
     * @brief Get AP IP address
     * @return IP address of the Access Point
     */
    IPAddress getIP();
    
    /**
     * @brief Get number of connected clients
     * @return Client count
     */
    int getClientCount();
    
private:
    WiFiServer* _server;      // Telnet server
    WiFiClient _client;       // Connected client
    bool _isConnected;        // Connection status
    String _commandBuffer;    // Buffer for accumulating command characters
};

#endif // WIFI_TERMINAL_H