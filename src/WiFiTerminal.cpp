/**
 * @file WiFiTerminal.cpp
 * @brief WiFi terminal for arduino-pico framework
 */

#include "WiFiTerminal.h"
#include "config.h"
#include <WiFi.h>

// ============================================================================
// CONSTRUCTOR
// ============================================================================

WiFiTerminal::WiFiTerminal()
{
    _server = nullptr;
    _client = WiFiClient();
    _isConnected = false;
}

// ============================================================================
// INITIALIZATION
// ============================================================================

bool WiFiTerminal::begin(const char* ssid, const char* password)
{
    Serial.println("\n=== WiFi Terminal Starting ===");
    
    Serial.print("Creating AP: ");
    Serial.println(ssid);
    
    // Start Access Point
    WiFi.mode(WIFI_AP);
    bool success = WiFi.softAP(ssid, password);
    
    if (!success) {
        Serial.println("ERROR: Failed to create AP");
        return false;
    }
    
    delay(1000);
    
    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);
    
    _server = new WiFiServer(TELNET_PORT);
    _server->begin();
    
    Serial.print("Telnet server started on port ");
    Serial.println(TELNET_PORT);
    Serial.print("Connect with: telnet ");
    Serial.println(IP);
    
    // Wait a bit for server to fully start
    delay(500);
    
    Serial.println("\n============================");
    Serial.println("READY! Waiting for connection...");
    Serial.println("============================\n");
    
    return true;
}

// ============================================================================
// CONNECTION MANAGEMENT
// ============================================================================

void WiFiTerminal::update()
{
    // Check for new client
    WiFiClient newClient = _server->accept();
    
    if (newClient) {
        // Disconnect previous client if exists
        if (_client && _client.connected()) {
            _client.stop();
        }
        
        _client = newClient;
        _isConnected = true;
        
        Serial.println("WiFi: Client connected");
        println("=== Robot Terminal ===");
        println("Type 'help' for commands");
        println("=====================\n");
    }
    
    // Check if client disconnected
    if (_isConnected && !_client.connected()) {
        _client.stop();
        _isConnected = false;
        Serial.println("WiFi: Client disconnected");
    }
}

bool WiFiTerminal::isConnected()
{
    return _isConnected && _client.connected();
}

// ============================================================================
// DATA TRANSMISSION
// ============================================================================

void WiFiTerminal::print(const char* str)
{
    if (isConnected()) {
        _client.print(str);
    }
}

void WiFiTerminal::print(const String& str)
{
    if (isConnected()) {
        _client.print(str);
    }
}

void WiFiTerminal::println(const char* str)
{
    if (isConnected()) {
        _client.println(str);
    }
}

void WiFiTerminal::println(const String& str)
{
    if (isConnected()) {
        _client.println(str);
    }
}

// Helper for formatted output
void WiFiTerminal::print(int val)
{
    if (isConnected()) {
        _client.print(val);
    }
}

void WiFiTerminal::println(int val)
{
    if (isConnected()) {
        _client.println(val);
    }
}

// ============================================================================
// DATA RECEPTION
// ============================================================================

int WiFiTerminal::available()
{
    if (isConnected()) {
        return _client.available();
    }
    return 0;
}

int WiFiTerminal::read()
{
    if (isConnected()) {
        return _client.read();
    }
    return -1;
}

String WiFiTerminal::readLine()
{
    String line = "";
    
    if (isConnected()) {
        while (_client.available()) {
            char c = _client.read();
            
            // Handle backspace
            if (c == '\b' || c == 127) {
                if (line.length() > 0) {
                    line.remove(line.length() - 1);
                    _client.print("\b \b");
                }
                continue;
            }
            
            // Echo character
            _client.print(c);
            
            // Check for end of line
            if (c == '\n' || c == '\r') {
                _client.println();
                break;
            }
            
            line += c;
        }
    }
    
    return line;
}

// ============================================================================
// UTILITY
// ============================================================================

IPAddress WiFiTerminal::getIP()
{
    return WiFi.softAPIP();
}

int WiFiTerminal::getClientCount()
{
    return WiFi.softAPgetStationNum();
}