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
    _commandBuffer = "";
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
    if (!_server) return;
    
    // Check for new client
    WiFiClient newClient = _server->available();
    
    if (newClient) {
        // Disconnect previous client if exists
        if (_client && _client.connected()) {
            _client.stop();
        }
        
        _client = newClient;
        _isConnected = true;
        _commandBuffer = "";  // Clear buffer on new connection
        
        Serial.println("WiFi: Client connected");
        println("\n=== Robot Terminal ===");
        println("Type 'help' for commands");
        println("=====================\n");
        print("> ");
    }
    
    // Check if client disconnected
    if (_isConnected && !_client.connected()) {
        _client.stop();
        _isConnected = false;
        _commandBuffer = "";
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

void WiFiTerminal::println(const char* str)
{
    if (isConnected()) {
        _client.println(str);
    }
}

void WiFiTerminal::print(const String& str)
{
    if (isConnected()) {
        _client.print(str);
    }
}

void WiFiTerminal::println(const String& str)
{
    if (isConnected()) {
        _client.println(str);
    }
}

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

void WiFiTerminal::print(float val, int decimalPlaces)
{
    if (isConnected()) {
        _client.print(val, decimalPlaces);
    }
}

void WiFiTerminal::println(float val, int decimalPlaces)
{
    if (isConnected()) {
        _client.println(val, decimalPlaces);
    }
}

void WiFiTerminal::println()
{
    if (isConnected()) {
        _client.println();
    }
}

// ============================================================================
// DATA RECEPTION - CORRIGIDO
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
    if (!isConnected()) {
        return "";
    }
    
    // Read all available characters into buffer
    while (_client.available()) {
        char c = _client.read();
        
        // Handle backspace (ASCII 8 or DEL 127)
        if (c == '\b' || c == 127) {
            if (_commandBuffer.length() > 0) {
                _commandBuffer.remove(_commandBuffer.length() - 1);
                // Send backspace sequence to terminal
                _client.print("\b \b");
            }
            continue;
        }
        
        // Ignore carriage return
        if (c == '\r') {
            continue;
        }
        
        // Check for newline (end of command)
        if (c == '\n') {
            if (_commandBuffer.length() > 0) {
                String command = _commandBuffer;
                _commandBuffer = "";  // Clear buffer for next command
                
                // Send newline
                _client.println();
                
                return command;
            }
            continue;
        }
        
        // Add printable characters (NO ECHO - Telnet client handles it)
        if (c >= 32 && c <= 126) {
            _commandBuffer += c;
        }
    }
    
    // No complete line yet
    return "";
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