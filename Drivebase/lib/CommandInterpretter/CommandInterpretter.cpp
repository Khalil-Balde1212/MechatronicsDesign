#include <Arduino.h>
#include "CommandInterpretter.h"
#include <vector>
#include <string>

namespace CommandInterpreter {
    void begin() {
        commands.begin();
        Serial.println("Command Interpreter Initialized");
    }

    bool registerCommand(const command& cmd) {
        for (const auto& existingCmd : commands) {
            if (strcmp(existingCmd.name, cmd.name) == 0) {
            return false; // Command name already exists
            }
        }
        commands.push_back(cmd);
        return true;
    }

    void periodic() {
        if(Serial.available() > 0) {
            String input = Serial.readStringUntil('\n');
            input.trim(); // Remove whitespace

            for (const auto& cmd : commands) {
                if (input.startsWith(cmd.name)) {
                    // parse arguments by delimiting with space
                    const std::string* args = input.length() > strlen(cmd.name) ? 
                        new std::string(input.substring(strlen(cmd.name)).c_str()) : nullptr;
                    
                    if(args[0] == "h" || args[0] == "help") // Print help message
                    {
                        Serial.println(cmd.help);
                        return;
                    }
                    
                    cmd.function(args);
                    return;
                }
            }
            Serial.println("Unknown command. Type 'help' for a list of commands.");
        }
    }
}