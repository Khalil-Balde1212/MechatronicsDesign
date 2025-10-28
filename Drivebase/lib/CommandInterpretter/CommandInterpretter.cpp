#include <Arduino.h>
#include "CommandInterpretter.h"
#include <vector>
#include <string>

namespace CommandInterpreter {
    std::vector<command> commands; // Define the commands vector
    
    void begin() {
        commands.clear(); // Use clear() instead of begin() for vector
        Serial.println("Command Interpreter Initialized");

        /*
        * Example command registration
        */
        registerCommand({"ping", [](const std::string* args)
        {
            Serial.println("pong");
        },
        "Usage: ping ## \n Sends 'pong' response. Optionally specify number of times to respond."
        });

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
                    // Extract arguments after the command name
                    String argsString = "";
                    if (input.length() > strlen(cmd.name)) {
                        argsString = input.substring(strlen(cmd.name));
                        argsString.trim(); // Remove leading/trailing whitespace
                    }
                    
                    // Check for help request
                    if(argsString == "h" || argsString == "help") {
                        Serial.println(cmd.help);
                        return;
                    }
                    
                    // Convert to std::string and call the function
                    std::string args(argsString.c_str());
                    cmd.function(&args);
                    return;
                }
            }
            Serial.println("Unknown command. Type 'help' for a list of commands.");
        }
    }
}