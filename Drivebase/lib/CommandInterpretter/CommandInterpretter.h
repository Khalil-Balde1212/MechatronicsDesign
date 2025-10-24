#ifndef COMMAND_INTERPRETER_H
#define COMMAND_INTERPRETER_H
#include <vector>
#include <string>

namespace CommandInterpreter {

    struct command {
        const char* name;
        void (*function)(const std::string* args);
        const char* help; //help message
    };
    
    extern std::vector<command> commands; // Declare as extern

    void begin();
    bool registerCommand(const command& cmd);
    void periodic();
};

#endif