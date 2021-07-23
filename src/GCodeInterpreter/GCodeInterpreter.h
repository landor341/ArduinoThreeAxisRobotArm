/*
  GCodeInterpreter.cpp - Class for interpreting gcode from serial monitor into commands for robot arm classes.
  Created by Landon R. Faris, June 2, 2021.
*/

#ifndef GCodeInterpreter_h
#define GCodeInterpreter_h

#include "Arduino.h"
#include "../ThreeAxisArm/ThreeAxisArm.h"

struct GCodeCommandField {
    char identifier;
    float fieldValue;

    GCodeCommandField(char identifier=NULL, float fieldValue=0)
    :identifier(identifier), fieldValue(fieldValue) {}
};

struct GCodeCommand {
    unsigned char codeType;
    uint8_t codeNum:3;
    GCodeCommandField commandParameters[7];

    GCodeCommand(char codeType, uint8_t codeNum, GCodeCommandField commandParameters[7]) 
    :codeType(codeType), codeNum(codeNum), //I couldn't find a better way to get a parameter array to a member array in an initializer list 
    commandParameters{commandParameters[0], commandParameters[1], commandParameters[2], commandParameters[3], commandParameters[4], commandParameters[5], commandParameters[6]} 
    {}

    GCodeCommand() :codeType(0), codeNum(0), commandParameters{ 0 } {}
};

class GCodeInterpreter {
    public:
        GCodeInterpreter(int bufferLength=100);

        int getBufferLength();

        boolean interpretCommandString(String str);
        boolean addCommand(GCodeCommand command);

        GCodeCommand getNextCommand();
        void emptyBuffer();


    private:
        void incrementBufferArray();

        unsigned int numberOfCommands = 0;
        unsigned int bufferLength;
        GCodeCommand * commandBuffer;
};

#endif

/*

Will use gcode reference these two sources:
https://github.com/cgxeiji/CGx-Gcode-RobotArm
https://makezine.com/2016/10/24/get-to-know-your-cnc-how-to-read-g-code/

for retrieving data the following functions would work:
https://www.arduino.cc/reference/en/language/functions/communication/serial/readstring/
https://www.arduino.cc/reference/en/language/functions/communication/serial/readbytes/

needs to compile who program before it runs to check for errors

*/