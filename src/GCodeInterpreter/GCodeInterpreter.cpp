/*
  GCodeInterpreter.cpp - Class for interpreting serial inputs into GCodeCommands for use with arduino based cnc.
  Created by Landon R. Faris, June 2, 2021.
*/


#include "Arduino.h"
#include "GCodeInterpreter.h"

GCodeInterpreter::GCodeInterpreter(int bufferLength=100)
:bufferLength(bufferLength), commandBuffer{new GCodeCommand[bufferLength]{ GCodeCommand() }} {}

int GCodeInterpreter::getBufferLength() { return bufferLength; }

boolean GCodeInterpreter::interpretCommandString(String str) {
    char codeType;
    if (isalpha(str[0])) codeType = str[0];
    else return false;

    unsigned int stringLength = sizeof(str)/sizeof(str[0]);
    int lastUsedIndex = 0;
    int currentIndex = 1;

    uint8_t codeNum = 255;
    
    GCodeCommandField commandParameters[7] = {GCodeCommandField()};
    uint8_t currentParameter = 0;
    bool foundIdentifier = false;

    while (currentParameter != 7 && currentIndex < stringLength) {
        if (codeNum == 255) {
            if (isWhitespace(str[currentIndex])) {
                //"G00" is a real command code but "G0L" is not. Checked using isalpha()
                for (int i=lastUsedIndex+1; i < (currentIndex-1); i++) if (isalpha(str[i])) return false;

                codeNum = str.substring(lastUsedIndex, currentIndex - 1).toInt();
                lastUsedIndex = currentIndex;
            } else if (isalpha(str[currentIndex])) return false;
        } else {
            if (isalpha(str[currentIndex])) {
                foundIdentifier = true;
                lastUsedIndex = currentIndex;
            } else if (isWhitespace(str[currentIndex]) && foundIdentifier) {
                for (int i=lastUsedIndex+1; i < (currentIndex-1) - (lastUsedIndex+1); i++) if (isalpha(str[i])) return false;
                commandParameters[currentParameter] = GCodeCommandField(str[lastUsedIndex], str.substring(lastUsedIndex + 1, currentIndex - 1).toInt());

                currentParameter++;
                foundIdentifier = false;
                lastUsedIndex = currentIndex;
            }
        }
        currentIndex++;
    }

    //GCodeCommand(char codeType, uint8_t codeNum, GCodeCommandField commandParameters[7]) 
    //GCodeCommandField(char identifier=NULL, float fieldValue=0)
    addCommand(GCodeCommand(codeType, codeNum, commandParameters));
    return true;
}

boolean GCodeInterpreter::addCommand(GCodeCommand command) {
    if (numberOfCommands < bufferLength) {
        incrementBufferArray();
        commandBuffer[0] = command;
    }
}

GCodeCommand GCodeInterpreter::getNextCommand() {
    GCodeCommand nextCommand = commandBuffer[numberOfCommands-1];
    commandBuffer[numberOfCommands-1] = GCodeCommand();
    return nextCommand;
}

void GCodeInterpreter::emptyBuffer() {for (int i=99; i>99-numberOfCommands; i--) { commandBuffer[i] = GCodeCommand(); }}

void GCodeInterpreter::incrementBufferArray() {
    for (int i=numberOfCommands; i>0; i++) {
        commandBuffer[i] = commandBuffer[i-1];
    }
    numberOfCommands++;
}

// unsigned int numberOfCommands = 0;
// GCodeCommand commandBuffer[100]{ GCodeCommand() };