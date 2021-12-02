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
    unsigned int lastUsedIndex = 0;
    unsigned int currentIndex = 1;

    uint8_t codeNum = NULL;
    
    GCodeCommandField commandParameters[7] = {GCodeCommandField()};
    uint8_t currentParameter = 0;
    bool foundIdentifier = false;

    //After it finds a "word" between an alpha identifier and the next whitespace it'll set the code/parameter and search for the next one  
    while (currentParameter < 7 && currentIndex < stringLength) {
        
        
        if (codeNum == NULL) {
            if (isWhitespace(str[currentIndex]) || currentIndex + 1 == stringLength) {
                for (int i=lastUsedIndex+1; i < (currentIndex); i++) if (isalpha(str[i])) return false;
                //last used index is 0, currentIndex is the sapce at the end of the code identifier ex. "G155 "
                codeNum = str.substring(lastUsedIndex, currentIndex - 1).toInt();
                lastUsedIndex = currentIndex;
            } else if (isalpha(str[currentIndex])) return false; //assumes index 0 is the initial identifier

        } else {
            if (isalpha(str[currentIndex])) { //Command parameter identifier
                if (foundIdentifier) return false; //alpha char where it shouldnt be

                foundIdentifier = true;
                lastUsedIndex = currentIndex;
            } else if (foundIdentifier && (isWhitespace(str[currentIndex]) || currentIndex + 1 == stringLength)) {
                for (int i=lastUsedIndex+1; i < (currentIndex); i++) if (isalpha(str[i])) return false;
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