#include "SDSpoofer.hh"
#include <Arduino.h>

SDSpoofer::SDSpoofer(){
    Serial.println("SDSpoofer Started");
    openFile();
}

void SDSpoofer::writeLog(){
    for(int i=0; i<100; i++){
        Serial.print("Writing to Log... ");
        Serial.print(i);
        Serial.println("%");
    }
}

void SDSpoofer::closeFile(){
    Serial.println("File Saved");
}

void SDSpoofer::openFile(){
    Serial.println("New File");
}