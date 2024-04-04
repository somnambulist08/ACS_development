#include "SDSpoofer.hh"
#include <Arduino.h>

SDSpoofer::SDSpoofer(){
    //Cannot make any calls to Serial in constructor or system will crash
    //Serial.println("SDSpoofer Started");
    //openFile();
}

void SDSpoofer::writeLog(){
    for(int i=0; i<100; i++){
        for(volatile int j=0; j<1000; j++);
        Serial.print("Writing to Log... ");
        Serial.print(i);
        Serial.println("%");
    }
}

void SDSpoofer::writeLog(float accel, float vel, float h, float ang, float tSim, float tLaunch, int state){
    Serial.print(accel);
    Serial.print(", ");
    Serial.print(vel);
    Serial.print(", ");
    Serial.print(h);
    Serial.print(", ");
    Serial.print(ang);
    Serial.print(", ");
    Serial.print(tSim);
    Serial.print(", ");
    Serial.print(tLaunch);
    Serial.print(", ");
    Serial.println(state);

}

void SDSpoofer::writeLog(String log){
    Serial.println(log);
}

void SDSpoofer::writeLine(String line){
  Serial.print(line);
  Serial.println();
}
void SDSpoofer::closeFile()
{
    Serial.println("File Saved");
}

void SDSpoofer::openFile()
{
    Serial.println("New File");
    Serial.println("Writing Headers:");

    //Write headers here...
    Serial.println("a, v, h, tSim, tLaunch, state");
}

void SDSpoofer::openFile(String headers)
{
    Serial.println(headers);
}
