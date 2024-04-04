/******************************************************************************
 * SDSpoofer.hh
 * 
 * Spoofs the SD card, replaces its functionality with some serial commands
 * 
 * 
 * 02/23/23 - created file
******************************************************************************/
#ifndef SD_SPOOFER_HH
#define SD_SPOOFER_HH

#include "SDInterface.hh"
#include <Arduino.h>

class SDSpoofer : public SDInterface {
public:
    //personal
    SDSpoofer();
    void openFile(String headers);
    void writeLog(float accel, float vel, float h, float ang, float tSim, float tLaunch, int state);
    void writeLine(String line);
    void writeLog(String log); //duplicate for compatability
    //inherited
  //  virtual void writeLine() override;
    virtual void openFile() override;
    virtual void writeLog() override;
    virtual void closeFile() override;
};










#endif //SD_SPOOFER_HH
