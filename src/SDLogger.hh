/******************************************************************************
 * SDLogger.hh
 * 
 * Contains the functions to operate the SD card
 * 
 * 
 * 02/23/24 - created file
 * 03/28/24 - adapted to teensy
******************************************************************************/
#ifndef SD_LOGGER_HH
#define SD_LOGGER_HH

#define MAX_FILE_NAME_LENGTH 256

#include "SDInterface.hh"
#include <SD.h>

const int chipSelect = BUILTIN_SDCARD;//Tells Teensy to use the on-board SD
class SDLogger : public SDInterface {
public:
    //personal
    SDLogger();
    SDLogger(String newFileName);
    void openFile(String headerString);
    void writeLog(float acc1, float acc2, float acc3, float saam1, float saam2, float saam3, float mag1, float mag2, float mag3, float ang, float alt, float t_now, int state);
    void writeLog(float accel, float vel, float h, float ang);
    void writeLog(float a_raw[], float m[], float a[]);
    void writeLine(String line);
    //inherited
    //virtual void writeLine() override;
    virtual void writeLog() override;
    virtual void closeFile() override;
    virtual void openFile() override;
private:
    //FILE
    File flightData;
    String fileName="";
};








#endif //SD_LOGGER_HH
