/******************************************************************************
 * SDLogger.hh
 * 
 * Contains the functions to operate the SD card
 * 
 * 
 * 02/23/23 - created file
******************************************************************************/
#ifndef SD_LOGGER_HH
#define SD_LOGGER_HH
#define RED 22
#define BLUE 23
#define GREEN 24

#include "SDInterface.hh"
#include <SD.h>

class SDLogger : public SDInterface {
public:
    //personal
    SDLogger();
    SDLogger(String newFileName);
    void openFile(String newFileName);
    void writeLog(float acc1, float acc2, float acc3, float saam1, float saam2, float saam3, float mag1, float mag2, float mag3, float ang, float alt, float t_now);
    void writeLog(float accel, float vel, float h, float ang);
    void writeLog(float a_raw[], float m[], float a[]);

    //inherited
    void writeLog();
    void closeFile();
    void openFile();
private:
    //FILE
    File flightData;
    String fileName="";
};








#endif //SD_LOGGER_HH
