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
    void writeLog(float acc1, float acc2, float acc3, float gy1, float gy2, float gy3, float mag1, float mag2, float mag3, float temp, float pressure);

    //inherited
    void writeLog()=0;
    void closeFile();
    void openFile();
private:
    //FILE
    static File flightData;
    String fileName="";
};








#endif //SD_LOGGER_HH
