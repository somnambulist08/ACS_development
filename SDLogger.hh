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

#include "SDInterface.hh"

class SDLogger : public SDInterface {
public:
    SDLogger();
    void writeLog();
    void closeFile();
    void openFile();
private:
    //FILE
}








#endif //SD_LOGGER_HH
