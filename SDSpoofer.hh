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
#include "SimulinkData.hh"

class SDSpoofer{// : public SDInterface {
public:
    //personal
    SDSpoofer();
    void writeLog(float accel, float vel, float h, float ang);

    //inherited
    void openFile();
    void writeLog();
    void closeFile();
};










#endif //SD_SPOOFER_HH
