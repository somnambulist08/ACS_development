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

class SDSpoofer : public SDInterface {
public:
    SDSpoofer();
    void writeLog();
    void closeFile();
    void openFile();
private:
    
};










#endif //SD_SPOOFER_HH
