#include "SimulinkData.hh"
#include <SD.h>

//float altitude[];
//float acceleration[];
//unsigned long t_step[];
File log;
class SimulinkData : public getData {
    bool readLine(File &f, char* line, size_t maxLen) {
        for (size_t n = 0; n < maxLen; n++) {
            int c = f.read();
            if ( c < 0 && n == 0) return false;  // EOF
            if (c < 0 || c == '\n') {
                line[n] = 0;
                return true;
            }
        line[n] = c;
        }
    return false; // line too long
    }

    bool readVals(float* alt, float* acc, unsigned long* t) {
        //float and long are both 4 chars, seperated by one char and terminated by \n
        //[4 + 1 + 4 + 1 + 4 + 1]=15
        char line[40], *ptr, *str;
        if (!readLine(file, line, sizeof(line))) {
            return false;  // EOF or too long
        }
        *alt = strtof(line, &ptr, 10);
        if (ptr == line) return false;  // bad number if equal
        while (*ptr) {
            if (*ptr++ == ',') break;
        }
        *acc = strtof(ptr, &str, 10);
        if (ptr == line) return false;  // bad number if equal
        while (*ptr) {
            if (*ptr++ == ',') break;
        }
        *t = strtoul(ptr, &str, 10);
        return str != ptr;  // true if number found
        
    }

    void startupTasks(){
        //Start the SD card
        while(!SD.begin());
        //Look for the source file in the root directory
        if(SD.exists("SimulinkLog.csv"))
            log = SD.open("SimulinkLog.csv", FILE_READ);
        else
          while(1==1) digitalWrite(23,1);
  }
    void readFile(float& alt, float& acc, )

    }
	void readAcceleration(float &x, float &y, float &z){

    }
    void readAltitude(float &H){

    }
    void readMagneticField(float &x, float &y, float &z){}
	void readGyroscope(float &x, float &y, float &z){}
	void readTemperature(float &T){}
	void readPressure(float &P){}

}
	