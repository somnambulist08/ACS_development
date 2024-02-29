#include "SimulinkData.hh"
#include <SD.h>
#include <list>

File log;
std::list<float> alt={};
std::list<float> acc={};
std::list<float> time={};
std::list<float>::iterator alt_iter;
std::list<float>::iterator acc_iter;
std::list<float>::iterator time_iter;
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

    bool readVals(float* alt, float* acc, float* t) {
        //floats are 4 char, seperated by one char and terminated by \r\n
        //[4 + 1 + 4 + 1 + 4 + 1 + 1]=16
        char line[16], *ptr, *str;
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
        *t = strtof(ptr, &str, 10);
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
        float h,a,t;
        while(readVals(&h,&a,&t)){
            alt.push_back(h);
            acc.push_back(a);
            time.push_back(t);
        }
        alt_iter=alt.begin();
        acc_iter=acc.begin();
        time_iter=time.begin();
  }
	void readAcceleration(float &x, float &y, float &z){
        x = 0.0f;
        y = 0.0f;
        z = *acc_iter;
        ++acc_iter;
    }
    void readAltitude(float &H){
        H= *alt_iter;
        ++alt_iter;
    }
    void readFrame(float &t){
        t = *time_iter;
        ++time_iter;
    }
    void readMagneticField(float &x, float &y, float &z){}
	void readGyroscope(float &x, float &y, float &z){}
	void readTemperature(float &T){}
	void readPressure(float &P){}

}
	