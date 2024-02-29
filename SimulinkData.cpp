#include "SimulinkData.hh"
#include <SD.h>
#include <list>

File simulation_log;
std::list<float> alt={};
std::list<float> acc={};
std::list<float> time_steps={};
std::list<float>::iterator alt_iter;
std::list<float>::iterator acc_iter;
std::list<float>::iterator time_iter;
class SimulinkData : public GetData {
// 5 X 4 array
#define ROW_DIM 5
#define COL_DIM 3

File file;

/*
 * Read a file one field at a time.
 *
 * file - File to read.
 *
 * str - Character array for the field.
 *
 * size - Size of str array.
 *
 * delim - String containing field delimiters.
 *
 * return - length of field including terminating delimiter.
 *
 * Note, the last character of str will not be a delimiter if
 * a read error occurs, the field is too long, or the file
 * does not end with a delimiter.  Consider this an error
 * if not at end-of-file.
 *
 */
size_t readField(File* file, char* str, size_t size, char* delim) {
  char ch;
  size_t n = 0;
  while ((n + 1) < size && file->read(&ch, 1) == 1) {
    // Delete CR.
    if (ch == '\r') {
      continue;
    }
    str[n++] = ch;
    if (strchr(delim, ch)) {
        break;
    }
  }
  str[n] = '\0';
  return n;
}

  // Array for data.
  int array[ROW_DIM][COL_DIM];
  int i = 0;     // First array index.
  int j = 0;     // Second array index
  size_t n;      // Length of returned field with delimiter.
  char str[20];  // Must hold longest field with delimiter and zero byte.
  char *ptr;     // Test for valid field.

  // Read the file and store the data.
  
  for (i = 0; i < ROW_DIM; i++) {
    for (j = 0; j < COL_DIM; j++) {
      n = readField(&file, str, sizeof(str), ",\n");
      if (n == 0) {
        errorHalt("Too few lines");
      }
      array[i][j] = strtol(str, &ptr, 10);
      if (ptr == str) {
        errorHalt("bad number");
      }
      while (*ptr == ' ') {
        ptr++;
      }
      if (*ptr != ',' && *ptr != '\n' && *ptr != '\0') {
        errorHalt("extra characters in field");
      }
      if (j < (COL_DIM-1) && str[n-1] != ',') {
        errorHalt("line with too few fields");
      }
    }
    // Allow missing endl at eof.
    if (str[n-1] != '\n' && file.available()) {
      errorHalt("missing endl");
    }    
  }

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
        if (!readLine(simulation_log, line, sizeof(line))) {
            return false;  // EOF or too long
        }
        *alt = strtof(line, &ptr);
        if (ptr == line) return false;  // bad number if equal
        while (*ptr) {
            if (*ptr++ == ',') break;
        }
        *acc = strtof(ptr, &str);
        if (ptr == line) return false;  // bad number if equal
        while (*ptr) {
            if (*ptr++ == ',') break;
        }
        *t = strtof(ptr, &str);
        return str != ptr;  // true if number found  
    }

    void startupTasks(){
        //Start the SD card
        while(!SD.begin());
        //Look for the source file in the root directory
        if(SD.exists("SimulinkLog.csv"))
            simulation_log = SD.open("SimulinkLog.csv", FILE_READ);
        else
          while(1==1) digitalWrite(23,1);
        float h,a,t;
        while(readVals(&h,&a,&t)){
            alt.push_back(h);
            acc.push_back(a);
            time_steps.push_back(t);
        }
        alt_iter=alt.begin();
        acc_iter=acc.begin();
        time_iter=time_steps.begin();
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

};
	