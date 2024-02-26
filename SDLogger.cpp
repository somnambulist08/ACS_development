#include "SDLogger.hh"

SDLogger::SDLogger(){
    source.startupTasks();
    Serial.println("Initializing SD card...");
  if (!SD.begin()) {
    Serial.println("initialization failed!");
    while (1) {
      digitalWrite(RED, HIGH);
      delay(1000);
      digitalWrite(RED, LOW);
      delay(1000);
    }
  } else {
    Serial.println("SD Initialized");
  }
  //less stupid naming convention - 0000.csv to 9999.csv
  //look through SD card, find the next available filename
  bool fileavail = false;
  int i = 0;
  String index="";
  while (!fileavail) {  
    if (i < 10)
      index = String("000" + String(i) + ".csv");
    else if (i < 100)
      index = String("00" + String(i) + ".csv");
    else if (i < 1000)
      index = String("0" + String(i) + ".csv");
    else
      index = String(String(i) + ".csv");
    fileavail = !SD.exists(index);
    i++;
  }
  flightData = SD.open(index, FILE_WRITE);
  Serial.println("Writing to File: " + index);
  // Write the headers:
  if (flightData) {
    Serial.print("Writing to " + index);
    flightData.print("Time[us], Temp[C], Pressure[hPa],");
    flightData.print(" Omega_1[rad/s], Omega_2[rad/s], Omega_3[rad/s], acc_1[g], acc_2[g], acc_3[g],");
    flightData.println(" mag_1[uT], mag_2[uT], mag_3[uT]");
    flightData.close();
    Serial.println("...headers done.");
    fileName = flightData.name();
  } else {
    Serial.println("error opening " + index);
    //red light means stop
    while (1) {
      digitalWrite(RED, HIGH);
      delay(500);
      digitalWrite(RED, LOW);
      delay(500);
    }
  }
  lastWrite = micros();
  openFile();
}

void SDLogger::writeLog(){
    float acc1, acc2, acc3;
    float gy1, gy2, gy3;
    float mag1, mag2, mag3;
    float temp;
    float pressure;
    source.readAcceleration(acc1, acc2, acc3);
    source.readGyroscope(gy1,gy2,gy3);
    source.readMagneticField(mag1,mag2,mag3);
    source.readPressure(pressure);
    source.readTemperature(temp);
    unsigned long t_now = micros();
  flightData.print(String(t_now) + ',' + String(temp) + ',' + String(pressure) + ',');
  flightData.print(String(gy1) + ',' + String(gy2) + ',' + String(gy3) + ',');
  flightData.print(String(acc1) + ',' + String(acc2) + ',' + String(acc3) + ',');
  flightData.println(String(mag1) + ',' + String(mag2) + ',' + String(mag3));
  flightData.flush();

}

void SDLogger::closeFile(){
    if (flightData){
        flightData = flightData.close();
    }

}

void SDLogger::openFile(){
    if(SD.exists(fileName))
        flightData = SD.open(index, FILE_WRITE);
}