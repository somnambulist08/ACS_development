#include "SDSpoofer.hh"
#include <Arduino.h>

SDSpoofer::SDSpoofer()
{
    Serial.println("SDSpoofer Started");
    source.startupTasks();
    Serial.println("Writing Headers:");
    Serial.print("Time[us], Temp[C], Pressure[hPa],");
    Serial.print(" Omega_1[rad/s], Omega_2[rad/s], Omega_3[rad/s], acc_1[g], acc_2[g], acc_3[g],");
    Serial.println(" mag_1[uT], mag_2[uT], mag_3[uT]");

    openFile();
}

void SDSpoofer::writeLog()
{
    for (int i = 0; i < 100; i++)
    {
        Serial.print("Writing to Log... ");
        Serial.print(i);
        Serial.println("%");
    }
    float acc1, acc2, acc3;
    float gy1, gy2, gy3;
    float mag1, mag2, mag3;
    float temp;
    float pressure;
    source.readAcceleration(acc1, acc2, acc3);
    source.readGyroscope(gy1, gy2, gy3);
    source.readMagneticField(mag1, mag2, mag3);
    source.readPressure(pressure);
    source.readTemperature(temp);
    unsigned long t_now = micros();
    Serial.print(String(t_now) + ',' + String(temp) + ',' + String(pressure) + ',');
    Serial.print(String(gy1) + ',' + String(gy2) + ',' + String(gy3) + ',');
    Serial.print(String(acc1) + ',' + String(acc2) + ',' + String(acc3) + ',');
    Serial.println(String(mag1) + ',' + String(mag2) + ',' + String(mag3));
}

void SDSpoofer::closeFile()
{
    Serial.println("File Saved");
}

void SDSpoofer::openFile()
{
    Serial.println("New File");
}