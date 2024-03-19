#include "BZZT.hh"

#include <Arduino.h>

void bzzt(int bzzBzzCount){
    for(int i=0; i<bzzBzzCount; i++){
        digitalWrite(BZZT, 1);
        delayMicroseconds(500000);
        digitalWrite(BZZT, 0);
        delayMicroseconds(500000);
    }
}
void longBzzt(int bzzBzzCount){
    for(int i=0; i<bzzBzzCount; i++){
        digitalWrite(BZZT, 1);
        delayMicroseconds(1000000);
        digitalWrite(BZZT, 0);
        delayMicroseconds(500000);
    }
}
