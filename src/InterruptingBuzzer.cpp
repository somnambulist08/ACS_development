#include <InterruptingBuzzer.hh>
#include <Arduino.h>

IntervalTimer buzzTick;
int sequenceIndex = 0;
int currentSequenceLength = 0;

void initializeBuzzer(){
    pinMode(BUZZ_PIN, OUTPUT);
    buzzTick.priority(BUZZER_TIMER_INTERRUPT_PRIORITY);

}
void enablePreBuzz(){
    currentSequenceLength = PRE_BUZZ_SEQUENCE_LENGTH;
    sequenceIndex = 0;
    buzzTick.begin(buzzInterupt, buzzSequence[0]);
}
void enablePostBuzz(){
    currentSequenceLength = POST_BUZZ_SEQUENCE_LENGTH;
    sequenceIndex = 0;
    buzzTick.begin(buzzInterupt, buzzSequence[0]);
}
void disableBuzz(){
    buzzTick.end();
    digitalWrite(BUZZ_PIN, 0);
}


void buzzInterupt(){
    digitalToggleFast(BUZZ_PIN);
    sequenceIndex = (sequenceIndex+1) % currentSequenceLength;
    buzzTick.update(buzzSequence[sequenceIndex]);
}
