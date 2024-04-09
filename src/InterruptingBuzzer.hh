#ifndef INTERRUPTING_BUZZER_HH
#define INTERRUPTING_BUZZER_HH

#include <IntervalTimer.h>

#define BUZZ_PIN 39

#define SHORT_BUZZ 250000 //0.25 sec
#define SHORT_PAUSE 500000 //0.5 sec
#define LONG_PAUSE 5000000 //5.0 sec

#define BUZZER_TIMER_INTERRUPT_PRIORITY 132

extern IntervalTimer buzzTick;
#define PRE_BUZZ_SEQUENCE_LENGTH 4
#define POST_BUZZ_SEQUENCE_LENGTH 2 //only do the first two of the sequence
const unsigned long buzzSequence[PRE_BUZZ_SEQUENCE_LENGTH] = {SHORT_BUZZ, LONG_PAUSE, SHORT_BUZZ, SHORT_PAUSE};

extern int currentSequenceLength;
extern int sequenceIndex;

void initializeBuzzer();
void enablePreBuzz();
void enablePostBuzz();
void disableBuzz();

void buzzInterupt();



#endif //INTERRUPTING_BUZZER_HH