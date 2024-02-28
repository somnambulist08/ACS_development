#include "RocketRTOS.hh"

#ifdef CUSTOM_SCHEDULER
rtos::Thread *threads[NUM_THREADS] = {}; //array of pointers to threads
#endif //CUSTOM_SCHEDULER


#ifdef SERIAL_STEPPER_TEST
SerialSpoofStepper stepper;
sixteenAMG sensor;
SDSpoofer dummySD;
unsigned long oldMicros;
float velocity;
float oldAccel;

/*
void loop(){
    yield();
}*/
#endif //SERIAL_STEPPER_TEST


void startRocketRTOS(){

#ifdef CUSTOM_SCHEDULER
    
#endif //CUSTOM_SCHEDULER

#ifdef LOOP_CHECKIN_TEST
    Scheduler.startLoop(loop2);
    Scheduler.startLoop(loop3);
#endif //LOOP_CHECKIN_TEST

#ifdef DUMMY_FUNCTIONS
    Scheduler.startLoop(getSensorsAndDoControlTask);
    Scheduler.startLoop(logToFileTask);
    Scheduler.startLoop(moveStepperTask);
#endif //DUMMY_FUNCTIONS

#ifdef SERIAL_STEPPER_TEST
    //stepper = new SerialSpoofStepper();
    //dummySD = new SDSpoofer();
    //sensor = new sixteenAMG();

    oldAccel = 0;
    velocity = 0;
    oldMicros = micros();
#ifndef CUSTOM_SCHEDULER
    Scheduler.startLoop(sensorAndControlTask);
    Scheduler.startLoop(logTask);
    Scheduler.startLoop(stepperTask);
#else //defined: CUSTOM SCHEDULER
    threads[0] = new rtos::Thread(osPriorityHigh, THREADS_STACK_DEPTH);
    threads[0]->start(sensorAndControlTask);
    threads[1] = new rtos::Thread(osPriorityHigh, THREADS_STACK_DEPTH);
    threads[1]->start(logTask);
    threads[2] = new rtos::Thread(osPriorityHigh, THREADS_STACK_DEPTH);
    threads[2]->start(stepperTask);
#endif //ifndef CUSTOM SCHEDULER

#endif //SERIAL_STEPPER_TEST

}



#ifdef CUSTOM_SCHEDULER




#endif //CUSTOM_SCHEDULER



#ifdef SERIAL_STEPPER_TEST
void stepperTask(){
#ifdef CUSTOM_SCHEDULER
    while(1){
#endif CUSTOM_SCHEDULER
    if(!stepper.getMoveSteps()) yield(); //if you don't need to run, don't //jonse

    for(int i=0; i<STEPS_PER_PREEMPT; i++){
        stepper.stepOnce();
    }
    
    delay(PULSE_PERIOD_MS);
}
#ifdef CUSTOM_SCHEDULER
}
#endif CUSTOM_SCHEDULER


void sensorAndControlTask(){
#ifdef CUSTOM_SCHEDULER
    while(1){
#endif CUSTOM_SCHEDULER
    float x=0, y=0, z=0;
    sensor.readAcceleration(x,y,z);
    Serial.print("Acceleration: ");
    Serial.print(x);
    Serial.print(", ");
    Serial.print(y);
    Serial.print(", ");
    Serial.println(z);

    
    float pressure = 80000;
    Serial.print("Pressure: ");
    Serial.println(pressure);

    float altitude = (1 - powf((pressure / 101325), 0.190284)) * 145366.45 * 0.3048;
    Serial.print("Altitude: ");
    Serial.println(altitude);

    float dt = micros() - oldMicros;
    velocity = (dt * (z + oldAccel) / 2.0);
    Serial.print("Velocity: ");
    Serial.println(velocity);

    float flaps = getControl(100, predictAltitude(altitude, velocity), dt);
    Serial.print("Control angle: ");
    Serial.println(flaps);

    stepper.setStepsTarget(stepper.microStepsFromFlapAngle(flaps));


    delay(CONTROL_PERIOD_MS);
}
#ifdef CUSTOM_SCHEDULER
}
#endif CUSTOM_SCHEDULER

void logTask(){
#ifdef CUSTOM_SCHEDULER
    while(1){
#endif CUSTOM_SCHEDULER
    dummySD.writeLog();

    delay(LOG_PERIOD_MS);
}
#ifdef CUSTOM_SCHEDULER
}
#endif CUSTOM_SCHEDULER



#endif //SERIAL_STEPPER_TEST









#ifdef DUMMY_FUNCTIONS
void loop(){

    yield(); // basically just means the main loop should be ignored
}


void getSensorsAndDoControlTask(){
    
    for(volatile unsigned int i=0; i<1000; i++); //waste thousands of clock cycles
    Serial.println("Sensor Data Aquired");

    delay(SENSOR_PERIOD_MS);
}
void logToFileTask(){

    for(volatile unsigned int i=0; i<100000; i++); //waste MORE cycles!
    Serial.println("Writing to File Complete");

    delay(FILE_PERIOD_MS);
}
void moveStepperTask(){

    for(volatile unsigned int i=0; i<100; i++); //waste a few cycles
    Serial.println("Stepper Moved");

    delay(STEPPER_PERIOD_MS);
}
#endif

#ifdef LOOP_CHECKIN_TEST
void loop2(){
    Serial.println("Loop2, checking in");
    delay(2000);
}

void loop3(){
    Serial.println("Loop3, checking in");
    delay(3000);
}

//this is the main loop. Will this even work or compile? //it sure does
void loop(){
  Serial.println("Hiiii! I'm the main thread!");
  delay(1000); 
}
#endif //LOOP_CHECKIN_TEST