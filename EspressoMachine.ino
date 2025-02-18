#include "max6675.h" // max6675.h file for reading thermocouples
#include "RBDdimmer.h" // RBDdimmer.h file for running dimmer module


// DIMMER PIN ALLOCATION
const int zeroCross = 2;
const int psm = 4; /// aka the output pin
dimmerLamp dimmer(psm); // initialase port for dimmer

// THERMOCOUPLE PIN ALLOCATION
const int soPin_boiler = 10; // assign pins for boiler thermocouple
const int csPin_boiler = 9;
const int sckPin_boiler =8;

const int soPin_grouphead = 13; // assign pins for grouphead thermocouple
const int csPin_grouphead = 12;
const int sckPin_grouphead = 11;

MAX6675 boilerTemp(sckPin_boiler, csPin_boiler, soPin_boiler); // create instance object of MAX6675
MAX6675 groupheadTemp(sckPin_grouphead, csPin_grouphead, soPin_grouphead); // create instance object of MAX6675

// RELAY PIN ALLOCATION
const int boiler_heater_trigger = 3;
const int grouphead_heater_trigger = 5;
const int grouphead_solenoid_trigger = 6;
const int dumpprime_solenoid_trigger = 7;


// BUTTON PIN ALLOCATION
const int boiler_purge_button_pin = 14;
const int grouphead_purge_button_pin = 15;
const int preheat_button_pin = 16;
const int start_button_pin = 17;
const int estop_button_pin = 18;

// PRESSURE SENSOR SETUP
const int pressure_sensor_signal_pin = A5;
const int pressureZero = 102.4; // analog reading of pressure sensor at 0psi, 0.5V = 0psi, arduino scale from 0-1023
const int pressureMax = 921.6; //analog reading of pressure sensor at 150psi, 4.5V = 150psi, arduino scale from 0-1023
const int pressuretransducermaxPSI = 150; // psi value of transducer being used
float pressureValue = 0; // psig of the boiler


// SETTING UP DIFFERENT STATE TYPES
enum States{
  STANDBY,
  WAITING_FOR_PREHEAT,
  PREHEAT,
  READY,
  RUN,
  ESTOP
};

char *enumStateConvert[] = {"STANDBY", "WAITING_FOR_PREHEAT", "PREHEAT", "READY", "RUN", "ESTOP"};

States state = STANDBY; // state starts in standby
int grouphead_purge_button_click_counter = 0;
int boiler_purge_button_click_counter = 0;


//setup PI control for boiler
double dt;
double last_time, integral, previous, output = 0;
double kp_boiler = 5;
double ki_boiler = 0;
double kd_boiler = 1;
double setpoint_boiler = 95; //sets the boiler temp setpoint in C
double dutyCycle = 0;

//SETTING UP TIMERS FOR BOILER
int boilerState = LOW; //boiler starts off
unsigned long lastTime = 0; //the last time  
double period = 3000; //period of the PWM milliseconds
bool new_cycle = true; //sets the start of the PWM
double interval_ON = 0;
double interval_OFF = period;



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); 

  // SETUP BUTTONS
  pinMode(boiler_purge_button_pin, INPUT_PULLUP); // BOILER PURGE BUTTON SETUP
  pinMode(grouphead_purge_button_pin, INPUT_PULLUP); // GROUPHEAD PURGE BUTTON SETUP
  pinMode(preheat_button_pin, INPUT_PULLUP); // PREHEAT BUTTON SETUP
  pinMode(start_button_pin, INPUT_PULLUP); // START BUTTON SETUP
  pinMode(estop_button_pin, INPUT_PULLUP); // E-STOP BUTTON SETUP

  //setup relayS
  pinMode(boiler_heater_trigger, OUTPUT);
  pinMode(grouphead_heater_trigger, OUTPUT);
  pinMode(grouphead_solenoid_trigger, OUTPUT);
  pinMode(dumpprime_solenoid_trigger, OUTPUT);

  //set relays to OFF initially, HIGH is OFF, LOW IS ON
  digitalWrite(boiler_heater_trigger, boilerState); //HIGH IS OFF, LOW IS ON
  digitalWrite(grouphead_heater_trigger, HIGH); //HIGH is OFF, LOW IS ON
  digitalWrite(grouphead_solenoid_trigger, HIGH); //HIGH is OFF, LOW IS ON
  digitalWrite(dumpprime_solenoid_trigger, LOW); //starts open HIGH is OFF, LOW IS ON

  //sets dimmer to off when machine turns on
  dimmer.begin(NORMAL_MODE, OFF);

  //setup pressure sensor
  pinMode(pressure_sensor_signal_pin, INPUT); // 
  

  Serial.write("Setup Complete");

}

void loop() {
  // put your main code here, to run repeatedly:

  // STATE MACHINE
  switch(state){
    case STANDBY:{
      int boiler_purge_button_value = digitalRead(boiler_purge_button_pin); //holds the status of the boiler purge button click
      int grouphead_purge_button_value = digitalRead(grouphead_purge_button_pin); //holds the status of the grouphead purge button click
      purgeEnable(boiler_purge_button_value, grouphead_purge_button_value);

      if(boiler_purge_button_value == 0){
        boiler_purge_button_click_counter = boiler_purge_button_click_counter + 1;
      }

      if(grouphead_purge_button_value == 0){
        grouphead_purge_button_click_counter = grouphead_purge_button_click_counter + 1;
      }
  
      if(boiler_purge_button_click_counter > 0 && grouphead_purge_button_click_counter > 0){
        state = WAITING_FOR_PREHEAT;
        boiler_purge_button_click_counter = 0;
        grouphead_purge_button_click_counter = 0;
      }
      break;
    }

    case WAITING_FOR_PREHEAT:{
      int boiler_purge_button_value = digitalRead(boiler_purge_button_pin); //holds the status of the boiler purge button click
      int grouphead_purge_button_value = digitalRead(grouphead_purge_button_pin); //holds the status of the grouphead purge button click
      purgeEnable(boiler_purge_button_value, grouphead_purge_button_value); //enables state to still be able to use purge valves
      
      int preheat_button_value = digitalRead(preheat_button_pin); // reading preheat button value to state change if requested
      if(preheat_button_value == 0){
        state = PREHEAT;
      }
      break;
    } 

    case PREHEAT:{
      double boiler_temperature_value = boilerTemp.readCelsius(); // reading boiler thermocouple temp
      double grouphead_temperature_value = groupheadTemp.readCelsius(); // reading grouphead thermocouple temp
      digitalWrite(dumpprime_solenoid_trigger, LOW); 
      digitalWrite(grouphead_solenoid_trigger, HIGH); 
      heatBoiler(boiler_temperature_value); //HEAT UP TO TARGET BOILER TEMP
      heatGrouphead(grouphead_temperature_value); // HEAT UP TO TARGET GROUPHEAD TEMP

      int boiler_purge_button_value = digitalRead(boiler_purge_button_pin); //holds the status of the boiler purge button click
      int grouphead_purge_button_value = digitalRead(grouphead_purge_button_pin); //holds the status of the grouphead purge button click
      purgeEnable(boiler_purge_button_value, grouphead_purge_button_value); //enables state to still be able to use purge valves

      if((boiler_temperature_value > 94 && boiler_temperature_value < 96) && (grouphead_temperature_value > 94 && grouphead_temperature_value < 96)){
        state = READY;
      }
      break;
    }

    case READY:{
      double boiler_temperature_value = boilerTemp.readCelsius(); // reading boiler thermocouple temp
      double grouphead_temperature_value = groupheadTemp.readCelsius(); // reading grouphead thermocouple temp
      digitalWrite(dumpprime_solenoid_trigger, LOW); 
      digitalWrite(grouphead_solenoid_trigger, HIGH); 
      heatBoiler(boiler_temperature_value); //HEAT UP TO TARGET BOILER TEMP
      heatGrouphead(grouphead_temperature_value); // HEAT UP TO TARGET GROUPHEAD TEMP

      int boiler_purge_button_value = digitalRead(boiler_purge_button_pin); //holds the status of the boiler purge button click
      int grouphead_purge_button_value = digitalRead(grouphead_purge_button_pin); //holds the status of the grouphead purge button click
      purgeEnable(boiler_purge_button_value, grouphead_purge_button_value); //enables state to still be able to use purge valves
      
      int start_button_value = digitalRead(start_button_pin); //reading start button value
      if(start_button_value == 0){ //if start button is clicked
        state = RUN;
      }
      break;
    }

    case RUN:{
    double boiler_temperature_value = boilerTemp.readCelsius(); // reading boiler thermocouple temp
      double grouphead_temperature_value = groupheadTemp.readCelsius(); // reading grouphead thermocouple temp
      heatBoiler(boiler_temperature_value); //HEAT UP TO TARGET BOILER TEMP
      heatGrouphead(grouphead_temperature_value); // HEAT UP TO TARGET GROUPHEAD TEMP

      int boiler_purge_button_value = digitalRead(boiler_purge_button_pin); //holds the status of the boiler purge button click
      int grouphead_purge_button_value = digitalRead(grouphead_purge_button_pin); //holds the status of the grouphead purge button click
      purgeEnable(boiler_purge_button_value, grouphead_purge_button_value); //enables state to still be able to use purge valves
      

      digitalWrite(dumpprime_solenoid_trigger, HIGH); // close dump valve
      digitalWrite(grouphead_solenoid_trigger, LOW); //open grouphead valve to start making espresso
      dimmer.setState(ON);
      dimmer.setPower(70);
      int start_button_value = digitalRead(start_button_pin);
      if(start_button_value == 0){ //if start button is clicked again
        dimmer.setState(OFF); //activate the pump

        state = PREHEAT;
      }
    }

    case ESTOP:{
      break;
    }
  }


  statusUpdate();
 
}

void purgeEnable (int boiler_purge_button_value, int grouphead_purge_button_value){
  if(boiler_purge_button_value == 0 || grouphead_purge_button_value == 0){ //if either of the buttons are clicked, run the pump
        dimmer.setState(ON);
        dimmer.setPower(60);
      }
      else {
        dimmer.setPower(0);
        dimmer.setState(OFF); //if buttons are not clicked turn the pump off
      }

      if(grouphead_purge_button_value == 0 && boiler_purge_button_value == 0){ //if you click both buttons at the same time open both solenoids

        digitalWrite(grouphead_solenoid_trigger, LOW);
        digitalWrite(dumpprime_solenoid_trigger, LOW);
      }
      else if (grouphead_purge_button_value == 0){ //if you click the grouphead button only, open grouphead solenoid, close dump solenoid
        digitalWrite(grouphead_solenoid_trigger, LOW);
        digitalWrite(dumpprime_solenoid_trigger, HIGH);
      }
      else{ //default config is opening dump prime solenoids all the time, no vacuum can be pulled
        digitalWrite(grouphead_solenoid_trigger, HIGH);
        digitalWrite(dumpprime_solenoid_trigger, LOW); //low is on
      }
}


void heatBoiler(double boiler_temperature_value){
  //code for control of boiler

  double now = millis();
  dt = (now-last_time)/1000.00;
  last_time = now;

  double error = setpoint_boiler - boiler_temperature_value;
  double proportional = error;
  integral += error *dt;
  double derivative = (error - previous)/dt;
  previous = error;

  //should output a desired duty cycle value between 0 and 100, 100 is max beans

  dutyCycle = (kp_boiler*proportional) + (ki_boiler*integral) + (kd_boiler*derivative); 
  Serial.print((String) " p = " + proportional);
  Serial.print((String) " i = " + integral);
  Serial.print((String) " d = " + derivative);

  if(dutyCycle >= 100){
    dutyCycle = 100;
  }
  else if (dutyCycle <= 0){
    dutyCycle = 0;
  }


  //code for setting period and duty cycle for sudo-PWM, arduino period too fast so manually coding something here
  //analogWrite(boiler_heater_trigger, dutyCycle);
  if(new_cycle == true){
    //set PWM intervals
    interval_ON = period*(dutyCycle/100.0);
    interval_OFF = period - interval_ON;

    //update lastTime
    lastTime = millis();

    //update status of where you are in PWM cycle
    new_cycle = false;

    //Turn On PWM to start
    boilerState = HIGH; 
    digitalWrite(boiler_heater_trigger, boilerState); //LOW IS OFF, HIGH IS ON
  
  }

  unsigned long currentTime = millis();

  //if the boiler is ON, check to see if you need to turn it off
  if((boilerState == HIGH) && (currentTime - lastTime >= interval_ON)){
      boilerState = LOW;
      digitalWrite(boiler_heater_trigger, boilerState);
      lastTime = currentTime;
  }

  //if the boiler is OFF, check to see if you need to turn on via a new cycle
  if((boilerState == LOW) && (currentTime - lastTime >= interval_OFF)){
    new_cycle = true;
  }

}

void heatGrouphead(double grouphead_temperature_value){
  if(grouphead_temperature_value < 95){
        digitalWrite(grouphead_heater_trigger, LOW);
      } 
  else{
        digitalWrite(grouphead_heater_trigger, HIGH);
      }
}

void runThisShit (){
  
  digitalWrite(dumpprime_solenoid_trigger, LOW); //open dump valve for 300ms
  delay(300);
  digitalWrite(dumpprime_solenoid_trigger, HIGH); // close dump valve
  digitalWrite(grouphead_solenoid_trigger, LOW); //open grouphead valve to start making espresso
  dimmer.setState(ON);
  dimmer.setPower(60);
  int start_button_value = digitalRead(start_button_pin);
  if(start_button_value == 0){ //if start button is clicked again
    dimmer.setState(OFF); //activate the pump
    state = STANDBY;
  }
  //now you are ready for pressure targeting sequence
  //pressureTargetSequencePSI[][] = 
  //[1 50; 2 50; 3; 50]

}

void targetPressure(){ // function to control pressure targeting, input is target pressure
  float analogpressureValue = analogRead(pressure_sensor_signal_pin);
  //if

}


void statusUpdate(){
  int boiler_purge_button_value = digitalRead(boiler_purge_button_pin); // reading boiler button value
  int grouphead_purge_button_value = digitalRead(grouphead_purge_button_pin); // reading grouphead button value
  int preheat_button_value = digitalRead(preheat_button_pin); // reading preheat button value
  int start_button_value = digitalRead(start_button_pin); //reading start button value
  int estop_button_value = digitalRead(estop_button_pin); // reading estop button value
  double boiler_temperature_value = boilerTemp.readCelsius(); // reading boiler thermocouple temp
  double grouphead_temperature_value = groupheadTemp.readCelsius(); // reading grouphead thermocouple temp
  float analogpressureValue = analogRead(pressure_sensor_signal_pin); // reads value from pressure sensor input pin
  pressureValue = ((analogpressureValue-pressureZero)*pressuretransducermaxPSI)/(pressureMax-pressureZero); // conversion to convert analog reading to psig

  Serial.print("\n");
  Serial.print(boiler_purge_button_value); // printing boiler button value
  Serial.print(grouphead_purge_button_value); //printing boiler button value
  Serial.print(preheat_button_value); //printing preheat button value  
  Serial.print(start_button_value); // printing start button value
  Serial.print(estop_button_value); //printing start button value
  Serial.print((String)" Boiler: " + boiler_temperature_value); // printing boiler thermocouple temp
  Serial.print((String)" Duty Cycle: " + dutyCycle); //prints value of pwm setting
  Serial.print((String)" Grouphead: " + grouphead_temperature_value); // printing boiler thermocouple temp
  Serial.print((String)" psig: " + pressureValue);
  Serial.print((String)" STATE: " + enumStateConvert[state]);
  Serial.print((String)" INTERVAL_ON: " + interval_ON);
  Serial.print((String)" INTERVAL_OFF: " + interval_OFF);
  delay(100);
}







