#include <stdio.h>
#include <time.h>
#include "E101.h"

//According to comments found on documentation, using 255 might
//cause the h-bridge to get stuck, so don't use this value for MAX_SPEED.
const int MAX_SPEED = 254; // Motor uses 100% capacity
const int MED_SPEED = (int) MAX_SPEED*0.5; //
const int MIN_SPEED = 0; //We may have a use for this or not
const int F_SENSOR = 0; //Front sensor pin
const int R_SENSOR = 0; //Right sensor pin
const int L_SENSOR = 0; //Left sensor pin
const int L_MOTOR = 1; //Left motor
const int R_MOTOR = 2; //Right motor

//Structure to store information about distance sensor readings.
struct Readings{
    double average;
    int max;
    int min;
};

//Reads the given a number of times and returns the average, max and min reading.
int readSensor(int sensor, int number_of_readings){
    //number_of_readings = 0, no reading is done at all
    //number_of_readings = 1, just one reading, i.e., no average
    //number_of_readings > 1, average of readings.
    //Use F_SENSOR, L_SENSOR or R_SENSOR for sensor.
    int    adc_reading     = 0;
    int    max_reading     = 0;
    int    min_reading     = 2000;
    double average_reading = 0;
    
    for (int i = 0; i < number_of_readings; i++){
        adc_reading     = read_analog(sensor);
        average_reading = (i*average_reading + adc_reading)/(i+1); //calculates average as it goes.
        if (adc_reading > max_reading){
            max_reading = adc_reading;
        }
        if (adc_reading < min_reading){
            min_reading = adc_reading;
        }
    }
    Readings results = {average_reading, max_reading, min_reading};
    return results;
}
    


int main(){
    int quad = 1; //Flag to signalize change of quadrants.
    int speed;
    
    //Starts moving robot
    init();
    speed = MAX_SPEED*0.3; //Sets initial speed to 30% of maximum velocity.
    
    while(1){
        if (quad == 1 || quad == 2){
            //Quadrant 1 and 2
            //Goal: follow straight line until Quad 3
            
            int front_reading = readSensor(F_SENSOR, 1).average;
            
            if (front_reading>300){ //We must test the sensors to determine values to use
                //There's something ahead and we don't want to
                //crash the robot or kill any minion, right? :P
                set_motor(L_MOTOR,0);
                set_motor(R_MOTOR,0);
            }
            else
                take_picture();
                // #todo determine the position of the line
                
                set_motor(L_MOTOR,MED_SPEED);
                set_motor(R_MOTOR,MED_SPEED);
                sleep1(1,0);
            
                //if(transversal line, i.e., 111111111111111)
                //    follow perpendicular line ahead until transversal line is gone
                //    break loop
                //else if(perpendicular line, i.e., 000000 111 000 or 00 1111 0000000)
                //    follow line
                //else if(no line, i.e., 00000000000000)
                //    stop and turn left until line is centralized
                //    //If it doesn`t work, try this: stop, turn the opposite direction of what it was turning before until line can be seen again.
                //else
                //    stop
                //    save picture and info for debugging
        } else if (quad == 3){
            //Quadrant 3
            //Goal: pass through the line maze.
        } else if (quad == 4){
            //Quadrant 4
            //Goal: pass through the walled maze.
        } else {
            //For testing.

            //Moves the robot forward and stops if an obstacle is close.
            Readings readings = getAverageMinMaxReading(10);
            if (readings.average < 500){ //moves forward (depends on how motors were assembled).
                set_motor(L_MOTOR,MED_SPEED);
                set_motor(R_MOTOR,MED_SPEED);
                sleep1(1,0);
            }
            else{
                set_motor(L_MOTOR,0);
                set_motor(R_MOTOR,0);
            }
        }
    }
    
    stop(L_MOTOR);
    stop(R_MOTOR);
}
