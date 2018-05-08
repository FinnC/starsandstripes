#include <stdio.h>
#include <time.h>
#include "E101.h"

struct Readings{
    double average;
    int max;
    int min;
};

//Reads the sensor n times and returns the average, max and min reading.
Readings getAverageMinMaxReading(int number_of_readings){
	int	   adc_reading     = 0;
	int    max_reading     = 0;
	int    min_reading     = 2000;
	double average_reading = 0;
	
	for (int i = 0; i < number_of_readings; i++){
		adc_reading     = read_analog (0);
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
    int quad = 5;
   
    init();
    while(1){
        if (quad == 1){
            //Quadrant 1
            //Goal: follow straight line until Quad 2
        } else if (quad == 2){
            //Quadrant 2
            //Goal: follow curved line until Quad 3
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
                set_motor(1,127); //127 = 50% speed.
                set_motor(2,127);
                sleep1(1,0);
            }
            else{
                stop(1);
                stop(2);
            }
        }
    }
}
