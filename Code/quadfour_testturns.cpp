#include <stdio.h>
#include <time.h>
#include "E101.h"

const int F_SENSOR = 6; //Front sensor pin
const int L_SENSOR = 5; //Left sensor pin
const int R_SENSOR = 0; //Right sensor pin
const int L_MOTOR  = 2; //Left motor
const int R_MOTOR  = 1; //Right 
const int MIN_DISTANCE = 400; //#todo Test and find a minimum distance to avoid collisions on quads 1 to 3.
const int MULTIPLIER = 3;
const int TURNTIME_SEC; //seconds component of time for a 90 degree turn
const int TURNTIME_MICRO; //microseconds component of time for a 90 degree turn

int main(){ //Probably needs the < & > signs flipped
	while(quad == 4){//Quadrant 4
        int front_reading = readSensor(F_SENSOR, 1).average;
        int left_reading = readSensor(L_SENSOR, 1).average;
        int right_reading = readSensor(R_SENSOR, 1).average;
        if (front_reading>MIN_DISTANCE){//if there is a wall try an go right, if not left
		set_motor(R_MOTOR, 0);
		set_motor(L_MOTOR, 0);
            	if(right_reading>front_reading*MULTIPLIER){//Look Right
			set_motor(R_MOTOR, -100);
			set_motor(L_MOTOR, 100); 
			sleep1(TURNTIME_SEC,TURNTIME_MICRO); 
			set_motor(R_MOTOR, 0);
			set_motor(L_MOTOR, 0);
         	}else if(left_reading>front_reading*MULTIPLIER){//Look Left
			set_motor(R_MOTOR, 100);
			set_motor(L_MOTOR, -100); 
			sleep1(TURNTIME_SEC,TURNTIME_MICRO); 
			set_motor(R_MOTOR, 0);
			set_motor(L_MOTOR, 0);
		} else{//180 degree turn for a dead end
			set_motor(R_MOTOR, 100);
			set_motor(L_MOTOR, -100); 
			sleep1(TURNTIME_SEC*2,TURNTIME_MICRO*2); 
			set_motor(R_MOTOR, 0);
			set_motor(L_MOTOR, 0);
		}
        }
        quad = 0;
    }
}

int quad4eq(){
	const int L_SENSOR = 5; //Left sensor pin
	const int R_SENSOR = 0; //Right sensor pin
	const int L_MOTOR  = 2; //Left motor
	const int R_MOTOR  = 1; //Right motor
	int sensor_IR_readingL = 0; //left sensor reading
	int sensor_IR_readingR = 0; //right sensor reading
	int sensor_IR_readingL_prev = 0; //left sensor reading previous
	int sensor_IR_readingR+prev = 0; //right sensor reading previous
	
	sensor_IR_readingL = read_analog(L_SENSOR);
	sensor_IR_readingR = read_analog(R_SENSOR);
	printf("L:%d\nR:%d\n", sensor_IR_readingL, sensor_IR_readingR)
	
	
	if(sensor_IR_readingL > sensor_IR_readingR){ //further from right wall curve right a little
		set_motor(L_MOTOR, 120);
		}
	if(sensor_IR_readingR > sensor_IR_readingL){ //further from left wall curve left a little
		set_motor(R_MOTOR, 120);
		}
	if(sensor_IR_readingR > x){
		break;
		}
	if(sensor_IR_readingL > x){
		break;
		}
	
	return 0;
	}

int quad4turn(){
	const int L_SENSOR = 5; //Left sensor pin
	const int R_SENSOR = 1; //Right sensor pin
	const int L_MOTOR  = 2; //Left motor
	const int R_MOTOR  = 1; //Right motor
	const int MIN_DISTANCE = 400; //#todo Test and find a minimum distance to avoid collisions on quads 1 to 3. NEED TO FIND
	const int MULTIPLIER = 3; //NEED TO FIND
	const int TURNTIME_SEC; //seconds component of time for a 90 degree turn NEED TO FIND
	const int TURNTIME_MICRO; //microseconds component of time for a 90 degree turn NEED TO FIND
	
	while(quad == 4){//Quadrant 4
        int front_reading = readSensor(F_SENSOR, 1).average;
        int left_reading = readSensor(L_SENSOR, 1).average;
        int right_reading = readSensor(R_SENSOR, 1).average;
        if (front_reading>MIN_DISTANCE){//if there is a wall try an go right, if not left
			set_motor(R_MOTOR, 0);
			set_motor(L_MOTOR, 0);
            if(right_reading>front_reading*MULTIPLIER){//Look Right
				set_motor(R_MOTOR, -100);
				set_motor(L_MOTOR, 100); 
				sleep1(TURNTIME_SEC,TURNTIME_MICRO); 
				set_motor(R_MOTOR, 0);
				set_motor(L_MOTOR, 0);
            }else if(left_reading>front_reading*MULTIPLIER){//Look Left
				set_motor(R_MOTOR, 100);
				set_motor(L_MOTOR, -100); 
				sleep1(TURNTIME_SEC,TURNTIME_MICRO); 
				set_motor(R_MOTOR, 0);
				set_motor(L_MOTOR, 0);
			}
        }
        quad = 0;
    }
	
	return 0;
	}

while(quad == 4){//Quadrant 4
        
        set_motor(R_MOTOR, 100);
        set_motor(L_MOTOR, 100);
        
    }
 
