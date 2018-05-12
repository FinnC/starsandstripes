#include <stdio.h>
#include <time.h>
#include "E101.h"

//Sensors and motor constants
const int F_SENSOR = 0; //Front sensor pin
const int L_SENSOR = 0; //Left sensor pin
const int R_SENSOR = 0; //Right sensor pin
const int L_MOTOR  = 1; //Left motor
const int R_MOTOR  = 2; //Right motor

//Velocity constants
/*According to comments found on documentation, using 255 may
  cause the H-bridge to get stuck, so MAX_SPEED is set to 254 instead. */
const int MAX_SPEED = 254; // Motor uses 100% capacity
const int SPEED     = (int) MAX_SPEED*0.5; //Speed of motors when error is zero.

//Image processing constants
const int PIC_WIDTH       = 320;
const int PIC_HEIGHT      = 240;
const int Y               = 240/4; //#todo determine a good value for the vertical coord.
const int MIN_TRACK_WIDTH = 30;  //#todo take pictures with robot to determine best value.
const int TRANSVERSAL     = (int)PIC_HEIGHT*0.85; //Mininum number of white pixels that makes a transversal line.
const int RED             = 0;
const int GREEN           = 1;
const int BLUE            = 2;
const int LUM             = 3;   //Luminosity = (red value + green value + blue value)/3
const int LUM_THRESHOLD   = 127; //#todo make experiments to determine threshold.
const int MAX_BLK_NOISE   = 5;   //Max. number of consecutive black pixels inside a track.

//Error constants. For now everything is arbitrary, we need tests.
const int MAX_ERROR = SPEED*0.5; //Remember to ensure SPEED + MAX_ERROR is less than 254.
const int KP = MAX_ERROR/(PIC_WIDTH/2.0);
const int KD = 0;

//Structure to store error data about tracks after image analysis.
struct Errors{
    double track1;
    int    white_counter1;
    double track2; //Not being used at the moment, but might be for quadrant 3
    int    white_counter2; //Same
};

//Structure to store information about distance sensor readings.
struct Readings{
    double average;
    int    max;
    int    min;
};

//Reads the given a number of times and returns the average, max and min reading.
Readings readSensor(int sensor, int number_of_readings){
    //Use F_SENSOR, L_SENSOR or R_SENSOR for sensor.
    //number_of_readings = 0, no reading is done at all
    //number_of_readings = 1, just one reading, i.e., no average
    //number_of_readings > 1, calculates the average.
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
    
//Analyses a picture and returns corresponding error signals and number of white pixels.
Errors getErrorFromPicture(int y){
    //y is the vertical coordinate of the row to be analyzed in the picture.
    //Pixels in the LEFT side of the image are assigned NEGATIVE values.
    //Pixels in the RIGHT side of the image are assigned POSITIVE values.
    int    pixel_value      = -PIC_WIDTH/2; //Value of the first pixel.
    int    track_number     = 0;
    double error[]          = {0,0}; //error[0] for first track detected, error[1] for second.
    int    white_counter[]  = {0,0};
    int    black_counter    = 0;
    double noise_correction = 0; //To account for small number of black pixels inside a track.
    for (int x = 0; x < PIC_WIDTH; x++){
        int luminosity = get_pixel(x, y, LUM); //Gets luminosity (whiteness) of pixel.
        if (luminosity > LUM_THRESHOLD){
            //Pixel is assumed to be white.
            white_counter[track_number]++;
            error[track_number] += pixel_value;
            if (black_counter > 0){
                int lum1 = get_pixel(x-1, y, LUM);
                int lum2 = get_pixel(x-2, y, LUM);
                if(lum1 > LUM_THRESHOLD && lum2 > LUM_THRESHOLD){
                    //Noise detected by the presence of black pixels inside the track
                    //is only incorporated if previous two pictures are also white.
                    error[track_number]         += noise_correction;
                    white_counter[track_number] += black_counter;
                    black_counter                = 0;
                    noise_correction             = 0;
                }
            }
        }
        else if (white_counter[track_number] > 0){
            //Pixel is assumed to be black and there is a chance it is inside a white track.
            black_counter++;
            noise_correction += pixel_value;
            if (black_counter > MAX_BLK_NOISE){
                //The amount of black pixels is too big, so the current region being analyzed
                //is assumed to either not be a track or be the right end of a track.
                black_counter    = 0;
                noise_correction = 0;
                if (white_counter[track_number] < MIN_TRACK_WIDTH){
                    //It is not a track: reset error and counter.
                    error[track_number]         = 0;
                    white_counter[track_number] = 0;
                }
                else{
                    //It is the right end of a track.
                    //Start getting error for a possible second track in the picture
                    track_number++;
                }
            }
        }
        pixel_value++;
        if (pixel_value == 0){
            //When pixel_value gets to zero, it means the loop has reached the right
            //side of the image, so the first pixel_value must be adjusted to 1.
            pixel_value = 1;
        }
    }
    //Dividing the error by the number of white pixels normalizes its value
    //to account for different widths in the white lines. In fact, it makes the
    //error assume the pixel value correspondent to middle of the line.
    if (white_counter[0] > 0)
        error[0] = error[0]/white_counter[0];
     if (white_counter[1] > 0)
        error[1] = error[1]/white_counter[1];
    Errors results = {error[0], white_counter[0], error[1], white_counter[1]};
    return results;
}

//Follows a white track according to the error provided.
void followTrack(Errors errors){
    //Still need to implement derivative and determine value for KD.
    double derivative = 0;
    
    double final_error = errors.track1*KP + derivative*KD;

    set_motor(L_MOTOR,SPEED+final_error);
    set_motor(R_MOTOR,SPEED-final_error);
}

int main(){
    int quad = 1; //Flag to signalize change of quadrants.
    Errors previous_errors;
    
    init();
    
    while(1){
        if (quad == 1 || quad == 2){
            //Quadrant 1 and 2
            //Goal: follow straight line until Quad 3
            
            int front_reading = readSensor(F_SENSOR, 1).average;
            
            if (front_reading>300){ //#todo test sensors to determine values to use
                //There's something ahead and we don't want to
                //crash the robot or kill any minion, right? :P
                set_motor(L_MOTOR,0);
                set_motor(R_MOTOR,0);
            }
            else {
                take_picture(); //Take a picture and loads it to the memory.
                Errors errors = getErrorFromPicture(Y);
                
                if(errors.white_counter1 >= TRANSVERSAL){
                    //Robot is reaching Quadrant 3.
                    //The loop bellow controls the transition between Quadrant 2 and 3.
                    while(errors.white_counter1 >= TRANSVERSAL){
                        errors = getErrorFromPicture(0); //Tries to detect a track ahead.
                        if(errors.white_counter1 < TRANSVERSAL)
                            followTrack(errors);
                        take_picture();
                        errors = getErrorFromPicture(Y);
                    }
                    quad = 3;
                }
                else if(errors.white_counter1 >= MIN_TRACK_WIDTH){
                    //Detected a track; must follow it.
                    followTrack(errors);
                    previous_errors = errors;
                }
                else { //errors.white_counter1 < MIN_TRACK_WIDTH
                    //Lost track; must use data from previous picture to find it.
                    set_motor(L_MOTOR,0);
                    set_motor(R_MOTOR,0);
                    while(errors.white_counter1 < MIN_TRACK_WIDTH){
                        if(previous_errors.track1 < 0){
                            //Track was on the left side before it was lost.
                            set_motor(L_MOTOR,-SPEED*0.5);
                            set_motor(R_MOTOR, SPEED*0.5);
                        }
                        else {    
                            //Track was on the right side before it was lost.
                            set_motor(L_MOTOR, SPEED*0.5);
                            set_motor(R_MOTOR,-SPEED*0.5);
                        }
                        take_picture();
                        errors = getErrorFromPicture(Y);
                    }
                    //Back on track!
                    followTrack(errors);
                    previous_errors = errors;
                }
            }

        } else if (quad == 3){
            //Quadrant 3
            //Goal: pass through the line maze.
        } else if (quad == 4){
            //Quadrant 4
            //Goal: pass through the walled maze.
        } else {
            //For testing.

            //Moves the robot forward and stops if an obstacle is close.
            Readings readings = readSensor(F_SENSOR, 10);
            if (readings.average < 500){ //moves forward (depends on how motors were assembled).
                set_motor(L_MOTOR,SPEED);
                set_motor(R_MOTOR,SPEED);
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
