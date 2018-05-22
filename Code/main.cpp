#include <stdio.h>
#include <time.h>
#include "E101.h"

//Sensors and motor constants
const int F_SENSOR = 6; //Front sensor pin
const int L_SENSOR = 5; //Left sensor pin
const int R_SENSOR = 1; //Right sensor pin
const int L_MOTOR  = 2; //Left motor
const int R_MOTOR  = 1; //Right motor

//Duty cycle constants
/*According to comments found on documentation, using 255 may
  cause the H-bridge to get stuck, so MAX_DUTY_CYCLE is set to 254 instead. */
const int MAX_DUTY_CYCLE  = 254; // Motor uses 100% capacity
const int MIN_DUTY_CYCLE  = 70;  // Minimum value for which the robot moves. Somewhere between 50 and 76.
const int BASE_DUTY_CYCLE = (int) MAX_DUTY_CYCLE*0.3; //Duty cycle when error is zero.

//Error calculation constants
const int KP = BASE_DUTY_CYCLE*0.5; //Remember to ensure BASE_DUTY_CYCLE + KP is less than 254.
const int KD = 0;

//Image processing constants
const int PIC_WIDTH       = 320;
const int PIC_HEIGHT      = 240;
const int ROW             = PIC_HEIGHT/4; //#todo determine a good value for the vertical coord.
const int MIN_TRACK_WIDTH = 30;  //#todo take pictures with robot to determine best value.
const int TRANSVERSAL     = (int)PIC_WIDTH*0.85; //Mininum number of white pixels that makes a transversal line.
const int PASSAGE         = (int)(TRANSVERSAL+MIN_TRACK_WIDTH)*0.5; //Mininum number of white pixels that makes a left or right passage.
const int RED             = 0;
const int GREEN           = 1;
const int BLUE            = 2;
const int LUM             = 3;   //Luminosity = (red value + green value + blue value)/3
const int LUM_THRESHOLD   = 127; //#todo make experiments to determine threshold.
const int MAX_BLK_NOISE   = 5;   //Max. number of consecutive black pixels inside a track.
const int RED_THRESHOLD   = 230; //Value for which the component of a pixel will be considered red.
const int GREEN_THRESHOLD = 200; //Value for which the component of a pixel will be considered green.
const int BLUE_THRESHOLD  = 200; //Value for which the component of a pixel will be considered blue.
const int MIN_RED_COUNTER = PIC_WIDTH/4; //Number or reddish pixels a line must have to be considered red.

//Distance control constants
const int MIN_DISTANCE = 400; //#todo Test and find a minimum distance to avoid collisions on quads 1 to 3.

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

//Reads the given sensor a number of times and returns the average, max and min reading.
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
        int luminosity = get_pixel(y, x, LUM); //Gets luminosity (whiteness) of pixel.
        if (luminosity > LUM_THRESHOLD){
            //Pixel is assumed to be white.
            white_counter[track_number]++;
            error[track_number] += pixel_value;
            if (black_counter > 0){
                int lum1 = get_pixel(y,x-1, LUM);
                int lum2 = get_pixel(y,x-2, LUM);
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
    double error_percentage      = errors.track1/(PIC_WIDTH/2.0);
    double derivative            = 0;
    double duty_cycle_correction = error_percentage*KP + derivative*KD;

    set_motor(L_MOTOR,BASE_DUTY_CYCLE+(int)duty_cycle_correction); //Final duty cycle must be an int.
    set_motor(R_MOTOR,BASE_DUTY_CYCLE-(int)duty_cycle_correction);
}

//Checks if there is a red line in the picture.
bool isRedLine(){
    bool result      = false;
    int  red_counter = 0;
    for (int x = 0; x < PIC_WIDTH; x++){
        int red_value   = get_pixel(ROW, x, RED);
        int green_value = get_pixel(ROW, x, GREEN);
        int blue_value  = get_pixel(ROW, x, BLUE);
        if (red_value > RED_THRESHOLD &&
            green_value < GREEN_THRESHOLD &&
            blue_value < BLUE_THRESHOLD)
            red_counter++;
    }
    if (red_counter >= MIN_RED_COUNTER)
        result = true;
    return result;
}

int main(){
    int quad = 1; //Flag to signalize change of quadrants.
    Errors previous_errors;
    
    init();
    
    while(quad == 1 || quad == 2){ //Quadrant 1 and 2
        
        //Goal: pen gate and follow straight line until Quad 3
        
        int front_reading = readSensor(F_SENSOR, 1).average;
        
        if (front_reading>MIN_DISTANCE){
            //There's something ahead and we don't want to
            //crash the robot or kill any minion, right? :P
            set_motor(L_MOTOR,0);
            set_motor(R_MOTOR,0);
            if (quad == 1){
                //Code to deal with the gate here.
                //quad = 2;
            }
        }
        else {
            take_picture(); //Take a picture and loads it to the memory.
            Errors errors = getErrorFromPicture(ROW);
            
            if(errors.white_counter1 >= TRANSVERSAL){
                //Robot is reaching Quadrant 3.
                //The loop bellow controls the transition between Quadrant 2 and 3.
                while(errors.white_counter1 >= TRANSVERSAL){
                    errors = getErrorFromPicture(0); //Tries to detect a track ahead.
                    if(errors.white_counter1 < TRANSVERSAL)
                        followTrack(errors);
                    take_picture();
                    errors = getErrorFromPicture(ROW);
                }
                quad = 3; //Change this to 2 if you're having trouble to perform tests for quads 1 and 2.
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
                        set_motor(L_MOTOR,-BASE_DUTY_CYCLE*0.5);
                        set_motor(R_MOTOR, BASE_DUTY_CYCLE*0.5);
                    }
                    else {    
                        //Track was on the right side before it was lost.
                        set_motor(L_MOTOR, BASE_DUTY_CYCLE*0.5);
                        set_motor(R_MOTOR,-BASE_DUTY_CYCLE*0.5);
                    }
                    take_picture();
                    errors = getErrorFromPicture(ROW);
                }
                //Back on track!
                followTrack(errors);
                previous_errors = errors;
            }
        }
    }
        
    while(quad == 3){ //Quadrant 3
        
        //Goal: pass through the line maze.
        
        int front_reading = readSensor(F_SENSOR, 1).average;
        
        if (front_reading>MIN_DISTANCE){
            //Avoid collisions.
            set_motor(L_MOTOR,0);
            set_motor(R_MOTOR,0);
        }
        else {
            take_picture(); //Take a picture and loads it to the memory.
            
            if (isRedLine()){
                //Robot is reaching Quadrant 4.
                //The loop bellow controls the transition between Quadrant 3 and 4.
                /*
                read left and right sensors
                while(no walls detected){
                    keep reading sensors
                }
                calculate error based on distances
                */
                quad = 4;
            }
            else {
                Errors errors = getErrorFromPicture(ROW);
                
                if(errors.white_counter1 >= TRANSVERSAL){
                    //The best option in this case is always to take the path of the left
                    //The error is set to its maximum negative magnitude.
                    errors.track1 = -PIC_WIDTH/2.0;
                    followTrack(errors);
                    previous_errors = errors;
                    /* Another idea for dealing with transversal tracks
                    while(errors.white_counter1 > MIN_TRACK_WIDTH){
                        //Keep advancing until track disappears
                        errors = getErrorFromPicture(ROW);
                    }
                    //Stop after track disappears
                    set_motor(L_MOTOR,0);
                    set_motor(R_MOTOR,0);
                    while(Math.abs(errors.track1) > PIC_WIDTH/4){
                        //Turns the robot left until the track is positioned in the central area of the picture
                        set_motor(L_MOTOR,-BASE_DUTY_CYCLE*0.5);
                        set_motor(R_MOTOR, BASE_DUTY_CYCLE*0.5);
                        errors = getErrorFromPicture(ROW);
                    }
                    //Start following the track again after it is centralized.
                    followTrack(errors);
                    previous_errors = errors;
                    */
                }
                else if(errors.white_counter1 >= PASSAGE && errors.track1 < 0){
                    //Robot found a passage to the left.
                    errors = getErrorFromPicture(0); //Tries to detect a track ahead.
                    if(errors.white_counter1 >= MIN_TRACK_WIDTH){
                        //If the track continues past the passage, follow the track.
                        followTrack(errors);
                        previous_errors = errors;
                    }
                    else{
                        //It is a sharp turn to the left. The error is set to its maximum negative magnitude.
                        errors.track1 = -PIC_WIDTH/2.0;
                        followTrack(errors);
                        previous_errors = errors;
                    }
                }
                else if(errors.white_counter1 >= PASSAGE && errors.track1 >= 0){
                    //Robot found a passage to the right.
                    errors = getErrorFromPicture(0); //Tries to detect a track ahead.
                    if(errors.white_counter1 >= MIN_TRACK_WIDTH){
                        //If the track continues past the passage, follow the track.
                        followTrack(errors);
                        previous_errors = errors;
                    }
                    else{
                        //It is a sharp turn to the right. The error is set to its maximum positive magnitude.
                        errors.track1 = PIC_WIDTH/2.0;
                        followTrack(errors);
                        previous_errors = errors;
                    }
                }
                else if(errors.white_counter1 >= MIN_TRACK_WIDTH){
                    //Detected a track with no passages; must follow it.
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
                            set_motor(L_MOTOR,-BASE_DUTY_CYCLE*0.5);
                            set_motor(R_MOTOR, BASE_DUTY_CYCLE*0.5);
                        }
                        else {    
                            //Track was on the right side before it was lost.
                            set_motor(L_MOTOR, BASE_DUTY_CYCLE*0.5);
                            set_motor(R_MOTOR,-BASE_DUTY_CYCLE*0.5);
                        }
                        take_picture();
                        errors = getErrorFromPicture(ROW);
                    }
                    //Back on track!
                    followTrack(errors);
                    previous_errors = errors;
                }
            }
        }
    }
    
    while(quad == 4){//Quadrant 4
        
        //Goal: pass through the walled maze.
        quad = 0;
    }
    
    while(quad == -1){
        //Use this loop for tests.

        //Moves the robot forward and stops if an obstacle is close.
        Readings readings = readSensor(F_SENSOR, 10);
        if (readings.average < 500){
            set_motor(L_MOTOR,BASE_DUTY_CYCLE);
            set_motor(R_MOTOR,BASE_DUTY_CYCLE);
            sleep1(1,0);
        }
        else{
            set_motor(L_MOTOR,0);
            set_motor(R_MOTOR,0);
        }
    }
    
    stop(L_MOTOR);
    stop(R_MOTOR);
}
