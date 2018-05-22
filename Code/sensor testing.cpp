#include <stdio.h>
#include <time.h>
#include “E101.h” // this is our custom ENGR 101 C library.
int main (){

	init ();
	for (i = 0; i<10 i = i + 1){
	   printf(“i =%d\n”,i);
	   //We declare an integer variable to store the ADC data 
		int adc_reading;
		//Reads from Analog Pin 0 (A0) through A7 
		adc_reading_right = read_analog(0);
		//Prints read analog value 
		printf(“  adc_r = %d\n”,adc reading_right);
		//Waits for 1 seconds (50000000 microseconds)
		sleep (5,0);
		
		
		adc_reading_left =  read_analog(5);
		//Prints read analog value 
	    printf(“  adc_l = %d\n”,adc reading_left);
		//Waits for 1 seconds (50000000 microseconds)
		sleep (5,0);
		
		adc_reading_front = read_analog(6);
		//Prints read analog value for pin 6
		printf(“   adc_f = %d\n”,adc reading_front);
		//Waits for 1 seconds (50000000 microseconds)
		sleep (5,0);
		
		
		
		
	}
	
	
return 0;} 
