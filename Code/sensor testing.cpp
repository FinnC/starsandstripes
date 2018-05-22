#include <stdio.h>
#include <time.h>
#include “E101.h” // this is our custom ENGR 101 C library.
int main (){
	int adc_reading_right;
	int adc_reading_left;
	int adc_reading_front;
	init ();
	for (int i = 0; i<100; i = i + 1) {
		adc_reading_right = read_analog(1);
		adc_reading_left = read_analog(5);
		adc_reading_front = read_analog(6);
		printf("%n \t %n \t %n", adc_reading_left, adc_reading_front, adc_reading_right);
		sleep1(0, 10000)

	}
	
	return 0;
} 
