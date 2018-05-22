#include <stdio.h>
#include <time.h>
#include “E101.h”
int main (){
	int adc_reading_right;
	int adc_reading_left;
	int adc_reading_front;
	int i = 0;
	init ();
	for (i = 0; i < 100; i = i + 1) {
		adc_reading_right = read_analog(1);
		adc_reading_left = read_analog(5);
		adc_reading_front = read_analog(6);
		printf("%d \t %d \t %d", adc_reading_left, adc_reading_front, adc_reading_right);
		sleep1(0, 100000);
	}
	return 0;
} 
