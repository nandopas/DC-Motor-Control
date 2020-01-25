//case5 group7

#include <xc.h>
#include <pic.h>

	#pragma config FOSC=HS, CP=OFF, DEBUG=OFF, BORV=20, BOREN=0, MCLRE=ON, PWRTE=ON, WDTE=OFF
	#pragma config BORSEN=OFF, IESO=OFF, FCMEN=0


/* Variable declarations */

#define PORTBIT(adr,bit) ((unsigned)(&adr)*8+(bit))
#define clockwise 0				// clockwise direction macro
#define counter_clockwise 1		// anti clockwise direction macro

// The function PORTBIT is used to give a name to a bit on a port
// The variable RC0 could have equally been used


	static bit blackButton @ PORTBIT(PORTA,4);
	static bit encoder @ PORTBIT(PORTC,6);
	static bit brake @ PORTBIT(PORTC,1);
	static bit eddy @ PORTBIT(PORTC,3);

	char Mode;
	int temp, speed, time, count, v, mean_v;
	int ref, count_encoder, pulse, output;
	int error, error_last, sum_error;
	int kp, kd, ki;




void setPWM(int speed) 					//sets the duty cycle of the pwm into CCPR1L
{
	CCPR1L = speed;
}




void initPORT(void) 					//initialization of ports
{
	PORTA = 0b00000000;					// Clear Port A output latches
	PORTB = 0b00000000; 				// Clear Port B output latches
	PORTC = 0b00000001; 				// Clear Port C output latches

	TRISA = 0b11111111; 				// Configure Port A pin 4 and pin 1 as input
	TRISB = 0b00000000; 				// Configure Port B as all output
	TRISC = 0b01111000; 				// Configure Port C as all input

	setPWM(0);
}



void SwitchDelay(void)					// Waits for switch debounce
{
	for (temp=200; temp > 0; temp--){}			// 1200 us delay
}



void initAtoD(void) 						// Initialize A/D
{
	ADCON0 = 0b10001101; 					// Select 8* oscillator, analog input 0, turn on
	ADCON1 = 0b01011101; 					// RA0,RA1,RA2,RA3 analog inputs, rest digital
	T2CON = 0b00000100;
	CCP1CON = 0b00001100;

	PR2 = 0b11111111;
	
	SwitchDelay(); 							// Delay a bit before starting A/D conversion
	GO = 1; 								// Start A/D
	count = 0;
	
	ADIF = 0;								//set interrupter control bits
	ADIE = 1;
	PEIE = 1;
	GIE = 1;
}



void interrupt isrAD(void)					//interrupt function, average 64 AD readings
{
	
	v += ADRESH;
	count++;
	
	if(count == 64)
	{
		mean_v = v/64;
		count = 0;
	}
	
	ADIF = 0;
	GO = 1;
	
}



void ms_delay(int time)						//millisecond delay function
{
     unsigned int i,j;
        for(i=0;i<time;i++)
		{
            for(j=0;j<1650;j++){}
		}
}



void Flash(void)							//stops pwm and flashes lights
{											//stops if black button pressed
	setPWM(0);
	
	while(1)
	{
		PORTB = 0b11111111;
		ms_delay(100);
		PORTB = 0;
		ms_delay(100);
		
		if(blackButton == 1)
		{
			break;
		}
	}
}



void initMode(void) 						//initialization routine for eddy sensor
{
	int count_init;							//counter for eddy current passes
	
	setPWM(145);
	count_init = 0;
	
	while(count_init < 4)					//3 passes through eddy sensor
	{
		if(eddy == 1)						//if eddy current detected
		{
			while(eddy == 1){}				//wait to pass through completely
			PORTB = 0x11;					//flash the LEDs to show eddy sensor active
			count_init = count_init + 1;	//increase eddy sensor count
		}
	}
	
	setPWM(0);								//stop motor and return to calling function
	brake = 1;
	
}



void Mode0(void)							//method for mode 0
{
	PORTB = Mode;							//set LED's to mode 0
	brake = 0;
	
	ref = 140;
	setPWM(ref);
	
	count_encoder = 0;						//count encoder passes
	pulse = 0;
	kp = 10;								//proportional gain
	
	while(1)
	{
		while(ref < 190)					//while the reference is less than 190
		{									//to increase from 140 to 190
			if(encoder == 1)				//increase count for every encoder pass
			{
				while(encoder == 1){}
				count_encoder++;
				pulse++;
			}
			
			if(count_encoder >= 100)		//if encoder count increases 100 (1 rev)
			{
				ref++;						//increase reference by 1
				count_encoder = 0;			//reset encoder count
			}
			
			error = ref - mean_v;			//calculate error every iteration
			PORTB = error;					//show error on port B
			output = kp * error;			//multiply error by proportional gain
			if(output < 0)					//if output less than 0, set to 0
			{
				output = 0;
			}
			else if(output > 255)			//if output greater than 255, set to 255
			{
				output = 255;
			}
			
			setPWM(output);					//set new motor speed
		}
		
		count_encoder = 0;					//reset counter values
		pulse = 0;
		while(pulse <= 6000)				//remain at 190 for 6000 encoder pulses
		{
			error = ref - mean_v;			//continue checking error
			PORTB = error;
			
			if(encoder == 1)
			{
				while(encoder == 1){}
				pulse++;
			}
			
			setPWM(output);
		}
		
		while(ref >= 140)					//bring down from 190 to 140 reference 
		{									//similar to increasing
			if(encoder == 1)
			{
				while(encoder == 1){}
				count_encoder++;
			}
			
			if(count_encoder >= 100)
			{
				ref--;
				count_encoder = 0;
			}
			
			error = ref - mean_v;
			PORTB = error;
			output = kp * error;
			if(output < 0)
			{
				output = 0;
			}
			else if(output > 255)
			{
				output = 255;
			}
			
			setPWM(output);
		}
		
		Flash();							//flash lights to show end of process
		
		if(blackButton == 1)
		{
			break;							//press black button to break out of process
		}
		
	}
}



void Mode3(void)
{
	PORTB = Mode;
	brake = 0;
	
	ref = 140;
	setPWM(ref);
	
	count_encoder = 0;
	pulse = 0;
	kp = 10;								//proportional gain
	kd = 3;									//derivative gain
	ki = 7;									//integral gain
	error_last = 0;							//previous error for use in derivative gain calculation
	
	while(1)								//very similar to mode 0
	{
		while(ref < 190)
		{
			if(encoder == 1)
			{
				while(encoder == 1){}
				count_encoder++;
				pulse++;
			}
			
			if(count_encoder >= 100)
			{
				ref++;
				count_encoder = 0;
			}
			
			error = ref - mean_v;
			sum_error += error;				//get sum of errors
			PORTB = error;
			
			output = kp * error + kd * (error - error_last) + ki * sum_error; //add all errors multiplied by gains
			
			if(output < 0)
			{
				output = 0;
			}
			else if(output > 255)
			{
				output = 255;
			}
			
			setPWM(output);
			error_last = error;
		}
		
		count_encoder = 0;
		pulse = 0;
		while(pulse <= 6000)
		{
			error = ref - mean_v;
			PORTB = error;
			
			if(encoder == 1)
			{
				while(encoder == 1){}
				pulse++;
			}
			
			setPWM(output);
		}
		
		while(ref > 140)
		{
			if(encoder == 1)
			{
				while(encoder == 1){}
				count_encoder++;
			}
			
			if(count_encoder >= 100)
			{
				ref--;
				count_encoder = 0;
			}
			
			error = ref - mean_v;
			sum_error += error;
			PORTB = error;
			
			output = kp * error + kd * (error - error_last) + ki * sum_error;  //add all errors multiplied by gains
		
			if(output < 0)
			{
				output = 0;
			}
			else if(output > 255)
			{
				output = 255;
			}
			
			setPWM(output);
			error_last = error;
		}
		
		
		Flash();
		
		if(blackButton == 1)
		{
			break;
		}
		
	}
}




void main (void)
{


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

	initPORT();
	
	initAtoD();
	
	initMode();

	
	
	
	
	while(1)
	{
		
		if(blackButton == 1)
		{
			while(blackButton == 1){}
			
			Mode = (PORTC>>4&1) + (PORTC>>5&1) * 2;
			Mode = Mode|0b11111100;
			Mode = ~Mode;
			
			if (Mode == 0).
			{
				Mode0();
			}
			
		
			else if (Mode == 3)
			{
				Mode3();
			}
		
		
			else
			{
				PORTB = 0xff;
			}
			
		}

	}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

}
