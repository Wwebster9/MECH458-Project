/*########################################################################
# MILESTONE: 5
# PROGRAM: 5
# PROJECT: Lab5 Demo
# GROUP: 7
# NAME 1: Wyatt Webster, V00927673
# NAME 2: Martin Yong, V00845874
# DESC: - This program runs the sorting apparatus to classify objects based on material and colour and sort them into the correct buckets on the 
		sorting tray.
		- This program uses the Hall effect sensor to home the stepper motor to the default black position, the reflectivity sensor to
		classify objects based on material type or colour, and the exit gate sensor to detect when an object is at the end of the conveyor.
		- A pause button connected to PE4 is available to stop the conveyor and display the current number of sorted
		parts and the partials (classified but not sorted into a bucket) to the LCD.
		- The ramp down pushbutton connected to PE5 will start a 10 second delay during which sorting can proceed. After the 10 seconds
		has passed the conveyor will stop and the number of sorted parts will be displayed to the LCD. At this point, the MCU must be reset to
		proceed. 
				
# REVISED: November 28, 2020
##########################################################################*/


/* include libraries and header files */
#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "LinkedQueue.h"
#include "lcd.h"


/* function headers */
void mTimer(int count);
void Move_Stepper_Motor(int direction, int steps);
void Operate_DC_Motor();
void Initialize_LCD();
void Initialize_PWM(int DC);
void Initialize_ADC();
void Stepper_Motor_Home();
void Determine_Bucket(int desired_bucket_loc);
void Timer3();


/* Defines */	
#define CLOCKWISE 0				// Stepper motor clockwise
#define COUNTER_CLOCKWISE 1		// Stepper motor counter-clockwise
#define DEGREES_180 100			// Steps to move stepper 180 degrees
#define DEGREES_90 50			// Steps to move stepper 90 degrees
#define DEGREES_60 33			// Steps to move stepper 60 degrees
#define DEGREES_45 25			// Steps to move stepper 45 degrees
#define DEGREES_30 17			// Steps to move stepper 30 degrees
#define STD_STEPPER_DELAY 20	// Initializes stepper motor delay to 20ms

#define DUTY_CYCLE 89		// Sets duty cycle for PWM signal

#define FORWARD 0			// DC motor forward direction
#define REVERSE 1			// DC motor reverse direction
#define STOP 2				// DC motor stop
#define DISABLE_DRIVER 3	// DC motor driver disable

#define AL_MAX 254			// Aluminum max value
#define ST_MIN 400			// Steel min value 
#define ST_MAX 700			// Steel max value 
#define WH_MIN 900			// White min value 
#define WH_MAX 955			// White max value 
#define BL_MIN 956			// Black min value 
#define BL_MAX 1023			// Black max value 


/* Global variables */
volatile int curPosition = 1;				// Variable to track the current position of the stepper motor
volatile char maxAccel = 20;				// Slow speed (longest delay between steps)
volatile char curAccel = STD_STEPPER_DELAY;	// Current speed set to default stepper delay of 20ms
volatile char minAccel = 8;					// Fast speed (shortest delay between steps)

unsigned int Current_DC_Motor_Dir;			// Keeps track of DC motor direction

volatile unsigned int ADC_result;			// Used to store result from ADC conversion
volatile unsigned int ADC_result_flag;		// Flag is set when ADC conversion finishes
volatile unsigned int ADC_result_min;		// Sets a minimum value for ADC

volatile char STATE;
volatile unsigned int Hall_Sensor_Flag = 0;	// Used to home the stepper motor to black position
char Part_Type;								// Temporary storage for the current part type
unsigned int unsortedItems = 0;				// Stores the number of bits that are classified but not yet sorted into a bin
unsigned int Desired_Bucket_Position;		// Desired position of the sorting tray
unsigned int Current_Bucket_Position;		// Current position of the sorting tray

char Al_numSorted = 0;	// Number of sorted aluminum parts
char St_numSorted = 0;	// Number of sorted steel parts
char Wh_numSorted = 0;	// Number of sorted white parts
char Bl_numSorted = 0;	// Number of sorted black parts

unsigned int counter = 0;	// Used to count the number of times Timer3 has counted to the maximum value

link *head;					/* The ptr to the head of the queue */
link *tail;					/* The ptr to the tail of the queue */
link *newLink;				/* A ptr to a link aggregate data type (struct) */
element eTest;				/* A variable to hold the aggregate data type known as element */


int main(int argc, char *argv[]){
	
	/* Variable declarations and initializations */
	STATE = 0;				
	newLink = NULL;			
	setup(&head, &tail);
	
	
	/* Timer setup */
	CLKPR = 0x80;		// Enables system clock to be changed
	CLKPR = 0x01;		// Sets system clock to 8MHz
	TCCR1B|=_BV(CS11);	// Sets timer 1 to run at 1MHz by using pre-scaler of 8
	TCCR3B|=_BV(CS11);	// Sets timer 3 to run at 1MHz by using a pre-scaler of 8


	/* Port direction setup */
 	DDRA = 0xFF;	// All port A bits set to outputs for stepper motor
 	DDRB = 0xFF;	// All port B bits set to outputs for DC motor
 	DDRC = 0xFF; 	// All port C bits set to outputs for LCD
 	DDRD = 0xF0;	// PD0 -> PD4 set to inputs for interrupts
	DDRE = 0x03;	// All port E bits (except first two) set to inputs
 	DDRF = 0x00;	// All port F bits set to inputs for ADC
	
	
	/* Port Initializations */
 	PORTA = 0x00;	// All I/O pins off
 	PORTB = 0x0F;	// Motor will initially be set to stop
 	PORTC = 0x00;	// All I/O pins off (LEDs off)
	
	
	/* Interrupt initializations */
	cli();	// Disables all interrupts

	//EICRA |= _BV(ISC01);				// set to falling edge 
	EICRA |= _BV(ISC11);				// set to falling edge 
	EICRA |= _BV(ISC21) | _BV(ISC20);	// set to rising edge 
	EICRA |= _BV(ISC31);				// set to falling edge 
	EICRA |= _BV(ISC41);				// set to falling edge 
	EICRA |= _BV(ISC51);				// set to falling edge 

	//EIMSK |= _BV(INT0);	// enable INT0 (OI sensor - Active Low)    NOT USED
	EIMSK |= _BV(INT1);		// enable INT1 (HE sensor - Active Low)		   
	EIMSK |= _BV(INT2);		// enable INT2 (OR sensor - Active High) 
	EIMSK |= _BV(INT3);		// enable INT3 (EX sensor - Active Low)
	EIMSK |= _BV(INT4);		// enable INT4 (Pause button - Active Low)
	EIMSK |= _BV(INT5);		// enable INT5 (RampDown - Active Low)

	/* Function initializations */
	Initialize_LCD();			
	Initialize_ADC();						
	Initialize_PWM(DUTY_CYCLE);	
	
	sei();	// Enable all interrupts


	/* Stepper motor put into default black position */
  	LCDWriteStringXY(0,0,"Stepper Home");	
  	Stepper_Motor_Home();						// Run function to home stepper motor position
	Current_Bucket_Position = 1;				// Set to 1 for black 
  	EIMSK = EIMSK & 0xFD;						// Disable Hall sensor INT1 (only needed to home sorting tray)
  	LCDWriteStringXY(0,0,"Stepper Motor at");
  	LCDWriteStringXY(1,1,"Black Position");
  	mTimer(2000);								// Wait 2 seconds
	
	
	/* Display message and start DC motor to run conveyor */
	LCDClear();
	LCDWriteStringXY(4,0,"Starting");
	LCDWriteStringXY(4,1,"Conveyor");
	mTimer(3000);						// Wait 3 seconds
	
	Operate_DC_Motor(FORWARD);			// Start conveyor
	
	LCDClear();
	LCDWriteStringXY(2,0,"Please place");
	LCDWriteStringXY(5,1,"parts");
	
	
	/* Sorting portion of the code starts here */
	/* STATE 1 is triggered by the exit gate
	   STATE 2 is triggered by the pause button
	   STATE 3 is triggered by ramp down function */	   
	
	while(1){	/* Main loop - continues always unless ramp down has been hit,
				   resulting in a 10s delay and then the program shuts down */
		
		while(STATE == 1){				// Runs while an item is in exit gate
			eTest = firstValue(&head);	// Check first item in linked list
			Desired_Bucket_Position = eTest.itemCode;			
			if (Desired_Bucket_Position == Current_Bucket_Position){	// If bucket is in correct position - start conveyor
				Operate_DC_Motor(FORWARD);
			}else{
				Determine_Bucket(Desired_Bucket_Position);	// Call function to determine the correct bucket and move stepper to that location
				Operate_DC_Motor(FORWARD);					// Re-start conveyor
			}// end else
			switch (Desired_Bucket_Position){	// Increment appropriate bucket counter
			case 1:
				Bl_numSorted++;
				break;
			case 2:
				Al_numSorted++;
				break;
			case 3:
				Wh_numSorted++;
				break;
			case 4:
				St_numSorted++;
				break;
			default:
				LCDClear();
				LCDWriteStringXY(0, 0, "Sorting error");	// Display error code if none of the above cases are met
				break;
			}// end switch
			dequeue(&head, &tail);	// Remove the top item in the list
			STATE = 0;				// Reset the state variable to exit loop
		}// end while
									
		while(STATE == 2){						// Runs once pause button has been pressed - waits for pause to be pressed again
			unsortedItems = size(&head,&tail);	// Grabs number of unsorted items (items not in a bucket yet)		
			/* Display currently sorted pieces and partials (unsorted) */
			LCDWriteStringXY(0,0,"Bl Al Wh St Ptl");
			LCDWriteIntXY(0,1,Bl_numSorted, 2);
			LCDWriteIntXY(3,1, Al_numSorted, 2);
			LCDWriteIntXY(6,1,Wh_numSorted, 2);
			LCDWriteIntXY(9,1,St_numSorted, 2);
			LCDWriteIntXY(12,1,unsortedItems,3);	
			mTimer(500);						// Delay added for readability of the LCD
			LCDClear();		
		}// end while
	
	}// end while
	
	return(0);

}// end Main



/**************************************************************************************/
/***************************** SUBROUTINES ********************************************/
/**************************************************************************************/




/* Driver for Timer3 background timer used for the ramp down */
void Timer3(){
	/* Set the waveform generation mode bit description to Clear Timer on Compare Math mode (CTC) only */
	TCCR3B |= _BV(WGM12);			/* set WGM bits to 0100 */
	OCR3A = 0x03E8;					/* Set Output Compare Register for 1000 cycles = 1ms */
	TIMSK3 = TIMSK3 | 0b00000010;	/* Enable the output compare interrupt enable */
	TCNT3 = 0x0000;					/* Sets initial value of Timer Counter to 0x0000 */
	TIFR3 |= _BV(OCF3A);			/* clear the timer interrupt flag and begin new timing */
}// end Timer3





/* Driver for the timer */
void mTimer(int count){
	// Variable declarations
	int i; /* keeps track of loop number */
	i = 0; /* initializes loop counter */
	
	/* Set the waveform generation mode bit description to Clear Timer on Compare Math mode (CTC) only */
	TCCR1B |= _BV(WGM12);			/* set WGM bits to 0100 */
	OCR1A = 0x03E8;					/* Set Output Compare Register for 1000 cycles = 1ms */
	TCNT1 = 0x0000;					/* Sets initial value of Timer Counter to 0x0000 */
	TIFR1 |= _BV(OCF1A);			/* clear the timer interrupt flag and begin new timing */
	
	/* Poll the timer to determine when the timer has reached 0x03E8 */
	while(i<count){
		if((TIFR1 & 0x02) == 0x02){
			TIFR1 |= _BV(OCF1A);		/* clear interrupt flag by writing a ONE to the bit */
			i++;						/* increment loop number */
			} /* end if */
	} /* end while */
			return;
} /* mTimer */





/**************************************************************************************
* DESC: Function to turn the stepper to the correct bucket location for the item currently in the exit gate
* INPUT: The desired bucket position
* RETURNS: None
*/
void Determine_Bucket(int desired_bucket_loc){
	/*              \   BL  /
				     \  1  /
					  \   /
					   \ /
				  AL	\     ST
				  2	   / \    4
					  /   \
					 / WH  \
					   3
	*/				
	
	int position = Desired_Bucket_Position - Current_Bucket_Position;	
	
	if (abs(position) == 2){
		Move_Stepper_Motor(CLOCKWISE, DEGREES_180);
		if (Current_Bucket_Position < 3){
			Current_Bucket_Position = Current_Bucket_Position + 2;
		}else{
			Current_Bucket_Position = Current_Bucket_Position - 2;
		}// end if else
	}else if (position == 1 || position == -3){
		Move_Stepper_Motor(CLOCKWISE, DEGREES_90);
		if (Current_Bucket_Position < 4){
			Current_Bucket_Position++;
		}else{
			Current_Bucket_Position = 1;
		}// end if else
	}else if (position == -1 || position == 3){
		Move_Stepper_Motor(COUNTER_CLOCKWISE, DEGREES_90);
		if (Current_Bucket_Position == 1){
			Current_Bucket_Position = 4;
		}else{
			Current_Bucket_Position--;
		}// end if else
	}// end else ifs
	
}// end Determine_Bucket





/**************************************************************************************
* DESC: Function to handle moving the stepper motor
* INPUT: The direction we want to move the motor and the number of steps
* RETURNS: None
*/
void Move_Stepper_Motor(int direction, int steps){
	
	/*  Stepper motor truth table for lab stepper motor (two-phase on full-stepping)
		Position 1 is step 1 = 110110 = 54
		Position 2 is step 2 = 101110 = 46
		Position 3 is step 3 = 101101 = 45
		Position 4 is step 4 = 110101 = 53
	*/
	
	/* Variable declarations */
	static int stepmotor_output[5] = {0, 54, 46, 45, 53}; 

	/* Check if clockwise */
	if(direction == CLOCKWISE){
		for (int i = 1; i <= steps; i++){
			
		/* Check if position is at position 4 and move to position 1 if true */
		if ((++curPosition) > 4){
			curPosition = 1;
		}// end if
		
		PORTA = stepmotor_output[curPosition];
		mTimer(curAccel);
		
		/* Acceleration profile */
		
		// Accelerate
		if(i < 21 && (curAccel >= minAccel)){
			curAccel = curAccel - 1;
		}// end if

		// Decelerate
		if(i > (steps - 20) && (curAccel < maxAccel)){
			curAccel = curAccel + 1;
		}// end if
				
		}// end for
	}// end if
	
	/* Check if counterclockwise */
	if(direction == COUNTER_CLOCKWISE){
		for (int k = 1; k <= steps; k++){
			
		/* Check if position is at position 1 and move to position 4 if true */
		if ((--curPosition) < 1){
			curPosition = 4;
		}// end if
			
		PORTA = stepmotor_output[curPosition];
		mTimer(curAccel);
			
		/* Acceleration profile */
		
		// Accelerate
		if(k < 21 && (curAccel >= minAccel)){
			curAccel = curAccel - 1;
		}// end if

		// Decelerate
		if(k > (steps - 20) && (curAccel < maxAccel)){
			curAccel = curAccel + 1;
		}// end if
			
		}// end for
	}// end if
	
	curAccel = STD_STEPPER_DELAY;
	return;
}//end Move_Stepper_Motor





/*
# Function : Stepper_Motor_Default
# Commands : None
# DESC : Moves the stepper motor to the default black position
*/
void Stepper_Motor_Home(){
	
	static int stepmotor_output[5]= {0, 54, 46, 45, 53};
	
	/* Continue moving stepper motor until Hall sensor is triggered on black bucket */
	while(Hall_Sensor_Flag == 0){
		
		/* Check if position is at position 4 and move to position 1 if true */
		if ((++curPosition) > 4){
			curPosition = 1;
		}// end if

		/* Turn stepper motor */
		PORTA = stepmotor_output[curPosition];
		mTimer(STD_STEPPER_DELAY);
		
	}// end while
	
}// end Stepper_Motor_Home





/**************************************************************************************
* DESC: Operates the DC motor by sending commands to the DC motor driver
		Always brakes to Vcc and delays for 10ms before sending the command to move 
		in a certain direction
* INPUT: The motor command (FORWARD, REVERSE OR STOP) 
* RETURNS: None
*/
void Operate_DC_Motor(int command){
	
	/* Truth table for DC motor
	INA INB DIAGA/ENA DIAGB/ENB OUTA OUTB     OPERATING MODE
	1	 1      1         1		 H    H       Brake to Vcc				output = 0x0F
	1	 0		1		  1	     H    L       Clockwise (CW)			output = 0x0B
	0	 1	    1		  1      L    H       Counterclockwise (CCW)	output = 0x07
	
	Port wiring
	INA  INB  ENA  ENB
	PB3  PB2  PB1  PB0
	
	*/
	switch(command){
		case(FORWARD):
			PORTB = 0x0F;					// Brake to Vcc first
			mTimer(10);						// Delay 10ms
			PORTB = 0x07;					// Send command to move motor CCW (FORWARD)
			Current_DC_Motor_Dir = FORWARD;	// Update current motor direction variable
			break;
		case(REVERSE):
			PORTB = 0x0F;					// Brake to Vcc first
			mTimer(10);						// Delay 10ms
			PORTB = 0x0B;					// Send command to move motor CW (REVERSE)
			Current_DC_Motor_Dir = REVERSE;	// Update current motor direction variable
			break;
		case(STOP):
			PORTB = 0x0F;					// Send command to stop motor 
			Current_DC_Motor_Dir = STOP;	// Update current motor direction variable
			break;
		case(DISABLE_DRIVER):				
			PORTB = 0x00;					// Command to disable DC motor driver
			Current_DC_Motor_Dir = STOP;	// Update current motor direction variable
			break;
	}// end switch
	
}// end Operate_DC_Motor





/**************************************************************************************
* DESC: Function to initialize the LCD display 
* INPUT: None
* RETURNS: None
*/
void Initialize_LCD(){
	
	InitLCD(LS_BLINK|LS_ULINE);				// Initialize the LCD module
	LCDClear();								// Clear the LCD
	
}//end Initalize_LCD





/**************************************************************************************
* DESC: Function to initialize the Pulse-width Modulation (PWM) signal
* INPUT: Integer value that sets duty cycle
* RETURNS: None
*/
void Initialize_PWM(int DC){
	
	TCCR0A &= 0x00;			// Set all bits initially to 0
	TCCR0A |= _BV(WGM00) | _BV(WGM01) | _BV(COM0A1);	// Sets WGM00 and WGM01 to 1 to set to fast PWM mode
														// Set COMOA1 to clear OC0A
	TCCR0B &= 0x00;			// Set all bits initially to 0
	TCCR0B |= _BV(CS00);	// Set to 1/64 pre-scaler
	TCCR0B |= _BV(CS01);
	OCR0A = DC;				// Value to be compared to in ORC0
	
}//end Initialize_PWM





/**************************************************************************************
* DESC: Function to initialize the ADC
* INPUT: None
* RETURNS: None
*/
void Initialize_ADC(){
	
	ADCSRA |= _BV(ADEN);				// Enable ADC
	ADCSRA |= _BV(ADIE);				// Enable interrupt of ADC 
	ADMUX |= _BV(REFS0) | _BV(MUX0);	// Reference voltage set to AVCC, and set to channel 0

}// end Initialize_ADC





/**********************************************Interrupt Service Routines**********************************************/

/* Interrupt will be triggered when the ADC is done */
ISR(ADC_vect){
	ADC_result = ADC;	// store result of ADC conversion in global variable
	
	if(ADC_result < ADC_result_min){	// Find lowest value
		ADC_result_min = ADC_result;
	}// end if
	
	if(PIND & 0x04){
		ADCSRA |= _BV(ADSC);	// Start another ADC Conversion
	}// end if
	else{	
		/* Display minimum ADC value found */
		LCDClear();
		LCDWriteStringXY(0,0,"ADC result:");
		LCDWriteIntXY(11, 0, ADC_result_min, 4);
		
		/* Classify object using min ADC value*/
		if(ADC_result_min <= AL_MAX){
			Part_Type = 2; // Aluminum
			LCDWriteStringXY(0,1,"Aluminum");
			
			}else if(ADC_result_min <= ST_MAX){
			Part_Type = 4; // Steel
			LCDWriteStringXY(0,1,"Steel   ");
			
			}else if(ADC_result_min <= WH_MAX){
			Part_Type = 3; // White
			LCDWriteStringXY(0,1,"White   ");
			
			}else if(ADC_result_min <= BL_MAX){
			Part_Type = 1; // Black
			LCDWriteStringXY(0,1,"Black   ");
			
			}else{
			LCDWriteStringXY(0,1,"Cannot Identify");
		}// end else
		
		/* Add integer value of part type to linked list */
		initLink(&newLink);
		newLink->e.itemCode = Part_Type;
		enqueue(&head, &tail, &newLink);
	}// end else
}// end ISR(ADC_vect)


/* Set up the External Interrupt 0 Vector */
//  ISR(INT0_vect){
	// Not used
//} // end ISR(INT0_vect)


/* Set up the External Interrupt 1 Vector
   (HE sensor - Active Low) */
ISR(INT1_vect){
	Hall_Sensor_Flag = 1;
}// end ISR(INT1_vect)


/* Set up the External Interrupt 2 Vector
   (OR sensor - Active High) */
ISR(INT2_vect){
	mTimer(1);				// 1ms delay
	if(PIND & 0x04){		// Check that OR sensor is still active
	ADC_result_min = 1024;	// Initialize min value
	ADCSRA |= _BV(ADSC);	// Start ADC Conversion
	}// end if
}// end ISR(INT2_vect)


/* Set up the External Interrupt 3 Vector
   (Exit (EX) sensor - Active Low) */
ISR(INT3_vect){
	Operate_DC_Motor(STOP);	// Stop DC motor
	STATE = 1;
}// end ISR(INT3_vect)


/* Set up the External Interrupt 4 Vector
   Pause button */
ISR(INT4_vect){
	mTimer(150); // Switch de-bouncing
	Operate_DC_Motor(STOP);	// Stop DC motor
	/* If already paused, start motor - else pause system */
 	if(STATE == 2){
 		STATE = 0;
 		LCDClear();
 		Operate_DC_Motor(FORWARD);
 	}// end if
 	else{
		STATE = 2;
	}// end else
}// end ISR(INT4_vect)


/* Set up the External Interrupt 4 Vector
   Ramp down button */
 ISR(INT5_vect){
	 mTimer(20);	// Switch de-bouncing
	 LCDClear();
	 LCDWriteStringXY(0,0,"Ramp Down");
	 Timer3();		// Call function to initialize and start Timer3 
 }// end ISR(INT5_vect)


ISR(TIMER3_COMPA_vect){		
	counter++;					// Increment counter
	if(counter == 10000){		// Check if counter has reached 10000 (10 seconds) 
		cli();					// Disable all interrupts
		Operate_DC_Motor(STOP);	// Stop conveyor
		LCDClear();
		/* Display currently sorted pieces and partials (unsorted) */
		LCDWriteStringXY(0,0,"Bl Al Wh St Ptl");
		LCDWriteIntXY(0,1,Bl_numSorted, 2);
		LCDWriteIntXY(3,1, Al_numSorted, 2);
		LCDWriteIntXY(6,1,Wh_numSorted, 2);
		LCDWriteIntXY(9,1,St_numSorted, 2);
		LCDWriteIntXY(12,1,unsortedItems,3);
		while(1){}	// Program sits here until reset
	}else{
		TIFR3 |= _BV(OCF3A); // reset flag and start new timing cycle
	}// end else
}// end ISR(TIMER3_COMPA_vect)
ISR(BADISR_vect){
	while (1){
		LCDClear();
		LCDWriteStringXY(6,0,"ERROR");
		LCDWriteStringXY(4,1,"RESET MCU");
	}// end while
}// end ISR(BADISR_vect)





/**********************************************Linked List Functions**********************************************/

/**************************************************************************************
* DESC: initializes the linked queue to 'NULL' status
* INPUT: the head and tail pointers by reference
*/

void setup(link **h,link **t){
	
	*h = NULL;		/* Point the head to NOTHING (NULL) */
	*t = NULL;		/* Point the tail to NOTHING (NULL) */
	return;
	
}/*setup*/
	
	
	
	
	
/**************************************************************************************
* DESC: This initializes a link and returns the pointer to the new link or NULL if error
* INPUT: the head and tail pointers by reference
*/
void initLink(link **newLink){
	
	*newLink = malloc(sizeof(link));
	(*newLink)->next = NULL;
	return;
	
}/*initLink*/





/****************************************************************************************
*  DESC: Accepts as input a new link by reference, and assigns the head and tail
*  of the queue accordingly
*  INPUT: the head and tail pointers, and a pointer to the new link that was created
*/
/* will put an item at the tail of the queue */
void enqueue(link **h, link **t, link **nL){
	
	if (*t != NULL){	/* Not an empty queue */		
		(*t)->next = *nL;
		*t = *nL; 		//(*t)->next;
	}// end if
	else{				/* It's an empty Queue */
		*h = *nL;
		*t = *nL;
	}// end else
	return;
	
}/*enqueue*/





/**************************************************************************************
* DESC : Removes the link from the head of the list
* INPUT: The head and tail pointers
* 		 which the removed link will be assigned to
*/
/* This will remove the link and element within the link from the head of the queue */
void dequeue(link **h, link **t){
	
	if (*h != NULL){	// Check empty list
	link *temp;			// Create temp variable
	temp = *h;			// Set temp variable to head
	*h = (*h)->next;	// Set new head to next item in list
	free(temp);			// Clear old head from memory
	if(*h == NULL){		// If the head is now NULL,
		*t = *h;		// Set the tail equal to the head (both are now NULL)
	}/*if*/
	}/*if*/
	return;
	
}/*dequeue*/	





/**************************************************************************************
* DESC: Peeks at the first element in the list
* INPUT: The head pointer
* RETURNS: The element contained within the queue
*/
/* This simply allows you to peek at the head element of the queue and returns a NULL pointer if empty */
element firstValue(link **h){
	
	return((*h)->e);
	
}/*firstValue*/





/**************************************************************************************
* DESC: deallocates (frees) all the memory consumed by the Queue
* INPUT: the pointers to the head and the tail
*/
/* This clears the queue */
void clearQueue(link **h, link **t){

	link *temp;
	while (*h != NULL){
		temp = *h;
		*h=(*h)->next;
		free(temp);
	}/*while*/
	/* Last but not least set the tail to NULL */
	*t = NULL;
	return;
		
}/*clearQueue*/





/**************************************************************************************
* DESC: Checks to see whether the queue is empty or not
* INPUT: The head pointer
* RETURNS: 1:if the queue is empty, and 0:if the queue is NOT empty
*/
/* Check to see if the queue is empty */
char isEmpty(link **h){
	
	return(*h == NULL);
	
}/*isEmpty*/





/**************************************************************************************
* DESC: Obtains the number of links in the queue
* INPUT: The head and tail pointer
* RETURNS: An integer with the number of links in the queue
*/
/* returns the size of the queue*/
int size(link **h, link **t){

	link 	*temp;				// Temp pointer to traverse list
	int 	numElements = 0;	// Initialize numElements
	temp = *h;					// Point to the head
	while(temp != NULL){		// Loop while the temp variable hasn't hit a NULL value (end of list)
	numElements++;				// Increment numElements
	temp = temp->next;			// Set temp to the next item in the list
	}// end while
	return(numElements);
	
}/*size*/

