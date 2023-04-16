/*  Time of Flight for 2DX4 -- Studio W8-0
                Code written to support data collection from VL53L1X using the Ultra Light Driver.
                I2C methods written based upon MSP432E4 Reference Manual Chapter 19.
                Specific implementation was based upon format specified in VL53L1X.pdf pg19-21
                Code organized according to en.STSW-IMG009\Example\Src\main.c
                
                The VL53L1X is run with default firmware settings.


            Written by Tom Doyle
            Updated by  Hafez Mousavi Garmaroudi
            Last Update: March 17, 2020
						
						Last Update: March 03, 2022
						Updated by Hafez Mousavi
						__ the dev address can now be written in its original format. 
								Note: the functions  beginTxI2C and  beginRxI2C are modified in vl53l1_platform_2dx4.c file
								
						Modified March 16, 2023 
						by T. Doyle
							- minor modifications made to make compatible with new Keil IDE
							
		Template purposed for final deliverable of 2DX3 project
		Written by Brent Menheere, 400362843
		Last Modified: April 12, 2023

*/
#include <stdint.h>
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "tm4c1294ncpdt.h"
#include "VL53L1X_api.h"





#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable

#define MAXRETRIES              5           // number of receive attempts before giving up
void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           													// activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          												// activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};																		// ready?

    GPIO_PORTB_AFSEL_R |= 0x0C;           																	// 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;             																	// 4) enable open drain on PB3 only

    GPIO_PORTB_DEN_R |= 0x0C;             																	// 5) enable digital I/O on PB2,3
//    GPIO_PORTB_AMSEL_R &= ~0x0C;          																// 7) disable analog functionality on PB2,3

                                                                            // 6) configure PB2,3 as I2C
//  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
    I2C0_MCR_R = I2C_MCR_MFE;                      													// 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                       	// 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
//    I2C0_MTPR_R = 0x3B;                                        						// 8) configure for 100 kbps clock
        
}

//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void){
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                // activate clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    // allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00;                                        // make PG0 in (HiZ)
  GPIO_PORTG_AFSEL_R &= ~0x01;                                     // disable alt funct on PG0
  GPIO_PORTG_DEN_R |= 0x01;                                        // enable digital I/O on PG0
                                                                                                    // configure PG0 as GPIO
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
  GPIO_PORTG_AMSEL_R &= ~0x01;                                     // disable analog functionality on PN0

    return;
}

//XSHUT     This pin is an active-low shutdown input; 
//					the board pulls it up to VDD to enable the sensor by default. 
//					Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input (HiZ)
    
}

void PortM0_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;                 // Activate the clock for Port M
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R11) == 0){};        // Allow time for clock to stabilize 
	GPIO_PORTM_DIR_R = 0b00000100;  //0b00000000    								      // Make PM0 input
  GPIO_PORTM_DEN_R = 0b00001111;  //0b00000001
	return;
}

void PortH0H1H2H3_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;                 // Activate the clock for Port H
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R7) == 0){};        // Allow time for clock to stabilize 
	GPIO_PORTH_DIR_R = 0b00001111;  //0b00000000    								      
  GPIO_PORTH_DEN_R = 0b00001111;  //0b00000011
	return;
}

void rotate_CW(int delay, int step){ //clock wise rotation
	
		for(int i=0; i<step; i++){
			GPIO_PORTH_DATA_R = 0b00001100;
			SysTick_Wait1us(delay);
			GPIO_PORTH_DATA_R = 0b00000110;
			SysTick_Wait1us(delay);
			GPIO_PORTH_DATA_R = 0b00000011;
			SysTick_Wait1us(delay);
			GPIO_PORTH_DATA_R = 0b00001001;
			SysTick_Wait1us(delay);
		}
		
}

void rotate_CCW(int delay, int step){ //counter clock wise rotation
	
		for(int i=0; i<step; i++){
			GPIO_PORTH_DATA_R = 0b00001001;
			SysTick_Wait1us(delay);
			GPIO_PORTH_DATA_R = 0b00000011;
			SysTick_Wait1us(delay);
			GPIO_PORTH_DATA_R = 0b00000110;
			SysTick_Wait1us(delay);
			GPIO_PORTH_DATA_R = 0b00001100;
			SysTick_Wait1us(delay);
		}
		
}

void PortQ_Init(void) { //for testing bus speed

    //Use PortQ pins for output
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R14;                // activate clock for Port Q
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R14) == 0){};    // allow time for clock to stabilize
    GPIO_PORTQ_DIR_R |= 0xFF;                                        // make PN0 out (PN0 built-in LED1)
		GPIO_PORTQ_DEN_R |= 0xFF;                                        // enable digital I/O on PN0

    return;
}


//*********************************************************************************************************
//*********************************************************************************************************
//***********					MAIN Function				*****************************************************************
//*********************************************************************************************************
//*********************************************************************************************************
uint16_t	dev = 0x29;			//address of the ToF sensor as an I2C slave peripheral
int status=0;

int main(void) {
  uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
  uint16_t wordData;
  uint16_t Distance;
  uint16_t SignalRate;
  uint16_t AmbientRate;
  uint16_t SpadNum; 
  uint8_t RangeStatus;
  uint8_t dataReady;

	//initialize
	PLL_Init();	
	SysTick_Init();
	onboardLEDs_Init();
	I2C_Init();
	UART_Init();
	PortM0_Init();
	PortH0H1H2H3_Init();
	PortQ_Init();
	
	// hello world!
	UART_printf("Program Begins\r\n");
	int mynumber = 1;
	sprintf(printf_buffer,"2DX ToF Program Studio Code %d\r\n",mynumber);
	UART_printf(printf_buffer);


/* Those basic I2C read functions can be used to check your own I2C functions */
	status = VL53L1X_GetSensorId(dev, &wordData);

	sprintf(printf_buffer,"(Model_ID, Module_Type)=0x%x\r\n",wordData);
	UART_printf(printf_buffer);

	// Booting ToF chip
	while(sensorState==0){
		status = VL53L1X_BootState(dev, &sensorState);
		SysTick_Wait10ms(10);
  }
	FlashAllLEDs();
	UART_printf("ToF Chip Booted!\r\n Please Wait...\r\n");
	
	status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
	
  /* This function must to be called to initialize the sensor with the default setting  */
  status = VL53L1X_SensorInit(dev);
	Status_Check("SensorInit", status);

status = VL53L1X_StartRanging(dev);   // This function has to be called to enable the ranging

int tof_enable = 0;
int enable = 0;
int delay = 1000; //delay to get fastest functional spin
int step = 1;
int count = 0;
int direction;
int capture;

uint8_t button_data_pressed = 0;

//FOR TESTING BUS SPEED, commented out when running regularly
//while(1) {
//		SysTick_Wait(1000000);
//    GPIO_PORTQ_DATA_R ^= 0b11111111;
//} 

while(1){
	
	if ((GPIO_PORTM_DATA_R & 0b00000001) == 0) { //MOTOR BUTTON
			enable = !enable;                         //Complements motor enable
			while ((GPIO_PORTM_DATA_R & 0b00000001) == 0) { SysTick_Wait1us(10); }
   }

	
	if ((GPIO_PORTM_DATA_R & 0b00000010) == 0) { //DATA BUTTON PRESSED
		if (!button_data_pressed) {  //enter here at begining
				tof_enable = !tof_enable; //tells TOF to start taking measurments
				FlashLED4(1);             
				button_data_pressed = 1;  //sets it back to 1
		
				if (tof_enable) { //if tof enabled, start data
						UART_printf("start scanning\n"); //Tells python to start printing data
				} 
				else{ //second button pressed turns tof off, stop data collection and go to OPEN 3D
						UART_printf("stop\n"); //if TOF enable gets turned off, send stop to python to stop taking data
			}
	  }
	} 
	else {
		button_data_pressed = 0;
	}

  if(enable){	//rotate motor
		FlashLED3(1);
		if (direction == 1){
			rotate_CW(delay, step);
			count += 1;
		} 
		else {
			rotate_CCW(delay, step);
			count += 1;
		}
	}

	if (capture) { //collect data
		
		UART_printf("s\n");
		if (!tof_enable) { //tells python that legitiment information is coming in
			UART_printf("start scanning\n");
		}

	  while (dataReady == 0){ 		//wait until the ToF sensor's data is ready
		  status = VL53L1X_CheckForDataReady(dev, &dataReady);
      VL53L1_WaitMs(dev, 5);
	  }
		
		dataReady = 0;
	  status = VL53L1X_GetDistance(dev, &Distance);	

		FlashLED4(1);
		FlashLED3(1);

	  status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
		
		sprintf(printf_buffer,"%u\r\n", Distance); // print the resulted readings to UART
		UART_printf(printf_buffer);
	  SysTick_Wait10ms(50);
		
		capture =0; //turn capture off so it doesnt continue taking data
		
  }
	
	if(tof_enable && count % 16 == 0 && count != 0){	// 360/11.25 = 32, 512/32 = 16
		capture = 1; //turn capture on every 11.25 degrees
	}

	if(count >= 512){ //512 steps is a full rotation, deemed to be 512 in studio 4B
		count = 0;		//reset count so we can operate again
		direction = !direction;
	 }

	
 }

  VL53L1X_StopRanging(dev);
  while(1) {}
		
}
