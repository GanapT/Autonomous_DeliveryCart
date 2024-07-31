#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKL46Z4.h"
#include "fsl_debug_console.h"

void delay(unsigned short delay_t) {
	SIM->SCGC6 |= (1 << 24); // Clock Enable TPM0
	SIM->SOPT2 |= (0x2 << 24); // Set TPMSRC to OSCERCLK
	TPM0->CONF |= (0x1 << 17); // Stop on Overflow
	TPM0->SC = (0x1 << 7) | (0x07); // Reset Timer Overflow Flag, Set Prescaler 128 (round up)
	TPM0->MOD = delay_t * 61 + delay_t/2; //
	TPM0->SC |= 0x01 << 3; // Start the clock!
	while(!(TPM0->SC & 0x80)){} // Wait until Overflow Flag
	return;
}

void delay_n(int times){
	for(int i=0;i<=times;i++){
		delay(1000);
	}
}

void ports() {
	SIM->SCGC5 |= (1 << 10) | (1 << 11) | (1 << 12) | (1 << 13);
	PORTB->PCR[0] &= ~0x700;
	PORTB->PCR[1] &= ~0x700;
	PORTB->PCR[2] &= ~0x700;
	PORTB->PCR[3] &= ~0x700;
	PORTB->PCR[0] |= 0x700 & (1 << 8);
	PORTB->PCR[1] |= 0x700 & (1 << 8);
	PORTB->PCR[2] |= 0x300;
	PORTB->PCR[3] |= 0x300;
	PORTC->PCR[1] &= ~0x700;
	PORTC->PCR[2] &= ~0x700;
	PORTC->PCR[1] |= 0x700 & (1 << 8);
	PORTC->PCR[2] |= 0x700 & (1 << 8);
	GPIOB->PDOR |= (1 << 0);
	GPIOB->PDOR |= (1 << 1);
	GPIOB->PDOR |= (1 << 2);
	GPIOB->PDOR |= (1 << 3);
	GPIOC->PDOR |= (1 << 1);
	GPIOC->PDOR |= (1 << 2);
	PORTE->PCR[16] &= ~0x700;
	PORTE->PCR[16] |= 0x700;
	PORTE->PCR[17] &= ~0x700;
	PORTE->PCR[17] |= 0x700;
	SIM->SCGC6 |= (1 << 24) | (1 << 26);
	SIM->SOPT2 |= (0x2 << 24);
	TPM2->CONTROLS[0].CnSC |= (0x2 << 2) | (0x2 << 4);
	TPM2->CONTROLS[1].CnSC |= (0x2 << 2) | (0x2 << 4);
	TPM2->CONF |= (1 << 6) | (1 << 7);
	TPM2->MOD = 7999;
	TPM2->SC |= 0x01 << 3;
	//	TPM2->CONTROLS[0].CnV = 7999;
	//	TPM2->CONTROLS[1].CnV = 7999;
}
void Motion_Motor(int turn) {
	SIM->SCGC6 |= (1 << 24) | (1 << 26);
	SIM->SOPT2 |= (0x2 << 24);
		TPM2->CONTROLS[0].CnSC |= (0x2 << 2) | (0x2 << 4);
		TPM2->CONTROLS[1].CnSC |= (0x2 << 2) | (0x2 << 4);
		TPM2->CONF |= (1 << 6) | (1 << 7);
		TPM2->MOD = 7999;
		TPM2->SC |= 0x01 << 3;
	switch(turn) {
	case 1: // forward
		TPM2->CONTROLS[0].CnV = 7999;
		TPM2->CONTROLS[1].CnV = 7999;
		GPIOB->PDDR |= (1 << 0);
				GPIOB->PDDR &= ~(1 << 1);

				GPIOC->PDDR |= (1 << 1);
				GPIOC->PDDR &= ~(1 << 2);
		GPIOB->PDDR |= (1<<3);
		GPIOB->PDDR |= (1<<2);
		break;
	case 2: // ACW
		TPM2->CONTROLS[0].CnV = 7999;
		TPM2->CONTROLS[1].CnV = 5000;
		GPIOB->PDDR |= (1 << 0);
				GPIOB->PDDR &= ~(1 << 1);

				GPIOC->PDDR |= (1 << 1);
				GPIOC->PDDR &= ~(1 << 2);
		GPIOB->PDDR |= (1<<3); //PWM
		GPIOB->PDDR |= (1<<2);
		break;
	case 3: // CW
		TPM2->CONTROLS[0].CnV = 5000;
		TPM2->CONTROLS[1].CnV = 7999;
		GPIOB->PDDR |= (1 << 0);
		GPIOB->PDDR &= ~(1 << 1);

		GPIOC->PDDR |= (1 << 1);
		GPIOC->PDDR &= ~(1 << 2);
		GPIOB->PDDR |= (1<<3); //Left
		GPIOB->PDDR |= (1<<2);
		break;
	case 4: // motor off
		GPIOB->PDDR &= ~(1<<0); //Left
		GPIOB->PDDR &= ~(1<<1);
		GPIOC->PDDR &= ~(1<<1);
		GPIOC->PDDR &= ~(1<<2);
		GPIOB->PDDR &= (1<<3); //PWM
		GPIOB->PDDR |= (1<<2);
		break;
	case 5: // short break
		GPIOB->PDDR |= (1<<0); //Left
		GPIOB->PDDR |= (1<<1);
		GPIOC->PDDR &= ~(1<<1);
		GPIOC->PDDR &= ~(1<<2);
		GPIOB->PDDR |= (1<<3); //PWM
		GPIOB->PDDR |= (1<<2);
	default:
		__asm volatile ("nop");
	}
}


void Speed(int shift){
	int powerLeft = 0;
	int powerRight = 0;

	if(shift ==1){//right wheel faster to shift left
		while (powerLeft < 7999) {
			TPM2->CONTROLS[0].CnV = powerLeft;
			powerLeft++;
		}
		while (powerRight < 5000) {
			TPM2->CONTROLS[1].CnV = powerRight;
			powerRight++;
		}
		TPM2->CONTROLS[0].CnV = powerLeft;
		TPM2->CONTROLS[1].CnV = powerRight;
		GPIOB->PDDR |= (1 << 0);
						GPIOB->PDDR &= ~(1 << 1);

						GPIOC->PDDR |= (1 << 1);
						GPIOC->PDDR &= ~(1 << 2);
	}
	else if(shift ==2){//left wheel faster to shift right
		while (powerLeft < 5000) {
			TPM2->CONTROLS[0].CnV = powerLeft;
			powerLeft++;
		}
		while (powerRight < 7999) {
			TPM2->CONTROLS[1].CnV = powerRight;
			powerRight++;
		}
		TPM2->CONTROLS[0].CnV = powerLeft;
		TPM2->CONTROLS[1].CnV = powerRight;
		GPIOB->PDDR |= (1 << 0);
								GPIOB->PDDR &= ~(1 << 1);

								GPIOC->PDDR |= (1 << 1);
								GPIOC->PDDR &= ~(1 << 2);

	}
	else {//right wheel faster to shift left
		GPIOB->PDDR &= ~(1<<0); //Left
		GPIOB->PDDR |= (1<<1);
		GPIOC->PDDR &= ~(1<<1);
		GPIOC->PDDR |= (1<<2);
		TPM2->CONTROLS[0].CnV |= 7999;
		TPM2->CONTROLS[1].CnV |= 7999;
		GPIOB->PDDR |= (1<<3);
		GPIOB->PDDR |= (1<<2);
	}
}


void ADC() {
	unsigned short cal_v = 0;

	// Clock Gating
	SIM->SCGC5 |= (1<<13); // Enable Light Sensor I/O Port
	SIM->SCGC5 |= (1<<12);
	SIM->SCGC6 |= (1<<27); // Enable ADC Module

	// Setup Analog Input - Default is analog (PTE22), No need to do anything.

	// Setup LED Pin for GPIO
	PORTD->PCR[5] &= ~0x700; // Clear First
	PORTD->PCR[5] |= 0x700 & (1 << 8); // Set MUX bits

	// Setup Pin 5 as output
	GPIOD->PDDR |= (1 << 5);

	// Setup ADC Clock ( < 4 MHz)
	ADC0->CFG1 = 0;  // Default everything.

	// Analog Calibrate
	ADC0->SC3 = 0x07; // Enable Maximum Hardware Averaging
	ADC0->SC3 |= 0x80; // Start Calibration

	// Wait for Calibration to Complete (either COCO or CALF)
	while(!(ADC0->SC1[0] & 0x80)){	}


	// Calibration Complete, write calibration registers.
	cal_v = ADC0->CLP0 + ADC0->CLP1 + ADC0->CLP2 + ADC0->CLP3 + ADC0->CLP4 + ADC0->CLPS;
	cal_v = cal_v >> 1 | 0x8000;
	ADC0->PG = cal_v;

	cal_v = 0;
	cal_v = ADC0->CLM0 + ADC0->CLM1 + ADC0->CLM2 + ADC0->CLM3 + ADC0->CLM4 + ADC0->CLMS;
	cal_v = cal_v >> 1 | 0x8000;
	ADC0->MG = cal_v;

	ADC0->SC3 = 0; // Turn off Hardware Averaging
}
int ADCLeft() {
	unsigned int light_val = 0;
	ADC0->SC1[0] = 0x01; // Set Channel, starts conversion.
	while(!(ADC0->SC1[0] & 0x80)){	}
	light_val = ADC0->R[0]; // Resets COCO
	return light_val;
}

int ADCRight() {
	unsigned int light_val = 0;
	ADC0->SC1[0] = 0x05; // Set Channel, starts conversion.
	while(!(ADC0->SC1[0] & 0x80)){	}
	light_val = ADC0->R[0]; // Resets COCO
	return light_val;
}

void motor_5() {
	delay_n(3);
	Motion_Motor(1); //FW
	delay(600);
	Motion_Motor(4);
	delay(210);

	Motion_Motor(2);//Left
	delay(410);
	Motion_Motor(4);
	delay(210);

	Motion_Motor(1); //FW
	delay(600);
	Motion_Motor(4);
	delay(210);

	Motion_Motor(2);//Left
	delay(410);
	Motion_Motor(4);
	delay(210);

	Motion_Motor(1); //FW
	delay(600);
	Motion_Motor(4);
	delay(220);

	Motion_Motor(3);//right
	delay(410);
	Motion_Motor(4);
	delay(220);

	Motion_Motor(1); //FW
	delay(600);
	Motion_Motor(4);
	delay(220);

	Motion_Motor(3);//right
	delay(410);
	Motion_Motor(4);
	delay(220);

	Motion_Motor(1); //FW
	delay(600);
	Motion_Motor(4);
	delay(220);



}


void motor_S(){

	delay_n(2);
	Motion_Motor(4);
	Speed(1);
	delay_n(1);
	Motion_Motor(4);

	Speed(2);
	delay_n(1);
	Motion_Motor(4);

}



void traceLine() {
	if (ADCRight() > 230) { // 240 black 210 white
		while(1) {
			if (ADCRight() > 230) {
				Motion_Motor(1);
			}
			if (ADCRight() < 230) {
				Motion_Motor(3);
			}
		}

	}
	else if (ADCLeft() > 225) { // 210 white 233 black
		while(1) {
			if (ADCLeft() > 225) {
				Motion_Motor(1);
			}
			if (ADCLeft() < 225) {
				Motion_Motor(2);
			}
		}
	}
}

int main(void) {
	/* Init board hardware. */
	BOARD_InitBootPins();
	BOARD_InitBootClocks();
	BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
	/* Init FSL debug console. */
	BOARD_InitDebugConsole();
#endif
	ports();
	ADC();
	PORTC->PCR[3] &= ~0X703; // Clear first
	PORTC->PCR[3] |= 0X703 & ((1 << 8) | 0x3); //set MUX bits, enable pull ups
	GPIOC->PDIR & (1 << 3); // Setup Pin 3 Port C as input

	PORTC->PCR[12] &= ~0X700; // Clear first
	PORTC->PCR[12] |= ((0X700 & (1 << 8) | 0x2)); //set MUX bits, enable pull ups
	GPIOC->PDDR &= ~(1 << 12);

	while(1) {
		if (!(GPIOC->PDIR & (1 << 12))) {
			//delay_n(1);
			while (1){
			traceLine();
		}

	}
	}
	return 0;
}

