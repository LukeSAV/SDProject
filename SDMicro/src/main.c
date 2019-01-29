/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include "stm32f0xx.h"
#include "stm32f0_discovery.h"
#include <stdio.h>
#include <string.h>

#define IR_RECEIVE_MAX 300
#define IR_NO_RECEIVE_MIN 1500
#define RX_BUFFER_MAX 1500
#define TX_BUFFER_MAX 500

enum encoders{RECEIVE, NO_RECEIVE};

enum encoders prev_adc0_state = NO_RECEIVE; // Previous ADC channel 0 state
enum encoders prev_adc1_state = NO_RECEIVE; // Previous ADC channel 1 state
enum encoders adc0_state = NO_RECEIVE; // Current ADC channel 0 state
enum encoders adc1_state = NO_RECEIVE; // Current ADC channel 1 state
static uint32_t adc0_val = 0; // Current ADC channel 0 reading
static uint32_t adc1_val = 0; // Current ADC channel 1 reading
static uint32_t adc0_slots = 0; // Number of times a reading on ADC channel 0 has crossed the threshold
static uint32_t adc1_slots = 0; // Number of times a reading on ADC channel 1 has crossed the threshold

static char last_message[RX_BUFFER_MAX]; // Contains last full string enclosed in curly braces

static char rx_buffer[RX_BUFFER_MAX]; // Buffer of received UART data
static uint16_t rx_buffer_index = 0;

static char tx_buffer[TX_BUFFER_MAX]; // Buffer of UART data to transmit
static uint16_t tx_buffer_index = 0;

static void ADCInit();
static void USART1Init();
int main(void) {
	ADCInit();
	USART1Init();

	// Test UART transfer
	char* t = "Connected";
	memcpy(tx_buffer, t, 9 * sizeof(char));
	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);   // Enable USART1 Transmit interrupt

	for(;;) {

	}
}


/*************************************************************************
*	IR Encoder Interrupt
*
*	Increments counter indicating that the IR receiver transitioned between receiving a reflection and not and vice versa. Multiply adcx_slots by resolution to get distance traveled.
*
**************************************************************************/
void TIM2_IRQHandler() {
	TIM2->SR &= 0x00;	// Clear the IRQ flag

	ADC1->CHSELR = 0;                 		// Deselect ADC Channels
	ADC1->CHSELR |= 1 << 8;           		// Select Channel 8 (PB0)
	while(!(ADC1->ISR & ADC_ISR_ADRDY)); 	// Wait for ADC to be ready
	ADC1->CR |= ADC_CR_ADSTART;       		// Start the ADC
	while(!(ADC1->ISR & ADC_ISR_EOC));		// Wait for end of conversion
	adc0_val = ADC1->DR;					// Store value in file var

    ADC1->CHSELR = 0;                 		// Deselect ADC Channels
    ADC1->CHSELR |= 1 << 9;           		// Select Channel 9 (PB1)
	while(!(ADC1->ISR & ADC_ISR_ADRDY));  	// Wait for ADC to be ready
	ADC1->CR |= ADC_CR_ADSTART;       		// Start the ADC
	while(!(ADC1->ISR & ADC_ISR_EOC));		// Wait for end of conversion
	adc1_val = ADC1->DR;					// Store value in file var


	// Update slot counters
	if(adc0_val > IR_NO_RECEIVE_MIN) {
		adc0_state = NO_RECEIVE;
	} else if(adc0_val < IR_RECEIVE_MAX) {
		adc0_state = RECEIVE;
	} else {
		adc0_state = prev_adc0_state;
	}
	if(adc1_val > IR_NO_RECEIVE_MIN) {
		adc1_state = NO_RECEIVE;
	} else if(adc1_val < IR_RECEIVE_MAX) {
		adc1_state = RECEIVE;
	} else {
		adc1_state = prev_adc1_state;
	}

	if(adc0_state != prev_adc0_state) {
		adc0_slots++;
		prev_adc0_state = adc0_state;
	}
	if(adc1_state != prev_adc1_state) {
		adc1_slots++;
		prev_adc1_state = adc1_state;
	}
}


/*************************************************************************
*	UART TX/RX Interrupt
*
*	Received UART data goes into the rx_buffer
*	To transmit UART data, set tx_buffer_index to 0, copy the data to send into the tx_buffer, and enable TXE interrupts.
*
**************************************************************************/
void USART1_IRQHandler() {
	if(USART1->ISR & USART_ISR_RXNE) { // Check if RXNE flag is set
		if(rx_buffer_index < RX_BUFFER_MAX) {
			char in_char = (char) USART_ReceiveData(USART1);
			if(rx_buffer_index == 0 && in_char != '{') {
				return; // Ignoring data that is not contained within braces
			}
			rx_buffer[rx_buffer_index++] = in_char;
			if(in_char == '}') {
				memcpy(last_message, rx_buffer, rx_buffer_index * sizeof(char)); // Copy completed message to memory
				memset(rx_buffer, '\0', RX_BUFFER_MAX); // Clear buffer
				rx_buffer_index = 0; // Reset buffer index
			}
		} else {
			// Error. Overran buffer without terminating JSON message.
			memset(rx_buffer, '\0', RX_BUFFER_MAX); // Clear buffer
			rx_buffer_index = 0; // Reset buffer index
		}
	}
	if(USART1->ISR & USART_ISR_TXE) { // Check if TXE flag is set
		if(tx_buffer[tx_buffer_index] != '\0') {
			USART1->TDR = tx_buffer[tx_buffer_index++]; // Load next character into data register
		} else {
			USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
			memset(tx_buffer, '\0', TX_BUFFER_MAX);
		}
	}
}


/*************************************************************************
*	Ultrasonic Pulse Interrupt
*
*	Hold the trigger pin high for 10us every 100ms and enable timer to count response time before echo pin goes high.
*
**************************************************************************/
void TIM3_IRQHandler() {

}

/*************************************************************************
*	ADC Initialization Routine
*
*	Initialize the ADC and its timer interrupt
*
**************************************************************************/
static void ADCInit() {
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;    	// Enable clock to Port B
	GPIOB->MODER &= 0x0F;		          	// Set Pin 0 and Pin 1 for analog input
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;   	// Enable clock to ADC unit
	RCC->CR2 |= RCC_CR2_HSI14ON;         	// Turn on Hi-spd internal 14MHz clock
	while(!(RCC->CR2 & RCC_CR2_HSI14RDY));  // Wait for 14MHz clock to be ready
	ADC1->CR |= ADC_CR_ADEN;              	// Enable ADC
	while(!(ADC1->ISR & ADC_ISR_ADRDY));  	// Wait for ADC to be ready
	while((ADC1->CR & ADC_CR_ADSTART));   	// Wait for ADCstart to be 0.

	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // Enable clock for timer 2
	TIM2->ARR = 48000; 					// Enable interrupt at 2ms
	TIM2->DIER |= 0x01; 				// Update interrupt enable
	TIM2->CR1 |= 0x01; 					// Enable counter
	NVIC_SetPriority(TIM2_IRQn, 1); 	// Set ADC interrupt priority
	NVIC_EnableIRQ(TIM2_IRQn); 			// Enable IRQ for TIM2 in NVIC
}


/*************************************************************************
*	USART Initialization Routine
*
*	Initialize the UART and its interrupt
*
**************************************************************************/
static void USART1Init() {
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	RCC->AHBENR |= RCC_AHBENR_GPIOAEN; 		// Enable clock to Port A
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN; 	// Enable USART1 clock

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1);		// Set up alternate function on Pin 9
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);		// Set up alternate function on Pin 10
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9  | GPIO_Pin_10; 	// Enable Pins 9 and 10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; 				// Alternate function mode
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 				// No pull
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 			// 50 MHz GPIO speed -- Reduce if noisy***
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 				// Push pull
	GPIO_Init(GPIOA, &GPIO_InitStructure);						// Initialize GPIOA with above settings

	USART_InitStructure.USART_BaudRate = 57600; 										// Set baud rate to 57600 b/s
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; 						// 8b word length
	USART_InitStructure.USART_StopBits = USART_StopBits_1;								// 1 stop bit
	USART_InitStructure.USART_Parity = USART_Parity_No;									// No parity
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;						// Enable TX and RX on pins 9 and 10 respectively of port A
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;		// No flow control

	USART_Init(USART1, &USART_InitStructure);	// Initialize USART1 with above settings
	USART_Cmd(USART1, ENABLE);					// Enable USART1

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);  // Enable USART1 Receive interrupt
	NVIC_SetPriority(USART1_IRQn, 2); 				// Set USART1 interrupt priority
	NVIC_EnableIRQ(USART1_IRQn);					// Enable USART1 NVIC interrupt
}


/*************************************************************************
*	Ultrasonic GPIO Initialization Routine
*
*	Initialize the GPIOs for the ultrasonics and their interrupt.
*	Ultrasonics will be pulsed in 100ms intervals on a timer. A timer will then be enabled to count until the echo pin goes high on all ultrasonics or 23 ms has passed (4m range exceeded).
*
**************************************************************************/
static void UltrasonicInit() {


	//NVIC_SetPriority(TIM3_IRQn, 0); // Set highest priority on ultrasonics

}





