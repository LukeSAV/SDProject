/**
  ******************************************************************************
  * @file    main.c
  * @author  Luke Armbruster
  * @version V1.0
  * @date    11-January-2019
  * @brief   Main file for D2U microcontroller source.
  ******************************************************************************
*/


#include "stm32f0xx.h"
#include <stdio.h>
#include <string.h>
#include "arm_math.h"
#include "stdbool.h"

#define IR_RECEIVE_MAX 300
#define IR_NO_RECEIVE_MIN 900
#define RX_BUFFER_MAX 100
#define TX_BUFFER_MAX 100
#define MC_ADDRESS 130

#define AMBIENT_SPEED 15

#define ARRAY_SIZE ((uint8_t)8)
#define RESOLUTION ((uint8_t)8)
//#define LOOK_AHEAD 1.0
#define LOOK_AHEAD_SQ 1.0
#define NORMAL_SPEED 25
#define TIC_LENGTH 0.053086
#define VELOCITY_COEFF 0.5 //TODO FIND AT LEAST AN ESTIMATE FOR THIS, CONVERTING VELOCITY TO MOTOR CONTROLLER NUMBERS
#define ACCEL 2
#define VEHICLE_WIDTH 0.575
#define MAX_SPEED 80
#define MAX_TURN 20

enum Encoders{RECEIVE, NO_RECEIVE};
enum Direction{FORWARD, REVERSE};
enum LastMotorSent{LEFT, RIGHT};
enum DisplayMode{START=0, ULTRASONICS=1, ENCODERS=2, DELIVERY=3, END=4};
typedef struct {
	char address;
	char command;
	char data;
	char checksum;
} Packet;

typedef struct _Point {
	float x;
	float y;
} Point;

/* Display mode info */
static uint8_t current_mode = ULTRASONICS;

/* Encoder state vars */
enum Encoders prev_adc0_state = NO_RECEIVE; // Previous ADC channel 0 state
enum Encoders prev_adc1_state = NO_RECEIVE; // Previous ADC channel 1 state
enum Encoders adc0_state = NO_RECEIVE; // Current ADC channel 0 state
enum Encoders adc1_state = NO_RECEIVE; // Current ADC channel 1 state

enum Direction left_direction = FORWARD;
enum Direction right_direction = FORWARD;

static uint32_t adc0_val = 0; // Current ADC channel 0 reading
static uint32_t adc1_val = 0; // Current ADC channel 1 reading

static uint32_t adc0_slots = 0; // Number of times a reading on ADC channel 0 has crossed the threshold - right
static uint32_t adc1_slots = 0; // Number of times a reading on ADC channel 1 has crossed the threshold - left

static uint32_t last_used_adc0_slots = 0; // Right
static uint32_t last_used_adc1_slots = 0; // Left

static uint32_t last_encoder_left_count_to_jetson = 0;
static uint32_t last_encoder_right_count_to_jetson = 0;

static uint8_t left_speed = 5; // Current left speed to be requested
static uint8_t right_speed = 0; // Current right speed to be requested

/* UART vars */
static char last_message[RX_BUFFER_MAX]; // Contains last full string enclosed in curly braces

static char rx_buffer[RX_BUFFER_MAX]; // Buffer of received UART data
static uint16_t rx_buffer_index = 0;

static char jetson_tx_buffer[TX_BUFFER_MAX]; // Buffer of UART data to transmit
static uint16_t jetson_tx_buffer_index = 0;

static char mc_tx_buffer[9]; // Buffer of UART data to transmit, place 255 in position 4 if only sending one packet
static uint8_t mc_tx_buffer_index = 0;

/* Ultrasonic vars */
static uint16_t elapsed_response_time = 0; // Response time since trigger was set high

static uint16_t us1_distance = 0; // Distance in centimeters of ultrasonic 1 to object
static uint8_t us1_started = 0;
static uint16_t us1_elapsed_echo_time = 0; // Elapsed ultrasonic echo time in us

static uint16_t us2_distance = 0; // Distance in centimeters of utlrasonic 2 to object
static uint8_t us2_started = 0;
static uint16_t us2_elapsed_echo_time = 0; // Elapsed ultrasonic echo time in us

static uint16_t us3_distance = 0; // Distance in centimeters of ultrasonic 3 to object
static uint8_t us3_started = 0;
static uint16_t us3_elapsed_echo_time = 0; // Elapsed ultrasonic echo time in us

static uint8_t disp_count = 0;

static uint16_t time_since_last_jetson_msg = 0; // Time in milliseconds since last message was received from jetson

static uint8_t delivery_requested = 0; // Indication that delivery was requested has been received from Jetson

/* Control Algorithm Variables */
static uint32_t encoder_diff_l;
static uint32_t encoder_diff_r;
//float points_x[] = {0.004, -0.004,  0.008, -0.008, -0.004, -0.080, -0.200, -0.500};
float points_x[] = {0,0,0,0,0,0,0,0};
float points_y[] = {1.000,  2.000,  3.000,  4.000,  5.000,  6.000,  7.000,  7.000};
float goal_x = 0.0;
float goal_y = 0.0;

/* Motor state vars */
Packet left_packet;
Packet right_packet;

/* Vars for debugging */
uint16_t num_chars_received = 0;
uint32_t loop_count = 0;

static void decodeJetsonString();

static void ADCInit();
static void USARTInit();
static void UltrasonicInit();
static void SPIInit();
static int Drive(enum Direction left_direction, char left_speed, enum Direction right_direction, char right_speed); // Return 1 if unable to send, 0 if commands sent
static void Cmd(char b);
static void Data(char b);
static void DispString(char* data);
static void SendEncoderCount();
static void nsWait(int t);
static void StraightLine();

bool find_goal();
float distance_squared(float x1, float y1, float x2, float y2);
void SetMotors (uint32_t diff_l, uint32_t diff_r, float32_t diff_t);
void PointUpdate (uint32_t diff_l, uint32_t diff_r);

int main(void) {
	ADCInit();
	USARTInit();
	SPIInit();
	UltrasonicInit();

	//RCC->AHBENR |= RCC_AHBENR_GPIOBEN; // Ensure clock to port B is enabled
	//GPIOB->MODER = 0x00000100;
	//GPIOB->ODR = 0xFFFF;
	//GPIOB->ODR = 0x0000;

	mc_tx_buffer[8] = 255; // Set end character in buffer

	/* Only one motor controller, so use this address */
	left_packet.address = MC_ADDRESS;
	right_packet.address = MC_ADDRESS;

	/* Send timeout packet */
	mc_tx_buffer[0] = MC_ADDRESS;
	mc_tx_buffer[1] = 14; // Serial timeout command
	mc_tx_buffer[2] = 10; // 1000ms timeout
	mc_tx_buffer[3] = (MC_ADDRESS + 10 + 14) & 127; // Checksum
	mc_tx_buffer[4] = 255; // Stop byte
	USART_ITConfig(USART2, USART_IT_TXE, ENABLE); // Send initial command to enable timeout
	nsWait(1000000000);

	// Test UART transfer
	//char* t = "Connected";
	//memcpy(jetson_tx_buffer, t, 9 * sizeof(char));
	//USART_ITConfig(USART1, USART_IT_TXE, ENABLE);   // Enable USART1 Transmit interrupt
	//DispString(t);

	bool drive_enable = true;

	//TODO Maybe replace these diff variables with global variables, and modify functions to accept them instead

	float32_t diff_t = 0.01;

	for(;;) {
/*		drive_enable = find_goal();
		if(drive_enable) {
			encoder_diff_l = adc0_slots - last_used_adc0_slots;
			last_used_adc0_slots = adc0_slots;

			encoder_diff_r = adc1_slots - last_used_adc1_slots;
			last_used_adc1_slots = adc1_slots;

			//TODO
			//diff_t = total_time - last_used_time;
			//last_used_time = total_time;

			SetMotors(encoder_diff_l, encoder_diff_r, diff_t);
			PointUpdate (encoder_diff_l, encoder_diff_r);
		}
		else {
			Drive(left_direction, 0, right_direction, 0);
		}
		*/
//		// This could include non-controller related messages so if using the controller be wary that if the node dies and the Jetson is still sending data over problems can occur
//		/*if(time_since_last_jetson_msg > 1000) { // Timeout if no data is received from Jetson to stop motors
//			left_speed = right_speed = 0;
//		}*/
//		//StraightLine();
//		Drive(left_direction, left_speed, right_direction, right_speed);
//		Drive(left_direction, 20, right_direction,(int) (20 * 0.85));
		nsWait(50000000); // wait 50 ms
//		loop_count++;
	}
}

/*
 * Drives the robot in a straight line based solely off encoder counts
 */
static void StraightLine() {
	int8_t slot_diff = adc1_slots - adc0_slots;
	if(slot_diff > 30 || slot_diff < -30) { // Veered too far off the line
		left_speed = right_speed = 0;
		return;
	}
	if(slot_diff == 0) {
		if(left_speed < AMBIENT_SPEED && right_speed < AMBIENT_SPEED) {
			left_speed++;
			right_speed++;
		}
	} else if(slot_diff < 0) { // Right wheel has not moved as much as left
		if(right_speed < AMBIENT_SPEED) {
			right_speed++;
		}
		else {
			if(left_speed >= 1) {
				left_speed--;
			}
		}

	} else { // Left wheel has not moved as much as right
		if(left_speed < AMBIENT_SPEED) {
			left_speed++;
		}
		else {
			if(right_speed >= 1) {
				right_speed--;
			}
		}
	}
}
/*
 * Send number of encoder ticks since last message was sent to Jetson
 */
static void SendEncoderCount() {
	uint32_t left_count_to_send = adc1_slots - last_encoder_left_count_to_jetson;
	uint32_t right_count_to_send = adc0_slots - last_encoder_right_count_to_jetson;
	/* If the encoder count deltas to the jetson are too high something is wrong - loop infinitely for now */
	last_encoder_left_count_to_jetson = adc1_slots;
	last_encoder_right_count_to_jetson = adc0_slots;
	if(left_count_to_send > 99 || right_count_to_send > 99) {
		return; // REALLY fast if this is true
	} else {
		char second_left_char = left_count_to_send % 10 + 48;
		left_count_to_send /= 10;
		char first_left_char = left_count_to_send % 10 + 48;

		char second_right_char = right_count_to_send % 10 + 48;
		right_count_to_send /= 10;
		char first_right_char = right_count_to_send % 10 + 48;

		jetson_tx_buffer[0] = '{';
		jetson_tx_buffer[1] = 'E';
		jetson_tx_buffer[2] = first_left_char;
		jetson_tx_buffer[3] = second_left_char;
		jetson_tx_buffer[4] = ',';
		jetson_tx_buffer[5] = first_right_char;
		jetson_tx_buffer[6] = second_right_char;
		jetson_tx_buffer[7] = '}';
		jetson_tx_buffer[8] = '\0';
		USART_ITConfig(USART1, USART_IT_TXE, ENABLE);   // Enable USART1 Transmit interrupt
	}
}

/*
 * Switch to next display mode
 */
static void nextMode() {
	current_mode++;
	if(current_mode >= END) {
		current_mode = START + 1;
	}
}

/*
 * Decode the remote controller message from the Jetson and set desired wheel speed.
 */
static void decodeControllerMsg() {
	time_since_last_jetson_msg = 0;
	left_speed = (last_message[5] - 48) + (last_message[4] - 48) * 10 + (last_message[3] - 48) * 100;
	right_speed = (last_message[10] - 48) + (last_message[9] - 48) * 10 + (last_message[8] - 48) * 100;
	if(last_message[2] == '+') {
		left_direction = FORWARD;
	} else if(last_message[2] == '-') {
		left_direction = REVERSE;
	}
	if(last_message[7] == '+') {
		right_direction = FORWARD;
	} else if(last_message[7] == '-') {
		right_direction = REVERSE;
	}
}

/*
 * Determine which type of message was received from the Jetson and parse the data accordingly.
 */
static void decodeJetsonString() {
	switch (last_message[1]) {
	case 'C': // Remote controller feedback
		decodeControllerMsg();
		break;
	case 'M': // Mode swtich
		nextMode();
		break;
	case 'N': // Cancel delivery request
		delivery_requested = 0;
		break;
	case 'D': // Delivery request received
		delivery_requested = 1;
		break;
	default:
		break;
	}
}


/*************************************************************************
*	Function to send commands to the motor controllers
*
*	Returns 1 if still sending the previous command, 0 otherwise.
*
**************************************************************************/
static int Drive(enum Direction left_direction, char left_speed, enum Direction right_direction, char right_speed) {
	if(mc_tx_buffer_index != 0) {
		return 1; // Haven't finished sending the last message
	}
	if(left_direction == FORWARD) {
		left_packet.command = 4;
	}
	else if(left_direction == REVERSE) {
		left_packet.command = 5;
	}
	if(right_direction == FORWARD) {
		right_packet.command = 0;
	}
	else if(right_direction == REVERSE) {
		right_packet.command = 1;
	}
	left_packet.data = left_speed;
	right_packet.data = right_speed;

	left_packet.checksum = (left_packet.address + left_packet.command + left_packet.data) & 127;
	right_packet.checksum = (right_packet.address + right_packet.command + right_packet.data) & 127;

	mc_tx_buffer[0] = left_packet.address;
	mc_tx_buffer[1] = left_packet.command;
	mc_tx_buffer[2] = left_packet.data;
	mc_tx_buffer[3] = left_packet.checksum;

	mc_tx_buffer[4] = right_packet.address;
	mc_tx_buffer[5] = right_packet.command;
	mc_tx_buffer[6] = right_packet.data;
	mc_tx_buffer[7] = right_packet.checksum;
	USART_ITConfig(USART2, USART_IT_TXE, ENABLE);

	return 0;
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
*	To transmit UART data, set jetson_tx_buffer_index to 0, copy the data to send into the jetson_tx_buffer, and enable TXE interrupts.
*
**************************************************************************/
void USART1_IRQHandler() {
	if(USART1->ISR & USART_ISR_ORE) { // Overran buffer
		USART1->ICR |= USART_ICR_ORECF;
		memset(rx_buffer, '\0', RX_BUFFER_MAX);
		rx_buffer_index = 0; // Reset buffer index
	}
	if((USART1->ISR & USART_ISR_RXNE) == USART_ISR_RXNE) { // Check if RXNE flag is set
		if(rx_buffer_index < RX_BUFFER_MAX) {
			char in_char = (uint8_t) USART1->RDR;
			num_chars_received++;
			if(rx_buffer_index == 0 && in_char != '{') {
				return; // Ignoring data that is not contained within braces
			}
			rx_buffer[rx_buffer_index++] = in_char;
			if(in_char == '}') {
				memcpy(last_message, rx_buffer, rx_buffer_index * sizeof(char)); // Copy completed message to memory
				memset(rx_buffer, '\0', RX_BUFFER_MAX); // Clear buffer
				decodeJetsonString();
				rx_buffer_index = 0; // Reset buffer index
			}
		} else {
			// Error. Overran buffer without terminating JSON message.
			memset(rx_buffer, '\0', RX_BUFFER_MAX); // Clear buffer
			rx_buffer_index = 0; // Reset buffer index
		}
	}
	if((USART1->ISR & USART_ISR_TC) == USART_ISR_TC) { // Check if TXE flag is set
		if(jetson_tx_buffer[jetson_tx_buffer_index] != '\0') {
			USART1->TDR = jetson_tx_buffer[jetson_tx_buffer_index++]; // Load next character into data register
		} else {
			USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
			memset(jetson_tx_buffer, '\0', TX_BUFFER_MAX);
			jetson_tx_buffer_index = 0;
		}
	}
}


/*************************************************************************
*	UART TX Interrupt
*
*	To transmit UART data, set mc_tx_buffer_index to 0, copy the data to send into the mc_tx_buffer, and enable TXE interrupts.
*
**************************************************************************/
void USART2_IRQHandler() {
	if(USART2->ISR & USART_ISR_TXE) { // Check if TXE flag is set
		if(mc_tx_buffer[mc_tx_buffer_index] != 255) {
			USART2->TDR = mc_tx_buffer[mc_tx_buffer_index++]; // Load next character into data register
		} else {
			USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
			memset(mc_tx_buffer, 255, 9);
			mc_tx_buffer_index = 0;
		}

	}
}


/*************************************************************************
*	200 ms Interrupt
*
*	Hold the trigger pin high for 10us every 200ms and enable timer to count response time before echo pin goes high.
*
**************************************************************************/
void TIM14_IRQHandler() {
	SendEncoderCount();
	time_since_last_jetson_msg += 200; // Increment timer for last jetson message
	/* Run ultrasonic pulse */
	TIM14->SR &= 0x00; 					// Clear the IRQ flag
	NVIC_EnableIRQ(TIM15_IRQn); 		// Enable IRQ for TIM15 in NVIC
	// Set the trigger pins high
	GPIO_SetBits(GPIOA, GPIO_Pin_0);
	GPIO_SetBits(GPIOA, GPIO_Pin_4);
	GPIO_SetBits(GPIOA, GPIO_Pin_6);
	nsWait(10000); // Idle while trigger is high (~10 us)
	// Set the trigger pins low
	GPIO_ResetBits(GPIOA, GPIO_Pin_0);
	GPIO_ResetBits(GPIOA, GPIO_Pin_4);
	GPIO_ResetBits(GPIOA, GPIO_Pin_6);
	// Reset elapsed echo time for each US
	us1_elapsed_echo_time = 0;
	us2_elapsed_echo_time = 0;
	us3_elapsed_echo_time = 0;
	// Reset elapsed response time
	elapsed_response_time = 0;

	/* Update Display */
	if(disp_count >= 5) { // Update the display every second
		Cmd(0x01); // clear entire display
		nsWait(6200000); // clear takes 6.2ms to complete
		Cmd(0x02); // put the cursor in the home position
		Cmd(0x06); // Set display to increment
		Cmd(0x38); // Enable 2 line mode
		if(current_mode == ULTRASONICS) {
			char out_data[17] = {'L', '0', '0', '0', ' ', 'M', '0', '0', '0', ' ', 'R', '0', '0', '0'}; // Write ultrasonic data to screen
			int data_loc = 3;
			int distance = us2_distance;
			while(distance > 0) {
				int num_to_disp = distance % 10;
				out_data[data_loc] = num_to_disp + 48;
				data_loc--;
				distance /= 10;
			}
			data_loc = 8;
			distance = us3_distance;
			while(distance > 0) {
				int num_to_disp = distance % 10;
				out_data[data_loc] = num_to_disp + 48;
				data_loc--;
				distance /= 10;
			}
			data_loc = 13;
			distance = us1_distance;
			while(distance > 0) {
				int num_to_disp = distance % 10;
				out_data[data_loc] = num_to_disp + 48;
				data_loc--;
				distance /= 10;
			}
			DispString(out_data);
		} else if(current_mode == ENCODERS) {
			char out_data[17] = {'I', 'R', ' ', 'L', ':', '0', '0', '0', '0', ' ', 'R', ':', '0', '0', '0', '0'}; // Write infrared encoder data to screen
			int data_loc = 8;
			int count = adc1_slots;
			while(count > 0) {
				int num_to_disp = count % 10;
				out_data[data_loc--] = num_to_disp + 48;
				count /= 10;
			}
			data_loc = 15;
			count = adc0_slots;
			while(count > 0) {
				int num_to_disp = count % 10;
				out_data[data_loc--] = num_to_disp + 48;
				count /= 10;
			}
			DispString(out_data);
		} else if(current_mode == DELIVERY) {
			if(delivery_requested) {
				char out_data[16] = {'D', 'E', 'L', 'I', 'V', 'E', 'R', 'Y', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' '};
				DispString(out_data);
			} else {
				char out_data[16] = {'N', 'O', ' ', 'D', 'E', 'L', 'I', 'V', 'E', 'R', 'Y', ' ', ' ', ' ', ' ', ' '};
				DispString(out_data);
			}
		}
		disp_count = 0;
	}
	disp_count++;
}


/*************************************************************************
*	Ultrasonic Echo Response Interrupt
*
*	Measure response time before echo pin goes high for each ultrasonic.
*
**************************************************************************/
void TIM15_IRQHandler() {
	TIM15->SR &= 0x00; // Clear the IRQ flag
	uint8_t us1_return = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1);
	uint8_t us2_return = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5);
	uint8_t us3_return = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_7);
	if(elapsed_response_time > 25000) { // More than 4 meters away
		us1_distance = 500;
		us2_distance = 500;
		us3_distance = 500;
		NVIC_DisableIRQ(TIM15_IRQn);
	}

	if(!us1_started && us1_return) { // Pulse went high
		us1_started = 1;
	}
	else if(us1_started && us1_return) { // Pulse stayed high
		us1_elapsed_echo_time += 60; // Increment by 60 us
	}
	else if(us1_started && !us1_return) { // Pulse went low
		us1_started = 0;
		us1_distance = us1_elapsed_echo_time * 0.0343 / 2; // Using speed of sound in cm/us to get distance
	}

	if(!us2_started && us2_return) { // Pulse went high
		us2_started = 1;
	}
	else if(us2_started && us2_return) { // Pulse stayed high
		us2_elapsed_echo_time += 60; // Increment by 60 us
	}
	else if(us2_started && !us2_return) { // Pulse went low
		us2_started = 0;
		us2_distance = us2_elapsed_echo_time * 0.0343 / 2; // Using speed of sound in cm/us to get distance
	}

	if(!us3_started && us3_return) { // Pulse went high
		us3_started = 1;
	}
	else if(us3_started && us3_return) { // Pulse stayed high
		us3_elapsed_echo_time += 60; // Increment by 60 us
	}
	else if(us3_started && !us3_return) { // Pulse went low
		us3_started = 0;
		us3_distance = us3_elapsed_echo_time * 0.0343 / 2; // Using speed of sound in cm/us to get distance
	}
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
	RCC->CR2 |= RCC_CR2_HSI14ON;         	// Turn on high speed internal 14MHz clock
	while(!(RCC->CR2 & RCC_CR2_HSI14RDY));  // Wait for 14MHz clock to be ready
	ADC1->CR |= ADC_CR_ADEN;              	// Enable ADC
	while(!(ADC1->ISR & ADC_ISR_ADRDY));  	// Wait for ADC to be ready
	while((ADC1->CR & ADC_CR_ADSTART));   	// Wait for ADCstart to be 0.

	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // Enable clock for timer 2
	TIM2->ARR = 96000; 					// Enable interrupt at 2ms
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
static void USARTInit() {
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	RCC->AHBENR |= RCC_AHBENR_GPIOAEN; 		// Enable clock to Port A
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN; 	// Enable USART1 clock
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN; 	// Enable USART2 clock

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1);		// Set up alternate function on Pin 9
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);		// Set up alternate function on Pin 10
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_1);		// Set up alternate function on Pin 2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_9  | GPIO_Pin_10; 	// Enable Pins 2, 9, and 10 (USART2 TX, USART1 TX, USART1 RX)
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; 				// Alternate function mode
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 				// No pull
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 			// 50 MHz GPIO speed -- Reduce if noisy***
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 				// Push pull
	GPIO_Init(GPIOA, &GPIO_InitStructure);						// Initialize GPIOA with above settings


	USART_InitStructure.USART_BaudRate = 19200; 										// Set baud rate to 57600 b/s
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; 						// 8b word length
	USART_InitStructure.USART_StopBits = USART_StopBits_1;								// 1 stop bit
	USART_InitStructure.USART_Parity = USART_Parity_No;									// No parity
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //| USART_Mode_Tx;						// Enable TX and RX on pins 9 and 10 respectively of port A
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;		// No flow control
	USART_Init(USART1, &USART_InitStructure);		// Initialize USART1 with above settings

	USART_InitStructure.USART_BaudRate = 9600; 		// Set baud rate to 38400 b/s
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; 						// 8b word length
	USART_InitStructure.USART_StopBits = USART_StopBits_1;								// 1 stop bit
	USART_InitStructure.USART_Parity = USART_Parity_No;									// No parity
	USART_InitStructure.USART_Mode = USART_Mode_Tx; // Enable TX on pin 2
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;		// No flow control
	USART_Init(USART2, &USART_InitStructure);		// Initialize USART2 with above settings

	USART_Cmd(USART1, ENABLE);						// Enable USART1
	USART_Cmd(USART2, ENABLE);						// Enable USART2

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);  // Enable USART1 Receive interrupt

	NVIC_SetPriority(USART1_IRQn, 0); 				// Set USART1 interrupt priority
	NVIC_SetPriority(USART2_IRQn, 0); 				// Set USART2 interrupt priority (higher for MC)
	NVIC_EnableIRQ(USART1_IRQn);					// Enable USART1 NVIC interrupt
	NVIC_EnableIRQ(USART2_IRQn);					// Enable USART2 NVIC interrupt
}


/*************************************************************************
*	Ultrasonic GPIO Initialization Routine
*
*	Initialize the GPIOs for the ultrasonics and their interrupt.
*	Ultrasonics will be pulsed in 100ms intervals on a timer. A timer will then be enabled to count until the echo pin goes high on all ultrasonics or 23 ms has passed (4m range exceeded).
*
**************************************************************************/
static void UltrasonicInit() {
	RCC->APB1ENR |= RCC_APB1ENR_TIM14EN; 	// Enable clock to timer
	TIM14->PSC = 47999; 					// Prescale clock to 1kHz
	TIM14->ARR = 200; 						// Trigger every 200 clock cycles (200 ms)
	TIM14->DIER |= 0x01; 					// Update interrupt enable
	TIM14->CR1 |= 0x01; 					// Enable counter

	RCC->APB2ENR |= RCC_APB2ENR_TIM15EN; 	// Enable clock to timer
	TIM15->PSC = 47; 						// Prescale clock to 1MHz
	TIM15->ARR = 60; 						// Trigger every 60 clock cycles (60 us)
	TIM15->DIER |= 0x01; 					// Update interrupt enable
	TIM15->CR1 |= 0x01; 					// Enable counter

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_4 | GPIO_Pin_6;	// Trigger pins
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; 				// Output mode
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 				// Pull up
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 			// 50MHz GPIO speed
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 				// Push pull
	GPIO_Init(GPIOA, &GPIO_InitStructure);						// Initialize GPIOA with above settings

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_5 | GPIO_Pin_7;	// Echo pins
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; 				// Input mode
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;				// Pull down
	GPIO_Init(GPIOA, &GPIO_InitStructure);						// Initialize GPIOA with above settings

	NVIC_SetPriority(TIM15_IRQn, 0); 		// Set highest priority on ultrasonics
	NVIC_SetPriority(TIM14_IRQn, 0); 		// Set highest priority on ultrasonics
	NVIC_EnableIRQ(TIM14_IRQn); 			// Enable IRQ for TIM14 in NVIC

}


/*************************************************************************
*	Send string to display
*
**************************************************************************/
static void DispString(char* data) {
	size_t data_length = strlen(data);
	for(unsigned int i = 0; i < data_length; i++) {
		Data(data[i]);
	}
}


/*************************************************************************
*	Send command to display
*
**************************************************************************/
static void Cmd(char b) {
    while((SPI2->SR & SPI_SR_TXE) != SPI_SR_TXE);
    SPI2->DR = b;
}


/*************************************************************************
*	Send character to display
*
**************************************************************************/
static void Data(char b) {
    while((SPI2->SR & SPI_SR_TXE) != SPI_SR_TXE);
    SPI2->DR = 0x200 | b;
}


/*************************************************************************
*	SPI Initialization Routine
*
**************************************************************************/
static void SPIInit() {
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN; // Ensure clock to port B is enabled
	GPIOB->MODER &= 0x30ffffff;
	GPIOB->MODER |= 0x8A000000;
	GPIOB->AFR[1] &= 0x0f00ffff;
	//GPIOB->ODR = 0xFFFF;

	RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
	SPI2->CR1 |= (SPI_CR1_MSTR | SPI_CR1_BR_0 | SPI_CR1_BR_1 | SPI_CR1_BR_2 | SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE);
	SPI2->CR2 = (SPI_CR2_SSOE | SPI_CR2_NSSP | SPI_CR2_DS_3 | SPI_CR2_DS_0);
	SPI2->CR1 |= SPI_CR1_SPE;

	nsWait(100000000); // Give it 100ms to initialize
	Cmd(0x38); // 0011 NF00 N=1, F=0: two lines
	Cmd(0x0c); // 0000 1DCB: display on, no cursor, no blink
	Cmd(0x01); // clear entire display
	nsWait(6200000); // clear takes 6.2ms to complete
	Cmd(0x02); // put the cursor in the home position
	Cmd(0x06); // 0000 01IS: set display to increment
}


/*************************************************************************
*	Wait specified number of microseconds
*
**************************************************************************/
void nsWait(int t) {
    asm("       mov r0,%0\n"
        "repeat:\n"
        "       sub r0,#83\n"
        "       bgt repeat\n"
        : : "r"(t) : "r0", "cc");
}

float distance_squared(float x1, float y1, float x2, float y2) {
	return (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2);
}

bool find_goal() {
	//Calculate all distances relative to origin/robot
	float dist_array[ARRAY_SIZE];
	for(int idx = 0; idx < ARRAY_SIZE; idx++) {
		dist_array[idx] = distance_squared(0.0, 0.0, points_x[idx], points_y[idx]);
	}

	//Locate point closest to origin/robot
	float min_dis = dist_array[0];
	uint8_t min_idx = 0;
	for(int idx = 1; idx < ARRAY_SIZE; idx++) {		//Index starts at 1 as points[0] is default
		//TODO Add some sort of exception for whatever "Ending Condition" distance creates
		//IE, what points does the Jetson send when destination is reached
		//Maybe just copy last valid point throughout the array
		if(dist_array[idx] < min_dis) {
			min_dis = dist_array[idx];
			min_idx = idx;
		}
	}

	//Find points of the line that Goal Point lies on
	Point target_1;
	Point target_2;
	if( (min_dis < LOOK_AHEAD_SQ) || (min_idx == (ARRAY_SIZE - 1)) ) {
		bool break_success = false;
		for(int offset = 0; offset + min_idx < ARRAY_SIZE; offset++) {
			if(dist_array[min_idx + offset] >= LOOK_AHEAD_SQ ) {
				target_2.x = points_x[min_idx + offset];
				target_2.y = points_y[min_idx + offset];
				target_1.x = points_x[min_idx + offset - 1];
				target_1.y = points_y[min_idx + offset - 1];
				break_success = true;
				break;
			}
		}
		if(!break_success) {
			//All next destination points within Look Ahead Distance, Probably Trigger a Slowdown Flag and Stop
			//You'd also arrive here if STM moved within the last point the Jetson sent, or if mini-EKF greatly misbehaved
			goal_x = 0.0;
			goal_y = 0.0;
			return false;
		}
	}
	else {
		float decision_dist = distance_squared(points_x[min_idx], points_y[min_idx], points_x[min_idx + 1], points_y[min_idx + 1]);
		if(dist_array[min_idx + 1] > decision_dist) {
			target_2.x = points_x[min_idx];
			target_2.y = points_y[min_idx];
			if(min_idx != 0) {
				target_1.x = points_x[min_idx - 1];
				target_1.y = points_y[min_idx - 1];
			}
			else {
				target_1.x = 0;
				target_1.y = 0;
			}
		}
		else {
			target_1.x = points_x[min_idx];
			target_1.y = points_y[min_idx];
			target_2.x = points_x[min_idx + 1];
			target_2.y = points_y[min_idx + 1];
		}
	}

	//Solve Equations of Line 1 and Line 2 from Target 1 and Target 2
	float slope_1 = (target_2.y - target_1.y) / (target_2.x - target_1.x);
	float b_1 = target_2.y - slope_1 * target_2.x;

	//Locate X-Coordinate of Goal Point
	target_1.x = b_1 / (-2.0 * slope_1);
	target_1.y = slope_1 * target_1.x + b_1;
	Point temp_point;

	for(int idx = 0; idx < RESOLUTION; idx++) {
		temp_point.x = ((target_2.x - target_1.x) / 2 ) + target_1.x;
		temp_point.y = slope_1 * temp_point.x + b_1;
		if(distance_squared(0.0, 0.0, temp_point.x, temp_point.y) > LOOK_AHEAD_SQ ) {
			target_2 = temp_point;
		}
		else {
			target_1 = temp_point;
		}
	}

	goal_x = ((target_2.x - target_1.x) / 2 ) + target_1.x;
	goal_y = goal_x * slope_1 + b_1;
	return true;
}

void SetMotors (uint32_t diff_l, uint32_t diff_r, float32_t diff_t) {
	float v_c = (diff_l + diff_r) * TIC_LENGTH / 2.00 / diff_t * VELOCITY_COEFF;
	//TODO MODIFY VELOCITY_COEFF

	if(v_c < 20) v_c += 20;
	if(v_c > NORMAL_SPEED) v_c -= ACCEL;

	//Allow negative for now, don't want to have to worry about overflow later
	int8_t v_l = (int) (v_c * (1.0f + VEHICLE_WIDTH * goal_x / LOOK_AHEAD_SQ));
	int8_t v_r = (int) (v_c * (1.0f - VEHICLE_WIDTH * goal_x / LOOK_AHEAD_SQ));

	if(v_l > MAX_SPEED) {
		v_r -= v_l - MAX_SPEED;
		v_l = MAX_SPEED;
	}
	if(v_r > MAX_SPEED) {
		v_l -= v_r - MAX_SPEED;
		v_r = MAX_SPEED;
	}
	if( (v_l - v_r) > MAX_TURN) v_l -= v_l - v_r - MAX_TURN;
	if( (v_r - v_l) > MAX_TURN) v_r -= v_r - v_l - MAX_TURN;
	if(v_l < 0) v_l = 0;
	if(v_r < 0) v_r = 0;

	//Optional, not sure if global variable is really even needed
	left_speed = v_l; // Current left speed to be requested
	right_speed = (int) ((float)v_r * 0.85); // Current right speed to be requested


	Drive(left_direction, left_speed, right_direction, right_speed);

	return;
}

void PointUpdate (uint32_t diff_l, uint32_t diff_r) {
//	float v_l = diff_l * TIC_LENGTH;
//	float v_r = diff_r * TIC_LENGTH;
//	float theta = (v_r - v_l) / VEHICLE_WIDTH;
//	float delta_x = -0.5 * (v_l + v_r) * sin(theta);
//	float delta_y =  0.5 * (v_l + v_r) * cos(theta);

	float theta = (diff_r - diff_l) * TIC_LENGTH / VEHICLE_WIDTH;
	float delta_x = -0.5 * TIC_LENGTH * (diff_l + diff_r) * sin(theta);
	float delta_y =  0.5 * TIC_LENGTH * (diff_l + diff_r) * cos(theta);

	float x_rot;
	float y_rot;

	for(int i = 0; i < ARRAY_SIZE; i++) {
		x_rot = points_x[i] * cos(theta) + points_y[i] * -1 * sin(theta);
		y_rot = points_x[i] * sin(theta) + points_y[i] * cos(theta);

		points_x[i] = x_rot - delta_x;
		points_y[i] = y_rot - delta_y;
	}

	return;
}







