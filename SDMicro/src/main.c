/**
  ******************************************************************************
  * @file    main.c
  * @author  Luke Armbruster and Troy Conlin
  * @version V1.0
  * @date    11-January-2019
  * @brief   Main file for D2U microcontroller source.
  ******************************************************************************
*/

#include "stm32f0xx.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "arm_math.h"
#include "stdbool.h"

#define IR_RECEIVE_MAX 500
#define IR_NO_RECEIVE_MIN 2600
#define RX_BUFFER_MAX 100
#define TX_BUFFER_MAX 100
#define MC_ADDRESS 130

#define PI_VAR 3.14159f
#define NORMAL_SPEED_3 20
#define TURN_COEFF 10
#define ARRAY_SIZE 8
#define RESOLUTION 8
#define LOOK_AHEAD    1.732f
#define LOOK_AHEAD_SQ 3.0f
#define NORMAL_SPEED 20
#define TIC_LENGTH 0.053086
#define VELOCITY_EQ_M 11.013
#define VELOCITY_EQ_B 20.0
//#define VELOCITY_EQ_B 9.5862
#define L_R_BIAS 1.2   	//Multiply to Right Wheel
#define ACCEL 2
#define VEHICLE_WIDTH 0.575
#define MAX_SPEED 30
#define MAX_MOTION_FAILURE_COUNT 30 //Each iteration is about a tenth of a second. So failure to move within 3 seconds

#define MAX_TURN 20//30
#define AVERAGE_SPEED 0.3f //Average human walking speed is about 1.4 m/s // .15
#define ACCEL_2 1
#define DECEL_2 1
#define MAX_TORQUE 40

#define KPL 10.0f;
#define KPR 15.0f;

#ifndef _USE_PID_CONTROLLER
//#define _USE_PID_CONTROLLER
#endif

enum Encoders{RECEIVE, NO_RECEIVE};
enum Direction{FORWARD, REVERSE};
enum LastMotorSent{LEFT, RIGHT};
enum DisplayMode{START=0, ULTRASONICS=1, ENCODERS=2, DELIVERY=3, JETSON=4, END=5};
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

static float TURN_MULT = 1.0f; // 3

/* Display mode info */
static uint8_t current_mode = ULTRASONICS;

/* Encoder state vars */
enum Encoders prev_adc0_state = NO_RECEIVE; // Previous ADC channel 0 state
enum Encoders prev_adc1_state = NO_RECEIVE; // Previous ADC channel 1 state
enum Encoders adc0_state = NO_RECEIVE; // Current ADC channel 0 state
enum Encoders adc1_state = NO_RECEIVE; // Current ADC channel 1 state

enum Direction left_direction = FORWARD;
enum Direction right_direction = REVERSE;

static uint32_t adc0_val = 0; // Current ADC channel 0 reading
static uint32_t adc1_val = 0; // Current ADC channel 1 reading

static uint32_t adc_left_slots = 0; // Number of times a reading on ADC channel 0 has crossed the threshold - right
static uint32_t adc_right_slots = 0; // Number of times a reading on ADC channel 1 has crossed the threshold - left

static uint32_t adc_left_same_counter = 0;
static uint32_t adc_right_same_counter = 0;

static uint32_t last_used_adc_left_slots = 0; // Right
static uint32_t last_used_adc_right_slots = 0; // Left

static uint32_t last_encoder_left_count_to_jetson = 0;
static uint32_t last_encoder_right_count_to_jetson = 0;

static int16_t left_speed = NORMAL_SPEED; // Current left speed to be requested
static int16_t right_speed = NORMAL_SPEED; // Current right speed to be requested

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

static uint8_t delivery_requested = 1; // Indication that delivery was requested has been received from Jetson

/* Control Algorithm Variables */
static uint32_t encoder_diff_l = 0;
static uint32_t encoder_diff_r = 0;
static uint8_t  accel_count = 1;
static uint8_t  decel_count = 1;
static uint8_t  accel_fail_count = 0;
static uint8_t  decel_fail_count = 0;

static bool jetson_connected = false;
bool new_points_ready = false;
static uint16_t time_since_screen_change = 0;

int left_speed_integral[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
int right_speed_integral[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//int integral_sum; // Positive on left wheel

float k_i = 0.0;
float k_p = 0.6;

/* New points from Jetson */
float new_points_x[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float new_points_y[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

/* Current working points */
float orig_points_x[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.4};
float orig_points_y[] = {1.0,  2.0,  3.0,  4.0,  5.0, 6.0,  7.0,  8.4};

float points_x[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float points_y[] = {3.0,  6.0,  9.0,  12.0,  15.0, 18.0, 21.0, 24.0};
//float points_x[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//float points_y[] = {0.0,  0.0,  0.0,  0.0,  0.0, 0.0,  0.0, 0.0};
//float points_x[] = {0.4, 0.56, 0.86, 0.94, 1.06, 1.23, 1.33, 1.4};
//float points_y[] = {0.6,  1.2,  1.8,  2.4, 3.0, 3.6,  4.2,  4.8};

float debug_goal_x[100];
int debug_goal_x_index = 0;
float goal_x = 0.0;
float goal_y = 0.0;
float prev_goal_x = 0.0;
float prev_goal_y = 0.0;

uint8_t v_t = 0;
uint32_t zero_count = 0; //DIAGNOSTIC ONLY TODO REMOVE

/* Motor state vars */
Packet left_packet;
Packet right_packet;

/* Vars for debugging */
uint16_t num_chars_received = 0;
uint32_t loop_count = 0;

int int_count = 0;
int total_int_count = 0;

uint8_t motor_cmd_count = 0;

static uint16_t failed_trajectory_counter = 0;
int8_t stall_count = 0;

//static uint8_t integral_counter = 0;


static void decodeJetsonString();

static void ADCInit();
static void USARTInit();
static void UltrasonicInit();
static void SPIInit();
static int Drive(enum Direction left_direction, int16_t left_speed, enum Direction right_direction, int16_t right_speed); // Return 1 if unable to send, 0 if commands sent
static void Cmd(char b);
static void Data(char b);
static void DispString(char* data);
static void SendEncoderCount();
static void nsWait(uint64_t t);

bool find_goal();
float distance_squared(float x1, float y1, float x2, float y2);
void SetMotors (uint32_t diff_l, uint32_t diff_r, float32_t diff_t);
void PointUpdate (uint32_t diff_l, uint32_t diff_r);
void SetMotors2 (uint32_t diff_l, uint32_t diff_r, float32_t diff_t);
void wheelControl (uint32_t diff_l, uint32_t diff_r, float32_t diff_t);
void PIMotors(uint32_t diff_l, uint32_t diff_r, float diff_t);

int main(void) {
	ADCInit();
	USARTInit();
	SPIInit();
	UltrasonicInit();

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
	nsWait(100000000);

	bool drive_enable = true;
	float32_t diff_t = 0.1f;
	//float32_t diff_t = 0.10;

	int set_encoder_diff_l = 0;
	int set_encoder_diff_r = 0;

	for(;;) {
		if(new_points_ready) {
			memcpy(points_x, new_points_x, sizeof(float) * 8);
			memcpy(points_y, new_points_y, sizeof(float) * 8);
			memset(new_points_x, 0.0f, sizeof(float) * 8);
			memset(new_points_y, 0.0f, sizeof(float) * 8);
			new_points_ready = false;
		}
		drive_enable = find_goal();
		if(!delivery_requested) { // If the delivery has not been requested from the Jetson, don't drive
			drive_enable = false;
		}
		if(drive_enable) {

			encoder_diff_l = adc_left_slots - last_used_adc_left_slots;
			last_used_adc_left_slots = adc_left_slots;

			encoder_diff_r = adc_right_slots - last_used_adc_right_slots;
			last_used_adc_right_slots = adc_right_slots;

			set_encoder_diff_l += encoder_diff_l;
			set_encoder_diff_r += encoder_diff_r;

			motor_cmd_count++;
			if(motor_cmd_count >= 1) {
				PointUpdate(set_encoder_diff_l, set_encoder_diff_r);
				PIMotors(set_encoder_diff_l, set_encoder_diff_r, diff_t);
				//SetMotors2(set_encoder_diff_l, set_encoder_diff_r, diff_t);
				//wheelControl(set_encoder_diff_l, set_encoder_diff_r, diff_t);

				motor_cmd_count = 0;
				set_encoder_diff_l = 0;
				set_encoder_diff_r = 0;
				loop_count++;
			}
		}
		else {
			Drive(left_direction, 0, right_direction, 0);
		}

		/* Send over goal points to the Jetson */
		int32_t goal_x_int = goal_x * 100.0f;
		int32_t goal_y_int = goal_y * 100.0f;
		if(jetson_tx_buffer_index == 0) {
			jetson_tx_buffer[0] = '{';
			jetson_tx_buffer[1] = 'C';
			if(goal_x_int < 0) {
				jetson_tx_buffer[2] = '-';
				goal_x_int *= -1.0f;
			} else {
				jetson_tx_buffer[2] = '+';
			}
			for(int i = 0; i < 3; i++) {
				char digit = goal_x_int % 10 + '0';
				goal_x_int /= 10;
				jetson_tx_buffer[3 + i] = digit;
			}
			jetson_tx_buffer[6] = ',';
			if(goal_x_int < 0) {
				jetson_tx_buffer[7] = '-';
				goal_y_int *= -1.0f;
			} else {
				jetson_tx_buffer[7] = '+';
			}
			for(int i = 0; i < 3; i++) {
				char digit = goal_y_int % 10 + '0';
				goal_y_int /= 10;
				jetson_tx_buffer[8 + i] = digit;
			}
			jetson_tx_buffer[11] = '}';
			USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
		}

		nsWait(50000000);
		nsWait(50000000);
	}
}


void PointUpdate (uint32_t diff_l, uint32_t diff_r) {
	float delta_theta = ((float)diff_r - (float)diff_l) * TIC_LENGTH / VEHICLE_WIDTH;
	float delta_x = -0.5 * TIC_LENGTH * ((float)diff_l + (float)diff_r) * sin(delta_theta);
	float delta_y = 0.5 * TIC_LENGTH * ((float)diff_l + (float)diff_r) * cos(delta_theta);

	float x_rot;
	float y_rot;

	for(int i = 0; i < ARRAY_SIZE; i++) {
		x_rot = (points_x[i] - delta_x) * cos(-delta_theta) + (points_y[i] - delta_y) * -sin(-delta_theta);
		y_rot = (points_x[i] - delta_x) * sin(-delta_theta) + (points_y[i] - delta_y) * cos(-delta_theta);

		points_x[i] = x_rot;
		points_y[i] = y_rot;
	}

	return;
}

/*
 * Send number of encoder ticks since last message was sent to Jetson
 */
static void SendEncoderCount() {
	uint32_t left_count_to_send = adc_right_slots - last_encoder_left_count_to_jetson;
	uint32_t right_count_to_send = adc_left_slots - last_encoder_right_count_to_jetson;
	/* If the encoder count deltas to the jetson are too high something is wrong - loop infinitely for now */
	last_encoder_left_count_to_jetson = adc_right_slots;
	last_encoder_right_count_to_jetson = adc_left_slots;
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
/*static void decodeControllerMsg() {
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
}*/

/*
 *  Decodes the points message sent from the Jetson
 */
static void decodePointUpdateMsg() {
	const char comma[2] = ",";
	// Ignoring { and K
	char* token = strtok(last_message, comma);

	for(int i = 0; i < 8; i++) {
		token = strtok(NULL, comma);
		new_points_x[i] = atof(token);
		token = strtok(NULL, comma);
		new_points_y[i] = atof(token);
	}
	new_points_ready = true;
}

/*
 * Determine which type of message was received from the Jetson and parse the data accordingly.
 */
static void decodeJetsonString() {
	switch (last_message[1]) {
	case 'K': // Points for control algorithm
		decodePointUpdateMsg();
		break;
	case 'C': // Remote controller feedback
		//decodeControllerMsg();
		break;
	case 'M': // Mode swtich
		nextMode();
		break;
	case 'N': // Cancel delivery request
		delivery_requested = 0;
		break;
	case 'D': // Delivery request received
		// Reassign points for continuous testing
		/*for(unsigned int i = 0; i < 8; i++) {
			points_x[i] = orig_points_x[i];
			points_y[i] = orig_points_y[i];
		}*/
		delivery_requested = 1;
		break;
	case 'S': // Switch modes for controller requested
		/*if(current_mode == CHANGE_MODE) {
			use_ps4_controller = !use_ps4_controller;
			left_speed = 0;
			right_speed = 0;
		}*/
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
static int Drive(enum Direction left_direction_v, int16_t left_speed_v, enum Direction right_direction_v, int16_t right_speed_v) {
	if(mc_tx_buffer_index != 0) {
		return 1; // Haven't finished sending the last message
	}
	/*
		float left_speed_integral_avg = 0.0f;
		float right_speed_integral_avg = 0.0f;
		bool add_new_integral_term = false;
		integral_counter++;
		if(integral_counter >= 2) {
			add_new_integral_term = true;
			integral_counter = 0;
		}
		for(int i = 0; i < 10; i++) {
			left_speed_integral_avg += left_speed_integral[i];
			right_speed_integral_avg += right_speed_integral[i];
			if(i < 9 && add_new_integral_term) {
				left_speed_integral[i] = left_speed_integral[i + 1];
				right_speed_integral[i] = right_speed_integral[i + 1];
			}
		}
		if(add_new_integral_term) {
			left_speed_integral[9] = left_speed_v;
			right_speed_integral[9] = right_speed_v;
		}
		left_speed_integral_avg /= 10.0f;
		right_speed_integral_avg /= 10.0f;

		left_speed_v = k_i * left_speed_integral_avg + k_p * (float)left_speed_v;
		right_speed_v = k_i * right_speed_integral_avg + k_p * (float)right_speed_v;
	*/
	if(left_speed_v < 0) {
		left_speed = -1 * left_speed_v;
		if(left_direction_v == FORWARD) {
			left_direction_v = REVERSE;
		} else {
			left_direction_v = FORWARD;
		}
	} else if(left_speed_v > 80) {
		left_speed_v = 80;
	}
	if(right_speed_v < 0) {
		right_speed = -1 * right_speed_v;
		if(right_direction_v == FORWARD) {
			right_direction_v = REVERSE;
		} else {
			right_direction_v = FORWARD;
		}
	} else if(right_speed_v > 80) {
		right_speed_v = 80;
	}


	//DIAGNOSTIC ONLY ZERO COUNT TODO REMOVE
	if(left_speed_v == 0 || right_speed_v == 0) {
		zero_count++;
	}

	if(left_direction_v == FORWARD) {
		left_packet.command = 4;
	}
	else if(left_direction_v == REVERSE) {
		left_packet.command = 5;
	}
	if(right_direction_v == FORWARD) {
		right_packet.command = 0;
	}
	else if(right_direction_v == REVERSE) {
		right_packet.command = 1;
	}
	left_packet.data = left_speed_v;
	right_packet.data = right_speed_v;

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
	total_int_count++;
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
		if(adc_left_same_counter > 1) {
			adc_left_slots++;
		}
		prev_adc0_state = adc0_state;
		adc_left_same_counter = 0;
	} else {
		adc_left_same_counter++;
	}
	if(adc1_state != prev_adc1_state) {
		if(adc_right_same_counter > 1) {
			adc_right_slots++;
		}
		prev_adc1_state = adc1_state;
		adc_right_same_counter = 0;
	} else {
		adc_right_same_counter++;
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
				jetson_connected = true;
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
	if(time_since_screen_change > 15) {
		time_since_screen_change = 0;
		nextMode();
	}
	else {
		time_since_screen_change++;
	}
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
			distance = us1_distance;
			while(distance > 0) {
				int num_to_disp = distance % 10;
				out_data[data_loc] = num_to_disp + 48;
				data_loc--;
				distance /= 10;
			}
			data_loc = 13;
			distance = us3_distance;
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
			int count = adc_left_slots;
			while(count > 0) {
				int num_to_disp = count % 10;
				out_data[data_loc--] = num_to_disp + 48;
				count /= 10;
			}
			data_loc = 15;
			count = adc_right_slots;
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
		} else if(current_mode == JETSON) {
			if(jetson_connected) {
				char out_data[16] = {'J', 'E', 'T', 'S', 'O', 'N', ' ', 'O', 'N', ' ', ' ', ' ', ' ', ' ', ' ', ' '};
				DispString(out_data);
			} else {
				char out_data[16] = {'J', 'E', 'T', 'S', 'O', 'N', ' ', 'O', 'F', 'F', ' ', ' ', ' ', ' ', ' ', ' '};
				DispString(out_data);
			}
		}
			/*else if(current_mode == CHANGE_MODE) {
			if(use_ps4_controller) {
				char out_data[13] = {'B', 'T', ' ', 'C', 'O', 'N', 'T', 'R', 'O', 'L', 'L', 'E', 'R'};
				DispString(out_data);
			}
			else {
				char out_data[10] = {'A', 'U', 'T', 'O', 'N', 'O', 'M', 'O', 'U', 'S'};
				DispString(out_data);
			}
		}*/
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
		us1_elapsed_echo_time += 120; // Increment by 60 us
	}
	else if(us1_started && !us1_return) { // Pulse went low
		us1_started = 0;
		us1_distance = us1_elapsed_echo_time * 0.0343 / 2; // Using speed of sound in cm/us to get distance
	}

	if(!us2_started && us2_return) { // Pulse went high
		us2_started = 1;
	}
	else if(us2_started && us2_return) { // Pulse stayed high
		us2_elapsed_echo_time += 120; // Increment by 60 us
	}
	else if(us2_started && !us2_return) { // Pulse went low
		us2_started = 0;
		us2_distance = us2_elapsed_echo_time * 0.0343 / 2; // Using speed of sound in cm/us to get distance
	}

	if(!us3_started && us3_return) { // Pulse went high
		us3_started = 1;
	}
	else if(us3_started && us3_return) { // Pulse stayed high
		us3_elapsed_echo_time += 120; // Increment by 60 us
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
	TIM2->ARR = 192000; 					// Enable interrupt at 2 ms
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
	TIM15->ARR = 120; 						// Trigger every 60 clock cycles (120 us)
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
void nsWait(uint64_t t) {
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
		if(dist_array[idx] < min_dis) {
			min_dis = dist_array[idx];
			min_idx = idx;
		}
	}

	//Find points of the line that Goal Point lies on
	Point target_1;
	Point target_2;
	if( (min_dis <= LOOK_AHEAD_SQ) || (min_idx == (ARRAY_SIZE - 1)) ) {
		bool break_success = false;
		for(int offset = 1; offset + min_idx < ARRAY_SIZE; offset++) {
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
			//All destination points within Look Ahead Distance, cease travel
			goal_x = 0.0f;
			goal_y = 0.0f;
			return false;
		}
	}
	// If the minimum distance is greater than the lookahead distance
	else {
		float decision_dist = distance_squared(points_x[min_idx], points_y[min_idx], points_x[min_idx + 1], points_y[min_idx + 1]);

		if(dist_array[min_idx + 1] > decision_dist) {
			if(min_idx != 0) {
				target_1.x = points_x[min_idx - 1];
				target_1.y = points_y[min_idx - 1];
				target_2.x = points_x[min_idx];
				target_2.y = points_y[min_idx];
			}
			else {
				target_1.x = points_x[0];
				target_1.y = points_y[0];
				target_2.x = points_x[1];
				target_2.y = points_y[1];
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
	if(target_2.x == target_1.x) {
		goal_x = 0.0f;
		goal_y = LOOK_AHEAD;
		return true;
	}
	float slope_1 = (target_2.y - target_1.y) / (target_2.x - target_1.x);
	float b_1 = target_2.y - slope_1 * target_2.x;

	//Locate X-Coordinate of Goal Point
	target_1.x = b_1 * slope_1 / ( -1 - slope_1 * slope_1);
	target_1.y = slope_1 * target_1.x + b_1;

	//If intersected point lies beyond Look Ahead, Robot is very off course. Set course for nearest forward point
	if(distance_squared(0.0, 0.0, target_1.x, target_1.y) > LOOK_AHEAD_SQ ) {
		goal_x = target_2.x;
		goal_y = target_2.y;
		return true;
	}

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
	float v_c = ((float)diff_l + (float)diff_r) * TIC_LENGTH / 2.00f / diff_t;
	//v_c above in units of m/s, v_c below in units of Motor Controller Torque
	v_c = VELOCITY_EQ_M * v_c + VELOCITY_EQ_B;

	bool accel_flag = false;
	bool decel_flag = false;

	if(v_c < NORMAL_SPEED) {
		accel_flag = true;
		v_c += ACCEL * accel_count;
		if(v_c >= MAX_SPEED) {
			v_c = MAX_SPEED;
			//Implement some sort of way to kill operation if failing to arrive at max_speed within
			//a certain count of loop iterations. Each loop
			accel_fail_count++;
		}
		else {
			accel_count++;
		}
	}

	if(v_c > NORMAL_SPEED && !accel_flag) {
		decel_flag = true;
		v_c -= ACCEL * decel_count;
		if(v_c <= 0) {
			v_c = 0;
			decel_fail_count++;
		}
		else {
			decel_count++;
		}
	}

	if(!accel_flag) {
		accel_count = 1;
		accel_fail_count = 0;
	}
	if(!decel_flag) {
		decel_count = 1;
		decel_fail_count = 0;
	}

/*	TODO Need some way to recover after COUNT exceeded
	if(accel_fail_count >= MAX_MOTION_FAILURE_COUNT || decel_fail_count >= MAX_MOTION_FAILURE_COUNT ) {
		Drive(left_direction, 0, right_direction, 0);
		return;
	}
*/

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
	right_speed = (int) ((float)v_r * L_R_BIAS); // Current right speed to be requested

	Drive(left_direction, left_speed, right_direction, right_speed);

	return;
}

void SetMotors2 (uint32_t diff_l, uint32_t diff_r, float32_t diff_t) {
	float v_c = ((float)diff_l + (float)diff_r) * TIC_LENGTH / 2.00f / diff_t;	//  m/s
	if(goal_x < 0.2 && goal_x > -0.2) {
		TURN_MULT = 0.3f;
	} else if(goal_x < 0.5 && goal_x > -0.5) {
		TURN_MULT = 0.5f;
	} else {
		TURN_MULT = 0.7f;
	}

//	bool accel_flag = false;
	bool decel_flag = false;

	if(v_c < AVERAGE_SPEED) {
//		accel_flag = true;
		if(v_t < MAX_TORQUE) v_t += ACCEL_2;
		else accel_fail_count++;
	}
	else if(v_c > AVERAGE_SPEED + 0.2f) {		// The 0.2f is so we are not constantly accel/decel, favors accel
		if(v_t > 0) {
			decel_flag = true;
			v_t -= DECEL_2;
			decel_count++;
		}
		else decel_fail_count++;
	}

/*	TODO Need some way to recover after COUNT exceeded
	if(accel_fail_count >= MAX_MOTION_FAILURE_COUNT || decel_fail_count >= MAX_MOTION_FAILURE_COUNT ) {
		Drive(left_direction, 0, right_direction, 0);
		return;
	}
*/

  //Assign left and right torques based on total torque
	int8_t v_l = (int) (v_t * (1.0f + VEHICLE_WIDTH * goal_x * TURN_MULT / LOOK_AHEAD_SQ));
	int8_t v_r = (int) (v_t * (1.0f - VEHICLE_WIDTH * goal_x * TURN_MULT / LOOK_AHEAD_SQ));

<<<<<<< HEAD

  //Change left and right torques to prevent overcorrection towards goal line
	if(v_l < v_r) { //Turning right
		if(prev_goal_x < goal_x) {//approaching goal x from left
			prev_goal_x = goal_x;
			v_t += DECEL_2;
			Drive(left_direction, left_speed, right_direction, right_speed);
			return;
=======
	float goal_delta = goal_x - prev_goal_x;
	if(v_l < v_r) {
		if(goal_delta > 0.0f) {
			v_r -= ACCEL_2;
>>>>>>> 0ff3a1ed8fa3a0b772b7c90a03d4abdf65a6e7d4
		}
	} else if(v_r < v_l) {
		if(goal_delta > 0.0f) {
			v_l -= ACCEL_2;
		}
	}
	prev_goal_x = goal_x;

	if(v_l > MAX_TORQUE) {
		//v_r -= v_l - MAX_TORQUE;
		v_r -= DECEL_2;
		v_l = MAX_TORQUE;
	}
	if(v_r > MAX_TORQUE) {
		//v_l -= v_r - MAX_TORQUE;
		v_l -= DECEL_2;
		v_r = MAX_TORQUE;
	}
	if( (v_l - v_r) > MAX_TURN) v_l -= (v_l - v_r - MAX_TURN);
	if( (v_r - v_l) > MAX_TURN) v_r -= (v_r - v_l - MAX_TURN);
	//if(v_l < 0) v_l = 0;
	//if(v_r < 0) v_r = 0;

	Drive(left_direction, v_l, right_direction, (int)((float)v_r * L_R_BIAS));

	left_speed = v_l; // Current left speed to be requested
	right_speed = (int)((float)v_r * L_R_BIAS); // Current right speed to be requested

	return;
}

<<<<<<< HEAD
//The ending to the critically acclaimed trilogy you've all been waiting for
void SetMotors3 (uint32_t diff_l, uint32_t diff_r, float32_t diff_t) {
  uint8_t v_l;
  uint8_t v_r;
  if(left_speed < NORMAL_SPEED_3 - 5) {
    v_l = left_speed + 2;
    v_r = left_speed + 2;
  } 
  else {
    //the REAL good stuff
    v_l = NORMAL_SPEED_3;
    v_r = NORMAL_SPEED_3;

    float diff_x = goal_x - prev_goal_x;
    float diff_y = goal_y - prev_goal_y;

    float heading = ((float)diff_r - (float)diff_l) * TIC_LENGTH / VEHICLE WIDTH;

    prev_goal_x = goal_x;
    prev_goal_y = goal_y;

    if(goal_x >= 0) {
      v_l = v_l + (TURN_COEFF*-cos(heading))+(TURN_COEFF*sin((PI_VAR/2)*(goal_x/5)));
      v_r = v_r + (TURN_COEFF*cos(heading))+(-TURN_COEFF*sin((PI_VAR/2)*(goal_x/5)));
    }
    else if(goal_x < 0) {
      v_l = v_l + (TURN_COEFF*-cos(heading))+(TURN_COEFF*sin((PI_VAR/2)*(goal_x/5)));
      v_r = v_r + (TURN_COEFF*cos(heading))+(-TURN_COEFF*sin((PI_VAR/2)*(goal_x/5)));
    }

    if(v_l > MAX_TORQUE) {
      //v_r -= v_l - MAX_TORQUE;
      v_r -= DECEL_2;
      v_l = MAX_TORQUE;
    }
    if(v_r > MAX_TORQUE) {
      //v_l -= v_r - MAX_TORQUE;
      v_l -= DECEL_2;
      v_r = MAX_TORQUE;
    }
    //if( (v_l - v_r) > MAX_TURN) v_l -= (v_l - v_r - MAX_TURN);
    //if( (v_r - v_l) > MAX_TURN) v_r -= (v_r - v_l - MAX_TURN);
    //if(v_l < 0) v_l = 0;
    //if(v_r < 0) v_r = 0;
  }
  
  Drive(left_direction, v_l, right_direction, (int)((float)v_r * L_R_BIAS));
  left_speed = v_l; // Current left speed to be requested
  right_speed = (int)((float)v_r * L_R_BIAS); // Current right speed to be requested

  return;
}





void PIMotors(uint32_t diff_l, uint32_t diff_r) {
	if(abs(goal_x) < 0.1) {
		if(left_speed < NORMAL_SPEED && right_speed < NORMAL_SPEED) {
			if(right_speed > left_speed) {
				left_speed = right_speed;
			} else {
				right_speed = left_speed;
=======


void PIMotors(uint32_t diff_l, uint32_t diff_r, float dt) {
	float prop_adj = 4.0f;
	float failed_multi = 1.0f;
	right_speed = NORMAL_SPEED;
	left_speed = NORMAL_SPEED;
	float goal_delta = goal_x - prev_goal_x;
	if((float)failed_trajectory_counter / dt > 1.0f) {
		failed_multi += 1.0f;
		failed_trajectory_counter = 0;
	}
	/*if(diff_l == 0 && diff_r == 0) {
		stall_count++;
	}
	else {
		if(stall_count >= 1) {
			stall_count--;
		}
	}
	left_speed += stall_count;
	right_speed += stall_count;*/

	if(goal_x < 0.1f) { // Want to turn the vehicle left
		if(prev_goal_x < 0.1f) {
			if(goal_delta > 0.0f) { // The vehicle is on a trajectory to correct left
				failed_trajectory_counter = 0;
				left_speed += (prop_adj * goal_delta);
				//left_speed -= 1.0f / (-1.0f * goal_x);
			}
			else { // Not yet on the correct trajectory
				failed_trajectory_counter++;
				left_speed -= (prop_adj * -1.0f * goal_x * failed_multi);
>>>>>>> 0ff3a1ed8fa3a0b772b7c90a03d4abdf65a6e7d4
			}
		}
		else {
			// Overcorrected so I want to decrease left speed
			failed_trajectory_counter = 0;
			left_speed -= (prop_adj * goal_x * goal_x);

		}
	}
	else { // Want to turn the vehicle right
		if(prev_goal_x > 0.1f) {
			if(goal_delta < 0.0f) { // The vehicle is on a trajectory to correct right
				failed_trajectory_counter = 0;
				left_speed -= (prop_adj * -1.0f * goal_delta);
				//left_speed += 1.0f / goal_x;
			}
			else { // Not yet on the correct trajectory
				failed_trajectory_counter++;
				left_speed += (prop_adj * goal_x * failed_multi);
			}
		}
		else {
			// Overcorrected so I want to increase left speed
			failed_trajectory_counter = 0;
			left_speed += (prop_adj * goal_x * goal_x);
		}
	}
	prev_goal_x = goal_x;

	// Need to add in motor stall logic here


	// Corrections in case the values exceed maximum specifications
	/*if(left_speed > MAX_SPEED) {
		right_speed = right_speed - (left_speed - MAX_SPEED);
		left_speed = MAX_SPEED;
	} else if(right_speed > MAX_SPEED) {
		left_speed = left_speed - (right_speed - MAX_SPEED);
		right_speed = MAX_SPEED;
	}*/
	//if((left_speed - right_speed) > MAX_TURN) left_speed -= left_speed - right_speed - MAX_TURN;
	//if((right_speed - left_speed) > MAX_TURN) right_speed -= right_speed - left_speed - MAX_TURN;

	// Drive the MF
	Drive(left_direction, left_speed, right_direction, (float)right_speed * L_R_BIAS);
	/*debug_goal_x[debug_goal_x_index++] = goal_x;
	if(debug_goal_x_index == 99) {
		int idx = 0;
		idx++;
	}*/
}



/**************************************************
 *
 *  Keith's Control algorithm Tunable Parameters
 *  
**************************************************/
int16_t old_power_l = 0;
int16_t old_power_r = 0;

float last_commanded_vl = 0;
float last_commanded_vr = 0;

float old_kr = 0.0;
float old_kl = 0.0;


// Velocity deltas can range from 0 to .3 m/s
// Power can range 0 - 128
// Want to add corrections by around 25% filtered
// 255/.3 ~= 765 so delta_velocity * 765 = linearly_scaled_power
// Want 10 % of that so want 191.25

void wheelControl(uint32_t diff_l, uint32_t diff_r, float32_t diff_t) {
	float measured_vl = (((float) diff_l) * TIC_LENGTH) / diff_t;
	float measured_vr = (((float) diff_r) * TIC_LENGTH) / diff_t;

	float desired_vl = (AVERAGE_SPEED * (1.0f + VEHICLE_WIDTH * goal_x * TURN_MULT / LOOK_AHEAD_SQ));
	float desired_vr = (AVERAGE_SPEED * (1.0f - VEHICLE_WIDTH * goal_x * TURN_MULT / LOOK_AHEAD_SQ));
	desired_vl = 0.3f;
	desired_vr = 0.4f;

	// Figure out what the power scale constants will be
	float kl;
	kl = (last_commanded_vl - measured_vl)*KPL; // TODO: Could add integral constants here
	kl += old_kl;
	float kr;
	kr = (last_commanded_vl - measured_vl)*KPR;
	kr += old_kr;

	// Adjust power outputs as a function of kl and kr
	float power_l = 20 + (desired_vl - measured_vl) * kl;
	float power_r = 20 + (desired_vr - measured_vr) * kr;
	old_power_l = power_l;
	old_power_r = power_r;

	old_kl = kl;
	old_kr = kr;
	last_commanded_vl = desired_vl;
	last_commanded_vr = desired_vr;

	Drive(FORWARD, (int16_t)power_l, FORWARD, (int16_t)power_r);
}
