#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/pwm.h"
#include "driverlib/adc.h"
#include "driverlib/systick.h"
#include "calibration.h"

// Debug instruction. This instruction comes from the TivaWare template for Keil.
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line) {
}
#endif

// Defines
#define PID_MAX				255							// Defines the maximum value that the PID control loop can have. This value is then scaled to match the dutyCycle scale.
#define PID_MIN				1								// Defines the minimum value that the PID control loop can have. This value is limited at 1 since the dutyCycle register can not be smaller than 1.
#define SENSOR				GPIO_PIN_3			// Input to the IR sensor. Pin is PE_3.
#define RED_LED   		GPIO_PIN_1			// Output to the oboard red led on the Tiva C development board. Pin is PF_1.
#define BLUE_LED  		GPIO_PIN_2			// Output to the oboard blue led on the Tiva C development board. Pin is PF_2.
#define GREEN_LED 		GPIO_PIN_3			// Output to the oboard green led on the Tiva C development board. Pin is PF_3.
#define COIL_EN				GPIO_PIN_0			// Output to the enable pin of the H-Bridge. Pin is PD_0.
#define COIL_ATRACT		GPIO_PIN_1			// Output to the direction 1 pin of the H-Bridge. Pin is PD_1.
#define COIL_REPEL		GPIO_PIN_2			// Output to the direction 2 pin of the H-Bridge. Pin is PD_2.

volatile bool doControlFlag = false;	// Flag set by the sysTick timer. Signals that it is time for the control task to execute. 
																			// Must be volatile since it is used during an interrupt routine.
uint32_t counter = 0;									// Counter for the amount of times that the control task executes.
uint32_t sensor = 0;									// Stores the current ADC value of the sensor.
float distance = 0;										// Calculated distance using the sensor ADC value.
uint32_t dutyCycle = 0;								// Final dutyCycle variable passed into PID library.
											
// PWM variables
uint32_t pwmPeriod = 0;								// Period of the pwm generator. Sets the pwm frequency. This value is received from the computer.
float distanceBuffer[3];							// Holds the last three previous distances. Used to estimate the centered derivative of the distance.
float error = 0;											// Calculated error, set point - distance.
float setPoint = 0;										// Sets the current set point in milimeters.										
float pControl = 0;										// Proportional term of the PID control loop.
float iControl = 0;										// Integral term of the PID control loop.
float dControl = 0;										// Derivative term of the PID control loop.
float kp = 0;													// Proportional gain of the PID control. This value is received from the computer.
float kd = 0;													// Derivative gain of the PID control. This value is received from the computer.
float ki = 0;													// Integral gain of the PID control. This value is received from the computer.
float dt = 0;													// Time step of the control task in seconds.
float pid = 0;												// Total output of the PID. It is the sum of pControl, iControl and dControl.

// Serial variables
uint8_t sendBuffer[4];										// Buffer for storing the values that will be sent to the computer.
uint8_t buffer[20];											// Buffer for storing the received values from the computer.
bool startSavingBytes = false;				// Flag for signaling that a start byte has been received and all subsequent bytes must be stored in the receive buffer.
uint32_t bufferCount = 0;							// Amout of bytes placed on the receive buffer.
uint8_t inByte = 0;										// Currently received byte from serial.
bool receivedParametersFlag = false;	// Flag set to true once the control constats were received from the computer.

void ISRSysTick(void) {
	/**
	 * Interrupt caused by the System Timer.
	 */
	
	// Enables the control task.
	doControlFlag = true;
}

void send() {
	/**
	 * Sends the current bytes placed in the sendBuffer array via serial.
	 */ 
	
	UARTCharPut(UART0_BASE, sendBuffer[0]);
	UARTCharPut(UART0_BASE, sendBuffer[1]);
	UARTCharPut(UART0_BASE, sendBuffer[2]);
	UARTCharPut(UART0_BASE, sendBuffer[3]);

}

void sendChar(uint8_t c) {
	/**
	 * Send a char via serial.
	 */
	
	UARTCharPut(UART0_BASE, c);
}

void sendUInt(uint32_t d) {
	/**
	 * Send an unsigned int via serial. The value of d is copied into the send buffer
	 * before sending it via serial.
	 */
	
	memcpy(&sendBuffer, &d, sizeof d);
	send();
	
}

void sendInt(int32_t d) {
	/**
	 * Send an int via serial. The value of d is copied into the send buffer
	 * before sending it via serial.
	 */
	
	memcpy(&sendBuffer, &d, sizeof d);
	send();
	
}

void sendFloat(float f) {
	/**
	 * Send a float via serial. The value of f is copied into the send buffer
	 * before sending it via serial.
	 */
	
	memcpy(&sendBuffer, &f, sizeof f);
	send();
	
}

int main(void) {
		
	// Enable the floating point unit. By enabling lazy stacking on the FPU, floating point
	// calculations can be made within an interrupt.
	// Found in pg. 243 of the TivaWare datasheet.
	FPUEnable();
	FPULazyStackingEnable();
	
	// Set processor clock at 80Mhz.
	// Found in pg. 479 of the TivaWare datasheet.
	SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
	
	// Set PWM clock at 80Mhz / 32 = 2,5 Mhz
	SysCtlPWMClockSet(SYSCTL_PWMDIV_32);
	
	// Enable GPIO ports F, A, D, E and wait for them to be ready.
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);						// Leds are in GPIOF.
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);						// Serial port is in GPIOA.
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);						// H-Bridge control is in GPIOD.
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);						// Analog inputs are in GPIOE.
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)) {}
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA)) {}
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD)) {}
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE)) {}
	
	// Enable pheripherals UART0, PWM1, and ADC0, then wait for them to be ready.
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0)) {}
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM1)) {}
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0)) {}
	
	// Configure the following pins as ADC inputs.
	GPIOPinTypeADC(GPIO_PORTE_BASE, SENSOR);
	
	// Configure the following pins as digital outputs.
	GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, COIL_ATRACT | COIL_REPEL);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, RED_LED|BLUE_LED|GREEN_LED);
	
	// Configure the following pins as PWM outputs.
	GPIOPinConfigure(GPIO_PD0_M1PWM0);					// Select the alternative function for this gpio pin.
	GPIOPinTypePWM(GPIO_PORTD_BASE, COIL_EN);
		
	// Configure the following pins as UART inputs/outputs.
	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	
	// Configure the PWM generator. Port PD_0 is the ouput of the PWM. 
	// According to the table 23.3 Signals by Signal Name on pg. 1339 of the Tiva Datasheet, PD_0
	// corresponds to PWM module 1, generator 0.
	PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
		
	// Sets the PWM period in PWM clock ticks. The value set here is not important since it will
	// be replaced by the value sent by the computer.
	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, 65000);
		
	// Sets the pulse width in PWM clock ticks. This value must be smaller than the pwm period but larger
	// than 0.
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, 1);
	
	// Enable the PWM module.
	PWMGenEnable(PWM1_BASE, PWM_GEN_0);
	PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, true);
		
	// Configure the ADC to trigger per demand basis. Also enable the hardware averaging to calculate
	// an average of 16 adc samples.
	// Hardware sample averaging circuit is found in pg. 807 of the Tiva Datasheet.
	ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
	ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);
	ADCHardwareOversampleConfigure(ADC0_BASE, 16);
	ADCSequenceEnable(ADC0_BASE, 3); 						// Enables the ADC to be triggered
	ADCIntClear(ADC0_BASE, 3);

	// Configure the UART peripheral to run at 1000000 bauds. 
	UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 1000000, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
	
	// Enable interrupts on the system.
	IntMasterEnable();
		
	// Configure the sysTick timer to fire an interrupt every 140 us. This interrupt signals
	// when the control task must be executed. Modifying this value changes the speed of the system.
	// The value is calculated used clock ticks such as follows:
	// 1/80 Mhz = 12.5 nS
	// 140 uS / 12.5nS = 11200
	dt = 0.000140;
	SysTickIntRegister(ISRSysTick);
	SysTickPeriodSet(11200);
	SysTickIntEnable();
	SysTickEnable();

	// Turn on LEDs to signal that the controller is on.
	// If the blue led is off this means that the controller is not running.
	GPIOPinWrite(GPIO_PORTF_BASE, BLUE_LED, BLUE_LED);

	// Start the main loop.
	while(1) {
		
		// controlTask: measure sensor, calculate distance, calculate pid, set manipulation.
		// Period is 140 us.
		// Flag for executing the control task. This flag is set by the SysTick timer. The frequency at which this interrupt
		// fires is equal to the frequency of execution.
		if (doControlFlag) {
			
			// Clear control flag. This allows the SysTick to set it again.
			doControlFlag = false;
			
			// Turn on RED led. This signals in the oscilloscope the start of the control loop.
			// The time that the RED led is on is equal to the time the control loop takes.
			// Therefore, this time must be smaller than the period of the control loop.
			GPIOPinWrite(GPIO_PORTF_BASE, RED_LED, RED_LED);
			
			// Do control only if parameters have been previously received from the computer.
			// Otherwise request them.
			if (receivedParametersFlag) {
			
				// Reads the analog value of the sensor.
				ADCProcessorTrigger(ADC0_BASE, 3);
				while (!ADCIntStatus(ADC0_BASE, 3, false)) {}
				ADCIntClear(ADC0_BASE, 3);
				ADCSequenceDataGet(ADC0_BASE, 3, &sensor);
					
				// Only perform control if the sensor is within sensing range. This means that the 
				// sensor value is inside the calibration table. The table is made up of 100 elements
				// starting from 0 to 10 mm. The index of the array coresponds to a distance of 0.1 mm.
				if (sensor <= calibration[0] && sensor > calibration[100]) {
					
					// Use the calibration loopup table to convert the analog value to distance.
					for (uint32_t i = 0; i < 100; i++) {
						
						// Find the adjacent cells of the current sensor reading.
						if ( sensor <= calibration[i] && calibration[i+1] < sensor) {
							
							// Do a linear regression between the adjacent cells and the current sensor value.
							distance = (sensor - calibration[i]) / (10.0 * (calibration[i+1] - calibration[i+1])) + i / 10.0;
							break;
							
						}
						
					}
					
					// Calculate the error of the system.
					error = setPoint - distance;
					
					// Calculate proportional term.
					pControl = error * kp;
					
					// Calculate derivative term using a centered derivative.
					// This produces more stable results than by using a backwards derivative.
					distanceBuffer[2] = distanceBuffer[1];
					distanceBuffer[1] = distanceBuffer[0];
					distanceBuffer[0] = distance;
					dControl = (distanceBuffer[0] - distanceBuffer[2]) * kp * kd / (2 * dt);
					
					// Calculate integral term. Constrain the integral to be within -PID_MAX and PID_MAX.
					iControl += error * dt * kp / ki;
					if (iControl > PID_MAX) {
						iControl = PID_MAX;
					} else if (iControl <  -PID_MAX) {
						iControl = -PID_MAX;
					}
					
					// Calculate the total pid control. Constrain the pid output to be within PID_MIN and PID_MAX.
					pid = pControl + dControl + iControl;
					if (pid > PID_MAX) {
						pid = PID_MAX;
					} else if (pid < PID_MIN) {
						pid = PID_MIN;
					}
					
					// Scale the pid output to the dutyCycle range.
					dutyCycle = pid * pwmPeriod / PID_MAX;
					
					// Set the coil duty cycle.
					PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, dutyCycle);
					
					// Set the direction of the coil. Coil can also be set to repel, but this requires a different
					// control strategy.
					GPIOPinWrite(GPIO_PORTD_BASE, COIL_ATRACT | COIL_REPEL, COIL_ATRACT);
					// GPIOPinWrite(GPIO_PORTD_BASE, COIL_ATRACT | COIL_REPEL, COIL_REPEL);
					
				}
			
				// Increase control loop counter.
				counter++;
				
			} else {
				
				// Since parameters have not yet been received, send the character sequence '<>' to request
				// the computer for parameters.
				UARTCharPut(UART0_BASE, '<');
				UARTCharPut(UART0_BASE, '>');
				
			}
					
			// Turn off RED led.
			GPIOPinWrite(GPIO_PORTF_BASE, RED_LED, 0);
				
		}
		
		// sendTask: send the current state of variables to the computer.
		// The period of the status loop is set by the counter variable. Since the counter increases
		// every SysTick period, then the timing of this loop can be easily calculated.
		// Period is 20 * 140 us
		if (counter >= 20) {
			
			// Turn on GREEN led, this signals the start of the status loop.
			// This signal can be used to measure the cycle time.
			GPIOPinWrite(GPIO_PORTF_BASE, GREEN_LED, GREEN_LED);
			
			// Sends the start byte $ to signal the receiver that a transmission is
			// starting. Then other variables are sent. The receiver is programmed to expect this
			// data pattern.
			sendChar('$');
			sendFloat(distance);
			sendFloat(error);
			sendFloat(pid);
			sendFloat(pControl);
			sendFloat(iControl);
			sendFloat(dControl);
		
			// Reset the counter.
			counter = 0;
			
			// Turn off GREEN led.
			GPIOPinWrite(GPIO_PORTF_BASE, GREEN_LED, 0);
			
		}
		
		// receiveTask: receives pid constants and pwm frequency from the computer.
		// This task has no period, it runs every time there are characters available.
		// However, characters are only received when the user changes a slider on the
		// computer interface, which doesn't occur that often. Moreover, the time used
		// to read the bytes is after executing the control loop. Therefore, the control task
		// does not miss a deadline.
		while (UARTCharsAvail(UART0_BASE)) {
			
			// Read the next character found in the receive buffer.
			inByte = UARTCharGet(UART0_BASE);
			
			// If a $ sign has been received, then start saving bytes into the buffer.
			if (startSavingBytes) {
				
				// Start saving bytes into the buffer.
				buffer[bufferCount] = inByte;
				bufferCount++;
				
				// If 20 bytes have been received, start to process them.
				if (bufferCount == 20) {
					
					// 4 bytes are received for every floating point value. Therefore 5 values were received.
					// Use memcpy to set the data on the buffer to their corresponding variables.
					memcpy(&setPoint, &buffer, 4);
					memcpy(&kp, &buffer[4], 4);
					memcpy(&ki, &buffer[8], 4);
					memcpy(&kd, &buffer[12], 4);
					memcpy(&pwmPeriod, &buffer[16], 4);
					
					// Update the pwm frequency with the newly received value.
					PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, pwmPeriod);
					
					// Reset the buffer count and be ready to receive new data.
					bufferCount = 0;
					startSavingBytes = false;
					receivedParametersFlag = true;
					
				}
				
			} else if (inByte == '$') {
				
				// If the incoming byte is a $, this means that a payload of 20 bytes is incomming.
				startSavingBytes = true;
			
			} else if (inByte == 'r') {
				
				// If the incoming byte is a r, reset the values of the pid controller.
				pControl = 0;
				iControl = 0;
				dControl = 0;
				
			}
			
		}
		
	}
	
}
