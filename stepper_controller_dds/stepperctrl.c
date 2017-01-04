/**
 * This file is part of Rotatory Inverted Pendulum.
 * 
 * Copyright 2016, 2017 Institute of Parallel and Distributed Systems (IPVS).  
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
  * limitations under the License.
 * 
 * Change history:
 * - 2016-03-13, Frank Duerr (frank.duerr@ipvs.uni-stuttgart.de):
 *   Initial implementation.
 * - 2017-01-02, Frank Duerr (frank.duerr@ipvs.uni-stuttgart.de):
 *   Added SPI input for dynamically setting motor acceleration.
 * - 2017-01-04, Frank Duerr (frank.duerr@ipvs.uni-stuttgart.de):
 *   Added SPI output of current motor speed.
 */

/*
  This implementation generates the stepper output signal using direct digital 
  synthesis (DDS). 

  An input value (n_speed) corresponding to the motor speed is periodically 
  added to a counter (phase accumulator; n) using a timer with fixed 
  frequency. The phase value is then mapped to a square wave signal for the 
  step output by inverting the step output whenever the accumulator overflows 
  a certain threshold (N).

  The input value n_speed modeling the motor speed is periodically increased or 
  decreased by n_accel using another fixed period timer to accelerate or 
  decelerate the motor.
 */

// We compile for Arduino Due featuring a SAM3X8E 
#include <system_sam3x.h>

// Master clock frequency [Hz].
// Arduino Due runs at 84 MHz.
#define F_MASTER 84000000UL

// Frequency of motor speed timer [Hz]
#define F_SPEED 500000UL

// Frequency of acceleration timer [Hz]
#define F_ACCEL 10000UL

// Invert step output, whenever variable n overflows N-1.
// Must be a power of 2 for fast overflow check.
#define N 0x40000000L

// LED is on pin PB27 on Port B on Arduino Due
#define PORT_LED PIOB
#define PIN_LED PIO_PB27

// PIO settings for stepper motor
#define PORT_STEPPER_DIR PIOC
#define PIN_STEPPER_DIR PIO_PC23
#define PORT_STEPPER_STEP PIOC
#define PIN_STEPPER_STEP PIO_PC24

// Settings for SPI.
// Using SPI0 since SPI1 is not brought out on Arduino Due board.
#define PORT_SPI PIOA
#define PIN_MISO PIO_PA25
#define PIN_MOSI PIO_PA26
#define PIN_SPCK PIO_PA27
#define PIN_SS PIO_PA28

#define PI 3.141592654f

// We use a 1.8 deg stepper.
#define STEPPER_DEG 1.8f
// We use 1/16 stepping mode.
#define STEPPER_DIV 16
// Effective stepping angle [radian].
#define STEP_ANGLE (2.0f*PI*(STEPPER_DEG/STEPPER_DIV)/360.0f)

// Maximum angular frequency [radian/s].
// 5 rotations per second.
#define MAX_OMEGA (5.0f*2.0f*PI)

// DDS phase accumulator.
static int32_t n = 0;
// The input value periodically added to the phase accumulator.
volatile static int32_t n_speed = 0;
// For changing the motor speed, this value is periodically added to the
// DDS input value n_speed. 
volatile static int32_t n_accel = 0;
 
static int32_t n_speed_max;

/**
 * Calculate the angular speed from phase accumulator increment.
 *
 * @param n_speed phase accumulator increment
 * @return angular speed [rad/s]
 */
static float n_speed_to_omega(int32_t n_speed)
{
     static const float c_n_speed_to_omega = (STEP_ANGLE*F_SPEED) / (2.0f*N);
     
     float omega = c_n_speed_to_omega*n_speed;
     
     return omega;
}

/**
 * Calculate phase accumulator increment modeling motor speed according to 
 * given angular frequency.
 *
 * @param omega angular frequency [radian/s]
 * @return phase accumulator increment
 */
static int32_t omega_to_n_speed(float omega)
{
     // Every time n overflows, the step output signal is inverted. Thus, a 
     // step is made every second time n overflows N. Therefore, we need
     // to multiply N by two.
     static const float c_omega_to_n_speed = (2.0f*N) / (STEP_ANGLE*F_SPEED);

     int32_t n_speed = (int32_t) (c_omega_to_n_speed*omega + 0.5f);

     return n_speed;
}

/**
 * Calculate acceleration step (increment of the phase accumulator input).
 *
 * @param omegadot change of angular frequency [radian/s**2]
 */
static int32_t omegadot_to_n_accel(float omegadot)
{
     static const float c_omegadot_to_n_accel = (2.0f*N) / 
	  (STEP_ANGLE*F_SPEED*F_ACCEL);

     int32_t n_accel = (int32_t) (c_omegadot_to_n_accel*omegadot + 0.5f);

     return n_accel;
}

/**
 * Bail out by going into endless loop.
 */
static void die()
{
     while (1);
}

/**
 * Setup IO pins of stepper driver.
 */
static void setup_io()
{
     // Enable digital IO.
     PORT_STEPPER_DIR->PIO_PER = PIN_STEPPER_DIR;
     // Configure pins as output.
     PORT_STEPPER_DIR->PIO_OER = PIN_STEPPER_DIR;
     // Disable pull-up resistor on pin.
     PORT_STEPPER_DIR->PIO_PUDR = PIN_STEPPER_DIR;
     // Disable output.
     PORT_STEPPER_DIR->PIO_CODR = PIN_STEPPER_DIR;

     // Enable digital IO.
     PORT_STEPPER_STEP->PIO_PER = PIN_STEPPER_STEP;
     // Configure pins as output.
     PORT_STEPPER_STEP->PIO_OER = PIN_STEPPER_STEP;
     // Disable pull-up resistor on pin.
     PORT_STEPPER_STEP->PIO_PUDR = PIN_STEPPER_STEP;
     // Disable output.
     PORT_STEPPER_STEP->PIO_CODR = PIN_STEPPER_STEP;
}

/**
 * Switch LED on.
 */
static void led_on()
{
     PORT_LED->PIO_SODR = PIN_LED;
}

/**
 * Turn LED off.
 */
static void led_off()
{
     PORT_LED->PIO_CODR = PIN_LED;
}

/**
 * Configure LED (used for showing status).
 */
static void setup_led()
{
     // Enable digital IO.
     PORT_LED->PIO_PER = PIN_LED; 
     // Configure pin as output.
     PORT_LED->PIO_OER = PIN_LED;
     // Disable pull-up resistor.
     PORT_LED->PIO_PUDR = PIN_LED;
     // Turn off LED by setting pin to high (LED is active low).
     led_off();
}

/**
 * Configuration of first channel of timer counter 0 (TC0) for controlling the 
 * motor speed.
 */
static void setup_speed_timer()
{
     volatile uint32_t timer_status;

     // Enable first channel of TC0 with the Power Management Controller (PMC).
     PMC->PMC_PCER0 = (1UL << ID_TC0);
    
     // Disable clock for TC0, channel 0 using Channel Control Register (CCR).
     TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKDIS;
 
     // Disable all interrupts.
     TC0->TC_CHANNEL[0].TC_IDR = 0xffffffff;
 
     // Clear timer status register by reading it.
     timer_status = TC0->TC_CHANNEL[0].TC_SR;

     // Set capture mode (CMR).
     // TC_CMR_TCCLKS_TIMER_CLOCK1: run clock at MCLK/2 (84 MHz/2 = 42 MHz)
     // TC_CMR_CPCTRG: compare trigger mode
     TC0->TC_CHANNEL[0].TC_CMR = TC_CMR_CPCTRG | TC_CMR_TCCLKS_TIMER_CLOCK1;

     // Set compare trigger value.
     // We want triggers with 1 MHz frequency. MCLK/2 = 42 MHz.
     // Thus, we need a comparator value of 42.
     TC0->TC_CHANNEL[0].TC_RC = (uint32_t) (((float) (F_MASTER)/2.0f) / 
					    F_SPEED + 0.5f);

     // Configure and enable interrupt on RC compare (TC_IER_CPCS).
     NVIC_EnableIRQ(TC0_IRQn);
     TC0->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
 
     // Enable clock (CLKEN).
     TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN;

     // Reset and start counter by software trigger (SWTRG).
     TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_SWTRG;
}

/**
 * Configuration of second channel of timer counter 0 (TC0) controlling the 
 * motor acceleration by periodically adding to the DDS input.
 */
static void setup_acceleration_timer()
{
     volatile uint32_t timer_status;

     // Enable second channel of TC0. Note that in ASF, the second channel of
     // TCO is also called TC1 sometimes, which is not consistent with 
     // the data sheet.
     PMC->PMC_PCER0 = (1UL << ID_TC1);

     // Disable clock for TC1, channel 0 using Channel Control Register (CCR).
     TC0->TC_CHANNEL[1].TC_CCR = TC_CCR_CLKDIS;
 
     // Disable all interrupts.
     TC0->TC_CHANNEL[1].TC_IDR = 0xffffffff;
 
     // Clear timer status register by reading it.
     timer_status = TC0->TC_CHANNEL[1].TC_SR;

     // Set capture mode (CMR).
     // TC_CMR_TCCLKS_TIMER_CLOCK1: run clock at MCLK/2 (84 MHz/2 = 42 MHz)
     // TC_CMR_CPCTRG: compare trigger mode
     TC0->TC_CHANNEL[1].TC_CMR = TC_CMR_CPCTRG | TC_CMR_TCCLKS_TIMER_CLOCK1;

     // Set compare trigger value.
     // We want triggers with 10 kHz frequency. MCLK/2 = 42 MHz.
     // Thus, we need a compare value of 4200.
     TC0->TC_CHANNEL[1].TC_RC = (uint32_t) (((float) (F_MASTER)/2.0f) / 
					    F_ACCEL + 0.5f);

     // Configure and enable interrupt on RC compare (TC_IER_CPCS)
     // Note that in ASF, the second channel of TCO is also called TC1 
     // sometimes, which is not consistent with the data sheet.
     NVIC_EnableIRQ(TC1_IRQn);
     TC0->TC_CHANNEL[1].TC_IER = TC_IER_CPCS;
 
     // Enable clock (CLKEN).
     TC0->TC_CHANNEL[1].TC_CCR = TC_CCR_CLKEN;

     // Reset and start counter by software trigger (SWTRG).
     TC0->TC_CHANNEL[1].TC_CCR = TC_CCR_SWTRG;
}

/**
 * Timer interrupt firing with frequency F_SPEED.
 * This timer controls the motor speed by periodically adding n_speed to 
 * the phase accumulator and mapping the phase accumulator value
 * to a square wave signal fed to the step signal output.
 */
void TC0_Handler(void)
{
     volatile uint32_t timer_status;
 
     // Read timer status register to clear it.
     timer_status = TC0->TC_CHANNEL[0].TC_SR;

     // Add to DDS phase accumulator.
     n += n_speed;

     if (n_speed >= 0) {
	  PORT_STEPPER_DIR->PIO_SODR = PIN_STEPPER_DIR;
     } else {
	  PORT_STEPPER_DIR->PIO_CODR = PIN_STEPPER_DIR;
     }

     // Map phase accumulator to square wave signel fed to step signal output.
     // Mapping is done by checking for an overflow of the phase accumulator n.
     // Every time n overflows, invert the step signal.
     if (n & N) {
	  PORT_STEPPER_STEP->PIO_SODR = PIN_STEPPER_STEP;
     } else {
	  PORT_STEPPER_STEP->PIO_CODR = PIN_STEPPER_STEP;
     }
}

/**
 * Timer interrupt firing with frequency F_ACCEL.
 * This timer controls the motor acceleration by periodically adding n_accel
 * to the phase accumulator.
 */
void TC1_Handler(void)
{
     volatile uint32_t timer_status;
 
     // Read timer status register to clear it
     timer_status = TC0->TC_CHANNEL[1].TC_SR;

     n_speed += n_accel;

     // Clamp to maximum speed
     if (n_speed > n_speed_max) {
	  n_speed = n_speed_max;
     } else if (n_speed < -n_speed_max) {
	  n_speed = -n_speed_max;
     }
}

/**
 * Setup SPI in slave mode.
 */
void setup_spi()
{
     // Enable peripheral clock for SPI through Power Management Controller 
     // (PMC).
     PMC->PMC_PCER0 = (1UL << ID_SPI0);    

     // Set SPI pins to peripheral (SPI) function.
     PORT_SPI->PIO_PDR = PIN_MISO | PIN_MOSI | PIN_SPCK | PIN_SS;

     // Disable SPI while configuring using Control Register (CR).
     SPI0->SPI_CR = SPI_CR_SPIDIS;
     
     // Define SPI mode (slave) and SS pin connection via Mode Register (MR)

     // Slave mode.
     SPI0->SPI_MR &= ~SPI_MR_MSTR;
     // Chip select pin directly connected to peripheral device.
     SPI0->SPI_MR &= ~SPI_MR_PCSDEC;
     // Mode fault detection disabled.
     SPI0->SPI_MR |= SPI_MR_MODFDIS;
     // Define peripheral select pin.
     SPI0->SPI_MR &= ~SPI_MR_PCS_Msk;
     switch (PIN_SS) {
     case PIO_PA28:
	  // NPCS0 is P.A28.
	  SPI0->SPI_MR |= SPI_MR_PCS(0);
	  break;
     case PIO_PA29:
	  // NPCS1 is P.A29.
	  SPI0->SPI_MR |= SPI_MR_PCS(1);
	  break;
     default:
	  // The other two NPCS pins are not available on Arduino Due.
	  die();
     }

     // Set transfer mode via Chip Select Register (CSR). 

     // SPI mode (0,0):
     // * Clock polarity (CPOL = 0): The inactive (idle) state value of SCLK is
     //   logic level zero (low).
     // * Clock phase (CPHA = 0): Data is captured on the leading/first edge 
     //   (i.e., the rising edge for CPOL = 0) of SPCK and changed on the 
     //   following edge of SPCK.
     SPI0->SPI_CSR[0] &= ~SPI_CSR_CPOL;
     SPI0->SPI_CSR[0] |= SPI_CSR_NCPHA;
     // 8 bit per transfer.
     SPI0->SPI_CSR[0] &= ~SPI_CSR_BITS_Msk;
     SPI0->SPI_CSR[0] |= SPI_CSR_BITS_8_BIT;

     // Enable SPI.
     SPI0->SPI_CR = SPI_CR_SPIEN;

     // Read Status Register (SR) once to clear status flags.
     volatile uint32_t dummy = SPI0->SPI_SR;
}

/**
 * Converts a frame received via SPI to omegadot value.
 *
 * @param frame the received frame. frame[0] is the first byte received.
 * @return omegadot value
 */
static float frame_to_omegadot(uint8_t frame[5])
{
     // Data is transferred in Big Endian order.
     // frame[0] contains highest order bits.
     uint32_t number;
     number = ((uint32_t) (frame[0]&0x7f)) << 25;
     number |= ((uint32_t) (frame[1])) << 18;
     number |= ((uint32_t) (frame[2])) << 11;
     number |= ((uint32_t) (frame[3])) << 4;
     number |= ((uint32_t) (frame[4]&0x78)) >> 3;

     float *omegadot = (float *) &number;
     
     return *omegadot;
}

/**
 * Converts omega value to a frame for sending to master via SPI.
 *
 * @param omega omega value to be sent.
 * @param frame frame to be sent.
 */
static void omega_to_frame(float omega, uint8_t frame[4])
{
     // Send in Big Endian byte order.
     uint32_t *number = (uint32_t *) &omega;
     frame[0] = (*number)>>24;
     frame[1] = ((*number)>>16)&0xff;
     frame[2] = ((*number)>>8)&0xff;
     frame[3] = (*number)&0xff;
}
 
int main()
{
     n_speed_max = omega_to_n_speed(MAX_OMEGA);

     // Initially, do not rotate or accelerate.
     n_speed = omega_to_n_speed(0.0f);
     n_accel = omegadot_to_n_accel(0.0f);

     // CMSIS: initialize system (clock, etc.)
     SystemInit();

     // Disable watchdog.
     WDT->WDT_MR = WDT_MR_WDDIS;

     // Setup GPIOs and peripherals.
     setup_io();
     setup_led();

     setup_spi();

     // Setup timer counter for motor speed control.
     setup_speed_timer();

     // Setup timer counter for motor acceleration control.
     setup_acceleration_timer();

     int ledon = 0;
     led_off();
     uint32_t spi_status;
     uint8_t bufferin[5];
     uint8_t bufferout[4];
     float omega = n_speed_to_omega(n_speed);
     omega_to_frame(omega, bufferout);
     unsigned int ibuffer = 0;
     while (1) {
	  // Busy waiting for new byte from SPI master.
	  // Receive Data Register Full (RDRF) flag signals data availability.
	  while ( ((spi_status = SPI0->SPI_SR)&SPI_SR_RDRF) != SPI_SR_RDRF);

	  // Copy data from Receive Data Register (RDR).
	  uint32_t data = SPI0->SPI_RDR;

	  // One transfer operations transmits 8 bits.
	  if (data&0x80) {
	       // MSB set -> start of frame.
	       ibuffer = 0;
	  } else {
	       ibuffer++;
	  }
	  bufferin[ibuffer] = data&0xff;
	  if (ibuffer == 4) {
	       // Frame of 5 bytes complete. Update acceleration.

	       if (bufferin[4]&0x01) {
		    // Stop flag set.
		    n_speed = 0;
		    n_accel = 0;
	       } else {
		    float omegadot = frame_to_omegadot(bufferin);
		    int32_t n_accel_temp = omegadot_to_n_accel(omegadot);
		    // Storing a 32 bit value is an atomic operation 
		    // (reference?) 
		    // Disabling interrupts to enforce atomic access leads to 
		    // disruptive movements.
		    //__disable_irq();
		    n_accel = n_accel_temp;
		    //__enable_irq();

		    // Prepare buffer for sending current angular speed (omega)
		    // to master. This buffer will be transmitted while
		    // receiving the next frame.
		    omega = n_speed_to_omega(n_speed);
		    omega_to_frame(omega, bufferout);
	       }

	       // The LED blinks, whenever a new acceleration value is received.
	       /*
	       if (ledon) {
		    ledon = 0;
		    led_off();
	       } else {
		    ledon = 1;
		    led_on();
	       }
	       */
	  } else {
	       // iBuffer < 4.
	       // Set SPI Transmit Data Register (TDR) to transmit another byte
	       // of the current speed with next transmission.
	       SPI0->SPI_TDR = bufferout[ibuffer];
	       // Wait until data written to the TDR is transferred to the
	       // serializer.
	       //while ( (SPI0->SPI_SR&SPI_SR_TDRE) != SPI_SR_TDRE);
	  }
     }
}
