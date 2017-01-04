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
 * - 2016-12-27, Frank Duerr (frank.duerr@ipvs.uni-stuttgart.de):
 *   Initial implementation.
 * - 2017-01-04, Frank Duerr (frank.duerr@ipvs.uni-stuttgart.de):
 *   Added PID controller for controlling rotation speed of pendulum base.
 */

#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <signal.h>
#include <stdint.h>
#include <math.h>
#include <bcm2835.h>
#include "kalmanfilter.h"
#include "pid_ctrl.h"

// Size of a frame transmitted over the Bluetooth serial connection.
#define FRAMESIZE 20

// Number of tolerated corrupt frames before the data stream is
// re-synchronized.
#define MAX_FRAME_ERR 1

// SPI core clock frequency.
// On RPi1 and RPi2, the core clock frequency is 250 MHz; on RPi3 it is 400 MHz.
#define CORE_SPI_CLOCK_FREQUENCY 250000000

// Gravitational acceleration, used to translate g values of accelerometer to
// acceleration in m/s**2.
const float grav_accel = 9.81f;

// Sampling period in seconds.
// Timestamps are sent together with samples. So manually defining the
// sampling period is not strictly required, but might be useful if
// recorded timestamps are thought to be inaccurate.
const float sampling_period = 0.005f;

// Standard deviations of the Gaussian distributions of angular measurements.
// The angle (phi) is only observed indirectly by measuring the 
// objects acceleration, assuming that acceleration is only due to gravity 
// and thus pointing towards the center of the earth. Obviously, this 
// assumption does not hold if the object is accelerated by other forces in 
// addition to gravity. 
// 0.004 is the standard deviation of an object at rest calculated from sample 
// measurements.
const float sigma_phi = 0.004f;

// Standard deviations of the Gaussian distributions of angular velocity
// measurements. Angular velocity is measured directly by the gyroscope. 
// 0.003 is the standard deviation calculated from sample measurements.
const float sigma_phidot = 0.003f;

// Standard deviation of the Gaussian distribution modeling uncontrolled
// angular acceleration. This noise depends on the external forces (e.g.,
// due to motors, wind, etc.) accelerating the object around its axes.  
const float sigma_angularaccel = 50.0/360.0 * 2.0*M_PI;

// Standard deviation of the Gaussian distribution modeling uncontrolled
// change of gyro bias. The Kalman filter can automatically estimate the gyro
// bias. However, you can set sigma_bias to 0 to switch off automatic bias 
// estimation (bias = 0). Typically, this value should be very small. 
const float sigma_bias = 0.000001f;
//const float sigma_bias = 0.0f;

// The sensitivity and value range of the accelerometer as defined by the
// following table (must match the configuration of the IMU):
//
// AFS_SEL | Full Scale Range | LSB Sensitivity
// --------+------------------+----------------
// 0       | +/- 2g           | 16384 LSB/g
// 1       | +/- 4g           | 8192 LSB/g
// 2       | +/- 8g           | 4096 LSB/g
// 3       | +/- 16g          | 2048 LSB/g
const float lsb_per_g = 8192.0f;

// The sensitivity and value range of the gyroscope as defined by the
// following table (must match the configuration of the IMU):
//
// FS_SEL | Full Scale Range   | LSB Sensitivity
// -------+--------------------+----------------
// 0      | +/- 250 degrees/s  | 131 LSB/deg/s
// 1      | +/- 500 degrees/s  | 65.5 LSB/deg/s
// 2      | +/- 1000 degrees/s | 32.8 LSB/deg/s
// 3      | +/- 2000 degrees/s | 16.4 LSB/deg/s
const float lsb_per_degs = 65.5f;

enum States {synced, unsynced1, unsynced2} state;

struct Sample {
     // Acceleration [m/s**2].
     float accel_x; 
     float accel_y; 
     float accel_z;
     // Angular velocity [rad/s].
     float gyro_x;
     float gyro_y;
     float gyro_z;
     // Timestamp in milliseconds since IMU was booted.
     uint32_t timestamp;
};

// Start of frame pattern.
const uint8_t start_of_frame[] = {0x5A, 0xA5};

// Static roll offset due to mounting errors.
const float roll_offset = 0.023f;
//const float roll_offset = 0.0f;

// Maximum angular acceleration [rad/s^2].
const float max_omegadot = 50.0 * 2.0*M_PI;

// Settings of PID controllers.
const float pid_ctrl_roll_kp = 1000.0f;
const float pid_ctrl_roll_kd = 20000.0f;
const float pid_ctrl_roll_ki = 1.0f;

/* A working set of parameters:
const float pid_ctrl_roll_kp = 1000.0f;
const float pid_ctrl_roll_kd = 20000.0f;
const float pid_ctrl_roll_ki = 1.0f;
*/

const float pid_ctrl_speed_kp = 2.0f;
const float pid_ctrl_speed_kd = 0.5f;
const float pid_ctrl_speed_ki = 1.0f;
// In order to adjust the base (rotation) speed, the pendulum might be rolled
// by this maximum angle.
const float max_roll = (3.0/360) * 2.0*M_PI;

/* A working set of parameters:
const float pid_ctrl_speed_kp = 2.0f;
const float pid_ctrl_speed_kd = 0.5f;
const float pid_ctrl_speed_ki = 1.0f;
// In order to adjust the base (rotation) speed, the pendulum might be rolled
// by this maximum angle.
const float max_roll = (3.0/360) * 2.0*M_PI;
*/

int serial = -1;

bool is_spi_open = false;

bool is_first_update = true;
uint32_t t_last_update;

/**
 * Send a stop signal to the motor controller.
 */
void send_stop_signal()
{
     uint8_t data;
     // Set MSB on first byte to signal start of frame.
     data = 0x80;
     bcm2835_spi_transfer(data);

     data = 0x00;
     bcm2835_spi_transfer(data);
     bcm2835_spi_transfer(data);
     bcm2835_spi_transfer(data);

     // Set LSB on last byte of frame to signal stop request.
     data = 0x01;
     bcm2835_spi_transfer(data);
}

/**
 * Gracefully exit the application.
 */
void die(int exitcode)
{
     if (serial != -1)
	  close(serial);

     if (is_spi_open) {
	  send_stop_signal();
	  bcm2835_spi_end();
	  bcm2835_close();
     }
	  
     exit(exitcode);
}

/**
 * SIGINT handler.
 */
void sig_int(int signo)
{
     die(0);
}

/**
 * Calculation of 16 bit checksum. 
 * 
 * This implementation uses the same algorithm as IP to calculate
 * IP header checksums. 
 * 
 * Performing the same calculation over the data *and*
 * checksum should yield 0 in case of no error.
 * 
 * The following code is adapted from Gary R. Wright, W. Richard Stevens: 
 * TCP/IP Illustrated, Volume 2 (there called a "naive implementation"; 
 * you can also find an optimized implementation there).
 */
uint16_t checksum(const uint8_t *data, size_t len)
{
     // Note that we do not need to consider byte order. From RFC 1071: 
     // "The sum of 16-bit integers can be computed in either byte order."

     uint32_t sum = 0;
     uint16_t *words = (uint16_t *) data;

     while (len > 1) {
         sum += *(words)++;
         if (sum&0x80000000)
             sum = (sum&0xFFFF) + (sum>>16);
         len -= 2;
     }

     if (len)
         sum += *((uint8_t *) words);
     
     while (sum>>16)
         sum = (sum & 0xFFFF) + (sum >> 16);

     return ~sum;
}

/**
 * Print usage information.
 */
void usage(const char *appl)
{
     fprintf(stderr, "%s -b BLUEIMU_SERIAL_DEVICE -f SPI_FREQUENCY -c SPI_CHANNEL \n",
	     appl);
}

void setup_serial(int fd)
{
     struct termios tty;
     memset(&tty, 0, sizeof tty);
     if (tcgetattr(fd, &tty) != 0) {
          perror("Could not get terminal attributes");
          die(-1);
     }
     
     // BlueIMU uses 230400 baud/s.
     cfsetospeed(&tty, B230400);
     cfsetispeed(&tty, B230400);

     tty.c_cflag &= ~CSIZE;   // 8-bit characters
     tty.c_cflag |= CS8;    
     tty.c_cflag &= ~CSTOPB;  // one stop bit (= no two stop bits)
     tty.c_cflag &= ~PARENB;  // no parity check
     tty.c_cflag |= CLOCAL;   // ignore modem control lines
     tty.c_cflag |= CREAD;    // enable receiver
     tty.c_cflag &= ~CRTSCTS; // disable hardware flow control

     // Turn off software flow control
     tty.c_iflag &= ~(IXON | IXOFF | IXANY);

     tty.c_iflag |= IGNBRK;  // no break processing
     tty.c_iflag &= ~IGNCR;  // don't discard CR characters
     tty.c_iflag &= ~INLCR;  // don't translate NL to CR
     tty.c_iflag &= ~ISTRIP; // don't strip off 8th bit
     tty.c_iflag &= ~ICRNL;  // don't translate CR to NL
     tty.c_oflag = 0;        // Turn off any output processing

     // Set non-canonical input mode (raw; non-line-oriented)
     tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

     tty.c_cc[VMIN] = 1;  // read blocks if there is not at least one byte
     tty.c_cc[VTIME] = 0; // no read timeout

     if (tcsetattr(fd, TCSANOW, &tty) != 0) {
          perror("Could not set terminal attributes");
          die(-1);
     }

     // Clear buffers.
     // Flush only seems to work with some sleep time -- possibly a USB 
     // problem, see:
     // http://stackoverflow.com/questions/13013387/clearing-the-serial-ports-buffer
     sleep(2);  
     tcflush(fd,TCIOFLUSH);
}

/**
 * Parse a sample transmitted in a frame.
 *
 * @param frame the frame containing the sample.
 * @param sample pointer to the allocated sample data structure.
 */
void parse_sample(uint8_t *frame, struct Sample *sample)
{
     // All values are in Big Endian byte order.
     int16_t lsb_accel_x = (((int16_t) frame[2])<<8) | frame[3];
     int16_t lsb_accel_y = (((int16_t) frame[4])<<8) | frame[5];
     int16_t lsb_accel_z = (((int16_t) frame[6])<<8) | frame[7];

     int16_t lsb_gyro_x = (((int16_t) frame[8])<<8) | frame[9];
     int16_t lsb_gyro_y = (((int16_t) frame[10])<<8) | frame[11];
     int16_t lsb_gyro_z = (((int16_t) frame[12])<<8) | frame[13];

     // Translate raw ADC readings to meaningful acceleration and 
     // angular velocity values in m/s**2 and rad/s, respectively.
     sample->accel_x = lsb_accel_x/lsb_per_g*grav_accel;
     sample->accel_y = lsb_accel_y/lsb_per_g*grav_accel;
     sample->accel_z = lsb_accel_z/lsb_per_g*grav_accel;

     sample->gyro_x = lsb_gyro_x/lsb_per_degs/360.0f*2.0f*M_PI;
     sample->gyro_y = lsb_gyro_y/lsb_per_degs/360.0f*2.0f*M_PI;
     sample->gyro_z = lsb_gyro_z/lsb_per_degs/360.0f*2.0f*M_PI;

     sample->timestamp = (((uint32_t) frame[14])<<24) |
	  (((uint32_t) frame[15])<<16) |
	  (((uint32_t) frame[16])<<8) |
	  (((uint32_t) frame[17]));

     // Default mounting orientation of the IMU is as follows:
     // 
     // * roll: around x axis
     // * pitch: around y axis
     // * yaw: around z axis
     // 
     // For roll = pitch = 0, the positive z axis is pointing away from the 
     // center of the earth.
     //
     // If the IMU is mounted differently, we need to rotate the axes.
     // The code below rotates the axes as follows: 
     //
     // * roll: around z axis
     // * pitch: around y axis
     // * yaw: around x axis
     //
     // For roll = pitch = 0, the positive x axis is pointing towards the 
     // center of the earth. 
     float tempX = sample->accel_x;
     float tempY = sample->accel_y;
     float tempZ = sample->accel_z;
     sample->accel_x = tempZ;
     sample->accel_y = tempY;
     sample->accel_z = -tempX;
     tempX = sample->gyro_x;
     tempY = sample->gyro_y;
     tempZ = sample->gyro_z;
     sample->gyro_x = tempZ;
     sample->gyro_y = tempY;
     sample->gyro_z = -tempX;
}

/**
 * Update a Kalman filter given a measurement.
 *
 * @param filter pointer to Kalman filter to be updated. 
 * @param phi measured angle [rad].
 * @param phidot measured angular velocity [rad/s].
 * @param t_last_update time of last update [ms].
 * @param t_sample time of sample [ms].
 */
void kalmanfilter_update(struct kf *filter, float phi, float phidot,
			 uint32_t t_last_update, uint32_t t_sample)
{
     float t_delta;
     if (t_last_update <= t_sample) {
	  t_delta = (float) (t_sample-t_last_update)/1000.0f;
     } else {
	  // wrap around
	  t_delta = (float) ((0xffffffffUL-t_last_update)+t_sample)/1000.0f;
     }

     // Use sampling period calculated from sample timestamps sent by BlueIMU.
     kf_update(filter, phi, phidot, t_delta);
     // Alternatively, we can use a pre-defined fixed sampling period.
     //kf_update(filter, phi, phidot, sampling_period);
}

/**
 * Calculate roll angle (rotation around x axis) from measured acceleration. 
 * We assume that acceleration is just gravity and, therefore, the acceleration 
 * vector points towards the center of the earth. 
 *
 * @param accel_y acceleration along Y axis of the IMU.
 * @param accel_z acceleration along Z axis of the IMU.
 * @return roll angle [rad]
 */
float roll_from_accel(float accel_y, float accel_z)
{
     return atan2f(accel_y, accel_z);
}

/**
 * Calculate pitch angle (rotation around y axis) from measured acceleration. 
 * We assume that acceleration is just gravity and, therefore, the acceleration 
 * vector points towards the center of the earth. 
 *
 * @param accel_x acceleration along X axis of the IMU.
 * @param accel_z acceleration along Z axis of the IMU.
 * @return pitch angle [rad]
 */
float pitch_from_accel(float accel_x, float accel_z)
{
     return -atan2f(accel_x, accel_z);
}

void omegadot_to_frame(float omegadot, uint8_t frame[5])
{    
     uint32_t *number = (uint32_t *) &omegadot;

     // On the first byte, set the MSB.
     frame[0] = ((*number>>25)&0x7f) | 0x80;
     // On all other bytes, clear the MSB.
     frame[1] = (*number>>18) & 0x7f;
     frame[2] = (*number>>11) & 0x7f;
     frame[3] = (*number>>4) & 0x7f;
     frame[4] = (*number&0x07)<<4;
}

/**
 * Send acceleration (omegadot) to slave via SPI and simultaneously receive current
 * angular speed (omega) from slave.
 *
 * @param omegadot angular acceleration (omegadot) to be send.
 * @param omega received angular speed (omega).
 */
void transfer_omegadot_omega(float omegadot, float *omega)
{
     uint8_t buffer[5];
     omegadot_to_frame(omegadot, buffer);
     uint32_t *number = (uint32_t *) omega;
     *number = 0;
     for (unsigned int i = 0; i < 5; i++) {
	  uint8_t r = bcm2835_spi_transfer(buffer[i]);
	  // Starting with the transfer of the second byte, the slave transmits the
	  // current angular speed to the master (32 bit float; Big Endian byte order).
	  if (i > 0) {
	       *number = (((*number)<<8) | r);
	  }
	  // If slave skips bytes, insert a wait statement here to
	  // give slave time to process each byte.
	  // At least at 250 kHz SPI clock frequency, everything works well.
	  //usleep(1000);
     }
}

int main(int argc, char *argv[])
{
     // Parse command line arguments.
     char *bluetooth_device_arg = NULL;
     char *spi_frequency_arg = NULL;
     char *spi_channel_arg = NULL;
     int c;
     while ((c = getopt(argc, argv, "b:f:c:")) != -1) {
          switch (c) {
          case 'b' :
               bluetooth_device_arg = malloc(strlen(optarg)+1);
               strcpy(bluetooth_device_arg, optarg);
               break;
	  case 'f' :
	       spi_frequency_arg = malloc(strlen(optarg)+1);
	       strcpy(spi_frequency_arg, optarg);
	       break;
	  case 'c' :
	       spi_channel_arg = malloc(strlen(optarg)+1);
	       strcpy(spi_channel_arg, optarg);
	       break;
	  }
     }
	  
     if (bluetooth_device_arg == NULL ||
	 spi_frequency_arg == NULL ||
	 spi_channel_arg == NULL ) {
	  usage(argv[0]);
	  die(-1);
     }

     unsigned int spi_frequency = atoi(spi_frequency_arg);
     unsigned int spi_channel = atoi(spi_channel_arg);
	  
     // Init SPI.
     
     bcm2835_init();
     bcm2835_spi_begin();
     
     if (spi_channel == 0)
	  bcm2835_spi_chipSelect(BCM2835_SPI_CS0);
     else
	  bcm2835_spi_chipSelect(BCM2835_SPI_CS1);

     // Set divider according to requested SPI frequency.
     uint16_t divider = (uint16_t) ((double) CORE_SPI_CLOCK_FREQUENCY/spi_frequency + 0.5);
     bcm2835_spi_setClockDivider(divider);
	  
     // SPI mode (0,0):
     // * Clock polarity: The inactive state value of SCLK is logic level zero/low.
     // * Clock phase: Data is changed on the leading/rising edge of SLCK and captured on
     //   the following falling edge of SLCK.
     bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);

     bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);

     bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);

     is_spi_open = true;
     
     // Kalman filters (later initialized after first sample).

     struct kf kf_roll;

     // PID controllers.

     // PID controller for controlling angle of pendulum.
     struct pid_ctrl pid_ctrl_roll;
     pid_ctrl_roll.kp = pid_ctrl_roll_kp;
     pid_ctrl_roll.kd = pid_ctrl_roll_kd;
     pid_ctrl_roll.ki = pid_ctrl_roll_ki;
     pid_ctrl_roll.setpoint = 0.0f;
     pid_ctrl_roll.out_clamp = max_omegadot;
     pid_ctrl_roll.i_clamp = max_omegadot;
     pid_ctrl_roll.e_i = 0.0f;
     pid_ctrl_roll.e_previous = 0.0f;
     
     // PID controller for controlling speed of pendulum base.
     struct pid_ctrl pid_ctrl_speed;
     pid_ctrl_speed.kp = pid_ctrl_speed_kp;
     pid_ctrl_speed.kd = pid_ctrl_speed_kd;
     pid_ctrl_speed.ki = pid_ctrl_speed_ki;
     // We strive for zero speed of the pendulum base (no rotation).
     pid_ctrl_speed.setpoint = 0.0f;
     pid_ctrl_speed.out_clamp = max_roll;
     pid_ctrl_speed.i_clamp = max_roll;
     pid_ctrl_speed.e_i = 0.0f;
     pid_ctrl_speed.e_previous = 0.0f;
     
     // Open and configure serial device.

     if ((serial = open(bluetooth_device_arg, O_RDWR | O_NOCTTY)) == -1) {
          perror("Could not open serial device");
          die(-1);
     }
     setup_serial(serial);
     
     // Install signal handler for SIGINT for graceful termination.

     if (signal(SIGINT, sig_int) == SIG_ERR) {
          perror("Could not set signal handler: SIGINT");
          die(-1);
     }

     /*
     float omegadot = 1.0 * 2.0*M_PI;
     while (1) {
	  send_omegadot(omegadot);
	  sleep(1);
     }
     */
     
     state = unsynced1;
     uint8_t buffer[FRAMESIZE];
     size_t nread, n;
     unsigned int nframeerr;
     struct Sample sample;
     while (1) {
	  switch (state) {
	  case unsynced1:
	       if ((n = read(serial, &buffer[0], 1)) == -1) {
		   perror("Error reading from serial port");
		   die(-1);
	       }
	       if (n == 1 && buffer[0] == start_of_frame[0])
		    state = unsynced2;
	       break;
	  case unsynced2:
	       if ((n = read(serial, &buffer[1], 1)) == -1) {
		    perror("Error reading from serial port");
		    die(-1);
	       }	    
	       if (n == 1 && buffer[1] == start_of_frame[1]) {
		    state = synced;
		    nread = 2;
		    nframeerr = 0;
	       }
	       break;
	  case synced:
	       if ((n = read(serial, &buffer[nread], FRAMESIZE-nread)) == -1) {
		    perror("Error reading from serial port");
		    die(-1);
	       }
	       nread += n;
	       if (nread == FRAMESIZE) {
		    // Complete frame read.
		    if (checksum(buffer, FRAMESIZE) != 0) {
			 // Bad frame.
			 nframeerr++;
			 if (nframeerr > MAX_FRAME_ERR) {
			      // Too many corrupt frames -> resync.
			      state = unsynced1;
			 }
		    } else {
			 // Frame OK.
			 parse_sample(buffer, &sample);
			 float roll = roll_from_accel(sample.accel_y,
			      sample.accel_z);
			 if (is_first_update) {
			      kf_init(&kf_roll, roll, sample.gyro_x, 0.0f, 
				      sigma_phi, sigma_phidot, 
				      sigma_angularaccel, sigma_bias);
			      is_first_update = false;
			 } else {
			     kalmanfilter_update(&kf_roll, roll, 
			         sample.gyro_x, t_last_update, 
			         sample.timestamp);
			 }
			 t_last_update = sample.timestamp;
			 nframeerr = 0;

			 // We now have a new sample with the following values:
			 // * Timestamp: sample.timestamp
			 // * Angle from Kalman filter [radian]: kf_roll.x[0]
			 // * Angular velocity from Kalman filter [radian/s]: kf_roll.x[1]
			 // * Gyro bias from Kalman filter [radian/s]: kf_roll.x[2]

			 // Get new acceleration from PID controller controlling pendulum
			 // angle (roll).

			 float r = kf_roll.x[0]-roll_offset;
			 float omegadot = pid_ctrl_control(&pid_ctrl_roll, r);

			 // Stop motor if angle is greater than 90 degrees and we cannot
			 // stop the pendulum from falling.
			 if (r > M_PI/2.0 || r < -M_PI/2)
			      die(0);
			 
			 // Send acceleration (omegadot) to motor controller.
			 // Receive current speed (omega) from motor controller.

			 float omega;
			 transfer_omegadot_omega(omegadot, &omega);

			 float roll_setpoint = pid_ctrl_control(&pid_ctrl_speed, omega);
			 pid_ctrl_roll.setpoint = -roll_setpoint;

			 /*
			 printf("%f\t%f\t%f\t%f\n", kf_roll.x[0], omegadot, omega,
			      roll_setpoint);
			 */
		    }
		    nread = 0;
	       }
	       break;
	  }
     }
}
