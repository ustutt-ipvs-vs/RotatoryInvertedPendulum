# The Rotatory Inverted Pendulum

The inverted pendulum is a text book example for implementing a control system. The goal is to keep the unstable inverted pendulum in an upright position by accelerating the center of the pendulum. Without acceleration, the pendulum will fall over. 

Here, we implement a rotatory inverted pendulum, where the pendulum is mounted on a base that can be rotated by a motor around an axis. Rotating the base will accelerate the center of the pendulum tangentially. A PID controller controls the angular acceleration of the rotation to control the angle of the instable inverted pendulum. The rotatory inverted pendulum avoids the problem of a finite path (e.g., a straight rail) to move the center of the pendulum. The following video shows the pendulum balanced through accelerating the rotation:

[Video of Rotatory Pendulum](https://youtu.be/wQY65LdlwGU) 

The angle of the pendulum is measured by a wireless IMU from the [BlueIMU project](https://github.com/duerrfk/BlueIMU). The IMU is connected via Bluetooth to a Raspberry Pi executing the PID controller to control the motor acceleration. As motor, we use a stepper motor controlled through an ARM Cortex M3 microcontroller (Arduino Due, actually not using the Arduino platform, but using the Due "bare-metal"). The microcontroller is connected to the RPi via SPI. The PID controller on the RPi sends angular acceleration values to the microcontroller, which accelerates the rotatory pendulum around the axis. The microcontroller sends the current rotation speed to the PID controller on the RPi. 

The following images show our prototype of the rotatory inverted pendulum. On the left hand side, you see the stepper motor connected through a stepper motor driver to the Arduino Due to control the motor acceleration, and the Raspberry Pi implementing the PID controllers. On the right hand side, you see the rotatory inverted pendulum with the IMU mounted on the pendulum:

![Rotatory Inverted Pendulum](/images/rotatory_inverted_pendulum_2.jpg)

![Rotatory Inverted Pendulum](/images/rotatory_inverted_pendulum_1.jpg)

Two PID controllers control the angle of the pendulum and the the rotation speed of the pendulum around the axis, respectively. The setpoint for the rotation speed is zero, i.e., the base should stand still. The speed is indirectly controlled by the second PID controller by controlling the setpoint of the angle of the pendulum controlled by the first PID controller. Then, the first PID controller controlling the angle of the pendulum has to accelerate the rotation to keep the pendulum at the given angle, thus, changing the rotation speed.      

This repository contains the following material:

* PID controller software for Raspberry Pi (folder `pid_controller`).
* Stepper motor controller software for bare-metal Arduino Due (folder `stepper_controller_dds`).
 
# Connecting the Raspberry Pi to the Arduino Due

In the following, we assume a Raspberry Pi 2, but you can use any Raspberry Pi model.

The Raspberry Pi and the Arduino Due communicate through SPI. To enable SPI communication, the following pins need to be connected:
 
 * MISO (Master in / Slave out: #21 on RPi2, A.25 on Due)
 * MOSI (Master out / Slave in: #19 on RPi2, A.26 on Due)
 * SCLK (SPI clock: #23 on RPi2, A.27 on Due)
 * SS (Slave Select: CE0 on RPi2, A.28/NCPS0 on Due)
 * GND

Note that the RPi has two SS pins called CE0 and CE1, and the Arduino Due also has two (actually four, but two are not brought out on the Due) SS pins, namely, A.28 (NCPS0) and A.29 (NCPS1). We use CE0 and A.28/NCPS0.

For the Raspberry Pi 2, pins are numbered from 1 to 40. Pin 1 is the left-most inner pin, where "left" means the PCB side with the micro SD card slot. 

```
           ^
           | Side with Micro SD card slot      

         #1  #2
         #3  #4
         #5  #6 -> GND
         #7  #8 
         #9  #10
         #11 #12
         #13 #14
         #15 #16
         #17 #18
MOSI <-  #19 #20
MISO <-  #21 #22
SCLK <-  #23 #24   SS / CE0
         #25 #26
         #27 #28
         #29 #30
         #31 #32
         #33 #34
         #35 #36
         #37 #38
         #39 #40
```
 
On the Arduino Due, the SPI port is the 6 pin header to the right of the microcontroller when USB connectors are on the left side of the PCB. This pin header is labeled "SPI", and there is a small dot at the top/left pin. 

```
  <- side with USB ports

  MISO (pin marked with small dot) <-  P.A25 5V
  SCLK                             <-  P.A27 P.A26 -> MOSI
                                       RESET GND   -> GND
```

On Arduino Due, the SS/NCPS0 pin is the pin labeled "10" (not "A10"!).

We use SPI mode (0,0):

* Clock polarity (CPOL = 0): The inactive (idle) state value of SCLK is logic level zero (low).
* Clock phase (CPHA = 0): Data is captured on the leading/first edge (i.e., the rising edge for CPOL = 0) of SPCK and changed on the following edge of SPCK.

Moreover, master and slave transfer data as follows:

* 8 bits per transfer operation.
* Most significant bit transferred first.

# Connecting and Configuring the Stepper Motor Driver

We use an Allegro A4988 stepper motor driver mounted on a Pololu A4988 board, but any similar driver should be fine.

## Powering and Configuring the Stepper Motor Driver

The stepper motor is connected through four pins. Pins 1A and 1B drive one coil of the stepper motor, pins 2A and 2B the other coil.

The motor is powered through a separate power supply connected to VMOT and GND. This power supply should be able to supply at least 4 A at at least 16 V. A common laptop power supply should be fine.

Besides full stepping mode, the A4988 supports the following micro-stepping modes: 1/16, 1/8, 1/4, 1/2 stepping. The stepping mode is selected through 3 pins called MS1-3 according to the following table:

```
MS1     MS2     MS3     Microstep Resolution
--------------------------------------------
Low     Low     Low     Full step
High    Low     Low     Half step
Low     High    Low     Quarter step
High    High    Low     Eighth step
High    High    High    Sixteenth step
```

We use 1/16 stepping, although you can change the stepping mode in the software.

## Connecting the Arduino Due to the Stepper Controller

To connect the A4988 stepper motor driver to the Arduino Due, besides the GND pin, two pins are required to control the direction and stepping of the motor. As direction and step pins, we use pin P.C23 (pin #7) and P.C24 (pin #6), respectively. However, you can change the pins in the software.

Moreover, the A4988 requires a power supply for the logic part besides the power supply for driving the stepper motor. The pins for the logic power supply are called VDD and GND, and are connected to the 3.3 V and GND pins of the Arduino Due. DON'T connect the power from the Arduino Due to the motor power supply (pin VMOT), but use a separate power supply for the motor instead!

# Motor Controller Software for Arduino Due

In order to influence the orientation of the inverted pendulum, we need to accelerate the center of the pendulum by accelerating the rotation of the axis. To this end, the stepper motor controller running on the Arduino Due only has one input parameter: omegadot (floating point number) defining the change of the angular frequency omega = 2*pi*f of the rotating axis over time. 

The motor controller software receives omegadot from the PID controller executed on the Raspberry Pi via SPI and then uses the direct digital synthesis (DDS) method to generate the stepper motor stepping signal.

In more detail, an input value (n_speed) corresponding to the motor speed is periodically added to a counter (phase accumulator; n) using a timer with fixed frequency. The phase value is then mapped to a square wave signal for the step output by inverting the step output whenever the accumulator overflows a certain threshold (N).

The input value n_speed modeling the motor speed is periodically increased or decreased by n_accel using another fixed period timer to accelerate or decelerate the motor.

omegadot is received via SPI by the Arduino Due (SPI slave) as a single single-precision IEEE 754 (32 bit) floating point number from the RPi (SPI master). In each transmission, 8 bits are transferred over SPI. Therefore, we need to transfer omegadot as a stream of several bytes. A sequence of 5 bytes encodes one omegadot value, i.e., one 32 bit floating point number. The first byte of the sequence has the most significant bit (MSB) set, the other four bytes do not have the MSB set. The following sequence shows the bit pattern of one floating point number as transfered via SPI:

```
Byte 1   Byte 2   Byte 3   Byte 4   Byte 4
1SEEEEEE 0EEMMMMM 0MMMMMMM 0MMMMMMM 0MMMM00T
```

MSB set to 1 signals the start of a new number. S, E, and M are the bits of the 32 bit floating point number in Big Endian byte order, i.e., the first bit is the sign-bit (S) of the floating point number, followed by 8 bits of the exponent (E), and 23 bits mantissa (M). T is a stop flag. If T = 1, speed and acceleration are set to 0, i.e., the rotation stops. If T = 0, the given acceleration will be set.

Starting with the transmission of Byte 2, the Arduino Due sends the current angular speed as a single precision floating point number (32 bit = 4 byte; Big Endian byte order) to the master.

## Prerequisites

The Arduino Due is used "bare metal", i.e., without real-time operating system and without using the the Arduino IDE, but by programming the Atmel SAM3X8E (ARM Cortex M3) directly. Since we are not using the Arduino IDE, we need to install a tool chain first for compiling and linking the program. We use the GCC ARM embedded tool chain (tested with version gcc-arm-none-eabi-5_2-2015q4). Binaries are available from the following link:

https://launchpad.net/gcc-arm-embedded/+download

To facilitate programming, the software uses the Atmel Software Framework (ASF) (tested with version 3.29.0.41), available from the following link:

http://www.atmel.com/tools/AVRSOFTWAREFRAMEWORK.aspx

The BOSSA flash utility for Atmel's SAM family is used to flash the program onto the SAM3X8E. We use the BOSSA version shipped with the Arduino 1.5.8 IDE. After installing the Arduino 1.5.8 IDE, BOSSA can be found in folder `arduino-1.5.8/hardware/tools/`.

## Building the Software

To build the program, we use make. First, set the following variables in the `Makefile`:

* `CROSS`: path to embedded compiler tools, e.g., `/usr/local/gcc-arm-none-eabi-5_2-2015q4/bin/arm-none-eabi-` (`gcc` or similar will be added automatically to this path). 
* `ASF_ROOT`: directory where ASF has been installed.
* `OPTIMIZATION` to defined optimization level (e.g., `-O0` for no optimization). 

Then simply use make to compile and link:

```
$ make
```

## Flashing

To flash the program, first connect the Arduino Due through the programming USB port to the host (note: the Arduino Due has two USB ports; only one can be used for flashing!). Then, execute the following commands to flash the program:

```
$ stty -F /dev/ttyACM0 1200
$ bossac --port=ttyACM0 -U false -e -w -v -b stepperctrl.bin -R
```

Note that before calling bossac, we open a serial connection at 1200 baud. The Atmega16U2 connected to the programming port will detect this connection, issues a reset, and clears the SAM MCU flash for making the SAM MCU ready for flashing a new program (that's the reason for the programming port on the Arduino Due).

Explanation of BOSSAC options:

* `-e`: erase the entire flash
* `-w`: write FILE to the flash
* `-v`: verify FILE matches flash contents
* `-b`: boot from flash
* `-R`: reset
* `-U`: override USB port autodetection
* `--port=PORT`: use serial PORT to communicate with device

# Connecting the Raspberry Pi to the IMU

As IMU to measure the angle of the inverted pendulum, we use [BlueIMU](https://github.com/duerrfk/BlueIMU). BlueIMU communicates through Bluetooth (serial profile, SPP) with the Raspberry Pi. To this end, first install the required Bluetooth tools:

```
$ sudo aptitude install bluetooth bluez
```

Then, in order to pair the Raspberry Pi and the BlueIMU device, execute the following commands:
 
```
$ sudo bluetoothctl
[bluetooth] scan on
[CHG] Controller 00:1A:7D:DA:71:0B Discovering: yes
[NEW] Device 00:11:12:31:04:12 00-11-12-31-04-12
[bluetooth] scan off
[bluetooth] agent on
[bluetooth] pair 00:11:12:31:04:12
Attempting to pair with 00:11:12:31:04:12
[CHG] Device 00:11:12:31:04:12 Connected: yes
Request PIN code
[agent] Enter PIN code: 1234
[CHG] Device 00:11:12:31:04:12 UUIDs:
        00001101-0000-1000-8000-00805f9b34fb
[CHG] Device 00:11:12:31:04:12 Paired: yes
Pairing successful
[CHG] Device 00:11:12:31:04:12 Connected: no
[bluetooth]# quit
```

Now, in order to connect to the device:

```
$ sudo rfcomm connect /dev/rfcomm1 00:11:12:31:04:12 1
```

`/dev/rfcomm1` is the serial device to be used by the controller software on the Raspberry Pi.

# PID Controller Software for Raspberry Pi

In directory `pid_controller`, you will find the code for a PID controller. Actually, this program implements two PID controllers.

The first PID controller is controlling the angle of the inverted pendulum such that the pendulum should not fall over and remains at a given angle. The setpoint of the pendulum angle is controlled by the second PID controller. The current angle of the pendulum is measured by the IMU. 

The second PID controller controls the rotation speed of axis. The setpoint of this controller is 0 rad/s, i.e., the base should not move. The current speed of the base is transmitted by the motor controller from the Arduino Due. The output is the setpoint of the first PID controller, i.e., the angle of the pendulum. The first PID controller accelerates the rotation into the direction of the angle to keep the angle at the angle setpoint. Thus, the rotation speed can be controlled indirectly through setting pendulum angle.

Configuring the parameters of both PID controllers is a little tricky. A working set of parameters---possibly not an optimal one---is included in the source code in file `pid-controller`. In order to change the parameters, you have to edit the source code (maybe we will make these command line parameters at some time).

## Prerequisites

The software requires the BCM2835 library for SPI communication. BCM2835 can be downloaded from the following link:

http://www.airspayce.com/mikem/bcm2835/

To install in `/usr/local`, use the usual commands:

```
$ configure
$ make install
```

## Building and Using the PID Controller

To build, execute the following commands:

```
$ cd pid_controller
$ make
```

Then, connect the Raspberry Pi via Bluetooth to the IMU (detailed instructions about how to set up the Bluetooth device can be found above):

```
$ sudo rfcomm connect /dev/rfcomm1 00:11:12:31:04:12 1
```

Finally, start the controller:

```
$ sudo/pid-controller -b /dev/rfcomm1 -f 250000 -c 0

Parameters:

* `-f`: SPI frequency [Hz]
* `-c`: SPI channel corresponding to a slave select pin (0 = CE0; 1 = CE1). 
```

The controller program will stop the motor and terminate if the pendulum angle is greater than 90 degrees (then, there's no chance anymore to stop the pendulum from falling). Thus, before starting the controller, you must manually bring the pendulum in an upright position and hold it there until the controller takes over.

# Licenses and Acknowledgements

This software includes code developed as part of [BlueIMU project](https://github.com/duerrfk/BlueIMU) by Frank Duerr.

