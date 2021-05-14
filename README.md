# ModularArticulation
Master Thesis's Repository

The industry tends to increasingly automate as many processes as possible, and to make this possible, they often resort to the use of robotic arms. So, this work has as main goal to develop a proposal for a modular joint for robotic arms that allows: to obtain the best possible torque/weight ratio; to be controlled in speed and/or position; to communicate with other joints and external microcontrollers; to keep the cost as low as possible; and to be easily reconfigurable.

<p align="center">
  <img src="https://raw.githubusercontent.com/Marco201604163/ModularArticulation/main/Images/CompleteGearbox.jpg" width="360">
</p>


The joint's development started from an initial concept for its operation, according to which the entire system architecture was developed. So, the general architecture can be seen explicitly in the figure below.

<p align="center">
  <img src="https://raw.githubusercontent.com/Marco201604163/ModularArticulation/main/Images/SystemArch.jpg" width="360">
</p>

This system has an ATmega328P as its operating brain. This micro receives through Serial Communication, using RS-485 as a physical layer, commands that represent the control that must be carried out. This control can be done in speed and/or position. When these commands are received, they have information about the references used for control.
Consequently, since the microcontroller carries out all the processing, it receives the data from the AS5600 magnetic encoder via I2C, which allows it to be aware of the speed and position of the joint. With that, it communicates with the motor driver BTS7960 through PWM inducing the proper movement of the motor, whis is a Surpass Hobby 550-35T.

In addition, in order to guarantee the motor's safe operation, a DS18B20 temperature sensor was added, which communicates the motor's temperature to the microcontroller through the 1-Wire protocol. The supply voltage is monitored using a resistive divider and the microcontroller's analog-to-digital converter (ADC). In order to monitor current, a current sensor that outputs a voltage proportional to the current is used, and this value is read through the ADC converter, more specifically a ACS712-30A.

A 12V DC Power Supply is used to power up the system. This way, only 4 wires are needed to make the joint work: 2 for communication and 2 for power supply.

The first version already is implemented and functional, as can be seen in the Figure below:

<p align="center">
  <img src="https://raw.githubusercontent.com/Marco201604163/ModularArticulation/main/Images/PrintedBox.jpeg" width="360">
</p>

The repository as the follow organization:
- Images: This folder only has project images.
- Master-RS485-PC: This folder has the Library needed to send commands to the microcontroller from any machine that runs Python. Aditionally also has an Example that explains how to send them.
- Slave-RS485: This folder has the Library needed to carry communication and to execute the control routines on the microcontroller. Aditionally, it has an Example.

Note: This library is developed in order to use a USB to RS485 Converter on the Master Side, and a MAX485CSA module on the Slave Side. It might work with other hardware, however it is not tested.
