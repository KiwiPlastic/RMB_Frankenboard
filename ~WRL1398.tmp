# RMB_Frankenboard
RMB-Frankenboard developed By RICHRD NICHOLSON from NEW ZEALAND
A Team.C.R.A.P Project

## Licensing
Non-commercial license is in effect. This license flows to derivative works.
You are free to remix / modify the blaster as you see fit. Please share
If you post a remix, please link back to the original
Commercial use is allowed with the purchase of a RMB-Frankenboard at a ratio of one board = one commercial build

## Attributions
(c) 2019-2021 Michael Ireland / Ireland Software / Airzone for example code used early on in project development work
BLE Server examples
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleNotify.cpp
    Ported to Arduino ESP32 by Evandro Copercini
    updated by chegewara and MoThunderz

## License
RMB-Frankenboard
by Team C.R.A.P is licensed under the Creative Commons - Attribution - Non Commercial Licence
https://creativecommons.org/licenses/by-nc/4.0/

## Introduction
RMB-Frankenboard. A mix of different pcb modules hooked up, flashed and ready to go with a Signal, or Two stage brushless Nerf Blaster type build. Designed to be used with a Neutron solenoid, ramping a 30 dps plus out of the box. Comes with an esp32 MCU and optic gates on the back of PCB for the closed loop feedback.

Controlled and configuration via Blue Tooth app on your phone and a select fire switch on the blaster for fire modes on the go. 
The RMB-Frankenboard can go into any type of Nerf type Blaster with a solenoid, and brushless flywheels. This could be a factory mod, or a 3d printed Blaster.

RMB-Frankenboard is integrated into the newest version of Mjölnir evolved, files can be found in this repository. 

### Features
- Supports dual Stage Brushless motors wired directly to headers on RMB-Frankenboard
- Will work with Half Darts or Long Darts
- Solenoid Pusher
- Optical Interrupt Gate on PCB for closed loop Pusher feed back. Front and Rear gates
- Single board no wiring loom
- Uses common through Hole parts and modules
- Controlled via Bluetooth App on Android phone for configuring your requirements. Download from google play store
- Battery connects directly to PCB via XT60/30
- Battery Type is auto detected. 4S-2S
- 3 Mode Select fire Switch
- Single Trigger Switch
- Rev Switch
- Magazine Sensor switch

- Via App: Display Battery Voltage
- Via App: Display Select Fire mode
- Via App: Display DPS
- Via App: Display Ammon Counter
- Via App: Trigger Button
- Via App: Game time countdown
- Via App: Burst Size slider
- Via App: Stage 1 Power slider
- Via App: Stage 2 Power slider
- Via App: Rev Ideal Speed slider
- Via App: Game Time slider

### Hardware
 - RMB-Frankenboard: Seeed XIAO ESP32C3, Mosfet, Opto Interrupts & I/O pins for ESC, Trigger, Select_Fire_Sw, Rev_Sw, Mag_Sw
 - 2 x Brushless Motors
 - 2 x ESC
 - Solenoid Pusher 
 - Trigger switch NO, COM
 - Select fire, 3 position switch, NO, COM, NO
 - Rev Sw NO, COM
 - Mag Sw NO, COM

 - Android phone & app for configuration, else is on factory defaults. App does not work on iPhone.

## Getting Started
On receiving your RMB-Frankenboard, the first thing to do BEFORE any soldering is started, is to confirm its operation.

Confirm PCB operation in one of two ways:-
- Option 1. Useing Phone App. Install App on to Android phone from this this link https://play.google.com/apps/internaltest/4701250598279133112
	or use APK file in this repository to install App.
	Power up RMB-Franken board, using USB-C lead in to a USB power supply or USB on a PC/Laptop
	Use App to connect to RMB-Frankenboard and confirm its working by scanning and connecting

- Option	2. Using Arduino IDE, plug in a USB C data sync lead (Not a charger lead) between RMB-Frankenboard and PC
	Start Arduino IDE and set serial port via tools menu to correct number	
	Open serial monitor window, set board rate to 115200. you sound see text output, probably "BAT low" from RMB-Frankenboard in the serial window
	If not try reboot by via power cycle on USB, with serial window open, you should see quite a few lines of text on startup if the serial window is already open.

Once you have confirmed you have a working board on the bench, you can start your build. Install board and solder up.

## Power Up and Operation
Once wires are connected, the RMB-Frankenboad needs to be powered from a Li-po battery, to power up correctly. 

### Before power up...
Solenoid pusher must have a aligned dufa attached to it, for the optical interrupts on the solder side of PCB to work correctly.
Once dufa is in place ensure you can hold the pusher in the fully out(on) position. 
The dufa should be sitting between the front Optic interrupt gate and NOT touching it.
Let the pusher go, in such a way that it snaps back via the spring. First time do it slow.
Once again ensure the dufa has not come in contact with the rear Gate, and is sitting nicely between its uprights.

Tech Note: If this is not configured before power up, the pcb will not function correctly. 
It will do single shot slowly, no burst or full auto.
Due to the Mosfet being turned hard on for an extended period of time you might damage the Mosfet missing this step. 
If there is a fault with the optics this is how it will respond.

Plug your Li-Po battery in.
You should here the start up beeps of the ESC's, twice.

Connect to RMB-Frankenboard via the app, confirm its reading battery voltage eg will be around 12v

Set Select fire switch to SINGLE shot, and pull trig. Pusher should snap fast, to do a single shot and flywheels spin.
Only if that worked then try Burst, then Full Auto.
Completed.


### RMB-Frankenboard Operation
The firmware comes with default configuration setting, and will work fine with out the App. 
However if you want to change the settings or use a remote trigger you will need the app.
The RMB-Frankenboard can be set back to defaults by holding trigger while power up.

The App displays status for Battery Volts, Select Fire Mode, Darts Shot
Two buttons, Trigger and Start (Game Start).

### Slider Settings:-
- Burst Size
- Stage 1 ESC Power
- Stage 2 ESC Power
- Ideal Rev Speed
- Game time

In addition to Trigger and Select Fire switches the RMB-Frankenboard, supports a Rev switch and Mag switch input

### Rev Idel function
Set Ideal Rev Speed > motor stall speed for pre spin. 
First shot will activate ideal, ie its idling after the first shot
Change Select fire Mode Switch to turn ideal off until next shot

If you have a rev button it works like normal, hold it in for on.

## Fault finding
The RMB-Frankenboard has a continual serial output that can be monitored via the serial window in Arduino IDE at 15200 baud
it has serial text output for most events. 
This can be used to see that switch's etc are working.
If you can not see the serial OutPut there's something not right. Power down. Does it give serial O/P when powered from USB?
Word of warning
When you have the USB plugged in, do not operate the trigger, thus making the solenoid work. 
Disconnect solenoid. 
It can damage the USB port with back EMF, i speck form experience.


## Links
https://play.google.com/apps/internaltest/4701250598279133112

https://github.com/KiwiPlastic/RMB_Frankenboard
