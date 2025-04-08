# RMB_Frankenboard
RMB-Frankenboard developed By RICHRD NICHOLSON from NEW ZEALAND
A Team.C.R.A.P Project

## Licensing
Non-commercial license is in effect. This license flows to derivative works.
You are free to remix / modify the blaster as you see fit. Please share
If you post a remix, please link back to the original.
Commercial use is allowed with the purchase of a RMB-Frankenboard at a ratio of one board = one commercial build.

## Attributions
(c) 2019-2021 Michael Ireland / Ireland Software / Airzone for example code used early on in project development work.
BLE Server examples
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleNotify.cpp
    Ported to Arduino ESP32 by Evandro Copercini
    updated by chegewara and MoThunderz

## License
RMB-Frankenboard
by Team C.R.A.P is licensed under the Creative Commons - Attribution - Non-Commercial Licence
https://creativecommons.org/licenses/by-nc/4.0/

## Introduction
RMB-Frankenboard. A mix of different PCB modules hooked up, flashed and ready to go with a Signal, or Two stage brushless Nerf Blaster type build. Designed to be used with a Neutron solenoid, ramping a 30 dps plus out of the box. Comes with an esp32 MCU and optic gates on the back of PCB for the closed loop feedback.

Controlled and configuration via Blue Tooth app on your phone and a select fire switch on the blaster for fire modes on the go. 
The RMB-Frankenboard can go into any type of Nerf type Blaster with a solenoid, and brushless flywheels. This could be a factory mod, or a 3d printed Blaster.

RMB-Frankenboard is integrated into the newest version of Mjölnir evolved files, which can be found in this repository. 

RMB-Frankenboard is actively evolving and currently consists of these Features

### Features
- RMB-Frankenboard: Seeed XIAO ESP32C3
- Supports dual Stage Brushless motors wired directly to headers on RMB-Frankenboard
- Will work with Half Darts or Long Darts
- Solenoid Pusher
- Optical Interrupt Gate on PCB for closed loop Pusher feedback. Front and Rear gates
- Single board no wiring loom
- Uses common through Hole parts and modules
- Controlled via Bluetooth App on Android phone for configuring your requirements. Download from google play store
- Battery connects directly to PCB via XT60/30
- Battery Type is auto detected. 4S-2S
- 3 Mode Select fire Switch
- Single Trigger Switch
- Rev Switch
- Magazine Sensor switch (code not completed)

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

### Hardware Required (you supply)
 - 2 x Brushless Motors
 - 2 x ESC or quad
 - Solenoid Pusher (Neutron for Mjolnir evolved) 
 - Trigger switch NO, COM
 - Select fire, 3 position switch, NO, COM, NO
 - Rev Sw NO, COM
 - Mag Sw NO, COM
 - USB type C Data Sync lead

 - Android phone & app for configuration, else is on factory defaults.	
	NOTE: App does not work on iPhone.
	
	NOTE2: Android phone does not need to be connected to a GSM network. Frankenboard uses the phones Blue Tooth for a connection. App can be downloaded to via local Wi-Fi connection.

## Getting Started - First Power Up
On receiving your RMB-Frankenboard, the first thing to do BEFORE any soldering is started, is to confirm its operation. Do not remove aerial.

First download the repository by clicking the green button over on the right says ‘CODE’ and selecting the zip.
You will also need Arduino IDE and a USB type C data sync cable

Confirm PCB operation in one of two ways: -
- Option 1. Using Phone App. 
Install App on to Android phone from this this link https://play.google.com/apps/internaltest/4701250598279133112
or use APK file in this repository to install App.

	Power up RMB-Franken board, using USB-C lead into a USB power supply or USB on a PC/Laptop
	
	Use App to connect to RMB-Frankenboard and confirm its working by scanning and connecting.

- Option 2. Using Arduino IDE. 
Plug in a USB C data sync lead (Not a charger lead) between RMB-Frankenboard and PC.
Start Arduino IDE and set serial port via tools menu to correct number	
	Open IDE serial monitor window, set board rate to 115200. You should see text output, probably "BAT low.

	If not, close serial window, try reboot by via power cycle (unplug USB lead and plug back in), immediately open serial window open, you should see quite a few lines of text on startup.

Once you have confirmed you have a working board on the bench, you can start your build. 
Install board and solder up.

## Leave one leg of Solenoid disconnected


## Optical Interrupt and Solenoid Duffer
BEFORE powering up...

- Solenoid pusher must have an aligned Duffer attached to it, for the optical interrupts on the solder side of PCB to work correctly.

- NOTE: If this is Duffer is not aligned correctly it will jam and the PCB will smoke and be damaged.

- The Solenoid PCB assembly may need to be pulled apart and put back together a few times to get the operation and tolerances correct.

- On first time assembly the Duffer will be to long.

- Manually move the solenoid in/out, noting where the Duffer touches. 

- Disassemble and file Duffer. 

- Repeat

- Once done correctly you should be able to manually push the pusher out with no resistance along its travel, until it reaches full travel and sits between the front Optic gate.
 
- Let it go slowly, and it should go back to rest between the rear Optic gate. If not dissemble and adjust. If you need to help it, it’s not correct.

- There cannot be the slightest touch between Optic gate and Solenoid Duffer. At high ROF this will course a jam and magic smoke will appear.

- Finaly, hold the pusher in the forward position and let the pusher go, in such a way that it snaps back via the spring. Ensure Duffer goes back through rear Optic gate without collision.

Tech Note: If this is not configured correctly before power up, the RMB-Frankenboard will not function correctly and could burn out.


### BLE Aerial
The BLE Aerial has a very delicate plug, we do not recommend removing it, as it may become damaged and not reconnect to PCB.
Do not power up RMB-Frankenboard without the Aerial connected, this will damage the ESP32 chip, with back RF.

## Power Up and Testing
Once wires are connected, the RMB-Frankenboard needs to be powered from a Li-po battery, to power up correctly. 

The RMB-Frankenboard uses serial output to indicate status.
We can use this to check for correct operation, before using the Trigger.

You will need a Laptop/PC with Arduino IDE running and a USB C type data sync lead, not a charger lead.

### Plug in Li-Po battery NO USB lead
1. You should here the startup beeps of the ESC's, (Beep, Beep, pause Beep).

2. Plug in USB lead 

3. Start IDE, open serial monitor, and configure com port and baud rate 112500

4. Power cycle RMB-Frankenboard by unplugging USB and plug back in. You should see text data in the IDE serial Window.

5. Change the Select Fire Switch position. Serial Monitor will indicate switch position: - Singal Burst Auto. Leave on Singal.

6. Manually move pusher forward, while watching serial Monitor. 
Rear Optic Gate will indicate 1 and 0, push forward and Front Optic Gate will indicate 1 and 0. If there is resistance, or a tight spot, it needs to be fixed. Disassemble and adjust (file).

7. With Solenoid DISCONNECTED. 
Disconnect USB lead. 
Set Select fire switch to SINGLE shot.
While watching Mosfet pcb, pull trigger once. 
Red LED on Mosfet pcb should light bright and momentary, motors should also spin up.
Try again to be sure.
Do not move the Select Fire Switch
Disconnect battery

8. With battery disconnected, solder other solenoid wire into correct hole on RMB-Frankenboard. NO USB lead.
Reconnect battery
Pull Trigger once
Pusher should snap fast, to do a single shot and flywheels spin.
Try again x 3 or 4 times
Only if that worked, then try Burst, then Full Auto.


Tech Note: 
If something is wrong with the alignment of optic gates and the Dufa, it will do single shot slowly. Burst and full auto will drive Mosfet fully hard on (red led n) making magic smoke if solenoid is connected. 


9. Connect to RMB-Frankenboard via the app, confirm its reading battery voltage eg will be around 12v. 


### RMB-Frankenboard App and Operation
The firmware comes with default configuration setting, and will work fine without the App. 

However, if you want to change the settings or use a remote trigger you will need the app.

The RMB-Frankenboard can be set back to defaults by holding trigger while power up.

If you have purchased an RMB-Frankenboard, you can download the app from this link: -
https://play.google.com/apps/internaltest/4701250598279133112

Alternatively, the apps, apk file is also in the repository and can be installed directly to the phone.

The Android phone does not need to be connected to a GSM network. Frankenboard uses the phones Blue Tooth for a connection, not the GSM network. 
The App can be installed via local Wi-Fi connection.

The App displays status for Battery Volts, Select Fire Mode, Darts Shot,
Trigger and Start (Game Start).

### Slider Settings: -
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
The RMB-Frankenboard has a event based serial output that can be monitored via the serial window in Arduino IDE at 15200 baud.

The serial output can be used to see that buttons, switch's etc are working.

If you cannot see the serial output, there's something not right. Power down. Does it give serial output when powered from USB only? Ie no Li-Po Battery.

## Word of warning
When you have the USB plugged in, DISCONNECT the solenoid or do not operate the trigger. 
The solenoid generates back EMF and can damage the USB port, I speck form experience.

Do not power up RMB-Frankenboard without the Aerial connected, this will damage the ESP32 chip with back RF.

This project is underdevelopment and actively evolving, please report issues so we can make improvements.


## How to Purchase RMB-Frankenboard
RMB-Frankenboard is $35usd plus shipping. 
Most countries, shipping is about $15usd (Ozy is cheaper).

Place order by contacting us via, 
Discord: Kiwi_7862
FB Messenger: Richard Fred

We require these details
- Name:
- Quanty:
- Email Address:
- Delivery address:
- Country:

We will send reply of receipt of your order
Then confirm shipping cost and send you a PayPal invoice via your email
Your email address is also used to register the Android App so u can download it from app store via supplied link. 
On receipt from PayPal that you have paid we ship.
We plan to have an Esty store shortly to make this more simple


## Flash Download Tool
This tool allows flashing of the firmware via *.Bin files.
It is a lot simpler than getting the correct libarys and a working compile from the Arduino IDE.

Firmware upgrades will be done using this tool.

The sub folder \bin in Flash download tool has the *.bin files it it.

Procedure: -
- Disconnect Li-Po Battery
- Close Arduino IDE
- Plug in USB data sync cable (not a charging cable)
- Open Flash Download Tool
- First Window Select: -
-   ESP32 C3
-   Development
-   USB

With reference to screen shoot in folder
Setup the download window. With correct path, and in this order.


 - Bootloader.bin 		0x0
 - Partions.Bin 		0x08000
 - RMB ESP32 V35.Bin 	0x010000

- 40Mhz 
- DIO
- DoNotChgBin = off
- Combine = off
- Select com port at 115200

Put chip in Bootloader mode. 
The chip has two buttons, one each side of the Arieal socket.
One is Reset = R, other is Boot = B
- Unplug USB
- While holding the B boot down plug the USB back in
- It will now power up in Bootloader mode
- Click Start…wait for green bar to move across screen
Flashing Complete

## Links
https://play.google.com/apps/internaltest/4701250598279133112

Note your email needs to be registered with us before this will work


https://github.com/KiwiPlastic/RMB_Frankenboard

https://www.arduino.cc/en/software/


