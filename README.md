Welcome to the J1772 Hydra project

The Hydra is a device that allows a J1772 EVSE (Electric Vehicle Supplying Equipment - an electric car charger)
to charge two vehicles simultaneously in a way that is compliant with the J1772 standard.

HARDWARE SPECIFICATION
----------------------

The Hydra is based on the Arduino Uno platform, which is itself based on a 16 MHz 5V Amtel ATmega328.

There are currently two basic hardware variants. The traditional variant is the "splitter." It has a J1772
inlet and the incoming J1772 pilot signal is normalized to TTL levels and fed into digital pin 2. The software
will configure interrupt 0 to trigger on any change. This will be used to detect the incoming duty cycle, and
thus the current availability. The proximity pin on the inlet is also fed into a comparator to detect
proximity transitions and transmit them quickly to the cars.

The other variant is the EVSE. In that version, the inlet processing is traded for a GFI and a real-time clock
chip. There's also a GFI test line that is rapidly pulsed at startup to induce a GFI interrupt to insure it's
working.

Two of the digital pins will be set up with ordinary open collector transistor triggers to turn on power
relays for each car. Two of the other digital pins will be configured with custom PWM timer setups (via the
Arduino PWM library) to output pilot signals for each car.

Two of the analog pins will be configured to follow the voltage of the outgoing pilot pin for each car.
This will be the mechanism for determining state change requests from each car. The remaining two analog
pins will be connected to current transformers to act as ammeters for each car. The last two analog pins
are taken by the i2c system, which will communicate with an LCD shield (via the LiquidTWI2 library).

SOFTWARE SPECIFICATION
----------------------

There are two operating modes - shared and sequential.

In shared mode, the basic rule of thumb is that the outgoing pilot signal to any given car is the same as the
incoming pilot as long as the other car is not also charging. The other rule is that if one car is charging
and the other wants to begin charging, it must wait while the other car's pilot is reduced and given time to
reduce its consumption accordingly.

To start with, neither car is connected. Both outgoing pilots are pegged at +12v. First one car is connected
and it transitions to state B. Its pilot begins at full power. The other car is then connected and its pilot
also gets full power. If one car requests charging, it is granted immediately and the other car's pilot is
immediately reduced to half power. If the second car requests power, then it will wait for a settling period
while the first car's pilot is reduced to half power. After the waiting period, it will receive power (its pilot
will have already been at half power when the first car originally powered up).

When either car turns off, the other car's pilot will be switched back to full power.

In sequential mode, one car is given a full power pilot, and the other car is given no pilot at all. When the
first car is finished, the two cars "switch," giving the other car a chance to charge. If neither car wants to
charge, the pilot will switch cars every five minutes just on the off chance one of the cars changes its mind.

If either car over-draws its current allocation, it will be given 5 seconds to correct. If it remains overcurrent
for 5 seconds, or if it fails a diode check at any time, or its positive pilot moves into an undefined state,
then it will be errored out until it transitions into state A (disconnected). This means that if anything
goes wrong, it can be remedied by disconnecting and reconnecting the car.

If the incoming pilot either stops oscillating at 1 kHz, or if its duty cycle goes lower than the minimum
current, then both cars will be errored out with an incoming pilot error. The minimum power is 12A, because
the hydra must be able to divide that power by half, and 6A is the minimum allowable power per the J1772 spec.

EV SIMULATOR
------------

It's been useful to me to be able to simulate an EV on occasion. This can be done rather simply with just an 882 ohm
resistor and a 1N4148 diode in series, but I've also occasionally desired to analyze the outgoing pilot signal
from an EVSE as well. Doing that requires either a dedicated piece of hardware or an oscilloscope.

ATMega chips are pretty cheap and programming one to do the job of reporting the frequency and duty cycle of a
J1772 pilot signal is pretty simple. The schematic, Eagle BRD and DigiKey BOM files for my homebrew EV simulator
are available for download from the link below, and boards can be had from OSHPark. You can either power the
board with a 5V wall wart and use the OpenEVSE LCD backpack to display the frequency, duty cycle, and amp
rating or you can connect up a 5V FTDI cable to the board (which will also power the circuit) and watch the
results get printed out.

HARDWARE
--------

You can buy quick kits and assembled boards at http://store.geppettoelectronics.com/

The boards can be ordered from OSHPark at http://oshpark.com/profiles/nsayer if you want to build your own.
