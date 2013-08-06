Welcome to the J1772 Hydra project

The Hydra is a device that allows a J1772 EVSE (Electric Vehicle Supplying Equipment - an electric car charger)
to charge two vehicles simultaneously in a way that is compliant with the J1772 standard.

HARDWARE SPECIFICATION
----------------------

The Hydra is based on the Arduino Uno platform, which is itself based on a 16 MHz 5V Amtel ATmega328.

The incoming J1772 pilot signal is normalized to TTL levels and fed into digital pin 2. The software will
configure interrupt 0 to trigger on any change. This will be used to detect the incoming duty cycle, and
thus the current availability.

Two of the digital pins will be set up with ordinary open collector transistor triggers to turn on power
relays for each car. Two of the other digital pins will be configured with custom PWM timer setups (via the
Arduino PWM library) to output pilot signals for each car.

Two of the analog pins will be configured to follow the voltage of the outgoing pilot pin for each car.
This will be the mechanism for determining state change requests from each car. The remaining two analog
pins will be connected to current transformers to act as ammeters for each car. The last two analog pins
are taken by the i2c system, which will communicate with an LCD shield (via the LiquidTWI2 library).

SOFTWARE SPECIFICATION
----------------------

The basic rule of thumb is that the outgoing pilot signal to any given car is the same as the incoming pilot
as long as the other car is not also charging. The other rule is that if one car is charging and the other
wants to begin charging, it must wait while the other car's pilot is reduced and given time to reduce its
consumption accordingly.

To start with, neither car is connected. Both outgoing pilots are pegged at +12v. First one car is connected
and it transitions to state B. Its pilot begins at full power. The other car is then connected and its pilot
also gets full power. If one car requests charging, it is granted immediately and the other car's pilot is
immediately reduced to half power. If the second car requests power, then it will wait for a settling period
while the first car's pilot is reduced to half power. After the waiting period, it will receive power (its pilot
will have already been at half power when the first car originally powered up).

When either car turns off, the other car's pilot will be switched back to full power.

If either car over-draws its current allocation, it will be given 5 seconds to correct. If it remains overcurrent
for 5 seconds, or if it fails a diode check at any time, or its positive pilot moves into an undefined state,
then it will be errored out until it transitions into state A (disconnected). This means that if anything
goes wrong, it can be remedied by disconnecting and reconnecting the car.

If the incoming pilot either stops oscillating at 1 kHz, or if its duty cycle goes lower than the minimum
current, then both cars will be errored out with an incoming pilot error. The minimum power is 12A, because
the hydra must be able to divide that power by half, and 6A is the minimum allowable power per the J1772 spec.

HARDWARE
--------

The boards can be ordered from OSHPark at http://oshpark.com/profiles/nsayer

There are two DigiKey BOM files that you can use to buy all the parts to build a Hydra. That, and the
Eagle .sch and .brd files and a PDF schematic are available in the project downloads.

REVISION HISTORY
----------------

0.1 alpha - initial commit. This version is not yet actually functional, as the hardware has not yet been
completely built.

0.8 beta - This version has been tested with the prototype hardware (except the ammeters) and is fully functional.
