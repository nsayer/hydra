Hydra EVSE version 2.3.5
========================

This release focuses on fixes and additions for the sequential mode. 

* Lack of sequential mode tiebreaking when both cars plugged during paused period is fixed.
When both cars are plugged during paused period, and EVSE is then unpaused either manually or by
an event, then both cars stay in "wait" state. 

* Tie break is selected by either:
  * last car charging before pause (if it wasn't unplugged during pause period) (same as before)
  * last car plugged during paused period (new)
  * there is no undetermined tiebreak state any more (new). On startup, default tiebreak is selected as car A.

* In the sequential mode, when cars are paused and both off, the current tiebreak status is indicated by an
'*' (asterisk) symbol next to the car status (off). The car marked by the asterisk will be first in 
the charging sequence once hydra enters the unpaused mode.

* Flipping the charging status. 

Previously, once one vehicle had transitioned from  C or D state (charging) into B
(plugged but not charging), the pilot was immediately withdrawn from that vehicle indicating no allowed amperage
(pause) and then another car was given a full pilot, thus allowing it entering C or D immediately. 

That created problems when both cars are fully charged, since both of my cars would still keep starting short 
charging sessions intermittently even that they are already charged, thus flipping contactors (which is pretty 
loud), generating events, sending emails etc. every few seconds. 

The version 2.3.1 firmware actually has a notion the flipping timeout (5 minutes) but that would only apply if the 
fully charged car did not initiate charging session when offered again (not the case for both of my two cars).

To fix the high frequency sequential charging sessions with 2 cars, the implementation now tracks "done" 
vs. "wait" statuses (also displayed during 2-car sequential charging). The "done" status indicates that the car 
has completed at least one full charging session since it had been unpaused or plugged in.

Thus, when a car is done charging, transition to another car in the "wait" status is still done immediately, whereas
transition to a car in the "done" status is subject to the same 5 minute flipping interval regardless of whether 
the current car did a charging session or not.

This gives cars a chance to still commence longer charging sessions after they are completely charged (e.g., if 
they are scheduled to condition in the morning), while eliminating overly frequent sequential flipping in case they
do initiate charging sessions even after having been fully charged.

* Firmware version is tracked separately from the hardware version. Current hardware release is stil 2.3.1 (as found 
on the EVSE logic board) and is tested again such. Both versions are shown at the startup.

Hydra EVSE version 2.4.0
========================

2.4.0 adds manual calibration to the ammeters and the pilot. 

Problem statement: 
------------------

My Spark EV is not a very good citizen (but my other car is, it underdraws diligently by 0.5A or so!). 
When being asked to draw a restricted amperage, my Spark EV yet may overdraw by about 5%. This normally is not a problem 
per se, since both residential single plug or commercial dual plug chargers have plenty of power to cover well above 
its maximum 3.3 kw chargin rate (~13.5A or so). 

However, I have a 30A line which means total available amperage for an EVSE is no more than 24A, or 12A per plug in 
the shared Hydra mode when two cars are plugged at the same time. So Spark finds itself under the limit of 12A, 
which it doesn't observe well. 
In addition, the ammeter readings vary by about 0.2A so it appears that Spark is overdrawing even more than on the other. Add to that some metering and draw variation, and Spark ends up throws error O (overdraw) from time to time in the shared mode on my line A (but not on B, on which it overdraws too, just slightly less due to ammeter reading variation).

Hydra processes overdraw error if the draw was exceeded by 1A or more for at least 5 seconds (IIRC). 
So in an unlikely event that two lines are overdrawing at the same time and exactly by that extreme amount, it would actually mean the draw may go up to 26A with configured 24A capacity before Hydra would error both out. That's why it 
is important to derate Hydra EVSE configuration compared to the breaker amperage.

So the desired fixes are as follows: 

* adjust ammeter natural variations manually by setting draw current adjustmetns of -0.5A to +0.5A in 0.1A increments;

* adjust additional amperage derate in allowed pilot amperage per line (between 0% and -10%, in 1% decrements). So, 
for example, if the half power in shared mode is 12A, then 9.8A will be actually restricted by the pilot while still allowing it to grow up to 12A half power +1A existing Hydra slack = 13A without throwing an overdraw error. This is 
actually done per line, not per actual car, assuming problematically overdawing car would be using the same line consistently.

Important:
----------

When flashing firmware 2.4.0 for the first time over previous versions **without erasing the EEPROM** make sure to check calibration menu settings to show all 0s after the first flash.
