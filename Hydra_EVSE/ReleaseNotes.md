Hydra EVSE version 2.3.5
==================

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
do initiate charging sessions even after was fully charged.

* Firmware version is tracked separately from the hardware version. Current hardware release is stil 2.3.1 (as found 
on the EVSE logic board) and is tested again such. Both versions are shown at the startup.

