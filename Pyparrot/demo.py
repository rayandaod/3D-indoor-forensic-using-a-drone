"""
Demo the Bebop indoors (takes off, lands)
Note, the bebop will hurt your furniture if it hits it.  Be sure you are doing this in an open area
and are prepared to catch!

Author: Rayan Daod
"""

from pyparrot.Bebop import Bebop

bebop = Bebop()

print("connecting")
success = bebop.connect(10)
print(success)

if (success):

    print("sleeping")
    bebop.smart_sleep(2)

    bebop.ask_for_state_update()

    bebop.safe_takeoff(10)

    bebop.smart_sleep(5)

    bebop.safe_land(10)

    print("DONE - disconnecting")
    bebop.smart_sleep(5)
    bebop.disconnect()