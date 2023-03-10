# This is a HomeAssistant integration for Solis inverters described below using ESPHome
A friend of mine asked me for a solution to get data out of his solis system
because he didn't find any working solution for his system on the net.

His system consists of a Solis S5-GR3P(15K)
https://www.ginlong.com/solarinverter3/3-20kw_s5_de.html

and the new Wifi stick S3-WiFi-ST
https://www.ginlong.com/accessories5/GPRS&WiFi_Data_Logging_Stick_de.html

the location is Germany (maybe it makes sense to know it)

I did a research and found a solution of Grob6000 on git.
https://github.com/grob6000/esphome-externalcomponents

But this solution didn't work, digging a bit deeper i spottet that the
communications is completely different.

Depending on that i'm completely new to ESPhome and not good with python and C++
(i prefer old C) , i took Grob6000's code as a base and applied several changes
to meet the different protocol and got a solution that works for my friend.

IMPORTANT: The inverter MUST be configured with (Modbus) address set to 1 !

The fortune to sniff the communication is that the inverter is still connected to
the cloud.

On the other hand you receive new values around every 5 minutes because the stick
does not request new values more often.
I think getting data more often is not really neccessary, however, maybe i'll
write another version requesting data more often and working without a stick sometimes.

I took GPIO4 for RX instead of pin 3 because a lot of the avaiable D1-mini vesions are
not able to receive data on the default Rx-pin bacause of the circuit around - and 
behavior of the CH340 itself.

You will find all documentation you need within the Docs folder including
a sample config yaml file.

Within the Tests folder you will find some tests for checking.

You can try if this solution works for you,
it works stabil for the above mentioned system.
