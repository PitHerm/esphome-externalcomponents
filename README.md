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

# s5_gr3p15kww3

A sniffing component dokumentation: Docs/s5_gr3p15kww3 
tests Tests/s5_gr3p15kww3


You can try if my solutions work for you,
it works stabil for the above mentioned system.
