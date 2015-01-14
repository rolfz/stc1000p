STC\-1000+ (minute based version)
========

This is a branch/fork of [STC\-1000+](https://github.com/matsstaff/stc1000p) (Improved firmware for fermentation and Arduino based uploader for the STC-1000 dual stage thermostat).

While it essentially the same functionality, the key differences are
* It uses minutes as timebase instead of hours
* It only stores state when next step is reached (original STC\-1000+ firmware stores state every hour), the reason this change is made is that if it where to save state every minute EEPRM wear would become a problem pretty soon.
* Setting step and duration (i.e. jumping in profile) is removed from the menu as duration is no longer stored in EEPROM.


Updates
-------

|Date|Release|Description|
|----|-------|-----------|
|TBA|TBA|TBA|

