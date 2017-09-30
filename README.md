      Arduino DHTs on Interrupt steroids
Files you need to place in your sketch directory:

Arduino_DHTs_on_Interrupt_steroids.ino

ISRs.h

misc_maskportitems.h

structs.h

The main purpose of the interrupt is to avoid the time waste of polling.  This project was developed because other DHT libraries that advertised interrupt useage still unnecessarily rely on polling as well, making their claimed use of interrupts virtually meaningless, not to mention their sad (and now unnecessary) limitation of only a single DHT device per board.  Have all the sensors you want with proper application of interrupt coding given here.  Connect them all up and go.  Optionally, add your main functionality to void loop() if you want more than the super-simple demo that is only intended to be a proof-of-concept.  This project is not a formal library, just some ISR code, ISR launcher code in setup(), DHT device detection and ISR pin change detection code, and a main loop() hello world demo with rudimentary examples of how RH and temperatures are read from the data structures.  

If you just want to see which pins of your board are served by PC ISRs and which ISR it is, this sketch is exactly what you need as well.  It is not hindered by the incomplete OEM definitions that support digitalPinToPCICRbit() and digitalPinToPCMSKbit() functions, as is the case with the official Mega 2560 IDE environment ISR1.  Screen shots of that functionality during board startup are posted here for the boards I had at hand.  Each Arduino type has a different amount of available RAM, and some boards are thusly limited to less than the maximum number of devices. I'll be working on more memory-efficient modifications while you try this version.  Note that LED_BUILTIN and any other LEDs will preclude their pins from being used for DHT devices because they prevent voltage pull-up.

I'll be working on some important streamlining and memory efficiency elements in the coming days.  At this point, the best I can say about the progress right now is that it is debugged and functions if your sketch can afford the memory hit.  

This project utilizes two different interrupt service routine types:  one "Pin Change Interrupt" pin is required for every DHT device connected serviced by an PC ISR code segment, and an additional over-all process-monitoring/watchdog code segment that resides in "Timer/Counter Compare Match A" interrupt code. (This code does not use any of the "2-wire Serial Interface", "External Interrupt", "Watchdog", "Timer/Counter Overflow", etc. types of interrupts.)

The pin change interrupt service routines come into play after the compare-match interrupt code triggers a device.  In response, a device will stream its data back while the pin change ISR code stores the timestamp of each falling edge until the final one.  Then the ISR sends the device into the wait state.  Note that storing timestamps in their raw unsigned long type is one memory useage vs speed compromise.  To gain a few more bytes of variable RAM at the expense of some extra execution time in the Pin Change ISRs, a person could move the translation function to the Pin Change ISRs.  For now, I am deciding against providing that option because it would have to be run-time determined every ISR call and thus add excess detection and decision execution time to a project like this meant for general consumption.

The compare-match code serves the other purposes, including translating timestamps into bits then into RH/temperature data, invalidating non-responding devices, etc.  Only a portion of it is executed every millisecond when it is launched, so the math won't equate as you might hope - translating timestamps to bits will require one ISR execution ( a whole millisecond ) for every bit and for every device.  In other words, if all 3 ISRs have incoming data streams, 3 time 42 = 126.  That is 126 ms time windows AT LEAST for timestamp translation alone.   This "small bite each pass" technique is done so that main code of your making will be allowed to run as well as can be allowed.

Note that a typical Arduino board may have 3 Pin Change Interrupts with Pin Change Interrupt Service Routines, so if the pins are chosen to do so, 3 devices may ALL be feeding their data streams back to the board simultaneously.  When those three devices are finished with their data streams, the next device in line on each ISR will get triggered, assuming their resting times are completed.  Note this project can actually handle the 4 Pin Change interrupt streams that some devices are capable of, so the example of 3 is just a typical scenario rather than its true limit.

The compare-match interrupt code routine (ISR) is triggered every millisecond, and the pin change ISRs are triggered every one hundred microseconds, more or less, and MUST NOT BE DELAYED BY ANOTHER PIN CHANGE ISR NOR BY LEAVING INTERRUPTS TURNED OFF INSIDE THE COMPARE-MATCH ISR.  Therefore, the pin change ISRs must take as little execution time as possible, and the compare-match ISR must keep interrupts enabled as much as possible during its execution while ensuring its execution time is WELL less than a millisecond.  This project was written under the assumption that the PC interrupts are higher priority than the compare-match interrupt, but that assumption may not really be necessary since interrupts are kept enabled everywhere that atomic writes of shared memory locations are not at stake.

More can be said, but I'm running out of time tonight.  Let me just post some working code for you to mold to your needs.  This is c++, so it is free.  An assembly code version awaits while I hope to see an interest demonstrated by [a] potential customer/employer[s].  Please let me know if that is you!

I merely got this functional, not beautiful.  It is not intended for first-project newbies who need handholding creating their own void loop() based on my demo loop().  That said, feel free to try anyway.  Please ensure your main code does not disable interrupts for more than roughly a microsecond at a time, since a four microsecond delay is all it takes to corrupt the data stream.  If you use DHT devices in an inhabited structure thermostat application, be aware that the DHT11 is notorious for frustratingly inconsistent manufactured quality in its operation, so have plenty of spares, make it easy to swap between them, and attempt to source from the best manufacturers you can find.


Progress being made: 

This sketch works just fine on the Mega 2560 because that board has plenty of memory.  This sketch is very limited on boards with less memory.  Example is on UNO (Nano shows exactly the same in this respect) - It only has enough memory for 13 devices, and the closer you get to 13, the less memory is left for your own main sketch.  I am removing memory bloat as I can to improve that situation and posting daily updates.  

Going back to the Leonardo with its differences in serial communications, I am unable to command it to list DHT readings for the time being.  It does support 7 DHT devices, possibly eight IF you want to remove (break) the RX LED and solder a connecting wire to the positive side of it...not likely anyone would do that, so 7 devices it is.
