# Arduino_DHTs_on_Interrupt_steroids
Interrupt's true prupose is to avoid polling.  This project was developed because other DHT libraries that advertised interrupt useage still unnecessarily rely on polling as well.  Also, it is not really a library, just some ISR code, ISR launcher code in setup(), DHT device detection and ISR pin change detection code, and main loop() examples of how RH and temperatures are read from the data structures.

This project utilizes two different interrupt service routine types - one "Pin Change Interrupt" pin for every DHT device connected and an additional over-all process-monitoring/watchdog code segment that resides in "Timer/Counter Compare Match A" interrupt code.  This code does not use any of the "2-wire Serial Interface", "External Interrupt", "Watchdog", "Timer/Counter Overflow", etc. types of interrupts.

The pin change interrupt service routines come into play after the compare-match code triggers a device.  A device in return will stream its data while the pin change ISR code stores the timestamp of each falling edge until the final falling edge is stored.  Then the ISR sends the device into the wait state.

The compare-match code serves the other purposes, including translating timestamps into bits then into RH/temperature data, invalidating non-responding devices, etc.  Only a portion of it is executed every millisecond when it is launched, so the math won't equate as you might hope - translating timestamps to bits will require one ISR execution ( a whole millisecond ) for every bit and for every device.  In other words, if all 3 ISRs have incoming data streams, 3 time 42 = 126.  That is a 126 ms time window AT LEAST for timestamp translation alone.   This is done so that main code of your making will be allowed to run as well as can be allowed.

Note that a typical Arduino board may have 3 Pin Change Interrupts with Pin Change Interrupt Service Routines, so if the pins are chosen to do so, 3 devices may ALL be feeding their data streams back to the board simultaneously.  When those three devices are finished with their data streams, the next device in line on each ISR will get triggered, assuming their resting times are completed.  Note this project can actually handle the 4 Pin Change interrupt streams that some devices are capable of, so the example of 3 is just a typical scenario rather than its true limit.

The compare-match interrupt code routine (ISR) is triggered every millisecond, and the pin change ISRs are triggered every one hundred microseconds, more or less, and MUST NOT BE DELAYED BY ANOTHER PIN CHANGE ISR NOR BY LEAVING INTERRUPTS TURNED OFF INSIDE THE COMPARE-MATCH ISR.  Therefore, the pin change ISRs must take as little execution time as possible, and the compare-match ISR must keep interrupts enabled as much as possible during its execution while ensuring its execution time is WELL less than a millisecond.  This project was written under the assumption that the PC interrupts are higher priority than the compare-match interrupt, but that assumption may not really be necessary since interrupts are kept enabled everywhere that atomic writes of shared memory locations are not critical.

More can be said, but I'm running out of time tonight.  Let me just load some working code for you to mold to your needs.  This is c++, so it is free.  A machine code version awaits a demonstrated interest from potential customers who are willing to pay a few bucks for it.  Please let me know if that includes you!

I just got this functional, not beautiful.  It is not intended for first-project newbies who'll have difficulty modifying to real-life needs.  That said, feel free to try anyway.  Please ensure your main code does not disable interrupts for more than roughly a microsecond at a time.
