# utx-embedded-launchpad-lab8
UTX Embedded - Lab 8 - Switch LED Interface

Purpose
Lab 8 is our first lab requiring you to build circuits on the breadboard and connect them to the LaunchPad. The purpose of this lab is to learn how to interface a switch and an LED. You will perform explicit measurements on the circuits in order to verify they are operational and to improve your understanding of how they work.

System Requirements
In this lab you will build a switch interface that implements positive logic, and you will build an LED interface that implements positive logic.  You will attach this switch and LED to your protoboard (the white piece with all the holes), and interface them to your TM4C123. Overall functionality of this system is similar to Lab 6, with five changes: 1) the pin to which we connect the switch is moved to PE0, 2) you will have to remove the PUR initialization because pull up is no longer needed. 3) the pin to which we connect the LED is moved to PE1, 4) the switch is changed from negative to positive logic, and 5) you should decrease the delay so it flashes about 5 Hz. To flash at 5 Hz means the LED comes on 5 times per second. If the switch is pressed we turn on the LED for 100 ms, turn off the LED for 100 ms, and repeat.
1) Make PE1 an output and make PE0 an input. 
2) The system starts with the LED on (make PE1 =1). 
3) Wait about 100 ms
4) If the switch is pressed (PE0 is 1), then toggle the LED once, else turn the LED on. 
5) Steps 3 and 4 are repeated over and over.

The grader will activate the PLL so the system runs at 80 MHz, you must not modify this rate.

https://www.youtube.com/watch?v=bS73Z-WGZz8
