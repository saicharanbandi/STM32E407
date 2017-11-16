This is a simple program to test the working of WKUP BUTTON(STM32E407) and using an External Push button on GPIO F1.

It is interesting to see that Externally used Push button is not so reliable because of static charge around the button is triggering the LEDs without any push. I have even tried to use pull-down resistors but it is not working. 

I still need to experiment with the priority options though.

Another interesting fact is that the I cannot configure both WKUP_BUTTON and LED_User at the same time. I think they are internally shorted or some hardware problem is there!!
