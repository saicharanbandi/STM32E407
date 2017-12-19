Important observations:

Date: 18.12.2017

I needed to change the logic bit for including the initial interrupt. I have changed the logic even in Slave code from number 2 to 3 i.e in line 283, where if ( tick_count == (bit_count * 8 + 3))!!
