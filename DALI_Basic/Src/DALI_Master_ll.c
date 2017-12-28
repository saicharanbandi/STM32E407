/*++++++++++++++ INCLUDES +++++++++++++++++++++++++++++++++++++++++++++++*/
#include <DALI_Master_ll.h>
#include <DALI_defs.h>

/*++++++++++++++ Variables +++++++++++++++++++++++++++++++++++++++++++++++*/
unsigned char dali_state;

volatile unsigned char dali_master_array_cmd[17];
volatile unsigned char dali_master_array_receive_buffer[9];

unsigned char actual_val;
unsigned char former_val;

/*++++++++++++++ Yet to add +++++++++++++++++++++++++++++++++++++++++++++*/
/* unsigned char *ptrAddr; */
/* volatile LightObjectType lightLeds[10]; */

/*++++++++++++++ Not sure +++++++++++++++++++++++++++++++++++++++++++++++*/
volatile unsigned char tmpg;

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
Function : DALI_Master_Init 
Description : Initialize DALI network

Note: Main function used for DALI initialization
      called in main.c
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

void DALI_Master_Init(void)
{
  //set start/default values for flags
  dali_state = NO_ACTION;

  /*++++++++++++++++++++ Yet to add +++++++++++++++++++++++++++++++++++++*/
  /* //Initialize memory space and default values */
  /* DALI_Mem_Init(); */
}

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
Function :     DALI_Master_Send_Cmd
Description :  Send DALI command over DALI protocol

Inputs :       * ballastAddr - Address of the ballast (dimmer)
           
               * cmd         - Command which is going to be sent, defined in DALI_defs.h

               * typeOfCmd   - It's used to define type of address:
	                        - BROADCAST_DIRECT
				- BROADCAST_CMD
				- SHORT_ADDRESS
				- GROUP_ADDRESS

               * followingType - Status of the last bit in address byte.
                                 - FOLLOWING_DIRECT_ARC_POWER_LVL
				 - FOLLOWING_COMMAND

Output :       TRUE
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
unsigned char DALI_Master_Send_Cmd(unsigned char ballastAddr, unsigned char cmd, unsigned char typeOfCmd, unsigned char followingType)
{
  unsigned char data_array[2] = {};
  unsigned char i = 0;

  // set Manch_Tx pin as high
  HAL_GPIO_WritePin(Manch_Tx_GPIO_Port, Manch_Tx_Pin, GPIO_PIN_SET);

  // reset tick_count and bit_count values
  tick_count = 0;
  bit_count = 0;

  // set DALI state to send data
  dali_state = SENDING_DATA;

  //fetch ballast address and command
  data_array[0] = (char)ballastAddr;
  data_array[1] = (char)cmd;

  /*??????????????? Necessity of initialization ?????????????????????????*/
  // reset dali_master_array_cmd values
  for (i = 0; i < 17; i++)
    {
      dali_master_array_cmd[i] = 0;
    }

  // prepare address byte to be sent
  PrepareAddressByte_Master(data_array, typeOfCmd, 0, followingType);

  //encode data - Manchester encoding
  PrepareDataToSend_Master(data_array, dali_master_array_cmd, 2);

  /*+++++++++++++ Yet to add ++++++++++++++++++++++++++++++++++++++++++++*/
  /* // Write ballastAddr to memory address */
  /* DALI_Write(ballastAddr, data_array[1]); */

  // check type of command
  // set back channel
  if((cmd >= 0x00) && (cmd <= 0x1F))    //Indirect arc power control commands
    {
      expect_backchannel = FALSE;
    }
  if((cmd >= 0x20) && (cmd <= 0x80))    // Configuration commands
    {
      expect_backchannel = FALSE;
    }
  if((cmd >= 0x90))                     // Query commands
    {
      expect_backchannel = TRUE;
      /* Set status to expect Backchannel. Possible answer:
	 1111 1111                            - YES
	 no response; no backchannel received - NO
	 8bit info                            - 8 bit +++++++++++++++++++*/
    }

  /*??????????????? YET TO UNDERSTAND ???????????????????????????????????*/
  //check for special command
  if(DALI_Check_Special_Cmd(data_array[0]))
    {
      expect_backchannel = TRUE;

      if(data_array[0] == TERMINATE_H_BITS || data_array[0] == DTR)
	{
	  expect_backchannel = FALSE;
	}
      else if(data_array[0] == VERIFY_SHORT_ADDRESS || data_array[0] == QUERY_SHORT_ADDRESS_H)
	{
	  expect_backchannel = TRUE;
	}
      else
	{
	  expect_backchannel = FALSE;
	}
    }
  /*?????????????????????????????????????????????????????????????????????*/

  // Start Timer
  Timer_Start();

  return TRUE;
}

/*!!!!!!!!!!!!!!!! CBTD !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/

unsigned char DALI_Check_Special_Cmd(unsigned char addrByte)
{
  volatile unsigned char addrToCheck;

  addrToCheck = addrByte;                                             // get address byte
  if((addrToCheck == 0x90) || (addrToCheck == 0xA0))                  // check for 1010 or 1011
    {
      if(addrToCheck & 0x01)                                          // LSB must be 1
	{
	  return TRUE;
	}
      else
	{
	  return FALSE;
	}
    }
  else
    {
      return FALSE;
    }
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
Function:       DALI_Master_Sending_Data
Description:    Send data using Manch_Tx pin

Note:           Used Manchester Encoding
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void DALI_Master_Sending_Data(void)
{
  unsigned char pulsePosition = 0;
  if(tick_count < 8)
    {
      if(tick_count < 4)
	{
	  HAL_GPIO_WritePin(Manch_Tx_GPIO_Port, Manch_Tx_Pin, GPIO_PIN_SET);
	}
      else
	{
	  HAL_GPIO_WritePin(Manch_Tx_GPIO_Port, Manch_Tx_Pin, GPIO_PIN_RESET);
	}
    }
  else if(bit_count < 17)
    {
      if(tick_count % 4 == 0)
	{
	  pulsePosition = tick_count / 4;
	  if(pulsePosition % 2 == 0)
	    {
	      if(dali_master_array_cmd[bit_count] == 0)
		{
		  HAL_GPIO_WritePin(Manch_Tx_GPIO_Port, Manch_Tx_Pin, GPIO_PIN_SET);
		}
	      else
		{
		  HAL_GPIO_WritePin(Manch_Tx_GPIO_Port, Manch_Tx_Pin, GPIO_PIN_RESET);
		}
	    }
	  else
	    {
	      if(dali_master_array_cmd[bit_count] == 0)
		{
		  HAL_GPIO_WritePin(Manch_Tx_GPIO_Port, Manch_Tx_Pin, GPIO_PIN_RESET);
		}
	      else
		{
		  HAL_GPIO_WritePin(Manch_Tx_GPIO_Port, Manch_Tx_Pin, GPIO_PIN_SET);
		}
	    }
	}
    }
  // increment tick_count
  tick_count++;

  // increment bit_count
  if(tick_count % 8 == 0)
    {
      bit_count++;
    }

  // transfer completed
  if(bit_count > 16)
    {
      dali_state = FORWARD_FRAME_SENT;
      HAL_GPIO_WritePin(Manch_Tx_GPIO_Port, Manch_Tx_Pin, GPIO_PIN_SET);
    }
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
Function :      PrepareDataToSend
Description :   Prepare command array to be encoded and create new
                array where every element is a bit

Parameters :    * commandArray  - Array of byte values
                * tx_array      - Return Array. Each element
                                       represents a bit state!!
		* bytesInCmd    - Number of bytes in command array

Note :          Manchester encoding
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void PrepareDataToSend(unsigned char *commandArray, unsigned char *tx_array, unsigned char bytesInCmd)
{
  //set default value for the mask
  unsigned char mask;
  //variable which hold one byte value - one element from commandArray
  unsigned char dummy = 0;
  //number of bytes in command
  unsigned char bytes_counter = 0;
  unsigned char i = 0;
  // number of active bit
  unsigned char bitCounter = 0;

  /*?????? Why does it need to initialized???????????????????????????????*/
  for(i = 0; i < 9; i++)
    {
      tx_array[0] = 0;
    }

  // loop through all bytes in commandArray
  for(bytes_counter = 0; bytes_counter < bytesInCmd; bytes_counter++)
    {
      // assign byte for use
      dummy = commandArray[bytes_counter];

      //set mask to default value
      mask = 0x80;

      // increment number of active bit
      bitCounter++;
      // check if active bit is the first one
      if(bitCounter == 1)
	{
	  // start bit is always 1 i.e DALI_END_BIT_PULSE
	  tx_array[0] = DALI_END_BIT_PULSE;
	}

      // 2 byte command
      for(i = 0; i < 9; i++)
	{
	  // check if bit is one
	  if(dummy & mask)
	    {
	      // assign pulse value
	      tx_array[i + (8 * bytes_counter)] = DALI_END_BIT_PULSE;
	    }
	  else
	    {
	      tx_array[i + (8 * bytes_counter)] = DALI_START_BIT_PULSE;
	    }
	  // check mask value
	  if(mask == 0x01)
	    {
	      mask <<= 7; // shift mask bit to MSB
	    }
	  else
	    {
	      mask >>= 1; // shift mask bit to 1 right
	    }
	}
    }
}

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
Function :      PrepareAddressByte
Description :   Prepare Address Byte

Parameters:     * commandArray - Array of byte values

                * addressType  - It's used to define type of address:
		                  - BROADCAST_DIRECT
				  - BROADCAST_CMD
				  - SHORT_ADDRESS
				  - GROUP_ADDRESS

		* byteAddressPosition - Index of element in array which holds address value

		* followingType       - value of the last bit in address byte.
		                        This defines if data byte holds command or direct arc
		                        value

					- FOLLOWING_DIRECT_ARC_POWER_LVL
					- FOLLOWING_COMMAND
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/*!!!!!!!!! UNCERTAIN - CHECK THOROUGHLY !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
void PrepareAddressByte(unsigned char *commandArray, unsigned char addressType, unsigned char byteAddressPosition, unsigned char followingType)
{
  unsigned char addr_tmp = 0;

  // broadcast command to all ballasts
  if(addressType == BROADCAST_CMD)
    {
      // set address byte to Broadcast command - value 0xFF
      commandArray[byteAddressPosition] = BROADCAST_CMD;
    }
  else
    {
      //fetch address value from array to operate
      addr_tmp = commandArray[byteAddressPosition];

      if(addressType == BROADCAST_DIRECT)
	{
	  // broadcast direct arc level to all ballasts - value 0xFE
	  commandArray[byteAddressPosition] = BROADCAST_CMD;
	}
      else
	{
	  // shift address value to left by 1
	  addr_tmp <<= 1;

	  // check if the command byte is following address byte
	  if(followingType == FOLLOWING_COMMAND)
	    {
	      // set LSB
	      addr_tmp |= 0x01;
	    }
	  if(addressType == GROUP_ADDRESS)
	    {
	      addr_tmp |= GROUP_ADDRESS;
	    }
	  // assign return value
	  commandArray[byteAddressPosition] = addr_tmp;
	}
    }
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
Function :      DALI_Receiving_Data
Description:    Check status of Manch_Rx pin and write to dali_master_array_receive_buffer array

Note:           Used Manchester Encoding

+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

void DALI_Receiving_Data(void)
{
  /*??????????? UNUSED BUT DEFINED ??????????????????????????????????????*/
  unsigned char pulsePosition = 0;
  /*?????????????????????????????????????????????????????????????????????*/
  
  /* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
     - backward frame - 9 bits to receive, last two don't change phase
     - ignore the first bit as it is start bit
     - also ignore last two bits as they are stop bits
     - FF - BF settling time (7Te - 22Te)[2Te = 8 interrupt intervals]
     - when change on pin is detected, tick_count is restarte 
     ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
  if(tick_count == (bit_count * 8 + 2))
    {
      if(HAL_GPIO_ReadPin(Manch_Rx_GPIO_Port, Manch_Rx_Pin) == GPIO_PIN_SET)
	{
	  dali_master_array_receive_buffer[bit_count] = 0;
	}
      else if(HAL_GPIO_ReadPin(Manch_Rx_GPIO_Port, Manch_Rx_Pin) == GPIO_PIN_RESET)
	{
	  dali_master_array_receive_buffer[bit_count] = 1;
	}
    }

  // increment ticks
  tick_count++;

  //increment bit_count
  if(tick_count % 8 == 0)
    {
      bit_count++;
    }

  // transfer completed
  if(bit_count > 8)
    {
      // set dali_state
      dali_state = BACKWARD_FRAME_RECEIVED;
    }
}


/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
Function :        DALI_Get_Ballast_Answer
Description :     Encode and write received data. Check in dali_array_receive_buffer

Output :          Return ballast answer
                  - YES  : 1111 1111
		  - NO   : 0
		  - 8bit : XXXX XXXX - 8 bit value

+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/*!!!!!!!!!! CAREFUL WITH THIS FUNCTION; KEY ASPECT CHANGED !!!!!!!!!!!!!*/
unsigned char DALI_Get_Ballast_Answer(void)
{
  unsigned char i = 0;
  unsigned char receivedData = 0;

  for(i = 0; i < 8; i++)
    {
      /* !!!!!!! Changed from i to 7-i : BEWARE !!!!!!!!!!!!!!!!!!!!!!!!!*/
      // shift bit to the right position
      dali_array_receive_buffer[i] <<= (7-i);
      // add bit to the received byte
      receivedData |= dali_array_receive_buffer[i];
    }

  // return received byte
  return receivedData;
}

/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
Function :       DALI_Master_Status
Description:     DALI master device main loop

Output    :      Return DALI state

+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
unsigned char DALI_Master_Status(void)
{
  unsigned char i = 0;
  unsigned char tmp_actual_val = 0;
  if(dali_state == NO_ACTION)
    {
      //idle state
      // reset variables
      tick_count = 0;
      bit_count = 0;
    }
  
  if(dali_state == SENDING_DATA)
    {
      // SENDING COMMANDS
    }

  // data can be received only if master device send a command
  if(dali_state == RECEIVING_DATA)
    {
      // backchannel
      // check for backchannel timeout; used for no answer
    }

  //forward frame sent. Check for settling time
  if(dali_state == FORWARD_FRAME_SENT)
    {
      tick_count = 0;
      bit_count = 0;

      //dali_cmd_repeat_time--;
      HAL_GPIO_WritePin(Manch_Tx_GPIO_Port, Manch_Tx_Pin, GPIO_PIN_SET);

      // set settling time
      if(expect_backchannel)
	{
	  dali_state = SETTLING_FF_TO_BF;
	}
      else
	{
	  dali_state = SETTLING_FF_TO_FF;
	}
      
    }

  //backward frame received. Set settling state
  if(dali_state == BACKWARD_FRAME_RECEIVED)
    {
      // backward frame full received
      // check for settling time
      // settling_state = SETTLING_FF_TO_FF; // FF -> FF & BF -> FF
      // dali_state = SETTLING_FF_TO_FF;

      dali_state = NO_ACTION;
      expect_backchannel = 0;
    }

  // check if settling finished
  if(dali_state == SETTLING_FF_TO_FF_FINISHED || dali_state == SETTLING_FF_TO_BF_FINISHED)
    {
      dali_state = NO_ACTION;

      if(expect_backchannel)
	{
	  dali_state = WAIT_FOR_BACKCHANNEL_TO_RECEIVE;

	  former_val = 1;
	  actual_val = 1;
	}
    }

  // if we wait for backchannel, check Manch_Rx Pin
  if(dali_state == WAIT_FOR_BACKCHANNEL_TO_RECEIVE)
    {
      former_val = actual_val;
      tmp_actual_val = HAL_GPIO_ReadPin(Manch_Rx_GPIO_Port, Manch_Rx_Pin);
      if(tmp_actual_val == GPIO_PIN_SET)
	{
	  actual_val = 1;
	}
      else if(tmp_actual_val == GPIO_PIN_RESET)
	{
	  actual_val = 0;
	}

      if(former_val != actual_val)
	{
	  tick_count = 0;
	  bit_count = 0; // add start bit
	  dali_state = RECEIVING_DATA;
	}
    }

  // error part
  if(dali_state == ERR)
    {
      HAL_GPIO_WritePin(Manch_Tx_GPIO_Port, Manch_Tx_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(Manch_Rx_GPIO_Port, Manch_Rx_Pin, GPIO_PIN_SET);
    }

  return dali_state;
}
