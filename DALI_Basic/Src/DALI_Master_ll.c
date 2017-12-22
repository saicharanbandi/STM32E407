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
  unsigned char data_array[2];
  unsigned char i;

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
Function :      DALI_Receiving_Data
Description:    Check status of Manch_Rx pin and write to dali_master_array_receive_buffer array

Note:           Used Manchester Encoding

+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

void DALI_Receiving_Data
