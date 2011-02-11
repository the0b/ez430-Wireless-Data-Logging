//******************************************************************************
//   Based on code from eZ430-RF2500 Temperature Sensor Access Point demo
//
//   Description: This is code for the Access Point for the eZ430-2500RF
//                wireless data acquisition system. The access point communicates
//                data between the computer and wireless End Device. The 
//                connection to the computer is through a virtual serial port.
//                The access point sends data received from the wireless End 
//                Device to the computer and also sends commands from the 
//                computer to the wireless End Device. 
//
//
//   T. Brower
//   Version    1.00
//   December 2010
//     IAR Embedded Workbench Kickstart (Version: 5.10.4)
//******************************************************************************
//Change Log:
//******************************************************************************
//Version:  1.00
//Comments: Initial Release Version
//******************************************************************************
#include <string.h>
#include "bsp.h"
#include "mrfi.h"
#include "bsp_leds.h"
#include "bsp_buttons.h"
#include "nwk_types.h"
#include "nwk_api.h"
#include "nwk_frame.h"
#include "nwk.h"
#include "virtual_com_cmds.h"

/****************** COMMENTS ON ASYNC LISTEN APPLICATION ***********************
Summary:
  This AP build includes implementation of an unknown number of end device peers
  in addition to AP functionality. In this scenario all End Devices establish a
  link to the AP and only to the AP. The AP acts as a data hub. All End Device
  peers are on the AP and not on other distinct ED platforms.

  There is still a limit to the number of peers supported on the AP that is
  defined by the macro NUM_CONNECTIONS. The AP will support NUM_CONNECTIONS or
  fewer peers but the exact number does not need to be known at build time.

  In this special but common scenario SimpliciTI restricts each End Device
  object to a single connection to the AP. If multiple logical connections are
  required these must be accommodated by supporting contexts in the application
  payload itself.

Solution overview:
  When a new peer connection is required the AP main loop must be notified. In
  essence the main loop polls a semaphore to know whether to begin listening for
  a peer Link request from a new End Device. There are two solutions: automatic
  notification and external notification. The only difference between the
  automatic notification solution and the external notification solution is how
  the listen semaphore is set. In the external notification solution the
  sempahore is set by the user when the AP is stimulated for example by a button
  press or a commend over a serial link. In the automatic scheme the
  notification is accomplished as a side effect of a new End Device joining.

  The Rx callback must be implemented. When the callback is invoked with a
  non-zero Link ID the handler could set a semaphore that alerts the main work
  loop that a SMPL_Receive() can be executed successfully on that Link ID.

  If the callback conveys an argument (LinkID) of 0 then a new device has joined
  the network. A SMPL_LinkListen() should be executed.

  Whether the joining device supports ED objects is indirectly inferred on the
  joining device from the setting of the NUM_CONNECTIONS macro. The value of
  this macro should be non-zero only if ED objects exist on the device. This
  macro is always non-zero for ED-only devices. But Range Extenders may or may
  not support ED objects. The macro should be be set to 0 for REs that do not
  also support ED objects. This prevents the Access Point from reserving
  resources for a joinng device that does not support any End Device Objects and
  it prevents the AP from executing a SMPL_LinkListen(). The Access Point will
  not ever see a Link frame if the joining device does not support any
  connections.

  Each joining device must execute a SMPL_Link() after receiving the join reply
  from the Access Point. The Access Point will be listening.

******************* END COMMENTS ON ASYNC LISTEN APPLICATION ******************/

/******  THIS SOURCE FILE REPRESENTS THE AUTOMATIC NOTIFICATION SOLUTION ******/

/*------------------------------------------------------------------------------
 * Prototypes
 *----------------------------------------------------------------------------*/
/* Frequency Agility helper functions */
static void    checkChangeChannel(void);
static void    changeChannel(void);

__interrupt void ADC10_ISR(void);
__interrupt void Timer_A (void);

/*------------------------------------------------------------------------------
 * Globals
 *----------------------------------------------------------------------------*/
/* reserve space for the maximum possible peer Link IDs */
static linkID_t sLID[NUM_CONNECTIONS] = {0};
static uint8_t  sNumCurrentPeers = 0;

/* callback handler */
static uint8_t sCB(linkID_t);

/* received message handler */
static void processMessage(linkID_t, uint8_t *, uint8_t);

/* work loop semaphores */
static volatile uint8_t sPeerFrameSem = 0;
static volatile uint8_t sJoinSem = 0;
static volatile uint8_t sSelfMeasureSem = 0;

/* blink LEDs when channel changes... */
static volatile uint8_t sBlinky = 0;

/* data for terminal output */
const char splash[] = {"\r\n------------------------------------------------------------  \r\n     ****\r\n     ****           eZ430-RF2500\r\n     ******o****    Gyroscope and Accelerometer Data Logger\r\n********_///_****   Theo Brower, 10-6-2010\r\n ******/_//_/*****  Use at your own risk\r\n  ** ***(__/*****   \r\n      *********     SimpliciTI1.1.1\r\n       *****\r\n        ***\r\n------------------------------------------------------------\r\n"};
volatile int * tempOffset = (int *)0x10F4;

// flag to send start message to end device(s)
//char sendStart = 0;
/* How many times to try a TX and miss an acknowledge before doing a scan */
#define MISSES_IN_A_ROW  200

/*------------------------------------------------------------------------------
 * Frequency Agility support (interference detection)
 *----------------------------------------------------------------------------*/
#ifdef FREQUENCY_AGILITY

#define INTERFERNCE_THRESHOLD_DBM (-70)
#define SSIZE    25
#define IN_A_ROW  3
static int8_t  sSample[SSIZE];
static uint8_t sChannel = 0;

#endif  /* FREQUENCY_AGILITY */

/*------------------------------------------------------------------------------
 * Main
 *----------------------------------------------------------------------------*/
void main (void)
{
  bspIState_t intState;
  //variables for wireless messaging
  uint8_t     msg[MAX_APP_PAYLOAD], len, i;

#ifdef FREQUENCY_AGILITY
  memset(sSample, 0x0, sizeof(sSample));
#endif

  /* Initialize board */
  BSP_Init();

  /* Initialize TimerA and oscillator */
  BCSCTL3 |= LFXT1S_2;                      // LFXT1 = VLO
  TACCTL0 = CCIE;                           // TACCR0 interrupt enabled
  TACCR0 = 1200;                           // ~1 second = 12000
  TACTL = TASSEL_1 + MC_1;                  // ACLK, upmode

  /* Initialize serial port */
  COM_Init();

  //Transmit splash screen and network init notification
  TXString( (char*)splash, (sizeof splash)-1);
  TXString( "\r\nInitializing Network....", 26 );

  SMPL_Init(sCB);

  // network initialized
  TXString( "Done\r\n", 6);
  TXString( "Waiting for peer connection...\r\n", 32);

  /* green and red LEDs on solid to indicate waiting for a Join. */
  BSP_TURN_ON_LED1();
  BSP_TURN_ON_LED2();

  /* main work loop */
  while (1)
  {
    /* Wait for the Join semaphore to be set by the receipt of a Join frame from
     * a device that supports an End Device.
     *
     * An external method could be used as well. A button press could be connected
     * to an ISR and the ISR could set a semaphore that is checked by a function
     * call here, or a command shell running in support of a serial connection
     * could set a semaphore that is checked by a function call.
     */
    if (sJoinSem && (sNumCurrentPeers < NUM_CONNECTIONS))
    {
      /* listen for a new connection */
      while (1)
      {
        if (SMPL_SUCCESS == SMPL_LinkListen(&sLID[sNumCurrentPeers]))
        {
          break;
        }
        /* Implement fail-to-link policy here. otherwise, listen again. */
      }

      sNumCurrentPeers++;

      BSP_ENTER_CRITICAL_SECTION(intState);
      sJoinSem--;
      BSP_EXIT_CRITICAL_SECTION(intState);
    }


    if(sSelfMeasureSem)
    {
      // Repurposed this measurement section to deal with receiving the
      // start adc trigger from the serial port
      uint8_t misses, done;
      uint8_t      noAck;
      smplStatus_t rc=0;
      //Just toggle the LED to show it's alive
      BSP_TOGGLE_LED1();
      
      //Send start message to all end devices, if necessary
      if(startAdcFlag == 1){
        msg[0] = 's'; 
        for (i=0; i<sNumCurrentPeers; ++i)
        {
          //Send data, no acknowledgment
          rc = SMPL_SendOpt(sLID[i], msg, sizeof(msg), SMPL_TXOPTION_NONE);
          BSP_TOGGLE_LED2();
        }
        startAdcFlag = 0;
      }

      // Done with measurement, disable measure flag
      sSelfMeasureSem = rc;
      sSelfMeasureSem = 0;
    }

    /* Have we received a frame on one of the ED connections?
     * No critical section -- it doesn't really matter much if we miss a poll
     */
    if (sPeerFrameSem)
    {
      int temp1;
      char ADCreads[] = {"XXXX, XXXX, XXXX, XXXX, XXXX, XXXX, XXXX\r\n"};

      /* process all frames waiting */
      for (i=0; i<sNumCurrentPeers; ++i)
      {
        if (SMPL_SUCCESS == SMPL_Receive(sLID[i], msg, &len))
        {
          ioctlRadioSiginfo_t sigInfo;

          processMessage(sLID[i], msg, len);

          sigInfo.lid = sLID[i];

          SMPL_Ioctl(IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_SIGINFO, (void *)&sigInfo);
          
          // Theo: send ADC data values to serial port
          if(msg[9] == 0xAA){
            /*Interpret message bytes into ADC values, this section uses the
            packed message format, allowing for transmission of 8 ADC values,
            however, only 6 ADC values are read at this time.
            ---------------------------------------------------------------------------------------------------------------
            message bytes
            |  byte 0  |  byte 1  |  byte 2  |  byte 3  |  byte 4  |  byte 5  |  byte 6  |  byte 7  |  byte 8  |  byte 9  |
            | 00000000 | 00111111 | 11112222 | 22222233 | 33333333 | 44444444 | 44555555 | 5555XXXX | XXXXXXXX | XXXXXXXX |
            ADC bits, numbers represent which ADC reading the bit is from, 
            each reading contains 10 bits. eg. ADC0 reading bits -> 0000000000
            X's represent unused message bits
            ---------------------------------------------------------------------------------------------------------------
            (caution: all parentheses required for order of operations to be correct)*/
            //A0 
            temp1 = (msg[0]<<2) + ((msg[1]>>6)&0x03);
            //to convert ADC value to volts, formula is:
            //temp1 = (temp1 * 1.53)/1023
            //Intentionally not doing this calculation on microprocessor, it
            //can be done more accurately through whatever program is reading
            //the data stream because the microprocessor can't do float math
            ADCreads[0] = '0'+((temp1/1000)%10);
            ADCreads[1] = '0'+((temp1/100)%10);
            ADCreads[2] = '0'+((temp1/10)%10);
            ADCreads[3] = '0'+(temp1%10);
            
            //A1
            temp1 = ((msg[1]&0x3F)<<4) + ((msg[2]>>4)&0x0F);
            ADCreads[6] = '0'+((temp1/1000)%10);
            ADCreads[7] = '0'+((temp1/100)%10);
            ADCreads[8] = '0'+((temp1/10)%10);
            ADCreads[9] = '0'+(temp1%10);
            
            //A2
            temp1 = ((msg[2]&0x0F)<<6) + ((msg[3]>>2)&0x3F);
            ADCreads[12] = '0'+((temp1/1000)%10);
            ADCreads[13] = '0'+((temp1/100)%10);
            ADCreads[14] = '0'+((temp1/10)%10);
            ADCreads[15] = '0'+(temp1%10);
            
            //A3
            temp1 = ((msg[3]&0x03)<<8) + msg[4];
            ADCreads[18] = '0'+((temp1/1000)%10);
            ADCreads[19] = '0'+((temp1/100)%10);
            ADCreads[20] = '0'+((temp1/10)%10);
            ADCreads[21] = '0'+(temp1%10);
            
            //A4
            temp1 = (msg[5]<<2) + ((msg[6]>>6)&0x03);
            ADCreads[24] = '0'+((temp1/1000)%10);
            ADCreads[25] = '0'+((temp1/100)%10);
            ADCreads[26] = '0'+((temp1/10)%10);
            ADCreads[27] = '0'+(temp1%10);
            
            //A12
            temp1 = ((msg[6]&0x3F)<<4) + ((msg[7]>>4)&0x0F);
            ADCreads[30] = '0'+((temp1/1000)%10);
            ADCreads[31] = '0'+((temp1/100)%10);
            ADCreads[32] = '0'+((temp1/10)%10);
            ADCreads[33] = '0'+(temp1%10);
            
            //Extra byte, shows ADC iteration
            temp1 = msg[8];
            ADCreads[36] = '0'+((temp1/1000)%10);
            ADCreads[37] = '0'+((temp1/100)%10);
            ADCreads[38] = '0'+((temp1/10)%10);
            ADCreads[39] = '0'+(temp1%10);
            
            
          }else{
            //Theo: This section interprets data sent in the simple message format
            //A0
            temp1 = msg[0] + (msg[1]<<8);
            ADCreads[0] = '0'+((temp1/1000)%10);
            ADCreads[1] = '0'+((temp1/100)%10);
            ADCreads[2] = '0'+((temp1/10)%10);
            ADCreads[3] = '0'+(temp1%10);
            
            //A1
            temp1 = msg[2] + (msg[3]<<8);
            ADCreads[6] = '0'+((temp1/1000)%10);
            ADCreads[7] = '0'+((temp1/100)%10);
            ADCreads[8] = '0'+((temp1/10)%10);
            ADCreads[9] = '0'+(temp1%10);
            
            //A2
            temp1 = msg[4] + (msg[5]<<8);
            ADCreads[12] = '0'+((temp1/1000)%10);
            ADCreads[13] = '0'+((temp1/100)%10);
            ADCreads[14] = '0'+((temp1/10)%10);
            ADCreads[15] = '0'+(temp1%10);
            
            //A3
            temp1 = msg[6] + (msg[7]<<8);
            ADCreads[18] = '0'+((temp1/1000)%10);
            ADCreads[19] = '0'+((temp1/100)%10);
            ADCreads[20] = '0'+((temp1/10)%10);
            ADCreads[21] = '0'+(temp1%10);
            
            //A4
            temp1 = msg[8] + (msg[9]<<8);
            ADCreads[24] = '0'+((temp1/1000)%10);
            ADCreads[25] = '0'+((temp1/100)%10);
            ADCreads[26] = '0'+((temp1/10)%10);
            ADCreads[27] = '0'+(temp1%10);
          }
  
          TXString( (char*)ADCreads, (sizeof ADCreads)-1);

          BSP_TOGGLE_LED2();

          BSP_ENTER_CRITICAL_SECTION(intState);
          sPeerFrameSem--;
          BSP_EXIT_CRITICAL_SECTION(intState);
        }
      }
    }
    if (BSP_BUTTON1())
    {
      __delay_cycles(2000000);  /* debounce (0.25 seconds) */
      changeChannel();
    }
    else
    {
      checkChangeChannel();
    }
    BSP_ENTER_CRITICAL_SECTION(intState);
    if (sBlinky)
    {
      if (++sBlinky >= 0xF)
      {
        sBlinky = 1;
        BSP_TOGGLE_LED1();
        BSP_TOGGLE_LED2();
      }
    }
    BSP_EXIT_CRITICAL_SECTION(intState);
  }

}

/* Runs in ISR context. Reading the frame should be done in the */
/* application thread not in the ISR thread. */
static uint8_t sCB(linkID_t lid)
{
  if (lid)
  {
    sPeerFrameSem++;
    sBlinky = 0;
  }
  else
  {
    sJoinSem++;
  }

  /* leave frame to be read by application. */
  return 0;
}

static void processMessage(linkID_t lid, uint8_t *msg, uint8_t len)
{
  /* do something useful */
  if (len)
  {
    BSP_TOGGLE_LED1();
  }
  return;
}

static void changeChannel(void)
{
#ifdef FREQUENCY_AGILITY
  freqEntry_t freq;

  if (++sChannel >= NWK_FREQ_TBL_SIZE)
  {
    sChannel = 0;
  }
  freq.logicalChan = sChannel;
  SMPL_Ioctl(IOCTL_OBJ_FREQ, IOCTL_ACT_SET, &freq);
  BSP_TURN_OFF_LED1();
  BSP_TURN_OFF_LED2();
  sBlinky = 1;
#endif
  return;
}

/* implement auto-channel-change policy here... */
static void checkChangeChannel(void)
{
#ifdef FREQUENCY_AGILITY
  int8_t dbm, inARow = 0;

  uint8_t i;

  memset(sSample, 0x0, SSIZE);
  for (i=0; i<SSIZE; ++i)
  {
    /* quit if we need to service an app frame */
    if (sPeerFrameSem || sJoinSem)
    {
      return;
    }
    NWK_DELAY(1);
    SMPL_Ioctl(IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_RSSI, (void *)&dbm);
    sSample[i] = dbm;

    if (dbm > INTERFERNCE_THRESHOLD_DBM)
    {
      if (++inARow == IN_A_ROW)
      {
        changeChannel();
        break;
      }
    }
    else
    {
      inARow = 0;
    }
  }
#endif
  return;
}

/*------------------------------------------------------------------------------
* ADC10 interrupt service routine
------------------------------------------------------------------------------*/
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR(void)
{
  __bic_SR_register_on_exit(CPUOFF);        // Clear CPUOFF bit from 0(SR)
}

/*------------------------------------------------------------------------------
* Timer A0 interrupt service routine
------------------------------------------------------------------------------*/
#pragma vector=TIMERA0_VECTOR
__interrupt void Timer_A (void)
{
  sSelfMeasureSem = 1;
}
