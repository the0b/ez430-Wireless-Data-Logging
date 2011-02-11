//******************************************************************************
//   Based on code from eZ430-RF2500 Temperature Sensor End Device demo
//
//   Description: This is code for the End Device for the eZ430-2500RF
//                wireless data acquisition system. The end device waits for
//                a start signal from either the Access Point or a button press
//                on the access point itself. When the signal is received, data
//                acquisition is started and continues until flash memory is
//                filled. After memory is full, the data stored in memory is
//                sent wirelessly to the Access Point where it is forwarded to
//                the computer via virtual serial port.
//
//
//   T. Brower
//   Version    1.00
//   January 2011
//     IAR Embedded Workbench Kickstart (Version: 5.10.4)
//******************************************************************************
//Change Log:
//******************************************************************************
//Version:  1.00
//Comments: Initial Release Version
//******************************************************************************

#include "bsp.h"
#include "mrfi.h"
#include "nwk_types.h"
#include "nwk_api.h"
#include "bsp_leds.h"
#include "bsp_buttons.h"
#include "vlo_rand.h"

#include "string.h"     //for memcpy()

/*------------------------------------------------------------------------------
 * Defines
 *----------------------------------------------------------------------------*/
/* How many times to try a TX and miss an acknowledge before doing a scan */
#define MISSES_IN_A_ROW  200

//Start and end addresses for flash memory
//The start and end addresses should be at the start of a flash memory segment
#define FLASH_MEM_START 0xA400
#define FLASH_MEM_END 0xFFD0
//interrupt vector code start and end addresses (need to restore after erasing flash)
#define INT_VECT_START 0xFFE0
#define INT_VECT_END 0xFFFF

//Number of values that will be read before transfering to flash memory
//This number should be divisible by 6 since there are 6 ADC channels
//If doing block writes to flash, the number should also be divisible by 64
#define READ_BUFFER_SIZE  192

//#define FLASH_STORE_ROUTINE_LOCATION 0x537
#define FLASH_STORE_ROUTINE_SIZE 0x84

//define code segments
#pragma segment="FLASHCODE"
#pragma segment="RAMCODE"

/*------------------------------------------------------------------------------
 * Prototypes
 *----------------------------------------------------------------------------*/
static void linkTo(void);
void createRandomAddress(void);
void eraseFlashMem(void);
void sendFlashData(void);
void adcReadAndStore(void);

__interrupt void ADC10_ISR(void);
__interrupt void Timer_A (void);
__interrupt void Port_1(void);

/*------------------------------------------------------------------------------
* Globals
------------------------------------------------------------------------------*/
static linkID_t sLinkID1 = 0;
/* Initialize radio address location */
char * Flash_Addr = (char *)0x10F0;
/* Work loop semaphores */
static volatile uint8_t sSelfMeasureSem = 0;
//variable to keep track of flash memory location
unsigned int flashMemLocation = FLASH_MEM_START;
//variable to keep track if flash memory is full
int flashFull = 0;
//variable for enabling/disabling data capture
int enableRead = 0;
//Flag to clear flash data
int clearFlashData = 0;
//variable for storing wireless messages
uint8_t msg[10];  //Maximum message length with SimpliciTI is 10 bytes, set by MAX_APP_PAYLOAD

/*------------------------------------------------------------------------------
 * Main
 *----------------------------------------------------------------------------*/
void main (void)
{
  addr_t lAddr;

  /* Initialize board-specific hardware */
  BSP_Init();

  /* Check flash for previously stored address */
  if(Flash_Addr[0] == 0xFF && Flash_Addr[1] == 0xFF &&
     Flash_Addr[2] == 0xFF && Flash_Addr[3] == 0xFF )
  {
    createRandomAddress(); // Create and store a new random address
  }

  /* Read out address from flash */
  lAddr.addr[0] = Flash_Addr[0];
  lAddr.addr[1] = Flash_Addr[1];
  lAddr.addr[2] = Flash_Addr[2];
  lAddr.addr[3] = Flash_Addr[3];

  /* Tell network stack the device address */
  SMPL_Ioctl(IOCTL_OBJ_ADDR, IOCTL_ACT_SET, &lAddr);

  // Initialize TimerA and oscillator
  BCSCTL3 |= LFXT1S_2;                      // LFXT1 = VLO
  TACCTL0 = CCIE;                           // TACCR0 interrupt enabled
  TACCR0 = 10;                              // 6 = 1.78 kHz
                                            // 8 = 1.37 kHz
                                            // 10 = 1.12 kHz
                                            // 12 = 943 Hz
                                            // 14 = 819 Hz
                                            // 16 = 735 Hz
                                            // 18 = 653 Hz
                                            // 19 = 621 Hz
                                            // 20 = 591 Hz
  TACTL = TASSEL_1 + MC_1;                  // ACLK, upmode

  /* Keep trying to join (a side effect of successful initialization) until
   * successful. Toggle LEDS to indicate that joining has not occurred.
   */
  while (SMPL_SUCCESS != SMPL_Init(0))
  {
    BSP_TOGGLE_LED1();
    /* Go to sleep (LPM3 with interrupts enabled)
     * Timer A0 interrupt will wake CPU up every second to retry initializing
     */
    __bis_SR_register(LPM3_bits+GIE);  // LPM3 with interrupts enabled
  }

  /* Unconditional link to AP which is listening due to successful join. */
  linkTo();

  while(1);
}

static void linkTo()
{
  /* Keep trying to link... */
  while (SMPL_SUCCESS != SMPL_Link(&sLinkID1))
  {
    BSP_TOGGLE_LED1();
    /* Go to sleep (LPM3 with interrupts enabled)
     * Timer A0 interrupt will wake CPU up every second to retry linking
     */
    __bis_SR_register(LPM3_bits+GIE);
  }

  /* Put the radio to sleep */
  //SMPL_Ioctl(IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_SLEEP, 0);
  
  //Set up flash timing generator frequency for writing/erasing flash memory
  //Frequency must be in range: 257-476 kHz
  //MCLK is running at 8MHz, so divide by 17 to get 470 kHz
  FCTL2 = FWKEY + FSSEL1 + FN4 + FN0;
  
  //Erase flash memory before starting loop
  eraseFlashMem();
  
  //Set up push button interrupt
  P1IE |= 0x04;       // P1.2 interrupt enabled
  P1IES |= 0x04;      // P1.2 Hi/lo edge
  P1IFG &= ~0x04;     // P1.2 IFG cleared
  
  //Set up digital output on pin 11 (P4.6) to aid in debugging
  P4DIR |= BIT6;
  P4SEL &= ~BIT6;
  
  //Configure pins for analog input (A0, A1, A2, A3, A4, A12)
  ADC10AE0 |= 0x1F;     //A0 - A4
  ADC10AE1 |= 0x08;     //A12
  
  //Turn on green LED to indicate that we're ready to take data
  BSP_TURN_ON_LED1();

  while (1)
  {
    /* Go to sleep, waiting for interrupt to acquire data */
    __bis_SR_register(LPM3_bits + GIE);     //LPM3 with interrupt

    /* Time to measure */
    if (sSelfMeasureSem) {
      volatile long temp;
      
      if(clearFlashData){
        //Erase the flash memory and set pointers to start of flash
        eraseFlashMem();
        flashFull = 0;
        flashMemLocation = FLASH_MEM_START;
        clearFlashData = 0;
        BSP_TURN_ON_LED1();   //Turn on green light to indicate ready to capture
        BSP_TURN_OFF_LED2();
      }
      
      if(enableRead){
        BSP_TURN_OFF_LED1();    //Turn off green LED to indicate that board is busy
        BSP_TOGGLE_LED2();      //Toggle red LED while taking data
        P4OUT |= BIT6;          //Toggle P2.6 (Pin 13)
        
        if(!flashFull){
          //If flash memory is not full, read data
          //Reference voltage for ADC will be 2.5V (internal), this means maximum
          //reading from ADC (1023) should be interpreted as 2.5V. As a result, 
          //ADC resolution = 0.0024V = 2.4mV

          //ADC configuration, sets reference voltage, sample/hold time, 
          //turns on reference voltage, turns on ADC10 module, enables ADC10 interrupt          
          ADC10CTL0 = SREF_1 + ADC10SHT_2 + REFON + ADC10ON + ADC10IE + REF2_5V;  
          __delay_cycles(240);      //Delay to allow reference voltage to settle (30us = 240 cycles)
                  
          // Configure ADC to read voltage off of Pin 3 (analog input A0)
          ADC10CTL1 = INCH_0 + ADC10SSEL_0 + ADC10DIV_0;
          adcReadAndStore();
          
          // Read voltage off of Pin 4 (analog input A1)
          ADC10CTL1 = INCH_1 + ADC10SSEL_0 + ADC10DIV_0;
          adcReadAndStore();
          
          // Read voltage from Pin 5 (analog input A2)
          ADC10CTL1 = INCH_2 + ADC10SSEL_0 + ADC10DIV_0;
          adcReadAndStore();
          
          // Read voltage from Pin 6 (analog input A3)
          ADC10CTL1 = INCH_3 + ADC10SSEL_0 + ADC10DIV_0;
          adcReadAndStore();
          
          // Read voltage from Pin 7 (analog input A4)
          ADC10CTL1 = INCH_4 + ADC10SSEL_0 + ADC10DIV_0;
          adcReadAndStore();
          
          // Read voltage from Pin 8 (analog input A12)
          ADC10CTL1 = INCH_12 + ADC10SSEL_0 + ADC10DIV_0;
          adcReadAndStore();
    
          // Stop and turn off ADC
          ADC10CTL0 &= ~(REFON + ADC10ON);
          P4OUT &= ~BIT6;          //Toggle P2.6 (Pin 13)
          
          //Check to see if flash is full
          if(flashMemLocation >= FLASH_MEM_END-6){
            flashFull = 1;
          }
          
        } else{   //Flash is full
          BSP_TURN_OFF_LED2();
          //Disable data capture (wait for button press to acquire more data)
          enableRead = 0;
          
          //Send data stored in flash to access point
          sendFlashData();
          
          //Erase flash memory to prepare for next data set
          clearFlashData = 1;
        }
      } else{ //Not taking data, process any waiting messages
        uint8_t len;
        
        if (SMPL_SUCCESS == SMPL_Receive(sLinkID1, msg, &len))
        {
          ioctlRadioSiginfo_t sigInfo;
          sigInfo.lid = sLinkID1;
          
          //Start datalogging if necessary
          if(msg[0] == 's'){
            BSP_TOGGLE_LED1();
            enableRead = 1;
          }

          SMPL_Ioctl(IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_SIGINFO, (void *)&sigInfo);
        }
      }
     
      /* Done with measurement, disable measure flag */
      sSelfMeasureSem = 0;
    }
  }
}

void adcReadAndStore(){
  ADC10DTC1 = 0x01;                       // Configure DTC to store one value
  ADC10SA = flashMemLocation;             // Memory location to put result
  FCTL3 = FWKEY;                          // Unlock flash
  FCTL1 = FWKEY + WRT;                    // Configure flash write mode
  ADC10CTL0 |= ENC + ADC10SC;             // Sampling and conversion start
  __bis_SR_register(CPUOFF + GIE);        // LPM0 with interrupts enabled
  ADC10CTL0 &= ~ENC;                      // Disable ENC so that control registers can change          
  while (ADC10CTL1 & BUSY);               // Wait if ADC10 core is active
  flashMemLocation += 2;                  // Increment flash memory location
  FCTL3 = FWKEY + LOCK;                   // Lock flash
}

void eraseFlashMem(){
  //This function erases all the adc data stored in flash memory
  unsigned int segmentAddress;
  int i;
  
  //Since we are erasing the last segment, save interrupt vectors before
  //the erase operation and restore them after the erase operation
  uint8_t interruptvecs[(INT_VECT_END-INT_VECT_START+1)] = {0};
  segmentAddress = INT_VECT_START;
  for(i=0;i<=(INT_VECT_END-INT_VECT_START);i++){
    interruptvecs[i] = *((uint8_t*)segmentAddress++);
  }
  
  //Disable interrupts (since we are erasing the interrupt vectors)
  _DINT();
  //Disable watchdog timer (we're not using WDT, but do it just in case)
  WDTCTL = WDTPW + WDTHOLD;
  
  //Set flash controller to segment erase mode
  FCTL3 = FWKEY;          //Unlock flash memory for writing/erasing
  FCTL1 = FWKEY + ERASE;  //Set to segment erase mode
  
  //Dummy write to the start of each segment to initialize erase operation
  for(segmentAddress=FLASH_MEM_START; segmentAddress<FLASH_MEM_END && segmentAddress >=FLASH_MEM_START; \
      segmentAddress+=512){
    FCTL1 = FWKEY + ERASE;  //Set to segment erase mode, must do this before each segment
    *((unsigned int *)segmentAddress) = 0;
    while(!(FCTL3 & WAIT));                 // WAIT until Flash is ready
    _NOP();               //Set breakpoint here for debugging
  }
  //Done with erasing, now restore interrupt vectors
  segmentAddress = INT_VECT_START;
  FCTL1 = FWKEY + WRT;
  for(i=0;i<=(INT_VECT_END-INT_VECT_START);i++){
    *((uint8_t*)segmentAddress++) = interruptvecs[i];
  }
  
  
  //Done with everything, lock flash memory
  FCTL1 = FWKEY;
  FCTL3 = FWKEY + LOCK;
  
  //Enable interrupts
  _EINT();
}

void sendFlashData(){
  //This function sends all the data stored in flash memory to the access point
  uint8_t misses, done;
  uint8_t      noAck;
  smplStatus_t rc;

  int adc0val, adc1val, adc2val, adc3val, adc4val, adc12val;
  unsigned int flashIndex = FLASH_MEM_START;
  unsigned int i;
  
  /*  Theo: This section packs the ADC values into the message bytes without
  wasting any bits. As a result, we will be able to send 8 ADC values in one
  message. 
  maximum message length is message is 10 bytes long = 80 bits.
  We have 6 ADC measurements of 10 bits each = 60 bits. We will combine
  different ADC readings in single message bytes as shown below.
  ---------------------------------------------------------------------------------------------------------------
  message bytes
  |  byte 0  |  byte 1  |  byte 2  |  byte 3  |  byte 4  |  byte 5  |  byte 6  |  byte 7  |  byte 8  |  byte 9  |
  | 00000000 | 00111111 | 11112222 | 22222233 | 33333333 | 44444444 | 44555555 | 5555XXXX | XXXXXXXX | XXXXXXXX |
  ADC bits, numbers represent which ADC reading the bit is from, 
  each reading contains 10 bits. eg. ADC0 reading bits -> 0000000000
  X's represent unused message bits
  ---------------------------------------------------------------------------------------------------------------
  */
/*
  //This block sends the readbuffer, used to test if ADC is working correctly  
  for(flashIndex = 0; flashIndex < 192; flashIndex+=6){
    adc0val = readbuffer[flashIndex];
    adc1val = readbuffer[flashIndex+1];
    adc2val = readbuffer[flashIndex+2];
    adc3val = readbuffer[flashIndex+3];
    adc4val = readbuffer[flashIndex+4];
    adc12val = readbuffer[flashIndex+5];
*/
  i=0;
  for(flashIndex = FLASH_MEM_START; flashIndex < FLASH_MEM_END; flashIndex+=12){
    adc0val = *((unsigned int *)(flashIndex));
    adc1val = *((unsigned int *)(flashIndex+2));
    adc2val = *((unsigned int *)(flashIndex+4));
    adc3val = *((unsigned int *)(flashIndex+6));
    adc4val = *((unsigned int *)(flashIndex+8));
    adc12val = *((unsigned int *)(flashIndex+10));


    msg[0] = ((adc0val>>2)&0xFF);
    msg[1] = ((adc0val<<6)&0xC0) + ((adc1val>>4)&0x3F);
    msg[2] = ((adc1val<<4)&0xF0) + ((adc2val>>6)&0x0F);
    msg[3] = ((adc2val<<2)&0xFC) + ((adc3val>>8)&0x03);
    msg[4] = ((adc3val)&0xFF);
    msg[5] = ((adc4val>>2)&0xFF);
    msg[6] = ((adc4val<<6)&0xC0) + ((adc12val>>4)&0x3F);
    msg[7] = ((adc12val<<4)&0xF0);
    msg[8] = i;
    msg[9] = 0xAA;    //This byte will tell the reciever that we are using the packed message format
    
    //Replace last data message with alternating ones and zeros, this will 
    //notify message receiver that the data set is complete
    if(flashIndex+12 >= FLASH_MEM_END){
          msg[0] = 0xAA;
          msg[1] = 0xAA;
          msg[2] = 0xAA;
          msg[3] = 0xAA;
          msg[4] = 0xAA;
          msg[5] = 0xAA;
          msg[6] = 0xAA;
          msg[7] = 0xAA;
          msg[8] = 0xAA;
          msg[9] = 0xAA;    //This byte will tell the reciever that we are using the packed message format
    }
    
    i+=1;
  
    //------------------------------------------------------------------------
    
    /* Get radio ready...awakens in idle state */
    SMPL_Ioctl( IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_AWAKE, 0);
    
    // Request that the AP sends an ACK back to confirm data transmission
    done = 0;
    while (!done)
    {
      noAck = 0;
  
      /* Try sending message MISSES_IN_A_ROW times looking for ack */
      for (misses=0; misses < MISSES_IN_A_ROW; ++misses){
        _NOP();
        if (SMPL_SUCCESS == (rc=SMPL_SendOpt(sLinkID1, msg, sizeof(msg), SMPL_TXOPTION_ACKREQ)))
        {
          /* Message acked. We're done. Toggle LED 1 to indicate ack received. */
          BSP_TOGGLE_LED1();
          break;
        }
        if (SMPL_NO_ACK == rc){
          /* Count ack failures. Could also fail becuase of CCA and
           * we don't want to scan in this case.
           */
          noAck++;
        }
        __delay_cycles(80000);        //Slow operation down
      }
      if (MISSES_IN_A_ROW == noAck){
        /* Message not acked */
        BSP_TOGGLE_LED2();
#ifdef FREQUENCY_AGILITY
        /* Assume we're on the wrong channel so look for channel by
         * using the Ping to initiate a scan when it gets no reply. With
         * a successful ping try sending the message again. Otherwise,
         * for any error we get we will wait until the next button
         * press to try again.
         */
        if (SMPL_SUCCESS != SMPL_Ping(sLinkID1)){
          done = 1;
        }
#else
        done = 1;
#endif  /* FREQUENCY_AGILITY */
      } else{
        /* Got the ack or we don't care. We're done. */
        done = 1;
      }
    }
  
    /* Put radio back to sleep */
    //SMPL_Ioctl( IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_SLEEP, 0);
    
    //Slow operation down so that serial port will not be overloaded
    __delay_cycles(400000);
  }
  
  //Finished sending data, send "end data" signal
}

void createRandomAddress()
{
  unsigned int rand, rand2;
  do
  {
    rand = TI_getRandomIntegerFromVLO();    // first byte can not be 0x00 of 0xFF
  }
  while( (rand & 0xFF00)==0xFF00 || (rand & 0xFF00)==0x0000 );
  rand2 = TI_getRandomIntegerFromVLO();

  BCSCTL1 = CALBC1_1MHZ;                    // Set DCO to 1MHz
  DCOCTL = CALDCO_1MHZ;
  FCTL2 = FWKEY + FSSEL0 + FN1;             // MCLK/3 for Flash Timing Generator
  FCTL3 = FWKEY + LOCKA;                    // Clear LOCK & LOCKA bits
  FCTL1 = FWKEY + WRT;                      // Set WRT bit for write operation

  Flash_Addr[0]=(rand>>8) & 0xFF;
  Flash_Addr[1]=rand & 0xFF;
  Flash_Addr[2]=(rand2>>8) & 0xFF;
  Flash_Addr[3]=rand2 & 0xFF;

  FCTL1 = FWKEY;                            // Clear WRT bit
  FCTL3 = FWKEY + LOCKA + LOCK;             // Set LOCK & LOCKA bit
}

/*------------------------------------------------------------------------------
 * ADC10 interrupt service routine
 *----------------------------------------------------------------------------*/
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR(void)
{
  __bic_SR_register_on_exit(CPUOFF);        // Clear CPUOFF bit from 0(SR)
}

/*------------------------------------------------------------------------------
 * Timer A0 interrupt service routine
 *----------------------------------------------------------------------------*/
#pragma vector=TIMERA0_VECTOR
__interrupt void Timer_A (void)
{
  sSelfMeasureSem = 1;
  __bic_SR_register_on_exit(LPM3_bits);        // Clear LPM3 bit from 0(SR)
}

/*------------------------------------------------------------------------------
 * Port 1 interrupt service routine
 *----------------------------------------------------------------------------*/
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
{
  enableRead = 1;
  P1IFG &= ~0x04;                           // P1.2 IFG cleared
}