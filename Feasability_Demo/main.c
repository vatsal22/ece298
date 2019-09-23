#include "main.h"
#include "driverlib/driverlib.h"
#include "hal_LCD.h"

/*
 * This project contains some code samples that may be useful.
 *
 */

char ADCState = 0;   //Busy state of the ADC
int16_t ADCResult = 0; //Storage for the ADC conversion result

void read_keypad()
{

    //        2 c2 1.6
    //        3 r1 1.3
    //        4 c1 5.0
    //        5 r4 1.4
    //        6 c3 1.5
    //        7 r3 2.5
    //        8 r2 2.7

    __disable_interrupt();

    //Stop watchdog timer unless you plan on using it
    WDT_A_hold(WDT_A_BASE);

    // Initializations - see functions for more detail
    //      Init_GPIO();    //Sets all pins to output low as a default
    Init_PWM();   //Sets up a PWM output
    Init_ADC();   //Sets up the ADC to sample
    Init_Clock(); //Sets up the necessary system clocks
    Init_UART();  //Sets up an echo over a COM port
    Init_LCD();   //Sets up the LaunchPad LCD display

    PMM_unlockLPM5(); //Disable the GPIO power-on default high-impedance mode to activate previously configured port settings

    //All done initializations - turn interrupts back on.

    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);



    //       GPIO_setAsInputPin(GPIO_PORT_P1, GPIO_PIN3);
    //       GPIO_setAsInputPin(GPIO_PORT_P2, GPIO_PIN7);
    //       GPIO_setAsInputPin(GPIO_PORT_P2, GPIO_PIN5);
    //       GPIO_setAsInputPin(GPIO_PORT_P1, GPIO_PIN4);

    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN0);
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN6);
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN5);

    GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN0);
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN6);
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN5);

     GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P1, GPIO_PIN3);
     GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P2, GPIO_PIN7);
     GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P2, GPIO_PIN5);
     GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P1, GPIO_PIN4);

    __enable_interrupt();

    while (1)
    {

//        for (int i = 0;i<4;i++) {
//
//
//
//        }




        GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN0);

        if (GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN3) == 0)
        {
            showChar('0', 4);

            __delay_cycles(2000);
            clearLCD();
        }
        if (GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN7) == 0)
        {
            showChar('1', 4);

            __delay_cycles(2000);
            clearLCD();
        }
        if (GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN5) == 0)
        {
            showChar('2', 4);

            __delay_cycles(2000);
            clearLCD();
        }
        if (GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN4) == 0)
        {
            showChar('3', 4);

            __delay_cycles(2000);
            clearLCD();
        }


        if (GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN3) == 1)
        {
            showChar('0', 4);

            __delay_cycles(2000);
            clearLCD();
        }
        if (GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN7) == 1)
        {
            showChar('1', 4);

            __delay_cycles(2000);
            clearLCD();
        }
        if (GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN5) == 1)
        {
            showChar('2', 4);

            __delay_cycles(2000);
            clearLCD();
        }
        if (GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN4) == 1)
        {
            showChar('3', 4);

            __delay_cycles(2000);
            clearLCD();
        }

        GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN3);
        GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P2, GPIO_PIN7);
        GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P2, GPIO_PIN5);
        GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN4);
//        GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN3);
//        GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN7);
//        GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN5);
//        GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN4);
//
////         GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3);
////         GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7);
////         GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN5);
////         GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN4);
//
//        showChar('n', 4);
//
//        __delay_cycles(2000);
//        clearLCD();
    }

           GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    //       GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    //       GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    //       GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    //       GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    //       GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    //       GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    //       GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
}


// 5.2 c2 2
// 5.3 r1 3
// 1.3 c1 4
// 1.4 r4 5
// 1.5 c3 6
// 2.7 r3 7
// 2.5 r2 8
struct pin_dat {

    int port;
    int pin;
};

struct k_col {
    int port;
    int pin;
};

//int getKeypadStatus () {
//    struct k_row rows[4];
//    struct k_col cols[3];
//    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3);
//    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P5, GPIO_PIN3);
//
//
//
//
//}

int getKeypadStatus()
{

    struct pin_dat r1;
    r1.port = GPIO_PORT_P5;
    r1.pin=GPIO_PIN3;

    struct pin_dat r2;
    r2.port = GPIO_PORT_P2;
    r2.pin=GPIO_PIN5;

    struct pin_dat r3;
    r3.port = GPIO_PORT_P2;
    r3.pin=GPIO_PIN7;

    struct pin_dat r4;
    r4.port = GPIO_PORT_P1;
    r4.pin=GPIO_PIN4;

    struct pin_dat c1;
    c1.port = GPIO_PORT_P1;
    c1.pin=GPIO_PIN3;

    struct pin_dat c2;
    c2.port = GPIO_PORT_P5;
    c2.pin=GPIO_PIN2;

    struct pin_dat c3;
       c3.port = GPIO_PORT_P1;
       c3.pin=GPIO_PIN5;

  struct pin_dat rowPins[4] = {r1, r2, r3, r4};  // row pins are 2, 7, 6, and 1 of the keypad
//  int row = 4;

  struct pin_dat colPins[3] = {c1, c2, c3};  // column pins are pins 2, 4, and 5 of the keypad
//  int col = 3;

  int keypadStatus = 0;  // this will be what's returned

  /* initialize all pins, inputs w/ pull-ups */
  int i;
  for ( i=0; i<4; i++)
  {
      GPIO_setAsInputPinWithPullUpResistor(rowPins[i].port, rowPins[i].pin);
  }


  for ( i=0; i<3; i++)
  {
      GPIO_setAsInputPinWithPullUpResistor(colPins[i].port, colPins[i].pin);
  }

  GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P5, GPIO_PIN3);


  int row;
  for ( row=0; row<4; row++)
  {  // initial for loop to check all 4 rows
//    pinMode(rowPins[row], OUTPUT);  // set the row pin as an output
    GPIO_setAsOutputPin(rowPins[row].port, rowPins[row].pin);

    GPIO_setOutputLowOnPin(rowPins[row].port, rowPins[row].pin);  // pull the row pins low


    int col;
    for ( col=0; col<3; col++)
    {  // embedded for loop to check all 3 columns of each row
      if (GPIO_getInputPinValue(colPins[col].port, colPins[col].pin) == 0)
      {
        keypadStatus |= 1 << ((row+1)*3 + (col+1) - 4);  // set the status bit of the keypad return value
      }
    }
    GPIO_setAsInputPinWithPullUpResistor(rowPins[row].port, rowPins[row].pin);

  }

  return keypadStatus;
}

void drive_motor(){





            GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN1);
            GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN0);
            GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN3);
            GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN2);

            GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN1);
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN3);
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN2);
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0);




while(1) {
    /*GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN1);
    GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN0);
    GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN3);
    GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN2);
*/
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN1);
    GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN3);
    GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN2);
    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0);
}

}

void main(void)
{
//    Init_GPIO();  //Sets all pins to output low as a default
//    while(1){
//    int val = getKeypadStatus();
//    printf("%", val);
//    }
    ////
//    drive_motor();

//    read_keypad();

    char buttonState = 0; //Current button press state (to allow edge detection)

    /*
     * Functions with two underscores in front are called compiler intrinsics.
     * They are documented in the compiler user guide, not the IDE or MCU guides.
     * They are a shortcut to insert some assembly code that is not really
     * expressible in plain C/C++. Google "MSP430 Optimizing C/C++ Compiler
     * v18.12.0.LTS" and search for the word "intrinsic" if you want to know
     * more.
     * */

    //Turn off interrupts during initialization
    __disable_interrupt();

    //Stop watchdog timer unless you plan on using it
    WDT_A_hold(WDT_A_BASE);

    // Initializations - see functions for more detail
    Init_GPIO();  //Sets all pins to output low as a default
    Init_PWM();   //Sets up a PWM output
    Init_ADC();   //Sets up the ADC to sample
    Init_Clock(); //Sets up the necessary system clocks
    Init_UART();  //Sets up an echo over a COM port
    Init_LCD();   //Sets up the LaunchPad LCD display

    PMM_unlockLPM5(); //Disable the GPIO power-on default high-impedance mode to activate previously configured port settings

    //All done initializations - turn interrupts back on.
    __enable_interrupt();

    // a1 p5.0
    // a2 p2.5
    // pwma p1.6
    // standby p1.3

    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN5);
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN3);
    GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN0);

    //
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN6);
    //

    //1.1 ain2  //
    // 8.0 ain1 ain2
    // 8.3 pwma  ///
    // 8.2 stdby a1in

    // 1.6 led
    // 5.0 hall effect

//    while (1)
//    {
//        __delay_cycles(0);
//    }

//        GPIO_setOutputHighOnPin (GPIO_PORT_P5,GPIO_PIN1);

//        GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN1);
//        GPIO_setAsInputPin(GPIO_PORT_P1, GPIO_PIN1); //1 is input


            int i = 0;
            volatile int valOfHallEffect = 0;
            while(1)
            {

                int keyVal = getKeypadStatus();
    //            valOfHallEffect = P1IN;
                    //printf("%d\n", P1IN);
                printf("%d", keyVal);
                switch(keyVal)
                  {
                    case 1:  // 0x001
                                           showChar('1',4);
                      break;
                    case 2:  // 0x002
                        showChar('2',4);
                      break;
                    case 4:  // 0x004
                        showChar('3',4);
                      break;
                    case 8:  // 0x008
                        showChar('4',4);
                      break;
                    case 16:  // 0x010
                        showChar('5',4);
                      break;
                    case 32:  // 0x020
                        showChar('6',4);
                      break;
                    case 64:  // 0x040
                        showChar('7',4);
                      break;
                    case 128:  // 0x080
                        showChar('8',4);
                      break;
                    case 256:  // 0x100
                        showChar('9',4);
                      break;
                    case 512:  // 0x200
                        showChar('*',4);
                      break;
                    case 1024:  // 0x400
                        showChar('0',4);
                      break;
                    case 2048:  // 0x800
                        showChar('#',4);
                      break;
                  }

               if ( GPIO_getInputPinValue(GPIO_PORT_P5, GPIO_PIN0 ) == 1)
               {
                   //displayScrollText("LED ON");
//                   showChar('H',4);
//                   showChar('I',6);
                   GPIO_setOutputHighOnPin (GPIO_PORT_P1,GPIO_PIN6);

                   GPIO_setOutputLowOnPin (GPIO_PORT_P8,GPIO_PIN0);
                   GPIO_setOutputHighOnPin (GPIO_PORT_P8,GPIO_PIN2);

                  // clearLCD();
               }
               else
               {
                   //displayScrollText("hello");
//                   showChar('L',4);
//                   showChar('O',6);
                   GPIO_setOutputLowOnPin (GPIO_PORT_P1,GPIO_PIN6);

                   GPIO_setOutputHighOnPin (GPIO_PORT_P8,GPIO_PIN0);
                   GPIO_setOutputLowOnPin (GPIO_PORT_P8,GPIO_PIN2);
               }
               __delay_cycles(2000);
               clearLCD();



            }

    //
    //     /*
    //     * The MSP430 MCUs have a variety of low power modes. They can be almost
    //     * completely off and turn back on only when an interrupt occurs. You can
    //     * look up the power modes in the Family User Guide under the Power Management
    //     * Module (PMM) section. You can see the available API calls in the DriverLib
    //     * user guide, or see "pmm.h" in the driverlib directory. Unless you
    //     * purposefully want to play with the power modes, just leave this command in.
    //     */
    //
    //
    //    displayScrollText("ECE 298");
    //
    //    while(1) //Do this when you want an infinite loop of code
    //    {
    //        //Buttons SW1 and SW2 are active low (1 until pressed, then 0)
    //        if ((GPIO_getInputPinValue(SW1_PORT, SW1_PIN) == 1) & (buttonState == 0)) //Look for rising edge
    //        {
    //            Timer_A_stop(TIMER_A0_BASE);    //Shut off PWM signal
    //            buttonState = 1;                //Capture new button state
    //        }
    //        if ((GPIO_getInputPinValue(SW1_PORT, SW1_PIN) == 0) & (buttonState == 1)) //Look for falling edge
    //        {
    //            Timer_A_outputPWM(TIMER_A0_BASE, &param);   //Turn on PWM
    //            buttonState = 0;                            //Capture new button state
    //        }
    //
    //        //Start an ADC conversion (if it's not busy) in Single-Channel, Single Conversion Mode
    //        if (ADCState == 0)
    //        {
    //            showHex((int)ADCResult); //Put the previous result on the LCD display
    //            ADCState = 1; //Set flag to indicate ADC is busy - ADC ISR (interrupt) will clear it
    //            ADC_startConversion(ADC_BASE, ADC_SINGLECHANNEL);
    //        }
    //    }
    //
    //    /*
    //     * You can use the following code if you plan on only using interrupts
    //     * to handle all your system events since you don't need any infinite loop of code.
    //     *
    //     * //Enter LPM0 - interrupts only
    //     * __bis_SR_register(LPM0_bits);
    //     * //For debugger to let it know that you meant for there to be no more code
    //     * __no_operation();
    //    */
}

void Init_GPIO(void)
{
    // Set all GPIO pins to output low to prevent floating input and reduce power consumption
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);

    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);

    //Set LaunchPad switches as inputs - they are active low, meaning '1' until pressed
//    GPIO_setAsInputPinWithPullUpResistor(SW1_PORT, SW1_PIN);
//    GPIO_setAsInputPinWithPullUpResistor(SW2_PORT, SW2_PIN);

    GPIO_setAsInputPin(GPIO_PORT_P5, GPIO_PIN0);

    //Set LED1 and LED2 as outputs
    //GPIO_setAsOutputPin(LED1_PORT, LED1_PIN); //Comment if using UART
    GPIO_setAsOutputPin(LED2_PORT, LED2_PIN);
}

/* Clock System Initialization */
void Init_Clock(void)
{
    /*
     * The MSP430 has a number of different on-chip clocks. You can read about it in
     * the section of the Family User Guide regarding the Clock System ('cs.h' in the
     * driverlib).
     */

    /*
     * On the LaunchPad, there is a 32.768 kHz crystal oscillator used as a
     * Real Time Clock (RTC). It is a quartz crystal connected to a circuit that
     * resonates it. Since the frequency is a power of two, you can use the signal
     * to drive a counter, and you know that the bits represent binary fractions
     * of one second. You can then have the RTC module throw an interrupt based
     * on a 'real time'. E.g., you could have your system sleep until every
     * 100 ms when it wakes up and checks the status of a sensor. Or, you could
     * sample the ADC once per second.
     */
    //Set P4.1 and P4.2 as Primary Module Function Input, XT_LF
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN1 + GPIO_PIN2, GPIO_PRIMARY_MODULE_FUNCTION);

    // Set external clock frequency to 32.768 KHz
    CS_setExternalClockSource(32768);
    // Set ACLK = XT1
    CS_initClockSignal(CS_ACLK, CS_XT1CLK_SELECT, CS_CLOCK_DIVIDER_1);
    // Initializes the XT1 crystal oscillator
    CS_turnOnXT1LF(CS_XT1_DRIVE_1);
    // Set SMCLK = DCO with frequency divider of 1
    CS_initClockSignal(CS_SMCLK, CS_DCOCLKDIV_SELECT, CS_CLOCK_DIVIDER_1);
    // Set MCLK = DCO with frequency divider of 1
    CS_initClockSignal(CS_MCLK, CS_DCOCLKDIV_SELECT, CS_CLOCK_DIVIDER_1);
}

/* UART Initialization */
void Init_UART(void)
{
    /* UART: It configures P1.0 and P1.1 to be connected internally to the
     * eSCSI module, which is a serial communications module, and places it
     * in UART mode. This let's you communicate with the PC via a software
     * COM port over the USB cable. You can use a console program, like PuTTY,
     * to type to your LaunchPad. The code in this sample just echos back
     * whatever character was received.
     */

    //Configure UART pins, which maps them to a COM port over the USB cable
    //Set P1.0 and P1.1 as Secondary Module Function Input.
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN1, GPIO_PRIMARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P1, GPIO_PIN0, GPIO_PRIMARY_MODULE_FUNCTION);

    /*
     * UART Configuration Parameter. These are the configuration parameters to
     * make the eUSCI A UART module to operate with a 9600 baud rate. These
     * values were calculated using the online calculator that TI provides at:
     * http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
     */

    //SMCLK = 1MHz, Baudrate = 9600
    //UCBRx = 6, UCBRFx = 8, UCBRSx = 17, UCOS16 = 1
    EUSCI_A_UART_initParam param = {0};
    param.selectClockSource = EUSCI_A_UART_CLOCKSOURCE_SMCLK;
    param.clockPrescalar = 6;
    param.firstModReg = 8;
    param.secondModReg = 17;
    param.parity = EUSCI_A_UART_NO_PARITY;
    param.msborLsbFirst = EUSCI_A_UART_LSB_FIRST;
    param.numberofStopBits = EUSCI_A_UART_ONE_STOP_BIT;
    param.uartMode = EUSCI_A_UART_MODE;
    param.overSampling = 1;

    if (STATUS_FAIL == EUSCI_A_UART_init(EUSCI_A0_BASE, &param))
    {
        return;
    }

    EUSCI_A_UART_enable(EUSCI_A0_BASE);

    EUSCI_A_UART_clearInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);

    // Enable EUSCI_A0 RX interrupt
    EUSCI_A_UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
}

/* EUSCI A0 UART ISR - Echoes data back to PC host */
#pragma vector = USCI_A0_VECTOR
__interrupt void EUSCIA0_ISR(void)
{
    uint8_t RxStatus = EUSCI_A_UART_getInterruptStatus(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG);

    EUSCI_A_UART_clearInterrupt(EUSCI_A0_BASE, RxStatus);

    if (RxStatus)
    {
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, EUSCI_A_UART_receiveData(EUSCI_A0_BASE));
    }
}

/* PWM Initialization */
void Init_PWM(void)
{
    /*
     * The internal timers (TIMER_A) can auto-generate a PWM signal without needing to
     * flip an output bit every cycle in software. The catch is that it limits which
     * pins you can use to output the signal, whereas manually flipping an output bit
     * means it can be on any GPIO. This function populates a data structure that tells
     * the API to use the timer as a hardware-generated PWM source.
     *
     */
    //Generate PWM - Timer runs in Up-Down mode
    param.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    param.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    param.timerPeriod = TIMER_A_PERIOD; //Defined in main.h
    param.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_1;
    param.compareOutputMode = TIMER_A_OUTPUTMODE_RESET_SET;
    param.dutyCycle = HIGH_COUNT; //Defined in main.h

    //PWM_PORT PWM_PIN (defined in main.h) as PWM output
    GPIO_setAsPeripheralModuleFunctionOutputPin(PWM_PORT, PWM_PIN, GPIO_PRIMARY_MODULE_FUNCTION);
}

void Init_ADC(void)
{
    /*
     * To use the ADC, you need to tell a physical pin to be an analog input instead
     * of a GPIO, then you need to tell the ADC to use that analog input. Defined
     * these in main.h for A9 on P8.1.
     */

    //Set ADC_IN to input direction
    GPIO_setAsPeripheralModuleFunctionInputPin(ADC_IN_PORT, ADC_IN_PIN, GPIO_PRIMARY_MODULE_FUNCTION);

    //Initialize the ADC Module
    /*
     * Base Address for the ADC Module
     * Use internal ADC bit as sample/hold signal to start conversion
     * USE MODOSC 5MHZ Digital Oscillator as clock source
     * Use default clock divider of 1
     */
    ADC_init(ADC_BASE,
             ADC_SAMPLEHOLDSOURCE_SC,
             ADC_CLOCKSOURCE_ADCOSC,
             ADC_CLOCKDIVIDER_1);

    ADC_enable(ADC_BASE);

    /*
     * Base Address for the ADC Module
     * Sample/hold for 16 clock cycles
     * Do not enable Multiple Sampling
     */
    ADC_setupSamplingTimer(ADC_BASE,
                           ADC_CYCLEHOLD_16_CYCLES,
                           ADC_MULTIPLESAMPLESDISABLE);

    //Configure Memory Buffer
    /*
     * Base Address for the ADC Module
     * Use input ADC_IN_CHANNEL
     * Use positive reference of AVcc
     * Use negative reference of AVss
     */
    ADC_configureMemory(ADC_BASE,
                        ADC_IN_CHANNEL,
                        ADC_VREFPOS_AVCC,
                        ADC_VREFNEG_AVSS);

    ADC_clearInterrupt(ADC_BASE,
                       ADC_COMPLETED_INTERRUPT);

    //Enable Memory Buffer interrupt
    ADC_enableInterrupt(ADC_BASE,
                        ADC_COMPLETED_INTERRUPT);
}

//ADC interrupt service routine
#pragma vector = ADC_VECTOR
__interrupt void ADC_ISR(void)
{
    uint8_t ADCStatus = ADC_getInterruptStatus(ADC_BASE, ADC_COMPLETED_INTERRUPT_FLAG);

    ADC_clearInterrupt(ADC_BASE, ADCStatus);

    if (ADCStatus)
    {
        ADCState = 0; //Not busy anymore
        ADCResult = ADC_getResults(ADC_BASE);
    }
}
