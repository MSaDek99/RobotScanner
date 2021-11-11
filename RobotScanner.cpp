/* File: Group13.c
   Author: Mohammed Elsayed 
   This program makes the robot's LEDs flash for 3 seconds, then scan the room for the beacon and reach it while avoiding obstacles
*/

#include <xc.h> //Inclding the libraries needed for all the functions 
#include <p18f2221.h>
#include <stdio.h>
#include <stdlib.h>
#pragma config OSC = HS //High speed resonator
#pragma config WDT = OFF //Watchdog timer off
#pragma config LVP = OFF //Low voltage programmer disabled
#pragma config PWRT = ON //Power up timer on
#define LED1 LATBbits.LB2 //LED1
#define LED2 LATBbits.LB3//LED2
#define LED3 LATBbits.LB4//LED3
#define LED4 LATBbits.LB5//LED4
#define LeftBeacon PORTCbits.RC3//Left Beacon
#define RightBeacon PORTCbits.RC4// Right Beacon
#define _XTAL_FREQ 10000000 // Define Clock Frequency for _delay_10ms()
void wait10ms(int del); // Function to generate a delay in multiples of 10ms
void LED_Flash(void); // Function to make LEDs flash for 3 seconds
void setupADC(void); //Function to configure A/D
unsigned int readADC_Left(void); // Function to read from left distance sensor from its A/D respective channel
unsigned int readADC_Right(void); // Function to read from Right distance sensor from its A/D respective channel
void rotate_anti(int markspace); // Function to rotate the robot anti-clockwise
void stop(void); // Function to stop the robot
void move_forward(int markspace); // Function to move the robot forward at a certain input speed between 0 and 255
void rotate_clock(int markspace); // Function to rotate the robot clockwise
void L_right(int markspace); // Function to avoid an obstacle if spotted by the right sensor by moving in an L shape
void L_left(int markspace); // Function to avoid an obstacle if spotted by the left sensor by moving in an L shape

int main(void)

{

int setpoint_distance=350; //Setting setpoint_distance as 350
TRISA=0b111001111;// Turn on the H Bridge forward
TRISB=0b110000000; //Configure Port B
TRISC=0b111111001; //Set CCP1(pin13) to an output pin
ADCON1=0b11001101; //Set up ADCON1 to configure which PORTA pins are I/O and which are A/D
PR2 = 0b11111111 ; //Set period of PWM
T2CON = 0b00000111 ; //Timer 2(TMR2) on, Prescaler = 16
CCP1CON = (0x0c); // 0x0c enables PWM module CCP1
CCP2CON = (0x0c); // 0x0c enables PWM module CCP2
stop(); // Calling stop function to stop the robot
int markspace=250, m; //Initializing  and defining m
LED_Flash();  //Calling LED_Flash function to make LEDs flash for 3 seconds
rotate_anti(200);//Calling rotate_anti function to rotate the robot anti-clockwise
wait10ms(100);//Wait 1 second
stop();// Calling stop function to stop the robot
unsigned int R,L;//Initializing R,l
for(m=0;m<4;m+1)//loop through 4 times for all possible cases of sensor detections
{
    unsigned int R,L;//Initializing R,l
    setupADC();//Calling the Function setupADC that configure A/D
    L=readADC_Left();//Defining L as readADC_Left Function
    R=readADC_Right();//Defining R as readADC_Right Function
    if(R<=setpoint_distance)//Compares the value of R with the predefined setpoint_distance
    {
        if(L<=setpoint_distance)//Compares the value of L if less than or equal to the predefined setpoint_distance
        {
           
    if(LeftBeacon==1&&RightBeacon==0)//If the LeftBeacon detects something unlike the RightBeacon
        {
        int markspace=200; //mark space value for PWM (50% mark space ratio)
LATAbits.LA4=0;// Left motor backward direction is enabled
LATAbits.LA5=1;// Left motor forward direction is disabled
LATBbits.LB0=1;// Right motor backward direction is enabled
LATBbits.LB1=0;// Right motor forward direction is disabled
CCPR1L = markspace; //Load duty cycle into CCP1CON, PWM begins
CCPR2L = markspace; //Load duty cycle into CCP2CON, PWM begins
        }
    if(LeftBeacon==1&&RightBeacon==1)//If the LeftBeacon detects something along with  the RightBeacon
        {
        LED1=0;//Turn LED1 off
        LED4=0;//Turn LED4 off
        }
    else
        {
            int markspace=225; //mark space value for PWM (50% mark space ratio)
LATAbits.LA4=0;// Left motor backward direction is enabled
LATAbits.LA5=1;// Left motor forward direction is disabled
LATBbits.LB0=0;// Right motor backward direction is enabled
LATBbits.LB1=1;// Right motor forward direction is disabled
CCPR1L = markspace; //Load duty cycle into CCP1CON, PWM begins
CCPR2L = markspace; //Load duty cycle into CCP2CON, PWM begins
        }

    }
        else if(L>setpoint_distance)//Compare L if more than predefined setpoint_distance
        {
            L_right(200);//Calling L_right function that enables the robot to avoid an obstacle if spotted by the right sensor by moving in an L shape
        }
    }
    setupADC();//Calling the Function setupADC that configure A/D
    L=readADC_Left();//Defining L as readADC_Left Function
    R=readADC_Right();//Defining R as readADC_Right Function
    if(R>setpoint_distance)//Comparing if R is more than the predefined setpoint_distance 
    {
        if(L<=setpoint_distance)//Comparing if L is more than the predefined setpoint_distance
        {
            L_left(200);//Calling L_left function that enables the robot to an obstacle if spotted by the left sensor by moving in an L shape
        }
        else if(L>setpoint_distance)//Comparing if L is more than the predefined setpoint distance
        {
            L_left(200);////Calling L_left function that enables the robot to avoid an obstacle if spotted by the left sensor by moving in an L shape
        }
    }
}
stop();// Calling stop function to stop the robot
}
void wait10ms(int del) //  Function to generate a delay in multiples of 10ms
{ 
    unsigned char c; // declaring a char c 
    for(c=0;c<del;c++) // loop the delay function del times
    __delay_ms(10); // call 10ms delay function "delay" times e.g. wait10ms(100) is a 1 second delay
    return; // return nothing
}

void LED_Flash(void) // function to make LEDs flash for 3 seconds
{
    int d; // declaring an integer d
    PORTB=0; // turn all LEDs off (TRISB is initially set)
    for(d=0;d<3;d++) // loop through 3 times to switch on and off 
        {
            LED1=1; // turn LED1 on
            LED2=1; // turn LED2 on
            LED3=1; // turn LED3 on
            LED4=1; // turn LED4 on
            wait10ms(50); //wait half a second
            LED1=0; //turn LED1 off
            LED2=0; //turn LED2 off
            LED3=0; //turn LED3 off
            LED4=0; //turn LED4 off
            wait10ms(50); //wait half a second
        }
    return; // return nothing
}
void setupADC(void) // function to configure A/D
{
    ADCON2bits.ADCS0=0; // Fosc/32
    ADCON2bits.ADCS1=1; // bits 0 to 2 are used for selecting the A/D conversion clock speed
    ADCON2bits.ADCS2=0;
    ADCON2bits.ADFM=1; //A/D result right justified
    ADCON1=0b00001110; //Set voltage reference and port A0 as A/D
    ADCON0bits.ADON=1; //Turn on ADC
}

unsigned int readADC_Left(void) // function to read from left distance sensor from its A/D respective channel
{
    ADCON0bits.CHS0=1; // 1000, channel is set
    ADCON0bits.CHS1=0; //use binary number to select ADC channel
    ADCON0bits.CHS2=0; 
    ADCON0bits.CHS3=0; //Channel set
    ADCON0bits.GO=1; // Conversion status bit. If it is 1 then conversion is in progress; if it is 0 then conversion is complete
    while (ADCON0bits.GO); //do nothing while conversion in progress
    return ((ADRESH<<8)+ADRESL); //Combines high & low bytes into one 16 bit value and returns result (A/D value 0-1023)
}

unsigned int readADC_Right(void) // function to read from right distance sensor from its respective A/D channel
{
    ADCON0bits.CHS0=0; // 0000,channel 0 is set,
    ADCON0bits.CHS1=0; // use binary number to select ADC channel
    ADCON0bits.CHS2=0; 
    ADCON0bits.CHS3=0; // Channel set
    ADCON0bits.GO=1; // Conversion status bit. If it is 1 then conversion is in progress; if it is 0 then conversion is complete
    while (ADCON0bits.GO); // do nothing while conversion in progress
    return ((ADRESH<<8)+ADRESL); // Combines high & low bytes into one 16 bit value and returns Result (A/D value 0-1023)
} 

void rotate_anti(int markspace) // function to rotate the robot anti-clockwise
{
    LATAbits.LA4=1; // Left motor backward direction is enabled
    LATAbits.LA5=0; // Left motor forward direction is disabled
    LATBbits.LB0=0; // Right motor backward direction is disabled
    LATBbits.LB1=1; // Right motor forward direction is enabled
    CCPR1L = markspace; //Load duty cycle into CCP1CON, PWM begins, to set speed
    CCPR2L = markspace; //Load duty cycle into CCP2CON, PWM begins, to set speed
    return; // return nothing
}
void stop(void) // function to stop the robot
{
    LATAbits.LA4=0; // Switching of all motor directions so the robot stops
    LATAbits.LA5=0;
    LATBbits.LB0=0;
    LATBbits.LB1=0;
    CCPR1L = 0; //Load duty cycle into CCP1CON, PWM begins, 0 speed so it doesn't move
    CCPR2L = 0; //Load duty cycle into CCP2CON, PWM begins, 0 speed so it doesn't move
    return; // return nothing
}
void move_forward(int markspace) // function to move the robot forward at a certain input speed between 0 and 255
{
    LATAbits.LA4=0; // Left motor backward direction is disabled
    LATAbits.LA5=1; // Left motor forward direction is enabled
    LATBbits.LB0=0; // Right motor backward direction is disabled
    LATBbits.LB1=1; // Right motor forward direction is enabled
    CCPR1L = markspace; //Load duty cycle into CCP1CON, PWM begins, set at certain speed    
    CCPR2L = markspace; //Load duty cycle into CCP2CON, PWM begins, set at certain speed
    return; // return nothing
}
void rotate_clock(int markspace) // function to rotate the robot clockwise
{
    LATAbits.LA4=0; // Left motor backward direction is disabled
    LATAbits.LA5=1; // Left motor forward direction is enabled
    LATBbits.LB0=1; // Right motor backward direction is enabled
    LATBbits.LB1=0; // Right motor forward direction is disabled
    CCPR1L = markspace; //Load duty cycle into CCP1CON, PWM begin
    CCPR2L = markspace; //Load duty cycle into CCP2CON, PWM begins
    return;// return nothing
}
void L_left(int markspace) // function to avoid an obstacle if spotted by the left sensor by moving in an L shape
{
    rotate_anti(200); // Rotating anti clockwise for 0.6 seconds and then stop
    wait10ms(60);
    stop();
    move_forward(150); // move forward for 1.5 seconds and then stop
    wait10ms(150);
    stop();
    rotate_clock(200); // Rotating clockwise for 0.6 seconds and then stop
    wait10ms(60);
    stop();
    move_forward(150); // move forward for 2.5 seconds
    wait10ms(250);
    return; // return nothing
}
void L_right(int markspace) // function to avoid an obstacle if spotted by the right sensor by moving in an L shape
{
    rotate_clock(200); // Rotating clockwise for 0.6 seconds and then stop
    wait10ms(60);//wait for 0.6 seconds
    stop();
    move_forward(150); // move forward for 1.5 seconds and then stop
    wait10ms(150);//wait 1 second and a half
    stop();
    rotate_anti(200); // Rotating anti clockwise for 0.6 seconds and then stop
    wait10ms(60); //wait 0.6 seconds
    stop();
    move_forward(150); // move forward for 2.5 seconds and then stop
    wait10ms(250);//wait  two seconds and a half
    return; // return nothing
}
