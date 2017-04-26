/*
 * File:   main.c
 * Author: john
 *
 * Created on April 26, 2017, 12:19 PM
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <xc.h>

//Configuration FOSC
#pragma config FCKSM = 3
#pragma config OSCIOFNC = 0
#pragma config POSCMD = 3

#pragma config IESO = 0
#pragma config FNOSC = 7

#pragma config PWMPIN = 0
#pragma config HPOL = 1
#pragma config LPOL = 1
#pragma config FWDTEN = 0

#define OUTPUT 0
#define INPUT 1
#define HIGH 1
#define LOW 0
#define TRUE 1
#define FALSE 0

/*
 *
 */

enum port_t{
    RA = 0,
    RB
};

struct gpio_t{
    const port_t port;
    const uint8_t pin;
    uint8_t dir;
};

uint8_t gpioSetDir(gpio_t * pin,  uint8_t dir)
{
    pin->dir = dir;
    uint8_t ret = TRUE;
    if(pin->port == RA)
    {
        if(dir == OUTPUT)
        {
            TRISA &= ~(1 << pin);
        }
        else
        {
            TRISA |= (1 << pin);
        }
    }
    else if(pin->port == RB)
    {
        if(dir == OUTPUT)
        {
            TRISB &= ~(1 << pin);
        }
        else
        {
            TRISB |= (1 << pin);
        }
    }
    else
    {
        ret = FALSE;
    }

    return ret;
}

void gpioWrite(gpio_t * pin, uint8_t val)
{
    uint8_t ret = TRUE;
    if(pin->dir == OUTPUT)
    {
        if(pin->port == RA)
        {
            if(val == LOW)
            {
                LATA &= ~(1 << pin);
            }
            else
            {
                LATA |= (1 << pin);
            }
        }
        else if(pni->port == RB)
        {
            if(val == LOW)
            {
                LATB &= ~(1 << pin);
            }
            else
            {
                LATB |= (1 << pin);
            }
        }
        else
        {
            ret = FALSE;
        }
    }
    else
    {
        ret = FALSE;
    }

    return ret;
}

uint8_t gpioRead(gpio_t * pin)
{
    uint8_t ret;
    if(pin->port == RA)
    {
        ret = (PORTA & (1 << pin)) ? 1 : 0;
    }
    else if(pin->port == RB)
    {
        ret = (PORTB & (1 << pin)) ? 1 : 0;
    }

    return ret;
}

void pwmWrite(uint8_t pin, uint8_t value)
{
    if(pin == 1)
    {
        P1DC1 = value;
    }
    else if(pin == 2)
    {
        P1DC2 = value;
    }
}

//value is -1.0 to 1.0
void motorWrite(uint8_t motor, float value)
{
    uint8_t dir = value >= 1 ? 1 : 0;
    uint8_t pwmVal = (value < 0 ? -value : value) * 255;

    if(motor == 1)
    {
        if(dir == 0)
        {
            gpioWrite(MOTOR_1_INA, HIGH);
            gpioWrite(MOTOR_1_INB, LOW);
        }
        else
        {
            gpioWrite(MOTOR_1_INA, LOW);
            gpioWrite(MOTOR_1_INB, HIGH);
        }

        pwmWrite(1, pwmVal);
    }
    else if(motor == 2)
    {
        if(dir == 0)
        {
            gpioWrite(MOTOR_2_INA, HIGH);
            gpioWrite(MOTOR_2_INB, LOW);
        }
        else
        {
            gpioWrite(MOTOR_2_INA, LOW);
            gpioWrite(MOTOR_2_INB, HIGH);
        }

        pwmWrite(2, pwmVal);
    }
}

//Stub
//Should return the robots tilt from vertical in degrees
float getTilt()
{

}

gpio_t MOTOR_1_INA = {RA, 0, OUTPUT};
gpio_t MOTOR_1_INB = {RA, 1, OUTPUT};
gpio_t MOTOR_2_INA = {RB, 0, OUTPUT};
gpio_t MOTOR_2_INA = {RB, 1, OUTPUT};

int main(int argc, char** argv) {

    OSCTUNbits.TUN = 23;

    CLKDIVbits.FRCDIV = 0;
    CLKDIVbits.DOZE = 0;
    CLKDIVbits.DOZEN = 1;

    P1TCONbits.PTEN = 0;
    P1TCONbits.PTCKPS = 0;
    P1TCONbits.PTMOD = 0;

    P1TMRbits.PTMR = 0;
    P1TPER = 128;

    PWM1CON1bits.PMOD3 = 1;
    PWM1CON1bits.PMOD2 = 1;
    PWM1CON1bits.PMOD1 = 1;
    PWM1CON1bits.PEN1L = 1;
    PWM1CON1bits.PEN1H = 1;
    PWM1CON1bits.PEN2L = 0;
    PWM1CON1bits.PEN2H = 0;
    PWM1CON1bits.PEN3L = 0;
    PWM1CON1bits.PEN3H = 0;

    PWM1CON2bits.IUE = 0;
    PWM1CON2bits.UDIS = 0;

    P1DC1 = 0;
    P1DC2 = 0;

    AD1PCFGL = 0xFFFF;
    gpioSetDir(MOTOR_1_INA, OUTPUT);
    gpioSetDir(MOTOR_1_INB, OUTPUT);
    gpioSetDir(MOTOR_2_INA, OUTPUT);
    gpioSetDir(MOTOR_2_INB, OUTPUT);

    P1TCONbits.PTEN = 1;

    while(true)
    {

    }

    return (EXIT_SUCCESS);
}

