
/* 
 * File:   main.c
 * Author: Hennie Kotze firmware@autonomech.co.za
 *
 * Created on 31 March 2019, 1:38 AM
 */

/******************************************************************************/
/*                             PROJECT PINOUT                                 */
/******************************************************************************/
/*                             ATMega328P
 *                                ______
 *            RESET/PCINT14/PC6 =|01* 28|= PC5/PCINT13/SCL/ADC5
 *               RX/PCINT16/PD0 =|02  27|= PC4/PCINT12/SDA/ADC4
 *               TX/PCINT17/PD1 =|03  26|= PC3/PCINT11/ADC3
 *             INT0/PCINT18/PD2 =|04  25|= PC2/PCINT10/ADC2
 *                  PCINT19/PD3 =|05  24|= PC1/PCINT9/ADC1
 *                  PCINT20/PD4 =|06  23|= PC0/PCINT8/ADC0
 *                          Vcc =|07  22|= GND
 *                          GND =|08  21|= Aref
 *             XTAL1/PCINT6/PB6 =|09  20|= AVcc
 *             XTAL2/PCINT7/PB7 =|10  19|= PB5/PCINT5/SCK
 *             OC0B/PCINT21/PD5 =|11  18|= PB4/PCINT4/MISO
 *        OC0A/AIN0/PCINT22/PD6 =|12  17|= PB3/PCINT3/MOSI/OC2A/OC2
 *             AIN1/PCINT23/PD7 =|13  16|= PB2/PCINT2/SS/OC1B
 *                   PCINT0/PB0 =|14  15|= PB1/PCINT1/OC1A
 *                                ------
 * 
 *                                ______
 *                              =|01* 28|= ***(Free)
 *                     UART___/ =|02  27|= PIN_RST (PC4)
 *                            \ =|03  26|= PIN_CS (PC3)
 *                 (PD2) PIN_02 =|04  25|= PIN_RS (PC2)
 *                 (PD3) PIN_03 =|05  24|= PIN_WR (PC1)
 *                 (PD4) PIN_04 =|06  23|= PIN_RD (PC0)
 *                          Vcc =|07  22|= GND
 *                          GND =|08  21|= Aref
 *                      XTAL__/ =|09  20|= AVcc
 *                            \ =|10  19|= \
 *                 (PD5) PIN_05 =|11  18|= |--SPI
 *                 (PD6) PIN_06 =|12  17|= /
 *                 (PD7) PIN_07 =|13  16|= ***(Free) DIAGNOSTIC OUTPUT / POWER PIN
 *                 (PB0) PIN_00 =|14  15|= PIN_01 (PB1) 
 *                                ------
 * 
 * 
 */



/******************************************************************************/
/*                                   DEFS                                     */
/******************************************************************************/

#define F_CPU           16000000
#define MEMSIZE         2048
#define TRUE            1
#define FALSE           0
#define ON              1
#define OFF             0
#define HIGH            1
#define LOW             0
#define NOP             asm("nop")
#define TX_BUF_LEN      8
#define RX_BUF_LEN      8

//---MACROS
#define CMD_EQ(A)           strcmp(A, rxbuf) == 0
#define CMD_EQN(A,N)        strncmp(A, rxbuf, N) == 0
#define ITOA(A)             itoa(A, txbuf, 10)
#define ITOA2(A)            itoa(A, txbuf, 2)
#define ITOA16(A)           itoa(A, txbuf, 16)

#define _readpin(a,b)       (a & b)
#define _setpin(a,b)        a |= b
#define _clearpin(a,b)      a &= ~(b)
#define _togglepin(a,b)     a ^= ba

//diagnostix
//#define PIN_DIAG    0x04
//#define PORT_DIAG   PORTB
#define PIN_POWER    0x04
#define PORT_POWER   PORTB

//
#define CHAR_COLS               0x08
#define CHAR_ROWS               0x06
#define CHAR_BIN_WITDH          0x28//40 pixels
#define CHAR_BIN_HEIGHT         0x50//80 pixels
#define CHAR_BBOX_WITDH         0x28//40 pixels
#define CHAR_BBOX_HEIGHT        0x50//80 pixels



/******************************************************************************/
/*                                 INCLUDES                                   */
/******************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include "math.h"
//#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
//#include <util/setbaud.h>
//#include <util/twi.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "ili9481.h"

/******************************************************************************/
/*                                VARIABLES                                   */

/******************************************************************************/

uint32_t timer_counter;
uint8_t status_byte = 0xff;


uint16_t white, black, red, green, blue;

bool flipflop = true;

/******************************************************************************/
/*                             STRUCTS AND ENUMS                              */

/******************************************************************************/


/******************************************************************************/
/*                            FUNCTION DECLARATIONS                           */
/******************************************************************************/

int main(void);
static void init(void);
void mainloop(void);
//usart
//i2c
//ADC
//eeprom
//timer
ISR(TIMER0_OVF_vect);
//util
uint8_t reverse_bits(uint8_t);
void delay_n_us(uint16_t);
void delay_n_ms(uint16_t);
int available_sram(void);

/******************************************************************************
 *                                FUNCTIONS                                   *
 ******************************************************************************/

int main(void)
{


    white = rgb_565(31, 63, 31);
    black = rgb_565(0, 0, 0);
    red = rgb_565(31, 0, 0);
    green = rgb_565(0, 63, 0);
    blue = rgb_565(0, 0, 31);

    init();
    _clearpin(PORT_POWER, PIN_POWER);
    _delay_ms(250);
    _setpin(PORT_POWER, PIN_POWER);
    _delay_ms(250);
    init_tftlcd();
    initFS(2, //size
           5, //font char px w
           7, //font char px h
           1, //font char pad h
           2, //font char pad v
           0, //xpos
           0, //ypos
           white);
    mainloop();
    return 0;
}

/*******************************************************************************
 *                                                                        INIT*/

static void init(void)
{

    //=======================================================================I/O
    //Direction registers port c 1=output 0=input
    /*If DDxn is written logic one, Pxn is configured as an output pin. 
     * If DDxn is written logic zero, Pxn is configured as an input pin.
     * If PORTxn is written logic one AND the pin is configured as an input pin (0), 
     * the pull-up resistor is activated.
     */
    DDRB |= PIN_TFT_00 | PIN_TFT_01 | PIN_POWER;
    PORTB |= 0xff;

    DDRC |= PIN_TFT_RST | PIN_TFT_CS | PIN_TFT_CMD | PIN_TFT_WR | PIN_TFT_RD;
    PORTC |= 0xff;

    DDRD |= PIN_TFT_02 | PIN_TFT_03 | PIN_TFT_04 | PIN_TFT_05 | PIN_TFT_06 | PIN_TFT_07;
    PORTD |= 0xff;

    //=======================================================================ADC

    //======================================================================UART

    //====================================================================TIMER0
    TCNT0 = 0x00;
    TCCR0B |= _BV(CS00); // | (1 << CS02);//prescaler
    TIMSK0 |= _BV(TOV0); //timer 0 overflow interrupt enable

    //=======================================================================I2C

    //================================================================INTERRUPTS
    sei();
}

/******************************************************************************
 *                                                                    MAINLOOP*/
void mainloop(void)
{
    
    
    
    draw_rect(0,0,319,320,white);
    
    
    int iX, iY;
    const int iXmax = 320;
    const int iYmax = 320;
    /* world ( double) coordinate = parameter plane*/
    double Cx, Cy;
    const double CxMin =0.3586499579;//-0.681249;//-2.5;//-1.264189487;// 
    const double CxMax =0.35945193842;//-0.575;// 1.5;//-1.262646922;//
    const double CyMin = 0.11175821721;//0.3999;// -2.0;//0.0410344597;//
    const double CyMax = 0.11256019771;//0.50625;// 2.0;//0.0425770239;//
    /* */
    double PixelWidth = (CxMax - CxMin) / iXmax;
    double PixelHeight = (CyMax - CyMin) / iYmax;
    static unsigned char color[3];
    /* Z=Zx+Zy*i  ;   Z0 = 0 */
    double Zx, Zy;
    double Zx2, Zy2; /* Zx2=Zx*Zx;  Zy2=Zy*Zy  */
    /*  */
    int Iteration;
    const int IterationMax = 500;
    /* bail-out value , radius of circle ;  */
    const double EscapeRadius = 2.0;
    double ER2 = EscapeRadius*EscapeRadius;
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    /*write ASCII header to the file*/
    //fprintf(fp, "P6\n %s\n %d\n %d\n %d\n", comment, iXmax, iYmax, MaxColorComponentValue);
    /* compute and write image data bytes to the file*/
    for (iY = 0; iY < iYmax; iY++)
    {
        Cy = CyMin + iY*PixelHeight;
        if (fabs(Cy) < PixelHeight / 2) Cy = 0.0; // Main antenna * /
        for (iX = 0; iX < iXmax; iX++)
        {
            Cx = CxMin + iX*PixelWidth;
            //initial value of orbit = critical point Z= 0 * /
            Zx = 0.0;
            Zy = 0.0;
            Zx2 = Zx*Zx;
            Zy2 = Zy*Zy;
          
            for (Iteration = 0; Iteration < IterationMax && ((Zx2 + Zy2) < ER2); Iteration++)
            {
                Zy = 2 * Zx * Zy + Cy;
                Zx = Zx2 - Zy2 + Cx;
                Zx2 = Zx*Zx;
                Zy2 = Zy*Zy;
            };
            // compute  pixel color (24 bit = 3 bytes) * /
            double iterationRatio  = (double)Iteration/(double)IterationMax;
            double green, red, blue;
            if (iterationRatio<=0.5) {
                green = 1.0-(iterationRatio*2);
                red = 1;//iterationRatio*2;
            }
            else {
                green = 0;
                red = 1.0-((0.5-iterationRatio)*2.0);
            }
            double nr, ng;
            nr = 31.0*red;
            ng=61*green;
            //  interior of Mandelbrot set = black * /
            if (Iteration == IterationMax) {
                color[0] = 0;
                color[1] = 0;
                color[2] = 0;
            } else {
                color[0] = (uint8_t) nr;
                color[1] = (uint8_t) ng;
                color[2] = 0;
            
            }
            
            //write color to the file* /
            //fwrite(color, 1, 3, fp);
            plot_pixel((uint16_t)iX,(uint16_t)iY,rgb_565(color[0],color[1],color[2]));
        }
    }
    
    draw_rect(0,0,319,320,white);
    
    
    
    
    pFS->position.y=330;
    print_str("Mandelbrot set\n");
    print_str("XMin: ");
    pFS->position.x = pFS->cursor.x;
    print_float(CxMin);
    print_str("\n");
    print_str("XMax: ");
    pFS->position.x = pFS->cursor.x;
    print_float(CxMax);
    print_str("\n");
    print_str("YMin: ");
    pFS->position.x = pFS->cursor.x;
    print_float(CyMin);
    print_str("\n");
    print_str("YMax: ");
    pFS->position.x = pFS->cursor.x;
    print_float(CyMax);
    print_str("\n");
    print_str("Iterations: ");
    pFS->position.x = pFS->cursor.x;
    print_int(IterationMax);
    print_str("\n");
    
    
    


    while (TRUE)
    {

        NOP;
    }
    return;
}


/******************************************************************************
 *                                                                       USART*/


/*****************************************************************************
 *                                                                     I2C LCD*/

/******************************************************************************
 *                                                                        ADC*/

/******************************************************************************
 *                                                                      TIMER0*/
ISR(TIMER0_OVF_vect)
{
    timer_counter++;
    if (timer_counter > 0x6FFF)
    {
        timer_counter = 0;
        if (status_byte > 1)
        {
            flipflop = !flipflop;
            if (flipflop)
            {
                fill_rect(310, 470, 5, 5, black);
            }
            else
            {

                fill_rect(310, 470, 5, 5, 0x1C3F);
            }
        }
    }
}

/*****************************************************************************
 *                                                                      EEPROM*/

/*****************************************************************************
 *                                                                        UTIL*/

uint8_t reverse_bits(uint8_t b)
{
    b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
    b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
    b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
    return b;
}

void delay_n_us(uint16_t n)
{
    while (n--)
    {
        _delay_us(1);
    }
    return;
}

void delay_n_ms(uint16_t n)
{
    while (n--)
    {
        _delay_ms(1);
    }
    return;
}

int available_sram(void)
{
    extern int __heap_start, *__brkval;
    int v;
    return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}