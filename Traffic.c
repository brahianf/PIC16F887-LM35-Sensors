/*
 * File:   Traffic.c
 * Author: BrnH
 *
 * Created on 3 de agosto de 2019, 01:32 AM
 * 
 * Description_
 * 
 * 
 */

#pragma config CONFIG1 = 0x2CD4
#pragma config CONFIG2 = 0x0700
 
#include <xc.h>
#define _XTAL_FREQ 8000000
#include <stdio.h>         // for sprintf
#include <stdint.h>        // include stdint header
#include <stdlib.h>

#include "lcd.h"


unsigned int readADC(unsigned char channel);


/********************** UART functions **************************/
void UART_Init(const uint32_t baud_rate)
{
  int16_t n = ( _XTAL_FREQ / (16 * baud_rate) ) - 1;
  
  if (n < 0)
    n = 0;
 
  if (n > 255)  // low speed
  {
    n = ( _XTAL_FREQ / (64 * baud_rate) ) - 1;
    if (n > 255)
      n = 255;
    SPBRG = n;
    TXSTA = 0x20;  // transmit enabled, low speed mode
  }
 
  else   // high speed
  {
    SPBRG = n;
    TXSTA = 0x24;  // transmit enabled, high speed mode
  }
 
  RCSTA = 0x90;  // serial port enabled, continues receive enabled
 
}
 
__bit UART_Data_Ready()
{
  return RCIF;  // return RCIF bit (register PIR1, bit 5)
}
 
uint8_t UART_GetC()
{
  while (RCIF == 0) ;  // wait for data receive
  if (OERR)  // if there is overrun error
  {  // clear overrun error bit
    CREN = 0;
    CREN = 1;
  }
  return RCREG;        // read from EUSART receive data register
}
 
void UART_PutC(const char data)
{
  while (TRMT == 0);  // wait for transmit shift register to be empty
  TXREG = data;       // update EUSART transmit data register
}
 
void UART_Print(const char *data)
{
  uint8_t i = 0;
  while (data[i] != '\0')
    UART_PutC (data[i++]);
}
/********************** end UART functions **************************/
                                        
const char message[] = "Sensores Lab Digitales \r\n";


// main function
void main(void){
    
    int adc;
    float temp;
    unsigned char bufferLCD[16];                        //buffer para mostrar en pantalla
    
  //PUERTOS
    TRISB=0;                        // salida pantalla lcd
    //TRISD=0;                        //PORTD salida, 8 LEDs bits menos significativos
    
    TRISCbits.TRISC6=0;             //Puertos para transmision EUSART
    TRISCbits.TRISC7=1;

    ANSELH=0;                     //  Entradas y salidas Digitales
    ANSELbits.ANS0=1;             
    ANSELbits.ANS1=1;
    ANSELbits.ANS2=1;
    
    ADCON1=0b10000000;              //Justificacion a la derecha, Vref pin RA3, //Frecuencia de conversion
    //ADCON1=0b10100000;
    ADCON0=0b10000001;               //Fosc/32 para tiempo de conversion de 2us ; CAD Activada 
    
    OSCCON = 0x70;    // set internal oscillator to 8MHz
    UART_Init(9600);  // initialize UART module with 9600 baud
    __delay_ms(2000);  // wait 2 seconds
    UART_Print(message);  // UART print message
    __delay_ms(1000);  // wait 1 second
    
     Lcd_Init();                         //inicializamos el lcd
     Lcd_Cmd(LCD_CLEAR);                 //limpiamos lcd
     Lcd_Cmd(LCD_CURSOR_OFF);            //apagamos el cursor
     __delay_ms(100);                    //esperamos 100ms

     
    while(1)
    {
      
     adc=readADC(0); // Reading ADC values
 
     sprintf(bufferLCD,"Temp1 %03d ",adc);
      Lcd_Out2(1,1,bufferLCD);
      UART_Print(bufferLCD);
      __delay_ms(1000);  
      UART_Print(",");  // start new line
      
      adc=readADC(1); // Reading ADC values
    
      sprintf(bufferLCD,"T2 %03d ",adc);
      Lcd_Out2(2,1,bufferLCD); 
      UART_Print(bufferLCD);
      __delay_ms(1000);  
      UART_Print(",");  // start new line
        
      adc=readADC(2); // Reading ADC values
    
      sprintf(bufferLCD,"T3 %03d ",adc);
      Lcd_Out2(2,8,bufferLCD); 
      UART_Print(bufferLCD);
      __delay_ms(1000);  
      UART_Print("\r\n");  // start new line
     
      
        if ( UART_Data_Ready() ){  // if a character available
          
            uint8_t c = UART_GetC();  // read from UART and store in 'c'
            UART_PutC(c);  // send 'c' via UART (return the received character back)
        }

    }
 
  
}

unsigned int readADC(unsigned char channel){              
    
    int adc;
    
    TRISA=0x00;                      //
    TRISAbits.TRISA0=1;
    TRISAbits.TRISA1=1;
    TRISAbits.TRISA2=1;
   
    ADCON0&=0xC5;  
    ADCON0|=channel<<2;
    __delay_us(20);   
    
    if(channel==0){
        
        ADCON0bits.CHS=0;
        ADCON0bits.GO_nDONE=1;           //Lanza conversion
        __delay_us(20);                 // Tiempo Adquisición, condensador se cargue 
        while(ADCON0bits.GO_nDONE);     // espera que la conversion termine cuando ADCONbits.go=0;
        
        //lEER resultado en 10 bits
        adc=((ADRESH<<8)+ADRESL);                 //<<8; // 2 bits más significativos de los 10 con corrimiento de 8 bits a la iz  0b'00 xxxxxx' 
        adc=adc>>1;
        
    }
     __delay_us(20);                     //Acquisition time to charge hold capacitor  
    
    if(channel==1){
   
        ADCON0bits.CHS=1;
        ADCON0bits.GO_nDONE=1;           //Lanza conversion
        __delay_us(20);                 // Tiempo Adquisición, condensador se cargue 
        while(ADCON0bits.GO_nDONE);     // espera que la conversion termine cuando ADCONbits.go=0;
        
        //lEER resultado en 10 bits
        adc=((ADRESH<<8)+ADRESL);                 //<<8; // 2 bits más significativos de los 10 con corrimiento de 8 bits a la iz  0b'00 xxxxxx'
        adc=adc>>1;   
    }
     __delay_us(20);                    //Acquisition time to charge hold capacitor  
      
      if(channel==2){
         
        ADCON0bits.CHS=2;
        ADCON0bits.GO_nDONE=1;           //Lanza conversion
        __delay_us(20);                 // Tiempo Adquisición, condensador se cargue 
        while(ADCON0bits.GO_nDONE);     // espera que la conversion termine cuando ADCONbits.go=0;
        
        //lEER resultado en 10 bits
        adc=((ADRESH<<8)+ADRESL);                 //<<8; // 2 bits más significativos de los 10 con corrimiento de 8 bits a la iz  0b'00 xxxxxx'
        adc=adc>>1;
            
    }
      
    return adc;   
     
}

 /*if(channel<=4){
        
         TRISA=0xFF&&ch;
         
    }
    if((4<channel)  &&  (channel<=7)){
        TRISE=0xFF&&ch;
    }
    
    if(channel>=8){
        TRISB=0xFF&&ch;
    }*/

