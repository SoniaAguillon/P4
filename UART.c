#include "lib/include.h"

extern void Configurar_UART6(void)
{
    SYSCTL->RCGCUART  = (1<<6);   //Paso 1 (RCGCUART) pag.344 UART/modulo6 0->Disable 1->Enable    ** UART6
    SYSCTL->RCGCGPIO |= (1<<3);     //Paso 2 (RCGCGPIO) pag.340 Enable clock port A *Se activa el registro de reloj del GPIO, para el UART6 corresponde puerto D, el cual es el bit 4*    
    //(GPIOAFSEL) pag.671 Enable alternate function
    GPIOA_AHB->AFSEL = (1<<4) | (1<<5); //*Corresponde puerto D, pines 4 y 5
    //GPIO Port Control (GPIOPCTL) PA0-> U0Rx PA1-> U0Tx pag.688
    GPIOA_AHB->PCTL = (GPIOD_AHB->PCTL&0xFFFFFF00) | 0x00110000;// (1<<0) | (1<<4);//0x00000011   ** El pin 4 y 5 corresponde al port mux 4 y 5, por ello se pone en 1 el 16 y 20, checar tabla***
    // GPIO Digital Enable (GPIODEN) pag.682
    GPIOA_AHB->DEN = (1<<4) | (1<<5);//PD4 PD5  **Puerto D, pines 4 y 5 habilitados como digitales**
    //UART0 UART Control (UARTCTL) pag.918 DISABLE!!
    UART0->CTL = (0<<9) | (0<<8) | (0<<0);

    // UART Integer Baud-Rate Divisor (UARTIBRD) pag.914
    /*
    BRD = 20,000,000 / (16 * 9600) = 130.2
    UARTFBRD[DIVFRAC] = integer(0.2 * 64 + 0.5) = 14

 //CALCULO VELOCIDAD COMUNICACIÓN
    /*

    55Mhz   115200

    BRD = 55,000,000/(16∗115,200) = 29.8394
    
    Redondeo --->    UARTFBRD =  (.8394∗64) + 0.5 = 54.2216 = 54
   
    */
    UART6->IBRD = 29;
    // UART Fractional Baud-Rate Divisor (UARTFBRD) pag.915
    UART6->FBRD = 54;
    //  UART Line Control (UARTLCRH) pag.916
/* La configuración de la trama se hace con el regitro de nombre generico UARTLCRH - Son 8 bits que se deben configurar:
        Solo nos intereza configurar WLEN -Longitud de palabra UART - Los bits indican el número de bits de datos transmitidos o recibidos 
        FEN -Habilita si es que esta en 1 o deshabilita si es que esta en 0 los FIFOS de recepción y transmición Se recomienda siempre habilitarlos para tener más espacio de almacenamiento de datos
    */
    UART6->LCRH = (0x3<<5)|(1<<4); //WLEN (bit5 conf) - para 8 bits indicado en  0x3      FEN (bit4 conf) - Habilita (1)  los FIFOS

    //  UART Clock Configuration(UARTCC) pag.939        controla la fuente de reloj en baudios para el módulo UART.
    UART6->CC =(0<<0);//Se pone en 0 para indicar Reloj del sistema

    //Disable UART0 UART Control (UARTCTL) pag.918
    UART6->CTL = (1<<0) | (1<<8) | (1<<9); //Termino la configuración y ahora si se activa el UART




}

extern char readChar(void)
{
    //UART FR flag pag 911
    //UART DR data 906
    int v;
    char c;
    while((UART6->FR & (1<<4)) != 0 );
    v = UART6->DR & 0xFF;
    c = v;
    return c;
}
extern void printChar(char c)
{
    while((UART6->FR & (1<<5)) != 0 );
    UART0->DR = c;
}
extern void printString(char* string)
{
    while(*string)
    {
        printChar(*(string++));
    }
}

extern char * readString(char delimitador)
{

   int i=0;
   char *string = (char *)calloc(10,sizeof(char));
   char c = readChar();
   while(c != delimitador)
   {
       *(string+i) = c;
       i++;
       if(i%10==0)
       {
           string = realloc(string,(i+10)*sizeof(char));
       }
       c = readChar();
   }

   return string;

}
