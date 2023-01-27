/*
 * ADC.c
 *
 *  Created on: 04/11/2019
 *      Author: ravenelco
 */
#include "lib/include.h"

extern void Configura_Reg_ADC0(void)
{
    /*
    Habilitar el modulo 0 del ADC con dos canales analogicos 
    en el puerto E a una velocidad de conversion de 250ksps
    dandole la mayor prioridad al secuenciador 2 con evento
    de procesador 
    */
     //Pag 396 para inicializar el modulo de reloj del adc RCGCADC
    SYSCTL->RCGCADC = (1<<0) | (1<<1); //Se inicializa el modulo 0 y 1 del ADC 
    //Pag 382 (RGCGPIO) Se habilitan los puertos que corresponden - Dependiendo de los pines que te tocaron ( me toco 1,3,4,5,11,9 - PUERTOS E, D Y B)
    //                     F     E      D       C      B     A
    SYSCTL->RCGCGPIO |= (0<<5)|(1<<4)|(1<<3)|(0<<2)|(1<<1)|(0<<0);
    //Pag 760 (GPIODIR) Habilta los pines como I/O un cero para entrada y un uno para salida (los ADC SON ENTRADAS - SE PONEN LOS PINES QUE NOS TOCARON)
    GPIOE_AHB->DIR = (0<<0) | (0<<2) | (0<<4); //PIN 0, 2,4  - PUERTO E COMO ENTRADAS
    GPIOD_AHB->DIR = (0<<3) | (0<<2); //PIN 3 Y 2 - PUERTO D COMO ENTRADAS
    GPIOB_AHB->DIR = (0<<5); //PIN 5 - PUERTO B COMO ENTRADAS
    
    
    //(GPIOAFSEL) pag.770 Habilitar funciones alternativas para que el modulo analógico tenga control de esos pines (PUERTO E, D, B) - SE PONEN EN 1
    GPIOE_AHB->AFSEL =  (1<<0) | (1<<2) | (1<<4);
    GPIOD_AHB->AFSEL =  (1<<2) | (1<<3);
    GPIOB_AHB->AFSEL =  (1<<5);

    //(GPIODEN) pag.781 desabilita el modo digital para los puertos E, D y B 
    GPIOE_AHB->DEN = (0<<4) | (0<<3) | (0<<1);
    GPIOD_AHB->DEN =  (0<<2) | (0<<3);
    GPIOB_AHB->DEN =  (0<<5);

    //Pag 787 GPIOPCTL registro combinado con el GPIOAFSEL y la tabla pag 1808
    //GPIOE_AHB->PCTL = GPIOE_AHB->PCTL & (0xFF00FFFF); 

    //(GPIOAMSEL) pag.786 Se habilita función analogica para cada uno de los puertos
    GPIOE_AHB->AMSEL |= (1<<0) | (1<<2) | (1<<4); //PIN 0, 2,4  - PUERTO E COMO ENTRADAS
    GPIOD_AHB->AMSEL |= (1<<3) | (1<<2); //PIN 3 Y 2 - PUERTO D COMO ENTRADAS
    GPIOB_AHB->AMSEL |= (1<<5); //PIN 5 - PUERTO B COMO ENTRADAS

    //Pag 1159 El registro (ADCPC) establece la velocidad de conversión de un millon por segundo para cada ADC y para cada puerto 
    ADC0->PC = (0<<3)|(1<<2)|(0<<1)|(1<<0);//1 Msps
    ADC1->PC = (0<<3)|(0<<2)|(1<<1)|(1<<0);//1 Msps
    
    //Pag 1099 Este registro (ADCSSPRI) configura la prioridad de los secuenciadores
    ADC0->SSPRI = 0x0000;   //Se pone por desault así que la prioridad sera 0, 1,2,3
    ADC1->SSPRI = 0x0000;   //Se pone por desault así que la prioridad sera 0, 1,2,3
    
    //Pag 1077 (ADCACTSS) Este registro controla la activación de los secuenciadores
    ADC0->ACTSS  =   (0<<3) | (0<<2) | (0<<1) | (0<<0); //SE DESACTIVAN PARA INICIAR LA CONFIGURACIÓN 
    ADC1->ACTSS  =   (0<<3) | (0<<2) | (0<<1) | (0<<0); //SE DESACTIVAN PARA INICIAR LA CONFIGURACIÓN 
    
    //Pag 1091 Este registro (ADCEMUX) selecciona el evento que activa la conversión (trigger)
    ADC0->EMUX  = (0x0000);  //Se eligio trigger
    ADC1->EMUX  = (0x0000);  //Se eligio trigger

    //Pag 1129 Este registro (ADCSSMUX2) define las entradas analógicas con el canal y secuenciador seleccionado
    ADC1->SSMUX1 = 0x00000931; //       que canales se van a leer de cada selector    
    ADC0->SSMUX2 = 0x00000B54; //       que canales se van a leer de cada selector -primer muestreo es el ultimo numero (9)

    //pag 868 Este registro (ADCSSCTL2), configura el bit de control de muestreo y la interrupción
    ADC1->SSCTL1 = 0x0644;    //AQUI SE LEE EL CANAL Y SE DICE Q
    ADC0->SSCTL2 = 0x0644;    //AQUI SE LEE EL CANAL Y SE DICE Q

    /* Enable ADC Interrupt */
    ADC1->IM |= (1<<1); /* Unmask ADC1 sequence 2 interrupt pag 1082*/
    ADC0->IM |= (1<<2); /* Unmask ADC0 sequence 2 interrupt pag 1082*/

    //NVIC_PRI4_R = (NVIC_PRI4_R & 0xFFFFFF00) | 0x00000020;
    //NVIC_EN0_R = 0x00010000;
    //Pag 1077 (ADCACTSS) Este registro controla la activación de los secuenciadores
    ADC1->ACTSS |= (0<<3) | (0<<2) | (1<<1) | (0<<0);
    ADC0->ACTSS |= (0<<3) | (1<<2) | (0<<1) | (0<<0);


    ADC0->PSSI |= (1<<2); //Se inicializa el muestreo en el secuenciador que se va a utilizar
    ADC1->PSSI |= (1<<1);
}

extern void ADC_leer_canal(uint16_t data[])
{
    ADC0 -> PSSI |= (1<<0); //Se inicializa el muestreo en el secuenciador que se va a utilizar
    delay_ms(1);

    while (ADC0 -> RIS & 0x01 == 0);
    delay_ms(1);

    while(ADC0  -> SSOP0 & (1<<0) == (1<<0));
    data[0] = ADC0 -> SSFIFO0 & 0xfff;
    delay_ms(1);

    while(ADC0 -> SSOP0 & (1<<4) == (1<<4));
    data[1] = ADC0 -> SSFIFO0 & 0xfff;
    delay_ms(1);

    while(ADC0 -> SSOP0 & (1<<8) == (1<<8));
    data[2] = ADC0->SSFIFO0 & 0xfff;
    delay_ms(1);

    while(ADC0 -> SSOP0 & (1<<12) == (1<<12));
    data[3] = ADC0 -> SSFIFO0 & 0xfff;
    delay_ms(1);
    while(ADC0 -> SSOP0 & (1<<16) == (1<<16));
    data[4] = ADC0 -> SSFIFO0 & 0xfff;
    delay_ms(1);
    while(ADC0 -> SSOP0 & (1<<20) == (1<<20));
    data[5] = ADC0 -> SSFIFO0 & 0xfff;
    delay_ms(1);

    ADC0->ISC |= (1<<0);
    delay_ms(1);
}

/////////////////////////////////////////////////////////////////////////////////////////////////// RECIBIR CHAR UART
extern void ADC_leer_canal(uint16_t data[])
{
    // --------------------------------------selector 0
    ADC0 -> PSSI |= (1<<0);
    delay_ms(1);

    while (ADC0 -> RIS & 0x01 == 0);
    delay_ms(1);
    while(ADC0 -> SSOP0 & (1<<0) == (1<<0));
    data[0] = ADC0 -> SSFIFO0 & 0xfff;
    delay_ms(1);
    while(ADC0->SSOP0 & (1<<4) == (1<<4));
    data[1] = ADC0->SSFIFO0 & 0xfff;
    delay_ms(1);
    while(ADC0->SSOP0 & (1<<8) == (1<<8));
    data[2] = ADC0->SSFIFO0 & 0xfff;
    delay_ms(1);

    ADC0->ISC |= (1<<0);
    delay_ms(1);

    // --------------------------------selector 1
    ADC0->PSSI |= (1<<1);
    delay_ms(1);

    while (ADC0->RIS & 0x02 == 0);
    delay_ms(1);
    while(ADC0->SSOP1 & (1<<0) == (1<<0));
    data[3] = ADC0->SSFIFO1 & 0xfff;
    delay_ms(1);
    while(ADC0->SSOP1 & (1<<4) == (1<<4));
    data[4] = ADC0->SSFIFO1 & 0xfff;
    delay_ms(1);
    while(ADC0->SSOP1 & (1<<8) == (1<<8));
    data[5] = ADC0->SSFIFO1 & 0xfff;
    delay_ms(1);

    ADC0->ISC |= (1<<1);
    delay_ms(1);
}
//*/




//*/



extern void ADC0_InSeq2(uint16_t *Result){

    //ADC Processor Sample Sequence Initiate (ADCPSSI)
       ADC0->PSSI = 0x00000004;
       while((ADC0->RIS&0x04)==0){}; // espera al convertidor
       Result[1] = ADC0->SSFIFO2&0xFFF; //  Leer  el resultado almacenado en la pila2
       Result[0] = ADC0->SSFIFO2&0xFFF;
       printChar('A');
       ADC0->ISC = 0x0004;  //Conversion finalizada

}


