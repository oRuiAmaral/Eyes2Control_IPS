/*
 * File:   Eyes2control.c
 * Author: Diana e Daniela
 *
 * Created on 26 de Setembro de 2017, 15:38
 */


#include <xc.h>
#include <stdlib.h>
#include <pic18f4550.h>

#define _XTAL_FREQ 4000000
#pragma config PLLDIV = 1         // PLL Prescaler Selection bits (No prescale (4 MHz oscillator input drives PLL directly))
#pragma config CPUDIV = OSC1_PLL2 // System Clock Postscaler Selection bits ([Primary Oscillator Src: /1][96 MHz PLL Src: /2])
#pragma config USBDIV = 1         // USB Clock Selection bit (used in Full-Speed USB mode only; UCFG:FSEN = 1) (USB clock source comes directly from the primary oscillator block with no postscale)

// CONFIG1H
#pragma config FOSC  = INTOSC_EC  // Oscillator Selection bits (Internal oscillator, CLKO function on RA6, EC used by USB (INTCKO))
#pragma config FCMEN = OFF        // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO  = OFF        // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT   = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOR    = OFF       // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV   = 3         // Brown-out Reset Voltage bits (Minimum setting)
#pragma config VREGEN = ON        // USB Voltage Regulator Enable bit (USB voltage regulator disabled)

// CONFIG2H
#pragma config WDT   = OFF        // Watchdog Timer Enable bit (WDT enabled)
#pragma config WDTPS = 32768      // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX  = ON      // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN  = OFF     // PORTB A/D Enable bit (PORTB<4:0> pins are configured as digital I/O on Reset)
#pragma config LPT1OSC = OFF     // Low-Power Timer 1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE   = ON      // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON       // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP    = OFF      // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
#pragma config ICPRT  = OFF      // Dedicated In-Circuit Debug/Programming Port (ICPORT) Enable bit (ICPORT disabled)
#pragma config XINST  = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-001FFFh) is not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (002000-003FFFh) is not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (004000-005FFFh) is not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (006000-007FFFh) is not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) is not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM is not code-protected)

// CONFIG6L
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (002000-003FFFh) is not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (004000-005FFFh) is not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (006000-007FFFh) is not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) are not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block (000000-0007FFh) is not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM is not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-001FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (002000-003FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (004000-005FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (006000-007FFFh) is not protected from table reads executed in other blocks)

// CONFIG7H
//#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) is not protected from table reads executed in other blocks)
////#pragma config PLLDIV   = 1
////#pragma config CPUDIV   = OSC1_PLL2
////#pragma config FOSC     = INTOSC_HS
////#pragma config WDT      = OFF
////#pragma config LVP      = OFF
////#pragma config BOR      = OFF
////#pragma config MCLRE    = ON
////#pragma config PWRT     = ON
////#pragma config PBADEN   = OFF
//
//
//
#define TRIS_A_SH  TRISAbits.TRISA0
#define TRIS_A_SV  TRISAbits.TRISA1
#define TRIS_button TRISBbits.TRISB0
#define TRIS_MUX1_S1 TRISCbits.TRISC1
#define TRIS_MUX1_S2 TRISCbits.TRISC2
#define TRIS_MUX2_S0 TRISDbits.TRISD0
#define TRIS_MUX2_S1 TRISDbits.TRISD1
#define TRIS_MUX2_S2 TRISDbits.TRISD2
#define TRIS_LED_WAIT_CALIBRATION  TRISEbits.TRISE0
#define TRIS_LED_DOING_CALIBRATION TRISEbits.TRISE1
#define TRIS_LED_END_CALIBRATION   TRISEbits.TRISE2
#define button PORTBbits.RB0
#define MUX1_S0 PORTCbits.RC0
#define MUX1_S1 PORTCbits.RC1
#define MUX1_S2 PORTCbits.RC2
#define MUX2_S0 PORTDbits.RD0
#define MUX2_S1 PORTDbits.RD1
#define MUX2_S2 PORTDbits.RD2

/**
  Section: Macro Declarations
 */
#define ACQ_US_DELAY 5


// --------[ PROGRAM DEFINITIONS ]----------
#define LED_WAIT_CALIBRATION  PORTEbits.RE0
#define LED_DOING_CALIBRATION PORTEbits.RE1
#define LED_END_CALIBRATION   PORTEbits.RE2
////-[ADC]------------------------------------
//#define ADC_Vfsr  5
//#define ADC_Nbits 10
//// #define ADC_Ra    (ADC_Vfsr/(2^ADC_Nbits))
//// #define ADC_Ra 0.0049   
//-[AI]-------------------------------------
#define ganho_max 5
//-[USART]----------------------------------
#define BAUDRATE  9600  //bps
//_-----------------------------------------

//---------[ PROGRAM VARIABLES ]------------
int Calibrar=0;
int Calibrar_SH=0;
int Calibrar_SV=0;

float Ganho_SH;
float Ganho_SV;

float ADC_Ra=0.0049;

//---------------------------------


//---------[ FUNCTION DEFINITIONS ]------------
void Initialization_Phase(void);

void InterruptS_Initialization(void);
void Timer0_Initialization(void);

void Calibration_Phase(void);
void Running_Phase(void);

//PS
//void interrupt_level2(void);


void ADC_Initialization(void);
unsigned int ADC_Read(unsigned char channel);


void UART_Initialization(void);
void SendByteSerially(unsigned char);
void SendStringSerially(const unsigned char*);
unsigned char ReceiveByteSerially(void);


void Initialization_Phase()
{
    TRISAbits.RA0= 1;            // configura RA0 como entrada 
    TRISAbits.RA1= 1;            // configura RA1 como entrada 
    TRISBbits.RB0= 1;            // configura RB0 como entrada 
    TRISC        = 0X1F;         // configurar <0,2> entradas (int0, int1, int2); <3,7> saidas (so, s1, s2)
    TRISD        = 0X1F;
    TRISEbits.RE0 =0;            // configura RE0 como saida 
    TRISEbits.RE1= 0;            // configura RE1 como saida 
    TRISEbits.RE2= 0;            // configura RE2 como saida 
    OSCCON       = 0X66;
    
    
   /* CONFIGURACAO DO REGISTO DE INTERRUPÇÕES*/
    InterruptS_Initialization();
    
    /* CONFIGURACAO DO REGISTO DO TIMER0 */
    Timer0_Initialization();
    
    /* CONFIGURACAO DO REGISTO DO ADC */    
    ADC_Initialization();
    
    /* CONFIGURACAO DO REGISTO DO UART */    
    UART_Initialization();    
    
    
    /*INICIALIZACAO DAS VARIAVEIS DO SISTEMA*/
    Calibrar=0;
    
}




void InterruptS_Initialization()
{
    /*CONFIGURACAO DO REGISTO PARA O TIMER0*/
    /*REGISTOS PARA AS INTERRUPCOES*/
    //INTCON=0x00;                // Enable INT0
    //INTCON2=0;                  // Set Falling Edge Trigger for INT0
    //INTCON3=0;
    
   /*CONFIGURACAO DO REGISTO DE CONTROLO DE INTERRUPÇÕES*/
    INTCONbits.GIE=1;            // habilita as interrupcoes não mascaradas

    //RCONbits.IPEN = 1;
    INTCONbits.PEIE = 1;        // habilita o uso da interrupcao dos periféricos
    
    INTCON2bits.INTEDG0=1;       // interrupção INT0 ocorre em bordo ascendente
    INTCONbits.INT0IE = 1;       // habilita o uso da interrupcao externa INT0
    INTCONbits.INT0IF = 0;       // desabilita a Flag IF de INT0
    
    INTCONbits.TMR0IF = 0;       // desabilita a Flag IF do TIMER0
    INTCONbits.TMR0IE = 1;       // habilita o uso da interrupcao do TIMER0           
    
}


void Timer0_Initialization()
{
    /*CONFIGURACAO DO REGISTO PARA O TIMER0*/
    //Timer0_Initialization()  ->  Diana
    TMR0 = 0;                // Clear TIMER0 register value

//    T0CONbits.TMR0ON = 1;    // Enable Timer0
    T0CONbits.T08BIT = 0;    // TIMER0 is a 16 bit counter
    T0CONbits.T0CS   = 0;    // TIMER0 use internal clock
    T0CONbits.PSA    = 0;    // TIMER0 use Prescaler
    T0CONbits.T0PS0  = 1;    // Prescaler value 1/3
    T0CONbits.T0PS1  = 1;    // Prescaler value 2/3
    T0CONbits.T0PS2  = 1;    // Prescaler value 3/3
    
}


//--------------------------------
//        ADC 
//--------------------------------
void ADC_Initialization()
{
    /*CONFIGURACAO DO REGISTO DO ADC*/    

    /*ADCON1: Registo de controlo do módulo ADC*/
//    ADCON1bits.VCFG0=0;     // Select -Vref = Vss and +Vref = Vdd 
//    ADCON1bits.VCFG0;       // Select -Vref = Vss and +Vref = Vdd 
	ADCON1bits.VCFG1 = 0; //Uso do GND para a fonte de  Vref-    LILIA
	ADCON1bits.VCFG0 = 0; // Uso do VCC para o fonte de  Vref+   LILIA
    ADCON1bits.PCFG0=0;       // Configurate A/D Port 
    ADCON1bits.PCFG1=1;       // ---------------------
    ADCON1bits.PCFG2=1;       // AN0 = Analog   
    ADCON1bits.PCFG3=1;       // AN1, ..., AN12 = Digital   

    /*ADCON0: Registo de controlo do módulo ADC*/
    ADCON0bits.CHS0=0;      // Select
    ADCON0bits.CHS1=0;      // AN0
    ADCON0bits.CHS2=0;      // analog
    ADCON0bits.CHS3=0;      // channel
    
    /*ADCON2: Registo de controlo do módulo ADC*/
    ADCON2bits.ADFM=1;       // Conversion bits Right justified
    ADCON2bits.ACQT2=1;      // Define 
    ADCON2bits.ACQT1=1;      // ADC 
    ADCON2bits.ACQT0=1;      // aquisition time
    ADCON2bits.ADCS0=1;      // Define 
    ADCON2bits.ADCS1=1;      // ADC 
    ADCON2bits.ADCS2=1;      // conversion clock

    // ADRESL 0x0 and  ADRESH 0x0 ; 
    ADRESL = 0x00;
    ADRESH = 0x00;

//    ADCON0bits.ADON=1;      // Turn ON de A/D Converter (Enable)
//    ADCON0bits.GO_nDONE=0;  // A/D Converter is IDLE to convert

}

unsigned int ADC_Read(unsigned char channel)
{
  //
  if(channel > 7)               //Channel range is 0 ~ 7
    return 0;

  ADCON0 &= 0xC5;               //Clearing channel selection bits
  ADCON0 |= channel<<3;         //Setting channel selection bits
  __delay_us(ACQ_US_DELAY);     // Acquisition time delay (Allows Hold capacitor to charge))
//  __delay_ms(2);              //Acquisition time to charge hold capacitor
  ADCON0bits.GO_nDONE = 1;      //inicializa a conversao
  while(ADCON0bits.GO_nDONE);   //espera pela finalizacao da conversao
  return ((ADRESH<<8)+ADRESL);  //retorna o  resultado
}


void Calibration_Phase()
{
//    float V1=0;float V2=0;
//    float V1adc=0; float V1max=0; float V1min=0; float V1media=0;
//    float V2adc=0; float V2max=0; float V2min=0; float V2media=0;
//    unsigned int sh;
//    unsigned int sv;

    float vADC_SH_avg=0;
    float vADC_SH_max=0;
    float vADC_SH_min=0;    
    float vADC_SH_i=0;
    float vADC_SV_avg=0;
    float vADC_SV_max=0;
    float vADC_SV_min=0;    
    float vADC_SV_i=0;
    int   nContagens=10;
    
//-[ADC]------------------------------------
//    float ADC_Ra            = 0.0049;   
    int     ADC_Vfsr          = 5;
    int     ADC_Nbits         = 10;
    float   ADC_Ramdom_Sample = 0.0;
    unsigned int j=1;     
    unsigned int ADC_sample;


    // Lancar TIMER
    T0CONbits.TMR0ON = 0;      // Disable Timer0
    TMR0H = 0x63;              // Cenário: Fosc=4MHz, Prescaler=256 => Delay 5 segundos
    TMR0L = 0x65;
    
    INTCONbits.TMR0IE = 1;     // Enable Timer interrupts   
    INTCONbits.TMR0IF = 0;
    T0CONbits.TMR0ON  = 1;      // Enable Timer0

    // Toca Bip
    ADRESL = 0x00;          // ADRESL 0x0 and  ADRESH 0x0 ; 
    ADRESH = 0x00;
    ADCON0bits.ADON=1;      //  Turn ON de A/D Converter (Enable)

    while(Calibrar==1)
    {
        for(int i=0; i<nContagens; i++)
        {
//            // Acquisition time delay (Allows Hold capacitor to charge))
//            __delay_us(ACQ_US_DELAY);
//            // Start the conversion
//            ADCON0bits.GO_nDONE = 1;
//            while(ADCON0bits.GO_nDONE);
//            ADC_sample=((ADRESH<<8)+ADRESL);
//            vADC_SH_i = ADC_sample*ADC_Ra;            // Lê conversão A/D do canal 0
//            ADC_Ramdom_Sample=rand()*1.0;
//            vADC_SH_i = ADC_Ramdom_Sample*ADC_Ra;            // Lê conversão A/D do canal 0
            vADC_SH_i = ADC_Read(0)*ADC_Ra;
            vADC_SH_avg=vADC_SH_avg+vADC_SH_i;
        }
        vADC_SH_avg=vADC_SH_avg/nContagens;
        
        // update ADC_SH max and min value
        if(vADC_SH_avg>vADC_SH_max)
            vADC_SH_max=vADC_SH_avg;
        if(vADC_SH_avg<vADC_SH_min)
            vADC_SH_min=vADC_SH_avg;     
    }
    
    // Define new Gain value
    Ganho_SH=ADC_Vfsr/(vADC_SH_max-vADC_SH_min);

    //__delay_ms(100);

    // Lancar TIMER
    // Toca Bip Bip
    T0CONbits.TMR0ON = 0;      // Disable Timer0
    TMR0H = 0x63;              // Cenário: Fosc=4MHz, Prescaler=256 => Delay 5 segundos
    TMR0L = 0x65;
    
    INTCONbits.TMR0IE = 1;     // Enable Timer interrupts   
    INTCONbits.TMR0IF = 0;
    T0CONbits.TMR0ON  = 1;      // Enable Timer0

    // Toca Bip
    ADRESL = 0x00;          // ADRESL 0x0 and  ADRESH 0x0 ; 
    ADRESH = 0x00;
    ADCON0bits.ADON=1;      //  Turn ON de A/D Converter (Enable)

    while(Calibrar==1)
    {
        for(int i=0; i<nContagens; i++)
        {
//            // Acquisition time delay
//            __delay_us(ACQ_US_DELAY);
//            // Start the conversion
//            ADCON0bits.GO_nDONE = 1;
//            while(ADCON0bits.GO_nDONE);
//            ADC_sample=((ADRESH<<8)+ADRESL);
//            vADC_SV_i = ADC_sample*ADC_Ra;            // Lê conversão A/D do canal 0
//            ADC_Ramdom_Sample=rand()*1.0;
//            vADC_SV_i = ADC_Ramdom_Sample*ADC_Ra;            // Lê conversão A/D do canal 0
//            
            vADC_SV_i = ADC_Read(1)*ADC_Ra;
            vADC_SV_avg=vADC_SV_avg+vADC_SV_i;
        }
        vADC_SV_avg=vADC_SV_avg/nContagens;
        
        // update ADC_SH max and min value
        if(vADC_SV_avg>vADC_SV_max)
            vADC_SV_max=vADC_SV_avg;
        if(vADC_SV_avg<vADC_SV_min)
            vADC_SV_min=vADC_SV_avg;     
    }
    
    // Define new Gain value
    Ganho_SV=ADC_Vfsr/(vADC_SV_max-vADC_SV_min);

}

void Running_Phase(){
    
    unsigned int sh;
    float resultado_sh;
    unsigned int sv;
    float resultado_sv;

    float vADC_SH=0;
    float vADC_SV=0;
//    float ADC_Ra=0.0049;

    do
    {
        vADC_SH=ADC_Read(0)*ADC_Ra; // Lê conversão A/D do canal 0
        __delay_ms(100);            //Delay
        vADC_SV=ADC_Read(1)*ADC_Ra; // Lê conversão A/D do canal 1
        
//        sv = ADC_Read(1);            // Lê conversão A/D do canal 1
//        resultado_sv=sv*resl_an;
        __delay_ms(100);            //Delay

        // TRANSMITE PRO PC

    }while(1);

}


/****************************************************************
*                   Interrupt Service Routines
****************************************************************/
//void interrupt InterruptRoutine(void)
void interrupt high_priority botao_pressionado_isr1(void)
{
    //check if the interrupt is caused by the:
    // -> Calibration Button (INT0 - external interrupt - pin RB0)
    // -> Calibration Clock  (TMR0 - internal interrupt)
    
    if(INTCONbits.INT0F == 1  )
    {
        INTCONbits.INT0F =0;     // clear the INT0 flag
        Calibrar=1;
    }
    else if(INTCONbits.TMR0IE && INTCONbits.TMR0IF) {
        INTCONbits.TMR0IE = 0;       // habilita o uso da interrupcao do TIMER0           
        INTCONbits.TMR0IF = 0;  // clear the TMR0 flag
        Calibrar=0;
    }
    else if(RCIF) {              // If UART Rx Interrupt
        if(OERR) // If over run error, then reset the receiver
        {
            CREN = 0;
            CREN = 1;
        }
        SendByteSerially(RCREG);    // Echo back received char
    }

}


//======================================================
//                       UART ROUTINES
//======================================================

void UART_Initialization(void)
{
    TRISB2 = 0;                       // TX Pin
    TRISB1 = 1;                       // RX Pin
    SPBRG = ((_XTAL_FREQ/16)/BAUDRATE) - 1;

    /* CONFIG TRANSMIT STATUS AND CONTROL REGISTER - pg240*/
    TXSTAbits.TX9   = 0;              // 9-bit transmission
    TXSTAbits.TXEN  = 0;              // Transmit disabled
    TXSTAbits.SYNC  = 0;              // Asynchronous Mode
    TXSTAbits.BRGH  = 1;              // High Baud Rate in Asynchronous Mode
    
    /* CONFIG RECEIVE STATUS AND CONTROL REGISTER - pg241*/
    RCSTAbits.SPEN  = 1;              // Serial port enabled (configures RX/DT and TX/CK pins as serial port pins)
    RCSTAbits.RX9   = 0;              // Selects 8-bit reception
    RCSTAbits.SREN  = 0;              // No effect in Asynchronous Mode
    RCSTAbits.CREN  = 1;              // Enable receiver in Asynchronous Mode
            
    /* CONFIG PIR Registers */
    PIR1bits.RCIF  = 0;               // EUSART receive buffer is empty (Interrupt Flag)
    PIR1bits.TXIF  = 1;               // EUSART transmit buffer is empty (Interrupt Flag)
    
    /* CONFIG PIE Registers */
    PIE1bits.TXIE  = 1;               // Enable EUSART transmit interrupts
    PIE1bits.RCIE  = 1;               // Enable EUSART receive interrupts

    /* CONFIG IPR Registers */
    IPR1bits.TXIP  = 1;               // EUSART transmit interrupt priority bit is HIGH
    IPR1bits.RCIP  = 1;               // EUSART receive interrupt priority bit is HIGH
    
    TXSTAbits.TXEN  = 0;              // Transmitter Enabled

}

void SendByteSerially(unsigned char Byte)  // Writes a character to the serial port
{
    while(!TXIF);  // wait for previous transmission to finish
    TXREG = Byte;
}

unsigned char ReceiveByteSerially(void)   // Reads a character from the serial port
{
    if(OERR) // If over run error, then reset the receiver
    {
        CREN = 0;
        CREN = 1;
    }

    while(!RCIF);  // Wait for transmission to receive

    return RCREG;
}

void SendStringSerially(const unsigned char* st)
{
    while(*st)
        SendByteSerially(*st++);
}


void main(void) {

    LED_WAIT_CALIBRATION  = 0;
    LED_DOING_CALIBRATION = 0;
    LED_END_CALIBRATION   = 0;
    Initialization_Phase();

    
    LED_WAIT_CALIBRATION  = 1;
    LED_DOING_CALIBRATION = 0;
    LED_END_CALIBRATION   = 0;
    while(!Calibrar);
//    {
//        int a=1;
////        IntExternal_INT();
//    }

    LED_WAIT_CALIBRATION  = 0;
    LED_DOING_CALIBRATION = 1;
    LED_END_CALIBRATION   = 0;
    
////    while(1)
////    {
//        //if(INTCONbits.INT0F == 0)
//
//    // Aguarda pedido de Calibracao    
//    LED_CALIBRACAO=0;
//    LED_FINALIZACAO=0;
//    LED_INICIO = 1;
//
//        EsperaPedidodeCalibrar=1;
//        while(EsperaPedidodeCalibrar==1)
//        {
//            int a=1;
//            IntExternal_INT();
//
//        }

    /* 
     * ---[ FASE de CALIBRACAO ]---
     */
    Calibration_Phase();

    LED_WAIT_CALIBRATION  = 0;
    LED_DOING_CALIBRATION = 0;
    LED_END_CALIBRATION   = 1;

//       LED_CALIBRACAO=0;
//       LED_FINALIZACAO=1;
    //}

    Running_Phase();
//    do
//    {
//        sh = ADC_Read(0);            // Lê conversão A/D do canal 0
//        resultado_sh=sh*resl_an;
//        //__delay_ms(100);            //Delay
//        sv = ADC_Read(1);            // Lê conversão A/D do canal 1
//        resultado_sv=sv*resl_an;
//        //__delay_ms(100);            //Delay
//
//        // TRANSMITE PRO PC
//
//    }while(1);
//    LED_CALIBRACAO=0;
//    LED_FINALIZACAO=1;
////PS
//    InitUART();                            // Initialize UART
//
//    SendStringSerially("Serial communication");    // Send string on UART
//    GIE  = 1;                              // Enable global interrupts
//    PEIE = 1;                              // Enable Peripheral Interrupts
//    while(1)
//    {
//
//    }
    
    return;
}
