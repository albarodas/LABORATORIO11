/* 
 * File:   MASTER_PL.c
 * Author: ALBA RODAS
 *
 * Created on 13 de mayo de 2022, 08:57 AM
 */

// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>

/*------------------------------------------------------------------------------
 * CONSTANTES 
 ------------------------------------------------------------------------------*/
#define _XTAL_FREQ 1000000
#define FLAG_SPI 0xFF
/*------------------------------------------------------------------------------
 * VARIABLES 
 ------------------------------------------------------------------------------*/
uint8_t contador_variable = 0;
uint8_t value = 0;                      // CONTADOR QUE VA: MASTER --> SLAVE
char value_variable = 0;

/*------------------------------------------------------------------------------
 * PROTOTIPO DE FUNCIONES 
 ------------------------------------------------------------------------------*/
void setup(void);
/*------------------------------------------------------------------------------
 * INTERRUPCIONES 
 ------------------------------------------------------------------------------*/
void __interrupt() isr (void){
    if(PIR1bits.ADIF){              // ASK: ¿HUBO INTERRUPCION EN EL ADC?
        if(ADCON0bits.CHS == 0)
        {                           // CHECK: AN0 CANAL SELECIONADO.       
           value = ADRESH;
        }
        PIR1bits.ADIF = 0; 
     }
    return;
}
/*------------------------------------------------------------------------------
 * CICLO PRINCIPAL
 ------------------------------------------------------------------------------*/
void main(void) {
    setup();
    while(1){
        if(ADCON0bits.GO == 0){             // SI NO HAY CONVERSIÓN ACTIVADA...
            ADCON0bits.GO = 1;              // LA INICIAMOS.
        }
        
            PORTAbits.RA6 = 1;
            PORTAbits.RA7 = 1;              // DESABILITAMOS: SS DEL SLAVE.
            __delay_ms(10);                 // DAMOS UN DELAY PARA QUE EL PIC ACEPTE Y ASIMILE EL DATO/CAMBIO.
            PORTAbits.RA7 = 0;              // ESCLAVO SE ACTIVA NUEVAMENTE
           
            SSPBUF = 0xFF;                  // ENVIAMOS UN NUEVO VALOR AL COUNTER
            while(!SSPSTATbits.BF){}        // LE DAMOS TIEMPO PARA EL ENVIO
            
            PORTAbits.RA6 = 1;              // ENCENDEMOS EL ESCLAVO AGAIN
            PORTAbits.RA7 = 1;              // TURN OFF AL SS DEL ESCLAVO
            __delay_ms(10);                 // DAMOS UN DELAY PARA QUE EL PIC PUEDA CONSIDERAR EL CAMBIO HECHO
            PORTAbits.RA7 = 0;              // ENCENDEMOS NUEVAMENTE AL ESCLAVO
            
            while(!SSPSTATbits.BF){}        // QUEDAMOS A LA ESPERA DE RECIBIR UN DATO
            PORTD = SSPBUF;                 // SHOW AL DATO EN EL PORT DESEADO, EN ESTE CASO EL PORTD
            
            PORTAbits.RA6 = 1;
            PORTAbits.RA7 = 1;              // TURN OFF AL SS DEL ESCLAVO
            __delay_ms(10);                 // DAMOS UN NUEVO DELAY PARA QUE EL PIC IDENTIFIQUE EL CAMBIO HECHO
            PORTAbits.RA6 = 0;              // DESABILITAMOS EL A7
           
            SSPBUF = value;                 // DAMOS NUEVO VALOR AL CONTADOR
            while(!SSPSTATbits.BF){}        // WIAT A QUE TERMINE EL ENVIO
            
            PORTAbits.RA6 = 1;              // HABILITAMOS EL A6
            PORTAbits.RA7 = 1;              // TURN OFF AL SS DEL ESCLAVO
            __delay_ms(10);                 // ESPERAMOS CON UN DELAY OTRA VEZ
            PORTAbits.RA6 = 0;              // TURN ON AL ESCLAVO
            
            
            __delay_ms(10);                 // DAMOS OTRO  DELAY POR SEGURIDAD.
    }
    return;
}

/*------------------------------------------------------------------------------
 * CONFIGURACION 
 ------------------------------------------------------------------------------*/
void setup(void){
    ANSEL = 0b00000001;         // AN0 --> ENTRADA
    
    ANSELH = 0;                 // INPUTS, OUTPUTS DIGITALES
    
    OSCCONbits.IRCF = 0b100;    // OSCILADOR: 1MHz
    OSCCONbits.SCS = 1;         // RELOJ INTERNO: ON
    
    TRISA = 0b00000001;         // SS y RA0 AS ENTRADAS     
    PORTA = 0;
    //SETEAMOS PUERTOS EN CERO.
    TRISD = 0;
    PORTD = 0;
    
    TRISB = 0;
    PORTB = 0;
    
    ADCON0bits.ADCS = 0b00;     // Fosc/2
    
    ADCON1bits.VCFG0 = 0;       // VDD *Referencias internas
    ADCON1bits.VCFG1 = 1;       // VSS
    
    ADCON0bits.CHS = 0b0000;    // Seleccionamos AN0
    ADCON1bits.ADFM = 0;        // Justificado a la izquierda
    ADCON0bits.ADON = 1;        // Habilitamos modulo ADC
    __delay_us(40);
    
    // CONFIG. INTERRUPCIONES
    PIR1bits.ADIF = 0;          // INT. ADC: FLAG ON
    PIE1bits.ADIE = 1;          // INT. de ADC: ON
    INTCONbits.GIE = 1;         // GLOBALES: ON
    INTCONbits.PEIE = 1;        // PERIFERICOS: ON
    
    // CONFIG. SPI
    // CONFIG PARA EL MAESTRO
    TRISC = 0b00010000;         // SDI INPUT
    PORTC = 0;
                                // SCK y SD0 --> OUTPUT
    
    // SSPCON PARA EL BIT DEL 0-5:
    SSPCONbits.SSPM = 0b0000;   // SPI Maestro --> Fosc/4
    SSPCONbits.CKP = 0;         // RELOJ EN OFF CUANDO = 0
    SSPCONbits.SSPEN = 1;       // SPI PINES : ON
    
    SSPSTATbits.CKE = 1;        
    SSPSTATbits.SMP = 1;        // ACTIVAMOS EL RECIBIR DATOS AL FINAL DEL PULSO POSITIVO DEL CLK
    SSPBUF = value;             
    
}
