#pragma config PLLDIV = 4       // PLL Prescaler Selection bits
#pragma config CPUDIV = OSC1_PLL2 // System Clock Postscaler Selection bits
#pragma config USBDIV = 2        // USB Clock Selection bit
#pragma config FOSC = HSPLL_HS   // Oscillator Selection bits (HS con PLL activado, 48MHz)
#pragma config FCMEN = OFF       // Fail-Safe Clock Monitor Enable bit
#pragma config IESO = OFF        // Internal/External Oscillator Switchover bit
#pragma config PWRT = OFF        // Power-up Timer Enable bit
#pragma config BOR = OFF         // Brown-out Reset Enable bits
#pragma config BORV = 3          // Brown-out Reset Voltage bits
#pragma config VREGEN = OFF      // USB Voltage Regulator Enable bit
#pragma config WDT = OFF         // Watchdog Timer Enable bit
#pragma config WDTPS = 32768     // Watchdog Timer Postscale Select bits
#pragma config CCP2MX = ON       // CCP2 MUX bit
#pragma config PBADEN = OFF      // PORTB A/D Enable bit (Desactiva AN0-AN4 en RB)
#pragma config LPT1OSC = OFF     // Low-Power Timer 1 Oscillator Enable bit
#pragma config MCLRE = ON        // MCLR Pin Enable bit
#pragma config STVREN = ON       // Stack Full/Underflow Reset Enable bit
#pragma config LVP = ON          // Single-Supply ICSP Enable bit
#pragma config ICPRT = OFF       // Dedicated In-Circuit Debug/Programming Port (ICPORT) Enable bit
#pragma config XINST = OFF       // Extended Instruction Set Enable bit (Legacy mode)
#pragma config CP0 = OFF, CP1 = OFF, CP2 = OFF, CP3 = OFF
#pragma config CPB = OFF, CPD = OFF
#pragma config WRT0 = OFF, WRT1 = OFF, WRT2 = OFF, WRT3 = OFF
#pragma config WRTC = OFF, WRTB = OFF, WRTD = OFF
#pragma config EBTR0 = OFF, EBTR1 = OFF, EBTR2 = OFF, EBTR3 = OFF
#pragma config EBTRB = OFF

#include <xc.h>
#include "lcd.h"
#include <stdio.h>

#define _XTAL_FREQ 48000000 

void Lcd_Init(void);
void Lcd_Set_Cursor(unsigned char a, unsigned char b); 
void Lcd_String(const char *a);

void UART_Init(void);
void UART_Write_Char(char data);
void ADC_Init(void);
unsigned int ADC_Read(void);

// --- FUNCIÓN DE INICIALIZACIÓN UART (PUERTO SERIAL) ---
void UART_Init(void) {
    // Configuración de pines TX/RX
    TRISCbits.TRISC6 = 0;       // RC6 (TX) como salida
    TRISCbits.TRISC7 = 1;       // RC7 (RX) como entrada

    // Configuración para 9600 Baudios a 48MHz (Valor: 1249)
    SPBRGH = 0x04;
    SPBRG = 0xE1; // Valor bajo 0xE1 (225) + Valor alto 0x04 (1024) = 1249
    
    BAUDCONbits.BRG16 = 1;      // Habilita el generador de baudios de 16 bits
    
    // TXSTA: Transmisión habilitada (TXEN=1), Asíncrona (SYNC=0), Alta velocidad (BRGH=1)
    TXSTA = 0b00100100; 
    
    // RCSTA: Habilitar puerto serie (SPEN=1), Habilitar recepción continua (CREN=1)
    RCSTA = 0b10010000; 
}

// --- FUNCIÓN DE TRANSMISIÓN UART ---
void UART_Write_Char(char data) {
    while(!TXSTAbits.TRMT); // Esperar a que el buffer de transmisión esté vacío
    TXREG = data;           // Enviar dato
}

// --- FUNCIÓN DE INICIALIZACIÓN ADC ---
void ADC_Init(void) {
    TRISAbits.TRISA0 = 1;       // RA0 como entrada (AN0)
    ADCON1 = 0x0E;              // AN0 como analógico (AN1-AN12 digitales)
    ADCON2 = 0b10101010;        // Fosc/32, Justificación a la derecha, 10 TAD
    ADCON0 = 0b00000001;        // Selecciona canal AN0 (CHS=0000), ADC encendido (ADON=1)
}

// --- FUNCIÓN DE LECTURA ADC ---
unsigned int ADC_Read(void) {
    ADCON0bits.GO = 1;          // Iniciar conversión
    while(ADCON0bits.GO);       // Esperar a que termine
    // Se fuerza la conversión a unsigned int para silenciar la advertencia
    return (unsigned int)((ADRESH << 8) + ADRESL); 
}

void main(void) {

    // Inicializaciones
    ADC_Init();
    UART_Init();
    Lcd_Init();

    // Configuración de pines de LED (RC0 y RC1)
    TRISCbits.TRISC0 = 0; 
    TRISCbits.TRISC1 = 0; 

    LATCbits.LATC0 = 0;   
    LATCbits.LATC1 = 0;

    unsigned int value;
    unsigned char estado_anterior = 255;    // Valor inicial para forzar la primera actualización

    #define UMBRAL 200

    Lcd_Set_Cursor(1,1);
    Lcd_String("JERINGA");

    while(1) {

        value = ADC_Read();

        // Determinar el estado actual: 1 si detectada (> UMBRAL), 0 si no detectada
        unsigned char estado_actual = (value > UMBRAL) ? 1 : 0;

        // Solo actualizar LCD y enviar UART cuando el estado cambia
        if(estado_actual != estado_anterior){
            
            Lcd_Set_Cursor(2,1); 
            
            if(estado_actual){
                // Jeringa Detectada
                LATCbits.LATC0 = 1; // LED DETECTADO ON
                LATCbits.LATC1 = 0; // LED NO DETECTADO OFF
                Lcd_String("DETECTADA   "); // espacios para borrar restos de "NO"
                UART_Write_Char('A'); // 'A' para ACEPTADO/ACTIVADO
            } else {
                // Jeringa No Detectada
                LATCbits.LATC0 = 0; // LED DETECTADO OFF
                LATCbits.LATC1 = 1; // LED NO DETECTADO ON
                Lcd_String("NO DETECTADA"); 
                UART_Write_Char('S'); // 'S' para SIN Jeringa/STANDBY
            }

            estado_anterior = estado_actual; // Guardar el nuevo estado
        }

        __delay_ms(50); // Pequeña pausa para estabilidad
    }
}
