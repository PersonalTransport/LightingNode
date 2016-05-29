// CONFIG4
#pragma config DSWDTPS = DSWDTPSF // DSWDT Postscale Select (1:2,147,483,648 (25.7 days))
#pragma config DSWDTOSC = LPRC // Deep Sleep Watchdog Timer Oscillator Select (DSWDT uses Low Power RC Oscillator (LPRC))
#pragma config RTCOSC = SOSC // RTCC Reference Oscillator Select (RTCC uses Secondary Oscillator (SOSC))
#pragma config DSBOREN = ON // Deep Sleep BOR Enable bit (BOR enabled in Deep Sleep)
#pragma config DSWDTEN = ON // Deep Sleep Watchdog Timer (DSWDT enabled)

// CONFIG3
#pragma config WPFP = WPFP63 // Write Protection Flash Page Segment Boundary (Highest Page (same as page 21))
#pragma config SOSCSEL = IO // Secondary Oscillator Pin Mode Select (SOSC pins have digital I/O functions (RA4, RB4))
#pragma config WUTSEL = LEG // Voltage Regulator Wake-up Time Select (Default regulator start-up time used)
#pragma config WPDIS = WPDIS // Segment Write Protection Disable (Segmented code protection disabled)
#pragma config WPCFG = WPCFGDIS // Write Protect Configuration Page Select (Last page and Flash Configuration words are unprotected)
#pragma config WPEND = WPENDMEM // Segment Write Protection End Page Select (Write Protect from WPFP to the last page of memory)

// CONFIG2
#pragma config POSCMOD = NONE // Primary Oscillator Select (Primary Oscillator disabled)
#pragma config I2C1SEL = PRI // I2C1 Pin Select bit (Use default SCL1/SDA1 pins for I2C1 )
#pragma config IOL1WAY = OFF // IOLOCK One-Way Set Enable (The IOLOCK bit can be set and cleared using the unlock sequence)
#pragma config OSCIOFNC = ON // OSCO Pin Configuration (OSCO pin functions as port I/O (RA3))
#pragma config FCKSM = CSDCMD // Clock Switching and Fail-Safe Clock Monitor (Sw Disabled, Mon Disabled)
#pragma config FNOSC = FRC // Initial Oscillator Select (Fast RC Oscillator (FRC))
#pragma config PLL96MHZ = OFF // 96MHz PLL Startup Select (96 MHz PLL Startup is enabled by user in software( controlled with the PLLEN bit))
#pragma config PLLDIV = NODIV // USB 96 MHz PLL Prescaler Select (Oscillator input used directly (4 MHz input))
#pragma config IESO = OFF // Internal External Switchover (IESO mode (Two-Speed Start-up) disabled)

// CONFIG1
#pragma config WDTPS = PS32768 // Watchdog Timer Postscaler (1:32,768)
#pragma config FWPSA = PR128 // WDT Prescaler (Prescaler ratio of 1:128)
#pragma config WINDIS = OFF // Windowed WDT (Standard Watchdog Timer enabled,(Windowed-mode is disabled))
#pragma config FWDTEN = OFF // Watchdog Timer (Watchdog Timer is disabled)
#pragma config ICS = PGx1 // Emulator Pin Placement Select bits (Emulator functions are shared with PGEC1/PGED1)
#pragma config GWRP = OFF // General Segment Write Protect (Writes to program memory are allowed)
#pragma config GCP = OFF // General Segment Code Protect (Code protection is disabled)
#pragma config JTAGEN = OFF // JTAG Port Enable (JTAG port is disabled)

#include <libpic30.h>
#include <xc.h>

int main()
{
    AD1PCFGL = 0xFFFF;
    AD1PCFG = 0xFFFF;

    // Setup a 5ms timer
    T1CONbits.TON = 1;
    T1CONbits.TCKPS = 0b10;
    T1CONbits.TCS = 0;
    T1CONbits.TGATE = 0;
    T1CONbits.TSYNC = 0;
    PR1 = FCY / 128ul;

    IEC0bits.T1IE = 1;
    IPC0bits.T1IP = 7;

    TRISAbits.TRISA0 = 0;
    TRISAbits.TRISA1 = 0;
    TRISAbits.TRISA2 = 0;
    TRISAbits.TRISA3 = 0;
    TRISBbits.TRISB9 = 0;
    TRISBbits.TRISB8 = 0;

    TRISBbits.TRISB4 = 1;
    TRISBbits.TRISB14 = 1;
    TRISBbits.TRISB13 = 1;

    TRISBbits.TRISB10 = 1;
    TRISBbits.TRISB11 = 1;
    TRISBbits.TRISB5 = 1;

    TRISBbits.TRISB2 = 1;

    while (1) {
        LATAbits.LATA2 = PORTBbits.RB14;
        LATAbits.LATA3 = PORTBbits.RB13;
        LATBbits.LATB9 = PORTBbits.RB5;
        LATBbits.LATB8 = PORTBbits.RB4;
        __delay_ms(10);
    }

    return -1;
}

void __attribute__((interrupt, no_auto_psv)) _T1Interrupt()
{
    if (IFS0bits.T1IF) {
        IFS0bits.T1IF = 0;
        if (PORTBbits.RB2 || PORTBbits.RB11) {
            LATAbits.LATA1 = ~LATAbits.LATA1;
        } else {
            LATAbits.LATA1 = 0;
        }
        if (PORTBbits.RB2 || PORTBbits.RB10) {
            LATAbits.LATA0 = ~LATAbits.LATA0;
        } else {
            LATAbits.LATA0 = 0;
        }
    }
}
