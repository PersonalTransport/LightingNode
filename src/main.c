// CONFIG4
#pragma config DSWDTPS = DSWDTPSF // DSWDT Postscale Select (1:2,147,483,648 (25.7 days))
#pragma config DSWDTOSC = LPRC // Deep Sleep Watchdog Timer Oscillator Select (DSWDT uses Low Power RC Oscillator (LPRC))
#pragma config RTCOSC = SOSC // RTCC Reference Oscillator Select (RTCC uses Secondary Oscillator (SOSC))
#pragma config DSBOREN = OFF // Deep Sleep BOR Enable bit (BOR disabled in Deep Sleep)
#pragma config DSWDTEN = OFF // Deep Sleep Watchdog Timer (DSWDT disabled)

// CONFIG3
#pragma config WPFP = WPFP63 // Write Protection Flash Page Segment Boundary (Highest Page (same as page 21))
#pragma config SOSCSEL = SOSC // Secondary Oscillator Pin Mode Select (SOSC pins in Default (high drive-strength) Oscillator Mode)
#pragma config WUTSEL = LEG // Voltage Regulator Wake-up Time Select (Default regulator start-up time used)
#pragma config WPDIS = WPDIS // Segment Write Protection Disable (Segmented code protection disabled)
#pragma config WPCFG = WPCFGDIS // Write Protect Configuration Page Select (Last page and Flash Configuration words are unprotected)
#pragma config WPEND = WPENDMEM // Segment Write Protection End Page Select (Write Protect from WPFP to the last page of memory)

// CONFIG2
#pragma config POSCMOD = HS // Primary Oscillator Select (HS Oscillator mode selected)
#pragma config I2C1SEL = PRI // I2C1 Pin Select bit (Use default SCL1/SDA1 pins for I2C1 )
#pragma config IOL1WAY = OFF // IOLOCK One-Way Set Enable (Once set, the IOLOCK bit cannot be cleared)
#pragma config OSCIOFNC = OFF // OSCO Pin Configuration (OSCO pin functions as clock output (CLKO))
#pragma config FCKSM = CSDCMD // Clock Switching and Fail-Safe Clock Monitor (Sw Disabled, Mon Disabled)
#pragma config FNOSC = FRCDIV // Initial Oscillator Select (Fast RC Oscillator with Postscaler (FRCDIV))
#pragma config PLL96MHZ = OFF // 96MHz PLL Startup Select (96 MHz PLL Startup is enabled automatically on start-up)
#pragma config PLLDIV = NODIV // USB 96 MHz PLL Prescaler Select (Oscillator input used directly (4 MHz input))
#pragma config IESO = ON // Internal External Switchover (IESO mode (Two-Speed Start-up) enabled)

// CONFIG1
#pragma config WDTPS = PS32768 // Watchdog Timer Postscaler (1:32,768)
#pragma config FWPSA = PR128 // WDT Prescaler (Prescaler ratio of 1:128)
#pragma config WINDIS = OFF // Windowed WDT (Standard Watchdog Timer enabled,(Windowed-mode is disabled))
#pragma config FWDTEN = OFF // Watchdog Timer (Watchdog Timer is disabled)
#pragma config ICS = PGx2 // Emulator Pin Placement Select bits (Emulator functions are shared with PGEC1/PGED1)
#pragma config GWRP = OFF // General Segment Write Protect (Writes to program memory are allowed)
#pragma config GCP = OFF // General Segment Code Protect (Code protection is disabled)
#pragma config JTAGEN = OFF // JTAG Port Enable (JTAG port is disabled)

#include <xc.h>
#include <lighting.h>

struct l_irqmask l_sys_irq_disable()
{
    IEC0bits.U1RXIE = 0;
    IEC0bits.U1TXIE = 0;
    IEC0bits.IC1IE = 0;
    IFS0bits.U1TXIF = 0;
    IFS0bits.U1RXIF = 0;
    struct l_irqmask mask = { IPC2bits.U1RXIP, IPC3bits.U1TXIP, IPC0bits.IC1IP };
    return mask;
}

void l_sys_irq_restore(struct l_irqmask previous)
{
    IPC2bits.U1RXIP = previous.rx_level;
    IFS0bits.U1TXIF = 0;
    IEC0bits.U1RXIE = 1;

    IPC3bits.U1TXIP = previous.tx_level;
    IFS0bits.U1RXIF = 0;
    IEC0bits.U1TXIE = 1;

    IPC0bits.IC1IP = previous.t1_level;
    IFS0bits.IC1IF = 0;
    IEC0bits.IC1IE = 1;
}

enum lighting_state {
    OFF,
    RIGHT,
    LEFT,
    HAZARDS
};

void main_task()
{
    if(l_flg_tst_lighting_state()) {
        l_flg_clr_lighting_state();
        LATAbits.LATA1 = 0;
        LATAbits.LATA2 = 0;
    }
}

int main()
{
    //RA1
    //RA2
    //RA3
    //RA4
    //RB0
    //RB1
    //RB8
    //RB9
    //RB13
    
    // Initialize the LIN interface
    if (l_sys_init())
        return -1;

    // Initialize the interface
    if (l_ifc_init_UART1())
        return -1;

    struct l_irqmask irqmask = { 6,6, 7 };
    l_sys_irq_restore(irqmask);
    
    T2CONbits.TCS = 0b0; //Timer2 Clock Source is Internal the clock (FOSC/2)
    T2CONbits.T32 = 0b0; //16 bit mode
    T2CONbits.TGATE = 0b0; //Gated time accumulation is disabled
    T2CONbits.TCKPS = 0b01; //Setting pre-scaler to 8

    TMR2 = 0x0000; //Clearing timer register
    PR2 = 0xC7F3; //Setting the period of timer to about 100ms

    IEC0bits.T2IE = 1;
    IPC1bits.T2IP = 5;
    IFS0bits.T2IF = 0;
    
    T2CONbits.TON = 1; //Turning timer2 on
    
    while (1) {
        main_task();
    }

    return -1;
}

void __attribute__((interrupt,no_auto_psv)) _T2Interrupt() {
    if(IFS0bits.T2IF) {
        switch(l_u8_rd_lighting_state()) {
            case OFF: {
                LATAbits.LATA1 = 0;
                LATAbits.LATA2 = 0;
                break;
            }
            case RIGHT: {
                LATAbits.LATA1 = ~LATAbits.LATA1;
                break;
            }
            case LEFT: {
                LATAbits.LATA2 = ~LATAbits.LATA2;
                break;
            }
            case HAZARDS: {
                LATAbits.LATA1 = ~LATAbits.LATA1;
                LATAbits.LATA2 = ~LATAbits.LATA2;
                break;
            }
            default: {
                LATAbits.LATA1 = 0;
                LATAbits.LATA2 = 0;
            }
        }
        IFS0bits.T2IF = 0;
    }
}

void __attribute__((interrupt, no_auto_psv)) _U1TXInterrupt()
{
    if (IFS0bits.U1TXIF) {
        // Clear TX interrupt flag.
        IFS0bits.U1TXIF = 0;

        l_ifc_tx_UART1();

        if (U1STAbits.FERR)
            U1STAbits.FERR = 0;
    }
}

void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt()
{
    if (IFS0bits.U1RXIF) {
        // Clear RX interrupt flag.
        IFS0bits.U1RXIF = 0;

        l_ifc_rx_UART1();

        if (U1STAbits.FERR)
            U1STAbits.FERR = 0;
    }
}
