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
#include <lighting.h>
#include <xc.h>

enum signal_light_state {
    SIGNAL_OFF,
    SIGNAL_RIGHT,
    SIGNAL_LEFT,
    SIGNAL_HAZARDS
};

enum head_light_state {
    HEADLIGHTS_OFF,
    HEADLIGHTS_LOW_BEAMS,
    HEADLIGHTS_HIGH_BEAMS
};

void main_task()
{
    if (l_flg_tst_signal_light_state()) {
        l_flg_clr_signal_light_state();
        LATAbits.LATA0 = 0;
        LATAbits.LATA1 = 0;
    }

    if (l_flg_tst_head_light_state()) {
        switch ((enum head_light_state)l_u8_rd_head_light_state()) {
        case HEADLIGHTS_OFF: {
            LATAbits.LATA2 = 0;
            LATAbits.LATA3 = 0;
            break;
        }
        case HEADLIGHTS_LOW_BEAMS: {
            LATAbits.LATA2 = 1;
            LATAbits.LATA3 = 0;
            break;
        }
        case HEADLIGHTS_HIGH_BEAMS: {
            LATAbits.LATA2 = 1;
            LATAbits.LATA3 = 1;
            break;
        }
        }
        l_flg_clr_head_light_state();
    }
}

int main()
{
    // Initialize the LIN interface
    if (l_sys_init())
        return -1;

    // Initialize the interface
    if (l_ifc_init_UART1())
        return -1;

    // Set UART TX to interrupt level 6
    // Set UART RX to interrupt level 6
    struct l_irqmask irqmask = { 6, 6 };
    l_sys_irq_restore(irqmask);

    l_bool configuration_ok = false;
    l_u16 configuration_timeout = 1000;
    do {
        if (l_ifc_read_status_UART1() & (1 << 6)) {
            configuration_ok = true;
            break;
        }
        __delay_ms(5);
        configuration_timeout--;
    } while (configuration_timeout || !configuration_ok);

    if (!configuration_ok) {
        // Master did not configure this node.
        return -1;
    }

    T2CONbits.TCS = 0b0; //Timer2 Clock Source is Internal the clock (FOSC/2)
    T2CONbits.T32 = 0b0; //16 bit mode
    T2CONbits.TGATE = 0b0; //Gated time accumulation is disabled
    T2CONbits.TCKPS = 0b10; //Setting pre-scaler to 64

    // (64)*PR2/FCY = 1/2
    // PR2/FCY = 1/128
    // PR2 = FCY/128
    TMR2 = 0x0000;
    PR2 = FCY / 128ul; // .5s delay

    IEC0bits.T2IE = 1;
    IPC1bits.T2IP = 5;
    IFS0bits.T2IF = 0;
    T2CONbits.TON = 1; //Turning timer2 on

    TRISAbits.TRISA2 = 0;
    TRISAbits.TRISA3 = 0;
    TRISAbits.TRISA4 = 0;
    TRISBbits.TRISB5 = 0;

    while (1) {
        main_task();
    }

    return -1;
}

void __attribute__((interrupt, no_auto_psv)) _T2Interrupt()
{
    if (IFS0bits.T2IF) {
        IFS0bits.T2IF = 0;
        switch ((enum signal_light_state)l_u8_rd_signal_light_state()) {
        case SIGNAL_OFF: {
            LATBbits.LATB5 = 0;
            LATAbits.LATA4 = 0;
            break;
        }
        case SIGNAL_RIGHT: {
            LATBbits.LATB5 = ~LATBbits.LATB5;
            break;
        }
        case SIGNAL_LEFT: {
            LATAbits.LATA4 = ~LATAbits.LATA4;
            break;
        }
        case SIGNAL_HAZARDS: {
            LATBbits.LATB5 = ~LATBbits.LATB5;
            LATAbits.LATA4 = ~LATAbits.LATA4;
            break;
        }
        }
    }
}

void __attribute__((interrupt, no_auto_psv)) _U1TXInterrupt()
{
    if (IFS0bits.U1TXIF) {
        IFS0bits.U1TXIF = 0;
        l_ifc_tx_UART1();
    }
}

void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt()
{
    if (IFS0bits.U1RXIF) {
        IFS0bits.U1RXIF = 0;
        l_ifc_rx_UART1();
    }
}

struct l_irqmask l_sys_irq_disable()
{
    IEC0bits.U1RXIE = 0;
    IEC0bits.U1TXIE = 0;
    IFS0bits.U1TXIF = 0;
    IFS0bits.U1RXIF = 0;
    struct l_irqmask mask = { IPC2bits.U1RXIP, IPC3bits.U1TXIP };
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
}
