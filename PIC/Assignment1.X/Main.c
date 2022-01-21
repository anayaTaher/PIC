#define _XTAL_FREQ   4000000UL  // needed for the delays, set to 4 MH= your crystal frequency
// CONFIG1H
#pragma config OSC = XT         // Oscillator Selection bits (XT oscillator)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown Out Reset Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = ON         // Watchdog Timer Enable bit (WDT enabled)
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = ON         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-003FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (004000-007FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (008000-00BFFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (00C000-00FFFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-003FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (004000-007FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (008000-00BFFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (00C000-00FFFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)

#include <xc.h>
#include <stdio.h>
#define STARTVALUE  3036

#ifndef XC_LCD_X8_H
#define _XTAL_FREQ   4000000UL 
#define LCD_TYPE 2
#define LCD_LINE_TWO 0x40
#define LCD_LINE_SIZE 16
#define lcd_output_enable(x) PORTEbits.RE1 = x
#define lcd_output_rs(x) PORTEbits.RE2 = x
struct lcd_pin_map {
    unsigned un1    : 1;
    unsigned rs     : 1;
    unsigned rw     : 1;
    unsigned enable : 1;
    unsigned data   : 4;
} lcd __at(0xF83);
#endif

void restartTimer0(void);
void clockNormalMode(void);
void changeClockMode(void);
void changeSetupMode(void);
void incrementClock(void);
void decrementClock(void);
void delay_ms(unsigned int n);
void send_string_no_lib(unsigned char *p);
unsigned char is_byte_available(void);
unsigned char read_byte_no_lib(void);
void send_byte_no_lib(unsigned char c);
void init_adc_no_lib(void);
float read_adc_voltage(unsigned char channel);
int read_adc_raw_no_lib(unsigned char channel);
void delay_cycles(unsigned char n);
void lcd_send_nibble(unsigned char n);
void lcd_send_byte(unsigned char cm_data, unsigned char n);
void lcd_init(void);
void lcd_gotoxy(unsigned char x, unsigned char y);
void lcd_putc(char c);
void lcd_puts(char *s);

unsigned short seconds = 0;
unsigned short minutes = 0;
unsigned short hours   = 0;
unsigned short clkMode = 0;
unsigned short stpMode = 0;
unsigned short initial = 1; // Initial State
unsigned char LCD_INIT_STRING[4] = {0x20 | (LCD_TYPE << 2), 0xc, 1, 6};

// Restart Timer0 So That We Create An Interrupt Every 1 Second.
void restartTimer0(void) {
    TMR0H = (unsigned char)((STARTVALUE >>  8) & 0x00FF);
    TMR0L = (unsigned char)(STARTVALUE & 0x00FF );   
}

void clockNormalMode(void) {
    INTCONbits.TMR0IF = 0;
    if(++seconds >= 60) { 
        seconds=0;
        if (++minutes >= 60) {
            minutes = 0;
            if (++hours >= 24 ) hours = 0;
        }
    }
    restartTimer0();
}

void changeClockMode(void) {
    INTCONbits.INT0IF = 0;
    initial = 0;
    clkMode = 1 - clkMode;
}

void changeSetupMode(void) {
    INTCON3bits.INT1IF = 0;
    if (stpMode == 0) stpMode = 1;
    else if(stpMode == 1) stpMode = 2;
    else stpMode = 0;
}

void incrementClock(void) {
    if (initial == 0 && clkMode == 1) { // Setup Mode
        if (stpMode == 0 && ++seconds >= 60) seconds=0;
        else if(stpMode == 1 && ++minutes >= 60) minutes=0;
        else if(stpMode == 2 && ++hours >= 24) hours=0;
    }
}

void decrementClock(void) {
    if (initial == 0 && clkMode == 1) { // Setup Mode
        if (stpMode == 0 && --seconds == -1) seconds=59;
        else if(stpMode == 1 && --minutes == -1) minutes=59;
        else if(stpMode == 2 && --hours == -1) hours=23;
    }
}

bit isValidChar(char command[9], char c, int i) {
    if (i == 0) return (c == 'w' || c == 'W'); // Small Or Capital
    else if (i == 1) return (c >= '0' && c <= '2');
    else if (i == 2) return (command[1] == '2' ? (c >= '0' && c <= '4') : (c >= '0' && c <= '9'));
    else if (i == 5 || i == 8) return (c >= '0' && c <= '9');
    else if (i == 3 || i == 6) return c == ':';
    else if (i == 4 || i == 7) return (c >= '0' && c <= '5');
    else return 0;
}

void delay_ms(unsigned int n) {
    int i;
    for (i=0; i < n; i++) __delaywdt_ms(1) ; 
}

void send_string_no_lib(unsigned char *p) {
    while (*p) {
        send_byte_no_lib(*p);
        p++;
    }
}

unsigned char is_byte_available(void) {
    if (RCSTAbits.FERR || RCSTAbits.OERR) {
        RCSTAbits.CREN = 0;
        RCSTAbits.CREN = 1;
    }
    if (PIR1bits.RCIF) return 1;
    else return 0;
}

unsigned char read_byte_no_lib(void) {
    unsigned char c;
    c = RCREG;
    return c;
}

void send_byte_no_lib(unsigned char c) {
    while (!TXSTAbits.TRMT) CLRWDT();
    TXREG = c;
}

void init_adc_no_lib(void) {
    ADCON0 = 0;
    ADCON0bits.ADON = 1;
    ADCON2 = 0b10001001;
}

int read_adc_raw_no_lib(unsigned char channel) {
    int raw_value;
    ADCON0bits.CHS = channel;
    ADCON0bits.GO = 1;
    while (ADCON0bits.GO) {};
    raw_value = ADRESH << 8 | ADRESL;
    return raw_value;
}

float read_adc_voltage(unsigned char channel) {
    int raw_value;
    float voltage;
    raw_value = read_adc_raw_no_lib(channel);
    voltage = (raw_value * 5) / 1023.0;
    return voltage;
}

void delay_cycles(unsigned char n) {
    int x;
    for (x = 0; x <= n; x++) CLRWDT();
}

void lcd_send_nibble(unsigned char n) {
    lcd.data = n;
    delay_cycles(1);
    lcd_output_enable(1);
    __delaywdt_us(2);
    lcd_output_enable(0);
}

void lcd_send_byte(unsigned char cm_data, unsigned char n) {
    lcd_output_rs(cm_data);
    delay_cycles(1);
    delay_cycles(1);
    lcd_output_enable(0);
    lcd_send_nibble(n >> 4);
    lcd_send_nibble(n & 0x0f);
    if (cm_data) __delaywdt_us(200);
    else delay_ms(2);
}

void lcd_init(void) {
    unsigned char i;
    lcd_output_rs(0);
    lcd_output_enable(0);

    delay_ms(25);   
    for (i = 1; i <= 3; ++i) {
        lcd_send_nibble(3);
        delay_ms(6);
    }

    lcd_send_nibble(2);
    for (i = 0; i <= 3; ++i) lcd_send_byte(0, LCD_INIT_STRING[i]);
}

void lcd_gotoxy(unsigned char x, unsigned char y) {
    unsigned char address;

    switch (y) {
        case 1: address = 0x80;
            break;
        case 2: address = 0xc0;
            break;
        case 3: address = 0x80 + LCD_LINE_SIZE;
            break;
        case 4: address = 0xc0 + LCD_LINE_SIZE;
            break;
    }
    address += x - 1;
    lcd_send_byte(0, (unsigned char) (0x80 | address));
}

void lcd_putc(char c) {
    switch (c) {
        case '\f': lcd_send_byte(0, 1);
            delay_ms(2);
            break;
        case '\n': lcd_gotoxy(1, 2);
            break;
        case '\b': lcd_send_byte(0, 0x10);
            break;
        default: lcd_send_byte(1, c);
            break;
    }
}

void lcd_puts(char *s) {
    while (*s) {
        lcd_putc(*s);
        s++;
    }
}

void __interrupt(high_priority) highIsr (void) {
    if(INTCONbits.TMR0IF) {
        if (initial == 1 || (initial == 0 && clkMode == 0) ) clockNormalMode();
        else INTCONbits.TMR0IF = 0; // Ignore Timer0 In Setup Mode
    } else if(INTCONbits.INT0IF) {
        changeClockMode();
    }else if(INTCON3bits.INT1IF) {
        if (initial == 0 && clkMode == 1) changeSetupMode();
        else INTCON3bits.INT1IF = 0;
    } 
}

void init(void) {
    
    // Setup Port
    ADCON0 = 0x00;
    ADCON1 = 0x0C; // 3 Analog Channels
    TRISA  = 0xFF; // All Inputs
    TRISB  = 0xFF; // All Inputs
    TRISC  = 0x80; // RX Input, Others Outputs
    TRISD  = 0x00; // All Outputs
    TRISE  = 0x00; // All Outputs
    PORTD   = 0   ;
    PIE1    = 0   ;
    
    // Setup Serial
    unsigned char dummy;
    dummy = RCREG;
    
    BAUDCONbits.BRG16 = 0;
    TXSTA             = 0;
    SPBRG             = 25;
    TXSTAbits.BRGH    = 1;  // Baud-Rate High-Speed Option
    TXSTAbits.TXEN    = 1;  // Enable Transmission
    RCSTA             = 0;  // SERIAL RECEPTION W/8 BITS,
    RCSTAbits.CREN    = 1;  // Enable Reception
    RCSTAbits.SPEN    = 1;  // Enable Serial Port
    
    T0CON = 0;
    T0CONbits.T0PS0  = 1; // 16 Pre_Scalar
    T0CONbits.T0PS1  = 1;
    T0CONbits.T0PS2  = 0;
    T0CONbits.TMR0ON = 1;
    restartTimer0();

    T1CONbits.TMR1CS  = 1;
    T1CONbits.TMR1ON  = 1;
    T1CONbits.T1CKPS1 = 0;
    T1CONbits.T1CKPS0 = 0;

    TMR1H = 0;
    TMR1L = 0;
    INTCONbits.GIEH   = 1;    // Enable global interrupt bits
    INTCONbits.GIEL   = 1;    // Enable global interrupt bits
    INTCONbits.T0IE = 1;
    INTCON2 = 0;
    INTCON3 = 0;
    INTCON2bits.INTEDG0 = 1;  // Interrupt 1 on rising edge
    INTCON2bits.INTEDG1 = 1;  // Interrupt 1 on rising edge
    INTCON3bits.INT1IE  = 1;  // Enable external interrupt 1
    
    TRISCbits.RC0 = 1; // Timer1 Clock
    PORTCbits.RC2 = 0; // Turn Off Cooler
    PORTCbits.RC5 = 0; // Turn Off Heater
    
    seconds = 0;
    minutes = 0;
    hours   = 0;
    clkMode = 1;
    stpMode = 0;
    initial = 1;   
    
    RCONbits.IPEN = 0;        // Disable Interrupt priority , All are high
    INTCONbits.TMR0IE = 1;    // Enable Timer0 Interrupt
    T0CONbits.TMR0ON  = 1;    // Start timer 0
    INTCONbits.INT0IE   = 1;  // Enable external interrupt 1
    
    restartTimer0();          // TR1Value = 3036, 1 second
}

void main(void) {
    
    init();
   
    unsigned char RecvedChar = 0;
    char Buffer[32]; 
    char command[9];
    int i = 0;
    short isRB4Release = 1;
    short isRB5Release = 1;
    float voltage;
    float t;
    
    lcd_init();
    init_adc_no_lib();
    lcd_putc('\f'); // Clears The Display
    
    sprintf(Buffer, "This Work Was Done By [Taher Anaya] And [Adam Siksik]\n\r");
    send_string_no_lib(Buffer);
    sprintf(Buffer, "1 - To View The Temperature: Press T \n\r");
    send_string_no_lib(Buffer);
    sprintf(Buffer, "2 - To View The Time: Press R \n\r");
    send_string_no_lib(Buffer);
    sprintf(Buffer, "3 - To Change Time: Enter The New Time In The Format WHH:MM:SS\n\r");
    send_string_no_lib(Buffer);
    
    while (1) {
        CLRWDT();
        
        if(PORTBbits.RB2 == 0) incrementClock();
        else if(PORTBbits.RB3 == 0) decrementClock();
        
        if(PORTBbits.RB4 == 1) isRB4Release = 1;
        if(PORTBbits.RB5 == 1) isRB5Release = 1;
        
        if(PORTBbits.RB4 == 0 && isRB4Release) { //If RB4 is pressed
            PORTCbits.RC2 = !PORTCbits.RC2;
            isRB4Release = 0;
        } else if(PORTBbits.RB5 == 0 && isRB5Release) { //If RB5 is pressed
            PORTCbits.RC5 = !PORTCbits.RC5;
            isRB5Release = 0;
        }
        
        voltage = read_adc_voltage(2);
        t = 100 * voltage;
       
        lcd_gotoxy(1, 1);
        sprintf(Buffer, "%02d:%02d:%02d", hours, minutes, seconds);
        lcd_puts(Buffer);

        lcd_gotoxy(10, 1);
        sprintf(Buffer, "T=%4.2f\n", t);
        lcd_puts(Buffer);
        
        lcd_gotoxy(1, 2);
        sprintf(Buffer, "[C:%s]  ", PORTCbits.RC2==1?"ON":"OFF");
        lcd_puts(Buffer);
        
        lcd_gotoxy(9, 2);
        sprintf(Buffer, "[H:%s]  ", PORTCbits.RC5==1?"ON":"OFF");
        lcd_puts(Buffer);
        
        lcd_gotoxy(1, 3);
        if (!clkMode) sprintf(Buffer, "Normal Mode     ");
        else sprintf(Buffer, "Setup=>[%s]   ", stpMode==0?"Seconds":stpMode==1?"Minutes":"Hours");
        lcd_puts(Buffer);
        
        lcd_gotoxy(1, 4);
        sprintf(Buffer, "  [Taher, Adam]  ");
        lcd_puts(Buffer);
        
        if (is_byte_available()) {
            RecvedChar = read_byte_no_lib();
            
            if (isValidChar(command, RecvedChar, i)) {
                command[i++] = RecvedChar;
            } else {
                i = 0;
                if (RecvedChar == 'R') {
                    sprintf(Buffer, "\n\rTime: %02d:%02d:%02d\n\r===================\n\r", hours, minutes, seconds);
                    send_string_no_lib(Buffer);
                } else if (RecvedChar == 'T') {
                    sprintf(Buffer, "\n\rTemp: %4.1f\n\r===================\n\r", t);
                    send_string_no_lib(Buffer);
                }
            }
            
            if (i == 9) {
                i = 0;
                hours   = 10*(command[1] - 48) + command[2] - 48;
                minutes = 10*(command[4] - 48) + command[5] - 48;
                seconds = 10*(command[7] - 48) + command[8] - 48;
            }
        }
        
        delay_ms(200);
    }
}