#include <rf430frl152h.h>
#include <rf13m_rom_config.h>
#include <stdint.h>
#include "event_queue.h"

EventQueue event_queue;

void device_init(void);
void process_event(const Event *event);
void start_timer(uint16_t duration_ms);


int main(void)
{
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer
	
	rf13m_setup();

    init_iso15693(CLEAR_BLOCK_LOCKS);  // clear all block locks
    device_init();
	event_queue_init(&event_queue);

	P1DIR |= BIT0 | BIT1;
	P1OUT |= BIT0 | BIT1;

	// Configure P1.3 to output ACLK
	P1DIR |= BIT3;         // Set P1.3 as output
	P1SEL0 |= BIT3;        // Set P1.3 function select bits
	P1SEL1 |= BIT3;        // Select quaternary function (ACLK output)

	start_timer(100);

	while (1) {
        Event event;
        if(event_queue_get(&event_queue, &event)) {
            process_event(&event);
        }

        __bis_SR_register(LPM3_bits | GIE);  // Enter LPM3
    }
}

void device_init(void)
{
//  P1SEL0 = 0xF0; //keep JTAG
//  P1SEL1 = 0xF0; //keep JTAG
    P1SEL0 = 0x00; //no JTAG
    P1SEL1 = 0x00; //no JTAG

    P1DIR &= ~0xEF;
    P1REN = 0;

    CCSCTL0 = CCSKEY;                        // Unlock CCS

    CCSCTL1 = 0;                             // do not half the clock speed
    CCSCTL4 = SELA_1 + SELM_0 + SELS_0;      // Select VLO for ACLK and select HFCLK/DCO for MCLK, and SMCLK
    CCSCTL5 = DIVA_2 + DIVM_1 + DIVS_1;      // Set the Dividers for ACLK (4), MCLK, and SMCLK to 1
    CCSCTL6 = XTOFF;                         // Turns of the crystal if it is not being used
    CCSCTL8 = ACLKREQEN + MCLKREQEN + SMCLKREQEN; //disable clocks if they are not being used

    CCSCTL0_H |= 0xFF;                       // Lock CCS

  return;
}


// Start timer for the specified duration in milliseconds
void start_timer(uint16_t duration_ms) {
    // seems to be 64kHz, not sure why

    // Configure Timer_A
    TA0CCTL0 = CCIE;                           // Enable interrupt
    TA0CCR0 = duration_ms * 64;                // ACLK is sourced from VLO at 64 kHz?
    TA0CTL = TASSEL__ACLK | MC__UP | TACLR;    // ACLK, Up mode, clear TAR
}

// Timer_A interrupt service routine
#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer_A_ISR(void) {
    // Stop Timer_A
    TA0CTL = MC__STOP;
    TA0CCTL0 &= ~CCIE;
    TA0CCTL0 &= ~CCIFG;  // Clear interrupt flag

    //P1OUT ^= BIT1;

    // Create a timer event
    Event event;
    event.type = EVENT_TIMER;
    // Add any additional data if necessary

    event_queue_put(&event_queue, &event);

    // Exit LPM3
    __bic_SR_register_on_exit(LPM3_bits);
}



//#pragma CODE_SECTION(RF13M_ISR, ".fram_driver_code")  // comment this line for using ROM's RF13M ISR, uncomment next one, see .cmd file for details
#pragma CODE_SECTION(RF13M_ISR, ".rf13m_rom_isr")       // comment this line for creating a custom RF13M ISR that will exist in FRAM, bypassing ROM's, uncomment previous
#pragma vector = RF13M_VECTOR
__interrupt void RF13M_ISR(void)
{
    // Right now this vector is pointing to the ROMs firmware location that runs the RF stack.
    // Entering code here will, without changing the CODE_SECTION lines
    // above, will cause an error.
    // Changing the code section above will cause the ROM RF stack to be bypassed.  New handler will need to be created.
}


// Process events from the event queue
void process_event(const Event *event) {
    switch (event->type) {
        case EVENT_NFC:
            //process_nfc_event(event->data.digit);
            break;
        case EVENT_TIMER:
            //process_timer_event();
            P1OUT ^= BIT0;
            start_timer(100);
            break;
        default:
            // Unknown event type
            break;
    }

}

