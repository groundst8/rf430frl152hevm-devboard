/*
 * tcal9539.c
 *
 *  Created on: Nov 10, 2024
 *      Author: nick
 */

#include <tcal9539.h>
#include <rf430frl152h.h>

void init_i2c(void) {
    // Put eUSCI_B0 into reset state
    UCB0CTLW0 |= UCSWRST;
    // Configure as I2C master, synchronous mode, SMCLK
    UCB0CTLW0 |= UCMODE_3 | UCMST | UCSSEL__SMCLK;
    // Set baud rate for ~100kHz; SMCLK is 2 MHz from the looks of it
    UCB0BRW = 20;
    // Set slave address
    UCB0I2CSA = TCAL9539_ADDRESS;

    // Configure I2C pins: P1.0 (SDA) and P1.1 (SCL)
    P1SEL0 |= BIT1 | BIT0;
    P1SEL1 &= ~(BIT1 | BIT0);

    // Release eUSCI_B0 from reset state
    UCB0CTLW0 &= ~UCSWRST;
}

void write_tcal9539_register(uint8_t reg, uint8_t data) {

    while (UCB0STATW & UCBBUSY);  // Wait if I2C bus is busy

    // Start condition and send register address
    UCB0CTLW0 |= UCTR | UCTXSTT;

    // Wait for START condition and slave address transmission to complete
    while (UCB0CTLW0 & UCTXSTT);

    // Wait for TX buffer to be ready for the register address
    while (!(UCB0IFG & UCTXIFG0));
    UCB0TXBUF = reg;

    // Send data
    while (!(UCB0IFG & UCTXIFG0));
    UCB0TXBUF = data;

    // Wait for transmission to complete and send stop condition
    while (!(UCB0IFG & UCTXIFG0));
    UCB0CTLW0 |= UCTXSTP;
    while (UCB0CTLW0 & UCTXSTP);
}
