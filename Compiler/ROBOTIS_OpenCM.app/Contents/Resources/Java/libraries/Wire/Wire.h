/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 LeafLabs LLC.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *****************************************************************************/

/**
 *  @brief Wire library, ported from Arduino. Provides a lean
 *  interface to I2C (two-wire) communication.
 */

#include "wirish.h"

#ifndef _WIRE_H_
#define _WIRE_H_

#define uint8_t uint8

typedef struct {
  uint8 scl;
  uint8 sda;
} Port;

/* You must update the online docs if you change this value. */
#define WIRE_BUFSIZ 32

/* return codes from endTransmission() */
#define SUCCESS   0        /* transmission was successful */
#define EDATA     1        /* too much data */
#define ENACKADDR 2        /* readd nack on transmit of address */
#define ENACKTRNS 3        /* readd nack on transmit of data */
#define EOTHER    4        /* other error */

#define SDA 20
#define SCL 21

#define I2C_WRITE 0
#define I2C_READ  1

#define I2C_DELAY do{for(int i=0;i<1;i++) {asm volatile("nop");}}while(0)

class TwoWire {
 private:
    uint8 rx_buf[WIRE_BUFSIZ];      /* read buffer */
    uint8 rx_buf_idx;               /* first unread idx in rx_buf */
    uint8 rx_buf_len;               /* number of bytes read */

    uint8 tx_addr;                  /* address transmitting to */
    uint8 tx_buf[WIRE_BUFSIZ];      /* transmit buffer */
    uint8 tx_buf_idx;  /* next idx available in tx_buf, -1 overflow */
    boolean tx_buf_overflow;
    Port port;
    uint8 writeOneByte(uint8);
    uint8 readOneByte(uint8, uint8*);
 public:
    TwoWire();
    void begin();
    void begin(uint8, uint8);
    void beginTransmission(uint8);
    void beginTransmission(int);
    uint8 endTransmission(void);
    uint8 requestFrom(uint8, int);
    uint8 requestFrom(int, int);
    void write(uint8);
    void write(uint8*, int);
    void write(int);
    void write(int*, int);
    void write(char*);
    uint8 available();
    uint8 read();
};

void    i2c_start(Port port);
void    i2c_stop(Port port);
boolean i2c_get_ack(Port port);
void    i2c_write_ack(Port port);
void    i2c_write_nack(Port port);
uint8   i2c_shift_in(Port port);
void    i2c_shift_out(Port port, uint8 val);

extern TwoWire Wire;

#endif // _WIRE_H_
