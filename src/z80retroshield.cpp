
// The MIT License (MIT)

// Copyright (c) 2019 Erturk Kocalar, http://8Bitforce.com/
// Copyright (c) 2019 Steve Kemp, https://steve.kemp.fi/

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.


#include <stdlib.h>
#include <Arduino.h>
#include <z80retroshield.h>

#ifdef ARDUINO_ARCH_AVR
#include <avr/pgmspace.h>

static void INITIALIZE(void) { }

/* Digital Pin Assignments */
static inline void DATA_OUT(uint8_t data) { (PORTL) = data; }
static inline uint8_t DATA_IN(void) { return (PINL); }
static inline uint8_t ADDR_H(void)  { return (PINC); }
static inline uint8_t ADDR_L(void)  { return (PINA); }
static inline unsigned int ADDR(void) { return ((unsigned int) (ADDR_H() << 8 | ADDR_L())); }

static const uint8_t uP_RESET_N = 38;
static const uint8_t uP_MREQ_N  = 41;
static const uint8_t uP_IORQ_N  = 39;
static const uint8_t uP_RD_N    = 53;
static const uint8_t uP_WR_N    = 40;
static const uint8_t uP_NMI_N   = 51;
static const uint8_t uP_INT_N   = 50;
static const uint8_t uP_CLK     = 52;

//
// Fast routines to drive clock signals high/low; faster than digitalWrite
// required to meet >100kHz clock
//
static inline void CLK_HIGH(void) { PORTB = PORTB | 0x02; }
static inline void CLK_LOW(void) { PORTB = PORTB & 0xFC; }
static inline uint8_t STATE_RD_N(void) { return (PINB & 0x01); }
static inline uint8_t STATE_WR_N(void) { return (PING & 0x02); }
static inline uint8_t STATE_MREQ_N(void) { return (PING & 0x01); }
static inline uint8_t STATE_IORQ_N(void) { return (PING & 0x04); }

static const uint8_t DIR_IN = 0x00;
static const uint8_t DIR_OUT = 0xFF;
static inline void DATA_DIR(uint8_t dir) { DDRL = dir; }
static inline void ADDR_H_DIR(uint8_t dir) { DDRC = dir; }
static inline void ADDR_L_DIR(uint8_t dir) { DDRA = dir; }

#endif  // ARDUINO_ARCH_AVR

#ifdef ADAFRUIT_GRAND_CENTRAL_M4

static const uint8_t uP_RESET_N = 38;
static const uint8_t uP_MREQ_N  = 41;
static const uint8_t uP_IORQ_N  = 39;
static const uint8_t uP_RD_N    = 53;
static const uint8_t uP_WR_N    = 40;
static const uint8_t uP_NMI_N   = 51;
static const uint8_t uP_INT_N   = 50;
static const uint8_t uP_CLK     = 52;

static const uint8_t DIR_IN = 0x00;
static const uint8_t DIR_OUT = 0xFF;

static const int PGA = 0;
static const int PGB = 1;
static const int PGC = 2;
static const int PGD = 3;

static void INITIALIZE(void) {
    // Set address and data pins to input mode
    for (int pin = 22; pin <= 37; pin++)
        pinMode(pin, INPUT);
    for (int pin = 42; pin <= 49; pin++)
        pinMode(pin, INPUT);
    static const uint32_t CONFIG_OUTPUT =
        ((1U << 30) |  // Write PINCFG
         (1U << 22) |  // Output Driver Strength Selection
         (1U << 17) |  // Input Buffer Enable
         (1U << 15) | (1U << 14) | (1U << 11) | (1U << 10) |
         (1U <<  6) | (1U <<  7) | (1U <<  4) | (1U <<  5));
    PORT->Group[PGC].WRCONFIG.reg = CONFIG_OUTPUT;
}

/* Digital Pin Assignments */

// A15 30 PA23    A7 29 PB19    D7 42 PC15    CLK      52 PD09
// A14 31 PA22    A6 28 PA14    D6 43 PC14    ~RD_N    53 PD10
// A13 32 PA21    A5 27 PA13    D5 44 PC11    ~WR_N    40 PC13
// A12 33 PA20    A4 26 PA12    D4 45 PC10    ~MREQ_N  41 PC12
// A11 34 PA19    A3 25 PC16    D3 46 PC06    ~IORQ_N  39 PB14
// A10 35 PA18    A2 24 PC17    D2 47 PC07
// A9  36 PA17    A1 23 PA15    D1 48 PC04
// A8  37 PA16    A0 22 PD12    D0 49 PC05

static inline void DATA_OUT(uint8_t data) {
    auto OUTC = &(PORT->Group[PGC].OUT.reg);
    static const uint32_t MASKC =
        ((1U << 15) | (1U << 14) | (1U << 11) | (1U << 10) |
         (1U <<  6) | (1U <<  7) | (1U <<  4) | (1U <<  5));
    uint32_t datac = 0;
    datac |=  (((uint32_t)(data & 0xc0)) << 8);
    datac |=  (((uint32_t)(data & 0x30)) << 6);
    datac |=  (((uint32_t)(data & 0x08)) << 3);
    datac |=  (((uint32_t)(data & 0x04)) << 5);
    datac |=  (((uint32_t)(data & 0x02)) << 3);
    datac |=  (((uint32_t)(data & 0x01)) << 5);
    *OUTC = (*OUTC & ~MASKC) | datac;
}

static inline uint8_t DATA_IN(void) {
    uint32_t datac = PORT->Group[PGC].IN.reg;
    uint8_t data = 0;
    data |=  ((datac >> 8) & 0xc0);
    data |=  ((datac >> 6) & 0x30);
    data |=  ((datac >> 3) & 0x08);
    data |=  ((datac >> 5) & 0x04);
    data |=  ((datac >> 3) & 0x02);
    data |=  ((datac >> 5) & 0x01);
    return data;
}

static inline void DATA_DIR(uint8_t dir) {
    auto DIRSETC = &(PORT->Group[PGC].DIRSET.reg);
    auto DIRCLRC = &(PORT->Group[PGC].DIRCLR.reg);
    static const uint32_t MASKC =
        ((1U << 15) | (1U << 14) | (1U << 11) | (1U << 10) |
         (1U <<  6) | (1U <<  7) | (1U <<  4) | (1U <<  5));

    if (dir == DIR_OUT)
        *DIRSETC = MASKC;
    else
        *DIRCLRC = MASKC;
}

static inline uint8_t ADDR_H(void)  {
    uint32_t dataa = PORT->Group[PGA].IN.reg;
    return (uint8_t)((dataa >> 16) & 0xff);
}

static inline void ADDR_H_DIR(uint8_t dir) {
    auto DIRSETA = &(PORT->Group[PGA].DIRSET.reg);
    auto DIRCLRA = &(PORT->Group[PGA].DIRCLR.reg);
    static const uint32_t MASKA = (0xff << 16);

    if (dir)
        *DIRSETA = MASKA;
    else
        *DIRCLRA = MASKA;
}

static inline uint8_t ADDR_L(void)  {
    uint32_t dataa = PORT->Group[PGA].IN.reg;
    uint32_t datab = PORT->Group[PGB].IN.reg;
    uint32_t datac = PORT->Group[PGC].IN.reg;
    uint32_t datad = PORT->Group[PGD].IN.reg;
    uint8_t addrl = 0;
    addrl |=  ((datab >> 12) & 0x80);
    addrl |=  ((dataa >>  8) & 0x70);
    addrl |=  ((datac >> 13) & 0x08);
    addrl |=  ((datac >> 15) & 0x04);
    addrl |=  ((dataa >> 14) & 0x02);
    addrl |=  ((datad >> 12) & 0x01);
    return addrl;
}

static inline void ADDR_L_DIR(uint8_t dir) {
    auto DIRSETA = &(PORT->Group[PGA].DIRSET.reg);
    auto DIRCLRA = &(PORT->Group[PGA].DIRCLR.reg);
    static const uint32_t MASKA =
        (1U << 14) | (1U << 13) | (1U << 12) | (1U << 15);
    auto DIRSETB = &(PORT->Group[PGB].DIRSET.reg);
    auto DIRCLRB = &(PORT->Group[PGB].DIRCLR.reg);
    static const uint32_t MASKB = (1U << 19);
    auto DIRSETC = &(PORT->Group[PGC].DIRSET.reg);
    auto DIRCLRC = &(PORT->Group[PGC].DIRCLR.reg);
    static const uint32_t MASKC = ((1U << 16) | (1U << 17));
    auto DIRSETD = &(PORT->Group[PGD].DIRSET.reg);
    auto DIRCLRD = &(PORT->Group[PGD].DIRCLR.reg);
    static const uint32_t MASKD = (1U << 12);

    if (dir) {
        *DIRSETA = MASKA;
        *DIRSETB = MASKB;
        *DIRSETC = MASKC;
        *DIRSETD = MASKD;
    } else {
        *DIRCLRA = MASKA;
        *DIRCLRB = MASKB;
        *DIRCLRC = MASKC;
        *DIRCLRD = MASKD;
    }
}

static inline unsigned int ADDR(void) {
    return ((unsigned int) (ADDR_H() << 8 | ADDR_L()));
}

static inline void CLK_HIGH(void) {
    auto OUTSETD = &(PORT->Group[PGD].OUTSET.reg);
    *OUTSETD = (1 << 9);
}

static inline void CLK_LOW(void) {
    auto OUTCLRD = &(PORT->Group[PGD].OUTCLR.reg);
    *OUTCLRD = (1 << 9);
}

static inline bool STATE_RD_N(void) {
    auto IND = &(PORT->Group[PGD].IN.reg);
    return (*IND & (1 << 10));
}

static inline bool STATE_WR_N(void) {
    auto INC = &(PORT->Group[PGC].IN.reg);
    return (*INC & (1 << 13));
}

static inline bool STATE_MREQ_N(void) {
    auto INC = &(PORT->Group[PGC].IN.reg);
    return (*INC & (1 << 12));
}

static inline bool STATE_IORQ_N(void) {
    auto INB = &(PORT->Group[PGB].IN.reg);
    return (*INB & (1 << 14));
}

#endif  // ADAFRUIT_GRAND_CENTRAL_M4


/*
 * Constructor
 */
Z80RetroShieldClassName::Z80RetroShieldClassName()
{
    //
    // Callbacks are all empty by default.
    //
    m_on_memory_read = NULL;
    m_on_memory_write = NULL;
    m_on_io_read = NULL;
    m_on_io_write = NULL;
    m_debug_output = NULL;
    m_debug = 0;
    m_cycle = 0;

    INITIALIZE();

    //
    // Set directions
    //
    DATA_DIR(DIR_IN);
    ADDR_H_DIR(DIR_IN);
    ADDR_L_DIR(DIR_IN);

    //
    // Handle other pins.
    //
    pinMode(uP_RESET_N, OUTPUT);
    pinMode(uP_WR_N, INPUT);
    pinMode(uP_RD_N, INPUT);
    pinMode(uP_MREQ_N, INPUT);
    pinMode(uP_IORQ_N, INPUT);
    pinMode(uP_INT_N, OUTPUT);
    pinMode(uP_NMI_N, OUTPUT);
    pinMode(uP_CLK, OUTPUT);

    Reset();
    digitalWrite(uP_CLK, LOW);
}

/*
 * Destructor
 */
Z80RetroShieldClassName::~Z80RetroShieldClassName()
{
}

void Z80RetroShieldClassName::show_status(const char* header)
{
    if (m_debug_output == nullptr) {
        return;
    }
    char buf[100];
    uint8_t mreq_n = STATE_MREQ_N();
    uint8_t iorq_n = STATE_IORQ_N();
    if ((m_debug & DEBUG_FLAG_CYCLE) ||
        (m_debug & DEBUG_FLAG_IO) && iorq_n == 0 ||
        (m_debug & DEBUG_FLAG_MEM) && mreq_n == 0) {
        sprintf(buf, "%s%4ld addr=%04x data=%02x ~MREQ=%s ~IOREQ=%s  RW=%s",
                header,
                m_cycle,
                ADDR(),
                DATA_IN(),
                mreq_n ? "H" : "L",
                iorq_n ? "H" : "L",
                STATE_RD_N() ? "" : "R",
                STATE_WR_N() ? "" : "W");
        m_debug_output(buf);
    } else {
        return;
    }
    if (m_debug & DEBUG_FLAG_VERBOSE) {
#ifdef ADAFRUIT_GRAND_CENTRAL_M4
        uint32_t dataa = PORT->Group[PGA].IN.reg;
        uint32_t datab = PORT->Group[PGB].IN.reg;
        uint32_t datac = PORT->Group[PGC].IN.reg;
        uint32_t datad = PORT->Group[PGD].IN.reg;
        sprintf(buf, "IN    PA: %08X  PB: %08X  PC: %08X  PD: %08X",
                dataa, datab, datac, datad);
        m_debug_output(buf);
        uint32_t outa = PORT->Group[PGA].OUT.reg;
        uint32_t outb = PORT->Group[PGB].OUT.reg;
        uint32_t outc = PORT->Group[PGC].OUT.reg;
        uint32_t outd = PORT->Group[PGD].OUT.reg;
        sprintf(buf, "OUT   PA: %08X  PB: %08X  PC: %08X  PD: %08X",
                outa, outb, outc, outd);
        m_debug_output(buf);
        uint32_t dira = PORT->Group[PGA].DIR.reg;
        uint32_t dirb = PORT->Group[PGB].DIR.reg;
        uint32_t dirc = PORT->Group[PGC].DIR.reg;
        uint32_t dird = PORT->Group[PGD].DIR.reg;
        sprintf(buf, "DIR   PA: %08X  PB: %08X  PC: %08X  PD: %08X",
                dira, dirb, dirc, dird);
        m_debug_output(buf);
#endif  // ADAFRUIT_GRAND_CENTRAL_M4
    }
}

/*
 * Run the processor.
 *
 * This will step the processor by a single clock-tick.
 */
void Z80RetroShieldClassName::Tick(int cycles)
{
    for (int cycle = 0; cycle < cycles; cycle++) {

        /*
         * The memory address we're reading/writing to.
         */
        static unsigned int  uP_ADDR = 0;

        /*
         * The I/O address we're reading/writing to.
         */
        static uint8_t prevIORQ = 0;

        // CLK goes high
        CLK_HIGH();

        //////////////////////////////////////////////////////////////////////
        // Memory Access?
        if (!STATE_MREQ_N())
            {
                // Store the contents of the address-bus in case we're going to use it.
                uP_ADDR = ADDR();

                // RAM Read?
                if (!STATE_RD_N())
                    {
                        // change DATA port to output to uP:
                        DATA_DIR(DIR_OUT);

                        if (m_on_memory_read)
                            DATA_OUT(m_on_memory_read(uP_ADDR));
                        else
                            DATA_OUT(0);
                        debug_show_status("MEMR: ");
                    }
                else if (!STATE_WR_N())
                    {
                        debug_show_status("MEMW: ");
                        // RAM write
                        if (m_on_memory_write != NULL)
                            m_on_memory_write(uP_ADDR, DATA_IN());
                    }

                goto tick_tock;
            }

        //////////////////////////////////////////////////////////////////////
        // IO Access?
        if (!STATE_IORQ_N())
            {
                // IO Read?
                if (!STATE_RD_N() && prevIORQ)
                    {
                        // change DATA port to output to uP:
                        DATA_DIR(DIR_OUT);

                        // output data at this cycle too
                        if (m_on_io_read)
                            DATA_OUT(m_on_io_read(ADDR_L()));
                        else
                            DATA_OUT(0);
                        debug_show_status("IOR : ");
                    }

                // IO Write?
                if (!STATE_WR_N() && prevIORQ)
                    {
                        debug_show_status("IOW : ");
                        if (m_on_io_write != NULL)
                            m_on_io_write(ADDR_L(), DATA_IN());
                    }

                goto tick_tock;
            }
        debug_show_status("    : ");

    tick_tock:
        prevIORQ = STATE_IORQ_N();

        //////////////////////////////////////////////////////////////////////
        // start next cycle
        CLK_LOW();
        debug_count_cycle();

        // natural delay for DATA Hold time (t_HR)
        DATA_DIR(DIR_IN);

    }  // for (int cycle = 0; cycle < cycles; cycle++)

}

/*
 * Reset the processor.
 *
 * This will run the clock a few cycles to ensure that the processor
 * is reset fully.
 */
void Z80RetroShieldClassName::Reset()
{
    // Drive RESET conditions
    digitalWrite(uP_RESET_N, LOW);
    digitalWrite(uP_INT_N, HIGH);
    digitalWrite(uP_NMI_N, HIGH);

    // Run for a few cycles.
    for (int i = 0; i < 4; i++)
        Tick();

    // Drive RESET conditions
    digitalWrite(uP_RESET_N, HIGH);
}
