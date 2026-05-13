#include "stm32u5xx.h"
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>

/* ------------------------------------------------------------------ */
/* Pin number constants                                                 */
/* ------------------------------------------------------------------ */
#define PIN_Chip_En   15u   /* PF15 */
#define PIN_MISO       3u   /* PD3  */
#define PIN_MOSI       4u   /* PD4  */
#define PIN_CK         1u   /* PD1  */
#define PIN_CS        12u   /* PB12 */
#define PIN_FLOW      15u   /* PG15 */
#define PIN_NOTIFY    14u   /* PD14 */
#define PIN_TX1        9u   /* PA9  */
#define PIN_RX1       10u   /* PA10 */

/* ------------------------------------------------------------------ */
/* Timing                                                               */
/* Running at 16 MHz HSI, loop calibrated at ~8 cycles per iteration.  */
/* 2,000,000 iterations ~ 1 second.                                     */
/* ------------------------------------------------------------------ */
#define DELAY_MS(ms)      delay_ms(ms)					
#define DELAY_SEC(s)      delay_ms((s) * 1000u) 

/*
 * Timeout counters are separate from the delay loop.
 * TIMEOUT_MS(n) gives a count suitable for a tight polling loop.
 * Calibrated so that 125 counts ~ 1 ms at 16 MHz.
 */
#define TIMEOUT_MS(ms)    ((uint32_t)(ms)  * 125UL)
#define TIMEOUT_SEC(s)    ((uint32_t)(s)   * 125000UL)

/* EMW3080 SPI header type bytes */
#define SPI_TYPE_WRITE   0x0Au
#define SPI_TYPE_READ    0x0Bu

/* Maximum payload we handle in one transaction */
#define EMW_MAX_PAYLOAD  768u

#define MIPC_HDR_SIZE       6u      /* req_id(4) + api_id(2) */
#define MIPC_MAX_PAYLOAD    250u    /* keep well under EMW_MAX_PAYLOAD */

/* Command IDs from mx_wifi_ipc.h */
#define MIPC_API_SYS_VERSION_CMD    0x0003u
#define MIPC_API_SYS_ECHO_CMD       0x0001u

#define MIPC_API_WIFI_CONNECT_CMD   0x0103u
#define MIPC_API_WIFI_GET_IP_CMD    0x0107u

static uint32_t g_req_id = 1u;     /* increments with each request */

volatile uint32_t g_rcc_test = 0;
volatile uint32_t g_rcc_after_all = 0;

/* ------------------------------------------------------------------ */
/* Delay                                                                */
/* ------------------------------------------------------------------ */
void delay(volatile uint32_t d)
{
    while (d--);
}

void delay_ms(uint32_t ms)
{
    /* Enable DWT if not already enabled */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

    uint32_t cycles = (SystemCoreClock / 1000u) * ms;
    uint32_t start  = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < cycles);
}

//#define DELAY_MS(ms)   delay_ms(ms)
//#define DELAY_SEC(s)   delay_ms((s) * 1000u)

/* ------------------------------------------------------------------ */
/* Clock enable                                                         */
/* ------------------------------------------------------------------ */
void Enable_Clocks(void)
{
    /* Step 1: Enable PWR clock on AHB3 */
    RCC->AHB3ENR |= RCC_AHB3ENR_PWREN;
    (void)RCC->AHB3ENR;

    /* Step 2: VDDIO2 enable sequence per RM0456
     * Required before using any PG[15:2] pin.
     *
     * a) Enable the IO2 voltage monitor page 414 */
    PWR->SVMCR |= PWR_SVMCR_IO2VMEN;
    (void)PWR->SVMCR;

    /* b) Wait for IO2CVM wake-up time (~20us at 16MHz = ~320 cycles) */
    delay(1000u);

    /* c) Wait until VDDIO2RDY is set in PWR_SVMSR */
    while (!(PWR->SVMSR & PWR_SVMSR_VDDIO2RDY));

    /* d) Disable IO2VM to save power (optional but recommended) */
    PWR->SVMCR &= ~PWR_SVMCR_IO2VMEN;

    /* Step 3: Remove VDDIO2 power isolation */
    PWR->SVMCR |= PWR_SVMCR_IO2SV;
    (void)PWR->SVMCR;

    /* Step 4: Enable GPIO clocks - GPIOG will now accept the enable */
    RCC->AHB2ENR1 |= RCC_AHB2ENR1_GPIOAEN
                   | RCC_AHB2ENR1_GPIOBEN
                   | RCC_AHB2ENR1_GPIODEN
                   | RCC_AHB2ENR1_GPIOFEN
                   | RCC_AHB2ENR1_GPIOGEN;
    (void)RCC->AHB2ENR1;

    (void)GPIOA->IDR;
    (void)GPIOB->IDR;
    (void)GPIOD->IDR;
    (void)GPIOF->IDR;
    (void)GPIOG->IDR;

    /* Step 5: Peripheral clocks */
    RCC->APB1ENR1 |= RCC_APB1ENR1_SPI2EN;
    RCC->APB2ENR  |= RCC_APB2ENR_USART1EN;
    (void)RCC->APB1ENR1;
    (void)RCC->APB2ENR;
}

/* ------------------------------------------------------------------ */
/* GPIO configuration                                                   */
/* ------------------------------------------------------------------ */
void Configure_Pins(void)
{
    /* ---- GPIOD: SPI2 AF pins (PD1=CK, PD3=MISO, PD4=MOSI) ---- */
    GPIOD->MODER &= ~( (0x3UL << (2u * PIN_CK))
                     | (0x3UL << (2u * PIN_MISO))
                     | (0x3UL << (2u * PIN_MOSI)) );
    GPIOD->MODER |=  ( (0x2UL << (2u * PIN_CK))    /* AF */
                     | (0x2UL << (2u * PIN_MISO))
                     | (0x2UL << (2u * PIN_MOSI)) );

    /* AF5 = SPI2 on these pins */
    GPIOD->AFR[0] &= ~( (0xFUL << (4u * PIN_CK))
                      | (0xFUL << (4u * PIN_MISO))
                      | (0xFUL << (4u * PIN_MOSI)) );
    GPIOD->AFR[0] |=  ( (5UL << (4u * PIN_CK))
                      | (5UL << (4u * PIN_MISO))
                      | (5UL << (4u * PIN_MOSI)) );

    /* Push-pull, no pull on SPI lines */
    GPIOD->OTYPER &= ~( (1UL << PIN_CK)
                      | (1UL << PIN_MISO)
                      | (1UL << PIN_MOSI) );
    GPIOD->PUPDR  &= ~( (0x3UL << (2u * PIN_CK))
                      | (0x3UL << (2u * PIN_MISO))
                      | (0x3UL << (2u * PIN_MOSI)) );

    /* ---- PD14: NOTIFY - Input, pull-down ---- */
    GPIOD->MODER &= ~(0x3UL << (2u * PIN_NOTIFY));   /* 00 = input */
    GPIOD->PUPDR &= ~(0x3UL << (2u * PIN_NOTIFY));
    GPIOD->PUPDR |=  (0x2UL << (2u * PIN_NOTIFY));   /* pull-down  */

    /* ---- PF15: Chip_En - Output push-pull ---- */
    GPIOF->MODER &= ~(0x3UL << (2u * PIN_Chip_En));
    GPIOF->MODER |=  (0x1UL << (2u * PIN_Chip_En));  /* output */
    GPIOF->OTYPER &= ~(1UL << PIN_Chip_En);           /* push-pull */
    GPIOF->PUPDR  &= ~(0x3UL << (2u * PIN_Chip_En)); /* no pull */

    /* ---- PB12: CS - Output push-pull, idle HIGH ---- */
    GPIOB->MODER &= ~(0x3UL << (2u * PIN_CS));
    GPIOB->MODER |=  (0x1UL << (2u * PIN_CS));
    GPIOB->OTYPER &= ~(1UL << PIN_CS);
    GPIOB->PUPDR  &= ~(0x3UL << (2u * PIN_CS));
    GPIOB->BSRR   =  (1UL << PIN_CS);                /* CS idle HIGH */

    /* ---- PG15: FLOW - Input, pull-down ---- */
    /*
     * FLOW is driven by the module. When the module is unpowered or
     * resetting the line floats; pull-down keeps it LOW (busy) so we
     * never accidentally think the module is ready.
     */
    GPIOG->MODER &= ~(0x3UL << (2u * PIN_FLOW));     /* input */
    GPIOG->PUPDR &= ~(0x3UL << (2u * PIN_FLOW));
    //GPIOG->PUPDR |=  (0x2UL << (2u * PIN_FLOW));     /* pull-down */

		/* Read back immediately */
		volatile uint32_t moder_check = GPIOG->MODER;
		volatile uint32_t idr_check   = GPIOG->IDR;
		(void)moder_check;
		(void)idr_check;
}

/* ------------------------------------------------------------------ */
/* USART1 - debug output                                                */
/* PA9=TX AF7, PA10=RX AF7, 115200 baud @ 16 MHz HSI                   */
/* ------------------------------------------------------------------ */
void USART1_Init(void)
{
    /* PA9, PA10 -> AF mode */
    GPIOA->MODER &= ~( (0x3UL << (2u * PIN_TX1))
                     | (0x3UL << (2u * PIN_RX1)) );
    GPIOA->MODER |=  ( (0x2UL << (2u * PIN_TX1))
                     | (0x2UL << (2u * PIN_RX1)) );

    /* AF7 for USART1 - pins 9 and 10 are in AFR[1] (high register) */
    GPIOA->AFR[1] &= ~( (0xFUL << (4u * (PIN_TX1 % 8u)))
                      | (0xFUL << (4u * (PIN_RX1 % 8u))) );
    GPIOA->AFR[1] |=  ( (7UL   << (4u * (PIN_TX1 % 8u)))
                      | (7UL   << (4u * (PIN_RX1 % 8u))) );

    /* Clock source: select HSI16 for USART1 (CCIPR1 bits [1:0] = 0b10) */
    RCC->CCIPR1 &= ~(3UL << 0u);
    RCC->CCIPR1 |=  (2UL << 0u);

    /* Make sure HSI16 is on */
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY));

    /* Configure USART1 */
    USART1->CR1 &= ~USART_CR1_UE;   /* disable before config */
    USART1->BRR  = 139u;            /* 16 000 000 / 115200 = 138.9 -> 139 */
    USART1->CR1 |= USART_CR1_TE
                 | USART_CR1_RE
                 | USART_CR1_UE;
}

void USART1_Print(const char *str)
{
    while (*str) {
        while (!(USART1->ISR & USART_ISR_TXE_TXFNF));
        USART1->TDR = (uint8_t)(*str++);
    }
    /* Wait for last byte to finish transmitting */
    while (!(USART1->ISR & USART_ISR_TC));
}

/* ------------------------------------------------------------------ */
/* SPI2 initialisation                                                  */
/*                                                                      */
/* STM32U5 SPI (RM0456) key differences from older STM32:              */
/*   - CR2.TSIZE must equal the number of bytes in the burst.           */
/*     TSIZE=0 means "indefinite" mode - only use that for full-duplex  */
/*     streams, NOT for the framed protocol used here.                   */
/*   - Set SPE AFTER all configuration registers.                        */
/*   - Set CSTART AFTER SPE, right before clocking bytes.               */
/*   - SSI=1 in CR1 must be set BEFORE MASTER bit in CFG2.              */
/* ------------------------------------------------------------------ */
void SPI2_Init(void)
{
    /* Reset peripheral cleanly */
    RCC->APB1RSTR1 |=  RCC_APB1RSTR1_SPI2RST;
    delay(100u);
    RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_SPI2RST;
    delay(100u);

    /* Clear all config registers before writing */
    SPI2->CR1  = 0u;
    SPI2->CR2  = 0u;
    SPI2->CFG1 = 0u;
    SPI2->CFG2 = 0u;
    SPI2->IFCR = 0xFFFFFFFFu;   /* clear all flags */

    /*
     * CR1: Set SSI=1 first.
     * SSI must be HIGH before the MASTER bit is set in CFG2 to prevent
     * a spurious MODF (mode fault) error.
     */
    SPI2->CR1 = SPI_CR1_SSI;

    /*
     * CFG2: Master mode, software NSS (SSM=1), CPOL=0, CPHA=0 (Mode 0)
     * MSB first is the default (LSBFRST=0).
     */
    SPI2->CFG2 = SPI_CFG2_MASTER | SPI_CFG2_SSM;

    /*
     * CFG1:
     *   DSIZE [4:0] = 7  -> 8-bit data frames (value = frame_size - 1)
     *   MBR   [5:3] = 3  -> clock prescaler /16
     *                        APB1 = 16 MHz -> SPI clock = 1 MHz
     *   FTHLV [8:5] = 0  -> FIFO threshold = 1 data frame (default)
     */
    SPI2->CFG1 = (7UL  << SPI_CFG1_DSIZE_Pos)
               | (3UL  << SPI_CFG1_MBR_Pos);

    /*
     * CR2.TSIZE is NOT set here globally. It must be set to the exact
     * byte count immediately before each CSTART call. See SPI2_Transfer().
     */

    /* Enable SPI - must come after all configuration */
    SPI2->CR1 |= SPI_CR1_SPE;

    /* Print register state for verification */
    char buf[96];
    sprintf(buf, "SPI2 CFG1=0x%08X CFG2=0x%08X CR1=0x%08X SR=0x%08X\r\n",
            (unsigned)SPI2->CFG1, (unsigned)SPI2->CFG2,
            (unsigned)SPI2->CR1,  (unsigned)SPI2->SR);
    USART1_Print(buf);
}

/* ------------------------------------------------------------------ */
/* CS helpers                                                           */
/* ------------------------------------------------------------------ */
static inline void CS_Low(void)  { GPIOB->BRR  = (1UL << PIN_CS); }
static inline void CS_High(void) { GPIOB->BSRR = (1UL << PIN_CS); }

/* ------------------------------------------------------------------ */
/* Handshake signal readers                                             */
/* ------------------------------------------------------------------ */
static inline uint8_t FLOW_IsReady(void)
{
    return (GPIOG->IDR & (1UL << PIN_FLOW)) ? 1u : 0u;
}

static inline uint8_t NOTIFY_HasData(void)
{
    return (GPIOD->IDR & (1UL << PIN_NOTIFY)) ? 1u : 0u;
}

/* ------------------------------------------------------------------ */
/* SPI2_Transfer - transfer exactly `len` bytes full-duplex            */
/*                                                                      */
/* This is the corrected core transfer function.                        */
/* Key fixes vs the original:                                           */
/*   1. CR2.TSIZE is set to `len` before CSTART so the SPI peripheral  */
/*      knows when the burst ends and asserts EOT correctly.            */
/*   2. CSTART is set once per burst, not once per byte.                */
/*   3. After the burst, we wait for SR.EOT and clear it via IFCR.EOTC */
/*      before returning, so the next burst starts from a clean state.  */
/*   4. TX and RX FIFOs are drained byte-by-byte inside the loop using  */
/*      the TXP (TX FIFO has space) and RXP (RX FIFO has data) flags.  */
/* ------------------------------------------------------------------ */
static int SPI2_Transfer(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    if (len == 0u) return 0;

    uint32_t timeout;

    /* Set exact transfer size - CRITICAL for STM32U5 SPI */
    SPI2->CR2 = (uint32_t)len;

    /* Trigger the burst */
    SPI2->CR1 |= SPI_CR1_CSTART;

    uint16_t tx_count = 0u;
    uint16_t rx_count = 0u;

    while (rx_count < len)
    {
        /* Push a byte into TX FIFO if there is space and we have bytes left */
        if (tx_count < len)
        {
            timeout = 100000u;
            while (!(SPI2->SR & SPI_SR_TXP))
            {
                if (--timeout == 0u)
                {
                    USART1_Print("SPI TX FIFO timeout\r\n");
                    CS_High();
                    return -1;
                }
            }
            /* 8-bit access to TXDR - must use byte pointer to avoid
             * accidentally writing 16 or 32 bits into the FIFO    */
            *(volatile uint8_t *)&SPI2->TXDR = tx ? tx[tx_count] : 0x00u;
            tx_count++;
        }

        /* Read a byte from RX FIFO when available */
        timeout = 100000u;
        while (!(SPI2->SR & SPI_SR_RXP))
        {
            if (--timeout == 0u)
            {
                USART1_Print("SPI RX FIFO timeout\r\n");
                CS_High();
                return -1;
            }
        }
        uint8_t b = *(volatile uint8_t *)&SPI2->RXDR;
        if (rx) rx[rx_count] = b;
        rx_count++;
    }

    /* Wait for end-of-transfer flag - burst is fully complete */
    timeout = 200000u;
    while (!(SPI2->SR & SPI_SR_EOT))
    {
        if (--timeout == 0u)
        {
            USART1_Print("SPI EOT timeout\r\n");
            break;
        }
    }

    /* Clear EOT and TXC flags so the next burst starts cleanly */
    SPI2->IFCR = SPI_IFCR_EOTC | SPI_IFCR_TXTFC;

    return 0;
}

/* ------------------------------------------------------------------ */
/* wait_flow_high                                                        */
/*                                                                      */
/* Wait up to `timeout_counts` for FLOW to go HIGH.                    */
/* Returns 0 on success, -1 on timeout.                                 */
/*                                                                      */
/* FIX: original code used (timeout--) as the loop condition which      */
/* wraps around to 0xFFFFFFFF when it hits zero, so the timeout check   */
/* `if (timeout == 0)` never triggered. We now decrement inside the     */
/* loop body and check for zero explicitly.                              */
/* ------------------------------------------------------------------ */

static int wait_flow_high(uint32_t timeout_counts)
{
    uint32_t waited = 0u;
    while (!FLOW_IsReady())
    {
        if (timeout_counts == 0u)
        {
            char dbg[48];
            sprintf(dbg, "FLOW timeout after %u counts GPIOG IDR=0x%08X\r\n",
                    waited, (unsigned)GPIOG->IDR);
            USART1_Print(dbg);
            return -1;
        }
        timeout_counts--;
        waited++;
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/* EMW_Transaction                                                       */
/*                                                                      */
/* Single complete SPI transaction with the EMW3080.                    */
/* Implements the full handshake protocol from mx_wifi_spi.c:           */
/*                                                                      */
/*   1. CS LOW                                                           */
/*   2. wait FLOW HIGH          (module ready for header)               */
/*   3. exchange 8-byte header                                           */
/*   4. validate slave header                                            */
/*   5. wait FLOW HIGH          (module ready for data)                 */
/*   6. exchange max(tx_len, slave_len) bytes                           */
/*   7. CS HIGH                                                         */
/*                                                                      */
/* tx_data / tx_len : bytes to send  (NULL / 0 for a pure read)        */
/* rx_buf / rx_buf_size : buffer to receive into                        */
/* Returns number of bytes received, or -1 on error.                   */
/* ------------------------------------------------------------------ */
int EMW_Transaction(const uint8_t *tx_data, uint16_t tx_len,
                    uint8_t *rx_buf,  uint16_t rx_buf_size)
{
    char dbg[80];

    /* ---- Build master header ---- */
    uint16_t tx_lenx = (uint16_t)(~tx_len);

    uint8_t m_hdr[8] = {
        SPI_TYPE_WRITE,
        (uint8_t)(tx_len  & 0xFFu),
        (uint8_t)(tx_len  >> 8u),
        (uint8_t)(tx_lenx & 0xFFu),
        (uint8_t)(tx_lenx >> 8u),
        0x00u, 0x00u, 0x00u          /* dummy bytes */
    };
    uint8_t s_hdr[8] = {0};

    /* ---- Step 1: CS LOW immediately ---- */
		CS_High();
DELAY_MS(10u);
CS_Low();

		/* ---- Step 2: wait FLOW HIGH (header phase) ----
		* On this firmware FLOW pulses HIGH briefly to signal readiness.
		* CS must already be LOW to catch it. Use a longer timeout since
		* the module may need a moment after seeing CS fall.           */
		USART1_Print("EMW_T: waiting FLOW header\r\n");
		if (wait_flow_high(TIMEOUT_MS(3000u)) != 0)
    {
        USART1_Print("Tx: FLOW timeout (header)\r\n");
        CS_High();
        return -1;
    }

    /* ---- Step 3: exchange 8-byte header ---- */
    if (SPI2_Transfer(m_hdr, s_hdr, 8u) != 0)
    {
        CS_High();
        return -1;
    }

    sprintf(dbg, "TX HDR: %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
            m_hdr[0], m_hdr[1], m_hdr[2], m_hdr[3],
            m_hdr[4], m_hdr[5], m_hdr[6], m_hdr[7]);
    USART1_Print(dbg);
    sprintf(dbg, "RX HDR: %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
            s_hdr[0], s_hdr[1], s_hdr[2], s_hdr[3],
            s_hdr[4], s_hdr[5], s_hdr[6], s_hdr[7]);
    USART1_Print(dbg);

    /* ---- Step 4: validate slave header ---- */
    if (s_hdr[0] != SPI_TYPE_READ)
    {
        sprintf(dbg, "Bad type: 0x%02X\r\n", s_hdr[0]);
        USART1_Print(dbg);
        CS_High();
        return -1;
    }

    uint16_t s_len  = (uint16_t)(s_hdr[1] | ((uint16_t)s_hdr[2] << 8u));
    uint16_t s_lenx = (uint16_t)(s_hdr[3] | ((uint16_t)s_hdr[4] << 8u));

    if ((s_len ^ s_lenx) != 0xFFFFu)
    {
        sprintf(dbg, "HDR integrity fail: len=0x%04X lenx=0x%04X\r\n",
                s_len, s_lenx);
        USART1_Print(dbg);
        CS_High();
        return -1;
    }

    sprintf(dbg, "Slave wants %u bytes, master sends %u bytes\r\n",
            s_len, tx_len);
    USART1_Print(dbg);

    /* Both zero - nothing to do */
    if (s_len == 0u && tx_len == 0u)
    {
        USART1_Print("Both lengths zero - nothing to transfer\r\n");
        CS_High();
        return 0;
    }

    /* Data length = max of master and slave lengths */
		uint16_t datalen = (tx_len > s_len) ? tx_len : s_len;

    if (datalen > EMW_MAX_PAYLOAD)
    {
        sprintf(dbg, "datalen %u exceeds buffer\r\n", datalen);
        USART1_Print(dbg);
        CS_High();
        return -1;
    }

    /* ---- Step 5: wait FLOW HIGH (data phase) ---- */
    /*
     * FIX: original code fell through to the data phase even when
     * FLOW was still LOW because the timeout underflowed. This second
     * FLOW wait is mandatory per the EMW3080 protocol - the module
     * pulses FLOW between the header and data phases.
     */
    USART1_Print("EMW_T: waiting FLOW data\r\n");
		if (wait_flow_high(TIMEOUT_MS(3000u)) != 0)
    {
        USART1_Print("Tx: FLOW timeout (data)\r\n");
        CS_High();
        return -1;
    }

    sprintf(dbg, "FLOW high before data phase (datalen=%u)\r\n", datalen);
    USART1_Print(dbg);

    /* ---- Step 6: exchange data ---- */
    /*
     * Build a zero-padded TX buffer of exactly `datalen` bytes.
     * Bytes beyond tx_len are padded with 0x00.
     * The RX buffer receives whatever the module sends.
     */
    static uint8_t tx_buf[EMW_MAX_PAYLOAD];
    static uint8_t rx_payload[EMW_MAX_PAYLOAD];

    memset(tx_buf,    0x00u, datalen);
    memset(rx_payload, 0x00u, datalen);

    if (tx_data && tx_len > 0u)
    {
        memcpy(tx_buf, tx_data, tx_len);
    }

    if (SPI2_Transfer(tx_buf, rx_payload, datalen) != 0)
    {
        CS_High();
        return -1;
    }

    /* ---- Step 7: CS HIGH ---- */
    CS_High();

    /* ---- Debug: print sent bytes 
    USART1_Print("Sent: [");
    for (uint16_t i = 0u; i < tx_len; i++)
    {
        char c[2] = { (tx_buf[i] >= 0x20u && tx_buf[i] < 0x7Fu)
                      ? (char)tx_buf[i] : '.', '\0' };
        USART1_Print(c);
    }
    USART1_Print("]\r\n");
		---- */
		
    /* ---- Debug: print raw RX bytes ---- */
    USART1_Print("RX raw: [");
for (uint16_t i = 0u; i < datalen; i++)
{
    if (rx_payload[i] != 0x00u)
    {
        char tmp[8];
        sprintf(tmp, "[%u]%02X ", i, rx_payload[i]);
        USART1_Print(tmp);
    }
}
USART1_Print("]\r\n");

    /* Copy received data - check both s_len and actual content */
int rx_count = 0;
if (rx_buf && rx_buf_size > 0u)
    {
        if (s_len > 0u)
        {
            uint16_t copy_len = (s_len < rx_buf_size) ? s_len : rx_buf_size;
            memcpy(rx_buf, rx_payload, copy_len);
            rx_count = (int)copy_len;
        }
    }
    return rx_count;
}

/* ------------------------------------------------------------------ */
/* EMW_SendAT                                                           */
/* Send a raw AT command string and print the response.                 */
/* ------------------------------------------------------------------ */

/* Build a minimal MIPC frame into buf[], return total length */
static uint16_t MIPC_BuildFrame(uint8_t *buf, uint16_t buf_size,
                                 uint16_t api_id,
                                 const uint8_t *params, uint16_t params_len)
{
    uint16_t total = (uint16_t)(MIPC_HDR_SIZE + params_len);
    if (total > buf_size) return 0u;

    /* req_id - little endian uint32_t */
    buf[0] = (uint8_t)(g_req_id        & 0xFFu);
    buf[1] = (uint8_t)((g_req_id >> 8) & 0xFFu);
    buf[2] = (uint8_t)((g_req_id >>16) & 0xFFu);
    buf[3] = (uint8_t)((g_req_id >>24) & 0xFFu);
    g_req_id++;

    /* api_id - little endian uint16_t */
    buf[4] = (uint8_t)(api_id       & 0xFFu);
    buf[5] = (uint8_t)((api_id >> 8) & 0xFFu);

    /* params */
    if (params && params_len > 0u)
    {
        memcpy(&buf[6], params, params_len);
    }

    return total;
}

/* Parse and print a received MIPC frame */
static void MIPC_PrintResponse(const uint8_t *data, uint16_t len)
{
    char buf[96];

    if (len < MIPC_HDR_SIZE)
    {
        USART1_Print("Response too short\r\n");
        return;
    }

    uint32_t resp_req_id = (uint32_t)(data[0])
                         | ((uint32_t)data[1] << 8u)
                         | ((uint32_t)data[2] << 16u)
                         | ((uint32_t)data[3] << 24u);

    uint16_t resp_api_id = (uint16_t)(data[4])
                         | ((uint16_t)data[5] << 8u);

    sprintf(buf, "MIPC resp: req_id=0x%08X api_id=0x%04X payload_len=%u\r\n",
            (unsigned)resp_req_id, (unsigned)resp_api_id,
            (unsigned)(len - MIPC_HDR_SIZE));
    USART1_Print(buf);

    /* Print payload as ASCII where printable, hex otherwise */
    if (len > MIPC_HDR_SIZE)
    {
        USART1_Print("Payload: [");
        for (uint16_t i = MIPC_HDR_SIZE; i < len; i++)
        {
            char c[2] = { (data[i] >= 0x20u && data[i] < 0x7Fu)
                          ? (char)data[i] : '.', '\0' };
            USART1_Print(c);
        }
        USART1_Print("]\r\n");
    }
}

/* ------------------------------------------------------------------ */
/* EMW_SendMIPC                                                          */
/* Send one MIPC command and wait for the response via NOTIFY.          */
/* ------------------------------------------------------------------ */

/* Drain any pending response the module has queued via NOTIFY */
static void EMW_DrainPending(void)
{
    if (!NOTIFY_HasData()) return;

    USART1_Print("Draining stale response...\r\n");
    static uint8_t drain_buf[EMW_MAX_PAYLOAD];
    int rx = EMW_Transaction(NULL, 0u, drain_buf, sizeof(drain_buf));
    if (rx > 0)
    {
        MIPC_PrintResponse(drain_buf, (uint16_t)rx);
    }
}

static void EMW_SendMIPC(uint16_t api_id,
                          const uint8_t *params, uint16_t params_len)
{
    char dbg[64];
    static uint8_t frame[EMW_MAX_PAYLOAD];
    static uint8_t response[EMW_MAX_PAYLOAD];

    /* Drain any queued response from previous command first */
    EMW_DrainPending();

    uint16_t frame_len = MIPC_BuildFrame(frame, sizeof(frame),
                                          api_id, params, params_len);
    if (frame_len == 0u)
    {
        USART1_Print("Frame build failed\r\n");
        return;
    }

    sprintf(dbg, "Sending MIPC api_id=0x%04X len=%u\r\n", api_id, frame_len);
    USART1_Print(dbg);

    /* Send the frame - expect s_len=0 since response is always deferred */
    EMW_Transaction(frame, frame_len, response, sizeof(response));

    /* Always wait for NOTIFY - response is never truly inline */
    USART1_Print("Waiting for NOTIFY...\r\n");
    uint32_t to = 3000u;
    while (!NOTIFY_HasData())
    {
        if (to == 0u)
        {
            USART1_Print("NOTIFY timeout\r\n");
            return;
        }
        DELAY_MS(1u);
        to--;
    }

    USART1_Print("NOTIFY high - reading response\r\n");

    /* Now drain the actual response */
    memset(response, 0, sizeof(response));
    int rx = EMW_Transaction(NULL, 0u, response, sizeof(response));

    if (rx > 0)
    {
        MIPC_PrintResponse(response, (uint16_t)rx);
    }
    else
    {
        USART1_Print("Read returned no data\r\n");
    }
}

#pragma pack(1)
typedef struct {
    int8_t  ssid[33];
    int8_t  key[65];
    int32_t key_len;
    uint8_t use_attr;
    uint8_t use_ip;
} wifi_connect_min_t;
#pragma pack()

static int EMW_WiFiConnect(const char *ssid, const char *password)
{
    char dbg[96];
    wifi_connect_min_t params;
    memset(&params, 0, sizeof(params));

    strncpy((char *)params.ssid, ssid,     32u);
    strncpy((char *)params.key,  password, 64u);
    params.key_len  = (int32_t)strlen(password);
    params.use_attr = 0u;
    params.use_ip   = 0u;

    /* Verify what we are actually sending */
    sprintf(dbg, "struct size=%u\r\n", (unsigned)sizeof(wifi_connect_min_t));
    USART1_Print(dbg);
    sprintf(dbg, "SSID: [%s]\r\n", (char *)params.ssid);
    USART1_Print(dbg);
    //sprintf(dbg, "KEY:  [%s]\r\n", (char *)params.key);
    //USART1_Print(dbg);
    sprintf(dbg, "key_len=%d use_attr=%u use_ip=%u\r\n",
            (int)params.key_len, params.use_attr, params.use_ip);
    USART1_Print(dbg);

    static uint8_t frame[EMW_MAX_PAYLOAD];
    uint16_t frame_len = MIPC_BuildFrame(frame, sizeof(frame),
                                          MIPC_API_WIFI_CONNECT_CMD,
                                          (const uint8_t *)&params,
                                          sizeof(params));
    if (frame_len == 0u)
    {
        USART1_Print("Connect frame build failed\r\n");
        return -1;
    }

    sprintf(dbg, "Connect frame len=%u\r\n", frame_len);
    USART1_Print(dbg);

    static uint8_t response[EMW_MAX_PAYLOAD];
    EMW_Transaction(frame, frame_len, response, sizeof(response));

    /* Wait up to 15 seconds for association */
    USART1_Print("Waiting for connect response (up to 15s)...\r\n");
    uint32_t to = 15000u;
    while (!NOTIFY_HasData())
    {
        if (to == 0u)
        {
            USART1_Print("WiFi connect timeout\r\n");
            return -1;
        }
        DELAY_MS(1u);
        to--;
    }

    sprintf(dbg, "NOTIFY after %u ms\r\n", (unsigned)(15000u - to));
    USART1_Print(dbg);

    USART1_Print("Reading connect response\r\n");
    memset(response, 0, sizeof(response));
    int rx = EMW_Transaction(NULL, 0u, response, sizeof(response));

    if (rx >= (int)(MIPC_HDR_SIZE + 4))
    {
        int32_t status = (int32_t)(response[MIPC_HDR_SIZE])
                       | ((int32_t)response[MIPC_HDR_SIZE + 1] << 8u)
                       | ((int32_t)response[MIPC_HDR_SIZE + 2] << 16u)
                       | ((int32_t)response[MIPC_HDR_SIZE + 3] << 24u);

				sprintf(dbg, "Connect status: %d %s\r\n",
						(int)status, status == 0 ? "OK" : "FAIL");
				USART1_Print(dbg);  
			
			USART1_Print("Waiting for DHCP event...\r\n");
uint32_t dhcp_to = 10000u;
while (!NOTIFY_HasData())
{
    if (dhcp_to == 0u)
    {
        USART1_Print("DHCP event timeout\r\n");
        break;
    }
    DELAY_MS(1u);
    dhcp_to--;
}
if (NOTIFY_HasData())
{
    USART1_Print("Draining DHCP event\r\n");
    static uint8_t dhcp_resp[EMW_MAX_PAYLOAD];
    EMW_Transaction(NULL, 0u, dhcp_resp, sizeof(dhcp_resp));
}

			return (int)status;
    }

    USART1_Print("Connect response too short\r\n");
    return -1;
}

static int EMW_GetIP(void)
{
    int32_t ip_status = -1;
    USART1_Print("Getting IP...\r\n");
    static uint8_t frame[EMW_MAX_PAYLOAD];
    static uint8_t response[EMW_MAX_PAYLOAD];
    uint16_t frame_len = MIPC_BuildFrame(frame, sizeof(frame),
                                          MIPC_API_WIFI_GET_IP_CMD,
                                          NULL, 0u);
    EMW_Transaction(frame, frame_len, response, sizeof(response));
    uint32_t to = 5000u;
    while (!NOTIFY_HasData())
    {
        if (to == 0u)
        {
            USART1_Print("Get IP timeout\r\n");
            return -1;
        }
        DELAY_MS(1u);
        to--;
    }
    memset(response, 0, sizeof(response));
    int rx = EMW_Transaction(NULL, 0u, response, sizeof(response));
    if (rx > (int)MIPC_HDR_SIZE)
    {
        /* NO int32_t here - assign to existing variable */
        ip_status = (int32_t)(response[MIPC_HDR_SIZE])
                  | ((int32_t)response[MIPC_HDR_SIZE+1] << 8u)
                  | ((int32_t)response[MIPC_HDR_SIZE+2] << 16u)
                  | ((int32_t)response[MIPC_HDR_SIZE+3] << 24u);
        char sdbg[32];
        sprintf(sdbg, "GetIP status=%d\r\n", (int)ip_status);
        USART1_Print(sdbg);

        char ip_str[17]   = {0};
        char mask_str[17] = {0};
        char gw_str[17]   = {0};
        uint16_t base = MIPC_HDR_SIZE + 4u;
        if (rx >= (int)(base + 48u))
        {
            memcpy(ip_str,   &response[base],       16u);
            memcpy(mask_str, &response[base + 16u], 16u);
            memcpy(gw_str,   &response[base + 32u], 16u);
            char dbg[96];
            sprintf(dbg, "IP:   %s\r\n", ip_str);  USART1_Print(dbg);
            sprintf(dbg, "Mask: %s\r\n", mask_str); USART1_Print(dbg);
            sprintf(dbg, "GW:   %s\r\n", gw_str);   USART1_Print(dbg);
        }
        else
        {
            USART1_Print("IP response too short\r\n");
            MIPC_PrintResponse(response, (uint16_t)rx);
        }
    }
    return (int)ip_status;
}

/* ------------------------------------------------------------------ */
/* ThingSpeak configuration                                             */
/* ------------------------------------------------------------------ */
#define THINGSPEAK_HOST     "api.thingspeak.com"
#define THINGSPEAK_PORT     80
#define THINGSPEAK_API_KEY  "YOUR_API_KEY"
#define THINGSPEAK_CH_ID    YOUR_CHANNEL_ID

/* Socket command IDs from mx_wifi_ipc.h */
#define MIPC_API_SOCKET_CREATE_CMD      0x0201u
#define MIPC_API_SOCKET_CONNECT_CMD     0x0202u
#define MIPC_API_SOCKET_SEND_CMD        0x0203u
#define MIPC_API_SOCKET_CLOSE_CMD       0x0208u
#define MIPC_API_WIFI_PING_CMD  ((uint16_t)(0x0100 + 0x000b))

/* Socket domain/type/protocol constants (standard BSD values) */
#define MX_AF_INET      2
#define MX_SOCK_STREAM  1
#define MX_IPPROTO_TCP  6

static int EMW_SockTransaction(const uint8_t *tx, uint16_t tx_len,
                                uint8_t *rx, uint16_t rx_size)
{
    /* Longer CS settle time for socket commands */
    CS_High();
    DELAY_MS(10u);
    CS_Low();

    /* Wait longer for FLOW - socket commands take more time */
		if (wait_flow_high(TIMEOUT_MS(5000u)) != 0)
		{
			USART1_Print("SockTX: FLOW timeout (data)\r\n");
			CS_High();
			return -1;
		}

    /* Rest is identical to EMW_Transaction */
    uint16_t tx_lenx = (uint16_t)(~tx_len);
    uint8_t m_hdr[8] = {
        SPI_TYPE_WRITE,
        (uint8_t)(tx_len  & 0xFFu),
        (uint8_t)(tx_len  >> 8u),
        (uint8_t)(tx_lenx & 0xFFu),
        (uint8_t)(tx_lenx >> 8u),
        0x00u, 0x00u, 0x00u
    };
    uint8_t s_hdr[8] = {0};

    if (SPI2_Transfer(m_hdr, s_hdr, 8u) != 0)
    {
        CS_High();
        return -1;
    }
		
    if (s_hdr[0] != SPI_TYPE_READ)
    {
        char dbg[32];
        sprintf(dbg, "SockTX: bad type 0x%02X\r\n", s_hdr[0]);
        USART1_Print(dbg);
        CS_High();
        return -1;
    }

    uint16_t s_len  = (uint16_t)(s_hdr[1] | ((uint16_t)s_hdr[2] << 8u));
    uint16_t s_lenx = (uint16_t)(s_hdr[3] | ((uint16_t)s_hdr[4] << 8u));

    if ((s_len ^ s_lenx) != 0xFFFFu)
    {
        char dbg[64];
				sprintf(dbg, "SockTX: integrity fail s_len=%u s_lenx=%u hdr=%02X %02X %02X %02X %02X\r\n",
        s_len, s_lenx, s_hdr[0], s_hdr[1], s_hdr[2], s_hdr[3], s_hdr[4]);
				USART1_Print(dbg);
				CS_High();
        return -1;
    }

		
    if (s_len == 0u && tx_len == 0u)
    {
        CS_High();
        return 0;
    }

    uint16_t datalen = (tx_len > s_len) ? tx_len : s_len;

		if (wait_flow_high(TIMEOUT_MS(5000u)) != 0)
		{
			USART1_Print("SockTX: FLOW timeout (data)\r\n");
			CS_High();
			return -1;
		}

    static uint8_t tx_buf[EMW_MAX_PAYLOAD];
    static uint8_t rx_payload[EMW_MAX_PAYLOAD];

    memset(tx_buf,     0x00u, datalen);
    memset(rx_payload, 0x00u, datalen);

    if (tx && tx_len > 0u) memcpy(tx_buf, tx, tx_len);

		if (SPI2_Transfer(tx_buf, rx_payload, datalen) != 0)
		{
				CS_High();
				return -1;
		}

    CS_High();

    int rx_count = 0;
    if (rx && rx_size > 0u && s_len > 0u)
    {
        uint16_t copy_len = (s_len < rx_size) ? s_len : rx_size;
        memcpy(rx, rx_payload, copy_len);
        rx_count = (int)copy_len;
    }

    return rx_count;
}

/* ------------------------------------------------------------------ */
/* Socket helper - send a command and wait for NOTIFY response          */
/* Returns number of rx bytes or -1 on error                           */
/* ------------------------------------------------------------------ */
static int EMW_SockCmd(uint16_t api_id,
                        const uint8_t *params, uint16_t params_len,
                        uint8_t *resp, uint16_t resp_size,
                        uint32_t timeout_ms)
{
		char dbg[48];
    sprintf(dbg, "SockCmd: api=0x%04X params_len=%u\r\n", api_id, params_len);
    USART1_Print(dbg);  /* ADD THIS */
		
    static uint8_t frame[EMW_MAX_PAYLOAD];

    uint16_t frame_len = MIPC_BuildFrame(frame, sizeof(frame),
                                          api_id, params, params_len);
		
		sprintf(dbg, "SockCmd: frame_len=%u\r\n", frame_len);
    USART1_Print(dbg);  /* ADD THIS */
		
    if (frame_len == 0u)
    {
        USART1_Print("SockCmd: frame build failed\r\n");
        return -1;
    }

		/* Drain any pending NOTIFY before starting new transaction */
    if (NOTIFY_HasData())
    {
        USART1_Print("SockCmd: draining before send\r\n");
        static uint8_t drain[EMW_MAX_PAYLOAD];
        EMW_Transaction(NULL, 0u, drain, sizeof(drain));
    }

		/* Print pin states before transaction */
    char dbg2[48];
    sprintf(dbg2, "Pre-TX FLOW:%u NOTIFY:%u CS should be HIGH\r\n",
            FLOW_IsReady(), NOTIFY_HasData());
    USART1_Print(dbg2);
		
    USART1_Print("SockCmd: calling EMW_Transaction\r\n");
		EMW_SockTransaction(frame, frame_len, resp, resp_size);
		
    USART1_Print("SockCmd: transaction done\r\n");  
		
    uint32_t to = timeout_ms;
		USART1_Print("SockCmd: waiting NOTIFY\r\n");  /* ADD */
    while (!NOTIFY_HasData())
    {
        if (to == 0u)
        {
            USART1_Print("SockCmd: NOTIFY timeout\r\n");
            return -1;
        }
        DELAY_MS(1u);
        to--;
    }
		USART1_Print("SockCmd: NOTIFY received\r\n");  /* ADD */

    memset(resp, 0, resp_size);
    return EMW_SockTransaction(NULL, 0u, resp, resp_size);
}

/* ------------------------------------------------------------------ */
/* Socket create                                                         */
/* Returns socket fd or -1 on error                                    */
/* ------------------------------------------------------------------ */
#pragma pack(1)
typedef struct {
    int32_t domain;
    int32_t type;
    int32_t protocol;
} sock_create_params_t;

typedef struct {
    int32_t fd;
} sock_create_resp_t;
#pragma pack()

static int EMW_SockCreate(void)
{
    USART1_Print("SockCreate: sending\r\n");
    static sock_create_params_t p;
    p.domain   = MX_AF_INET;
    p.type     = MX_SOCK_STREAM;
    p.protocol = MX_IPPROTO_TCP;

		/* Add this dump */
    USART1_Print("SockCreate params: ");
    uint8_t *raw = (uint8_t *)&p;
    for (int i = 0; i < (int)sizeof(p); i++)
    {
        char tmp[8];
        sprintf(tmp, "[%d]%02X ", i, raw[i]);
        USART1_Print(tmp);
    }
    USART1_Print("\r\n");
	
    static uint8_t resp[EMW_MAX_PAYLOAD];
    int rx = EMW_SockCmd(MIPC_API_SOCKET_CREATE_CMD,
                          (const uint8_t *)&p, sizeof(p),
                          resp, sizeof(resp), 5000u);

    if (rx < (int)(MIPC_HDR_SIZE + 4))
    {
        USART1_Print("SockCreate: short response\r\n");
        return -1;
    }

			/* Print all response bytes */
USART1_Print("SockCreate raw resp: ");
for (int i = 0; i < rx; i++)
{
    char tmp[8];
    sprintf(tmp, "[%d]%02X ", i, resp[i]);
    USART1_Print(tmp);
}
USART1_Print("\r\n");

/* Try reading fd from every possible offset */
for (int off = 0; off <= rx - 4; off++)
{
    int32_t fd_try = (int32_t)(resp[off])
                   | ((int32_t)resp[off+1] << 8)
                   | ((int32_t)resp[off+2] << 16)
                   | ((int32_t)resp[off+3] << 24);
    char tmp[32];
    sprintf(tmp, "fd@offset[%d]=%d\r\n", off, (int)fd_try);
    USART1_Print(tmp);
}

    int32_t fd = (int32_t)(resp[MIPC_HDR_SIZE])
               | ((int32_t)resp[MIPC_HDR_SIZE + 1] << 8u)
               | ((int32_t)resp[MIPC_HDR_SIZE + 2] << 16u)
               | ((int32_t)resp[MIPC_HDR_SIZE + 3] << 24u);

    char dbg[32];
    sprintf(dbg, "Socket fd=%d\r\n", (int)fd);
    USART1_Print(dbg);

    /* Check if fd=0 which may indicate error */
    if (fd == 0)
    {
        USART1_Print("SockCreate: WARNING fd=0 may be leaked socket\r\n");
    }

    return (int)fd;
}

/* ------------------------------------------------------------------ */
/* Socket connect                                                        */
/* ------------------------------------------------------------------ */

#pragma pack(1)
typedef struct {
    int32_t  socket;       /* fd                        - 4 bytes  */
    uint8_t  s2_len;       /* = 28 (sizeof storage)     - 1 byte   */
    uint8_t  ss_family;    /* = MX_AF_INET (2)          - 1 byte   */
    uint8_t  s2_data1[2];  /* port big-endian           - 2 bytes  */
    uint32_t s2_data2[3];  /* IP in [0] + padding       - 12 bytes */
    uint32_t s2_data3[3];  /* padding                   - 12 bytes */
    uint32_t length;       /* = 28  (mx_socklen_t)      - 4 bytes  */
} sock_connect_params_t;   /* Total params = 4+28+4 = 36 bytes     */
#pragma pack()

static int EMW_SockConnect(int fd, const char *ip, uint16_t port)
{
    static sock_connect_params_t p;
    memset(&p, 0, sizeof(p));

    p.socket = (int32_t)fd;

    /* Parse IP */
    unsigned a, b, c, d;
    if (sscanf(ip, "%u.%u.%u.%u", &a, &b, &c, &d) != 4)
    {
        USART1_Print("SockConnect: bad IP\r\n");
        return -1;
    }

    /* Fill mx_sockaddr_storage (28 bytes total) */
    p.s2_len      = 28u;                       /* sizeof mx_sockaddr_storage */
    p.ss_family   = (uint8_t)MX_AF_INET;       /* 2 */
    p.s2_data1[0] = (uint8_t)(port >> 8u);     /* port high byte */
    p.s2_data1[1] = (uint8_t)(port & 0xFFu);   /* port low byte  */

    /* IP address packed into first 4 bytes of s2_data2 */
    uint8_t *ip_bytes = (uint8_t *)&p.s2_data2[0];
    ip_bytes[0] = (uint8_t)a;
    ip_bytes[1] = (uint8_t)b;
    ip_bytes[2] = (uint8_t)c;
    ip_bytes[3] = (uint8_t)d;

    p.length = 28u;   /* mx_socklen_t = uint32_t */

    /* Debug print */
    char dbg[96];
    sprintf(dbg, "Connecting fd=%d to %u.%u.%u.%u port=%u\r\n",
            fd, a, b, c, d, port);
    USART1_Print(dbg);

    USART1_Print("Connect params raw: ");
    uint8_t *raw = (uint8_t *)&p;
    for (int i = 0; i < (int)sizeof(p); i++)
    {
        char tmp[8];
        sprintf(tmp, "[%d]%02X ", i, raw[i]);
        USART1_Print(tmp);
    }
    USART1_Print("\r\n");

    static uint8_t resp[EMW_MAX_PAYLOAD];
    int rx = EMW_SockCmd(MIPC_API_SOCKET_CONNECT_CMD,
                          (const uint8_t *)&p, sizeof(p),
                          resp, sizeof(resp), 30000u);

    char rxdbg[32];
    sprintf(rxdbg, "SockCmd rx=%d\r\n", rx);
    USART1_Print(rxdbg);

    USART1_Print("All resp bytes: ");
    for (int i = 0; i < rx; i++)
    {
        char tmp[12];
        sprintf(tmp, "[%d]%02X ", i, resp[i]);
        USART1_Print(tmp);
    }
    USART1_Print("\r\n");

    if (rx < (int)(MIPC_HDR_SIZE + 4))
    {
        USART1_Print("SockConnect: short response\r\n");
        return -1;
    }

    int32_t status = (int32_t)(resp[MIPC_HDR_SIZE])
                   | ((int32_t)resp[MIPC_HDR_SIZE + 1] << 8u)
                   | ((int32_t)resp[MIPC_HDR_SIZE + 2] << 16u)
                   | ((int32_t)resp[MIPC_HDR_SIZE + 3] << 24u);

    sprintf(dbg, "Connect status=%d\r\n", (int)status);
    USART1_Print(dbg);

    return (int)status;
}

/* ------------------------------------------------------------------ */
/* Socket send                                                           */
/* ------------------------------------------------------------------ */
#pragma pack(1)
typedef struct {
    int32_t  fd;
    uint32_t  size;
    int32_t  flags;
    uint8_t  data[EMW_MAX_PAYLOAD - 12u];
} sock_send_params_t;
#pragma pack()

static int EMW_SockSend(int fd, const char *data, uint16_t len)
{
		USART1_Print("SockSend: sending\r\n");  /* ADD THIS */
    static sock_send_params_t p;
    memset(&p, 0, sizeof(p));

    p.fd    = (int32_t)fd;
    p.size  = (int32_t)len;
    p.flags = 0;

    if (len > sizeof(p.data))
    {
        USART1_Print("SockSend: data too long\r\n");
        return -1;
    }
    memcpy(p.data, data, len);

    /* Total param size = fd(4) + size(4) + flags(4) + data(len) */
    uint16_t param_len = (uint16_t)(12u + len);

    static uint8_t resp[EMW_MAX_PAYLOAD];
    int rx = EMW_SockCmd(MIPC_API_SOCKET_SEND_CMD,
                          (const uint8_t *)&p, param_len,
                          resp, sizeof(resp), 10000u);

    if (rx < (int)(MIPC_HDR_SIZE + 4))
    {
        USART1_Print("SockSend: short response\r\n");
        return -1;
    }

    int32_t sent = (int32_t)(resp[MIPC_HDR_SIZE])
                 | ((int32_t)resp[MIPC_HDR_SIZE + 1] << 8u)
                 | ((int32_t)resp[MIPC_HDR_SIZE + 2] << 16u)
                 | ((int32_t)resp[MIPC_HDR_SIZE + 3] << 24u);

    char dbg[32];
    sprintf(dbg, "Sent %d bytes\r\n", (int)sent);
    USART1_Print(dbg);
    return (int)sent;
}

/* ------------------------------------------------------------------ */
/* Socket close                                                          */
/* ------------------------------------------------------------------ */
#pragma pack(1)
typedef struct {
    int32_t fd;
} sock_close_params_t;
#pragma pack()

static void EMW_SockClose(int fd)
{
    sock_close_params_t p;
    p.fd = (int32_t)fd;

    static uint8_t frame[EMW_MAX_PAYLOAD];
    uint16_t frame_len = MIPC_BuildFrame(frame, sizeof(frame),
                                          MIPC_API_SOCKET_CLOSE_CMD,
                                          (const uint8_t *)&p, sizeof(p));

    /* Just send the close frame, don't wait for NOTIFY response */
    EMW_SockTransaction(frame, frame_len, NULL, 0u);
    
    /* Give module a moment to process */
    DELAY_MS(200u);
    
    /* Drain whatever the module sends back */
    if (NOTIFY_HasData())
    {
        static uint8_t drain[EMW_MAX_PAYLOAD];
        EMW_SockTransaction(NULL, 0u, drain, sizeof(drain));
    }

    USART1_Print("Socket closed\r\n");
}

#define MIPC_API_SOCKET_GETHOSTBYNAME_CMD  ((uint16_t)(0x0200u + 0x0011u))

static int EMW_ResolveDNS(const char *hostname, char *ip_out)
{
    /* Matches socket_gethostbyname_cparams_t exactly */
    static uint8_t params[253];
    memset(params, 0, sizeof(params));
    strncpy((char *)params, hostname, 252);

    static uint8_t resp[EMW_MAX_PAYLOAD];
    int rx = EMW_SockCmd(MIPC_API_SOCKET_GETHOSTBYNAME_CMD,
                         params, sizeof(params),
                         resp, sizeof(resp), 10000u);

    /* Response: 6 header bytes + int32_t status + uint32_t s_addr */
    if (rx < (int)(MIPC_HDR_SIZE + 8))
    {
        USART1_Print("DNS: short response\r\n");
        return -1;
    }

    int32_t status = (int32_t)(resp[MIPC_HDR_SIZE])
                   | ((int32_t)resp[MIPC_HDR_SIZE+1] << 8)
                   | ((int32_t)resp[MIPC_HDR_SIZE+2] << 16)
                   | ((int32_t)resp[MIPC_HDR_SIZE+3] << 24);

    if (status != 0)
    {
        char dbg[32];
        sprintf(dbg, "DNS: failed status=%d\r\n", (int)status);
        USART1_Print(dbg);
        return -1;
    }

    /* Read s_addr as little-endian uint32_t then extract octets */
		uint32_t s_addr = (uint32_t)(resp[MIPC_HDR_SIZE + 4])
                | ((uint32_t)resp[MIPC_HDR_SIZE + 5] << 8u)
                | ((uint32_t)resp[MIPC_HDR_SIZE + 6] << 16u)
                | ((uint32_t)resp[MIPC_HDR_SIZE + 7] << 24u);

		sprintf(ip_out, "%u.%u.%u.%u",
        (s_addr)       & 0xFFu,
        (s_addr >> 8)  & 0xFFu,
        (s_addr >> 16) & 0xFFu,
        (s_addr >> 24) & 0xFFu);

    char dbg[48];
    sprintf(dbg, "DNS resolved: %s\r\n", ip_out);
    USART1_Print(dbg);
    return 0;
}

/* ------------------------------------------------------------------ */
/* ThingSpeak POST                                                       */
/* Resolves hostname to IP then sends HTTP POST with field1 value      */
/* ------------------------------------------------------------------ */
static void ThingSpeak_Send(float value)
{
		char ts_ip[24] = {0};

		// Command to find IP's 
		//[System.Net.Dns]::GetHostAddresses("api.thingspeak.com") | Select-Object IPAddressToString
		//strcpy(ts_ip, "34.203.56.254");
		//strcpy(ts_ip, "18.204.112.16");
		strcpy(ts_ip, "184.106.153.149");
		USART1_Print("TS: using hardcoded IP\r\n");
		
		//USART1_Print("TS: resolving hostname\r\n");
		
		//if (EMW_ResolveDNS(THINGSPEAK_HOST, ts_ip) != 0)
		//{
		//	USART1_Print("DNS failed\r\n");
		//	return;
		//}
		
    /* Build HTTP GET request */
    static char request[512];
    int req_len = sprintf(request,
        "GET /update?api_key=%s&field1=%.2f HTTP/1.1\r\n"
        "Host: api.thingspeak.com\r\n"
        "Connection: close\r\n"
        "\r\n",
        THINGSPEAK_API_KEY, value);

    /* Create socket */
    int fd = EMW_SockCreate();
    if (fd < 0)
    {
        USART1_Print("Socket create failed\r\n");
        return;
    }
    DELAY_MS(200u);

    /* Connect */
    if (EMW_SockConnect(fd, ts_ip, THINGSPEAK_PORT) != 0)
    {
        USART1_Print("Socket connect failed\r\n");
        EMW_SockClose(fd);
        return;
    }

    /* Send */
    if (EMW_SockSend(fd, request, (uint16_t)req_len) < 0)
    {
        USART1_Print("Socket send failed\r\n");
        EMW_SockClose(fd);
        return;
    }
		
		DELAY_MS(500u);
    if (NOTIFY_HasData())
    {
        static uint8_t drain[EMW_MAX_PAYLOAD];
        EMW_SockTransaction(NULL, 0u, drain, sizeof(drain));
        USART1_Print("Response drained\r\n");
    }

    USART1_Print("ThingSpeak POST sent!\r\n");
    EMW_SockClose(fd);
}

/* ------------------------------------------------------------------ */
/* Temperature Sensor configuration                                             */
/* ------------------------------------------------------------------ */

// Pin assignments
#define SCL_PIN     4    // PH4 = I2C2_SCL
#define SDA_PIN     5    // PH5 = I2C2_SDA

// HTS221 I2C address & registers fro
#define HTS221_ADDR         0x5F
#define CTRL_REG1           0x20
#define PD_ODR_1HZ          0x87    // power on, block update, 1Hz

// Temperature calibration registers
#define TEMP_LOW_DEGC_X8    0x32
#define TEMP_HIGH_DEGC_X8   0x33
#define TEMP_LOW_HIGH_MSB   0x35
#define TEMP_LOW_OUT_L      0x3C
#define TEMP_HIGH_OUT_L     0x3E

// Temperature data register
#define TEMP_L              0x2A

// Temperature calibration values 
static float   temp_low_degC, temp_high_degC;
static int16_t temp_low_out, temp_high_out;

// I2C 

// Sets up PH4 and PH5 as an I2C bus to communicate with the HTS221 sensor at 100kHz
void I2C2_Init(void) {
    RCC->APB1ENR1 |= RCC_APB1ENR1_I2C2EN;
    RCC->AHB2ENR1 |= RCC_AHB2ENR1_GPIOHEN;

    GPIOH->MODER = (GPIOH->MODER & ~((3U<<(2*SCL_PIN)) | (3U<<(2*SDA_PIN))))
                   | (2U<<(2*SCL_PIN)) | (2U<<(2*SDA_PIN));
    GPIOH->AFR[0] = (GPIOH->AFR[0] & ~((0xFU<<(4*SCL_PIN)) | (0xFU<<(4*SDA_PIN))))
                    | (4U<<(4*SCL_PIN)) | (4U<<(4*SDA_PIN));
    GPIOH->OTYPER  |= (1U<<SCL_PIN) | (1U<<SDA_PIN);
    GPIOH->OSPEEDR |= (3U<<(2*SCL_PIN)) | (3U<<(2*SDA_PIN));
    GPIOH->PUPDR = (GPIOH->PUPDR & ~((3U<<(2*SCL_PIN)) | (3U<<(2*SDA_PIN))))
                   | (1U<<(2*SCL_PIN)) | (1U<<(2*SDA_PIN));

    I2C2->CR1    &= ~I2C_CR1_PE;
    I2C2->ICR     =  I2C_ICR_STOPCF | I2C_ICR_NACKCF | I2C_ICR_BERRCF;
    I2C2->TIMINGR =  0x30420F13; // 100kHz @ 4MHz
    I2C2->CR1    |=  I2C_CR1_PE;
}

// Sends a register address followed by one value byte to the sensor
void I2C_Write(uint8_t reg, uint8_t val) {
    I2C2->CR2 = (HTS221_ADDR << 1) | (2 << 16) | I2C_CR2_START | I2C_CR2_AUTOEND;
    while (!(I2C2->ISR & I2C_ISR_TXIS));
    I2C2->TXDR = reg;
    while (!(I2C2->ISR & I2C_ISR_TXIS));
    I2C2->TXDR = val;
    while (!(I2C2->ISR & I2C_ISR_STOPF));
    I2C2->ICR = I2C_ICR_STOPCF;
}

// Tells the sensor which register to read from and then reads back the requested number of bytes
void I2C_Read(uint8_t reg, uint8_t* buf, uint8_t len) {
    I2C2->CR2 = (HTS221_ADDR << 1) | (1 << 16) | I2C_CR2_START | I2C_CR2_AUTOEND;
    while (!(I2C2->ISR & I2C_ISR_TXIS));
    I2C2->TXDR = reg;
    while (!(I2C2->ISR & I2C_ISR_STOPF));
    I2C2->ICR = I2C_ICR_STOPCF;

    I2C2->CR2 = (HTS221_ADDR << 1) | (len << 16) | I2C_CR2_RD_WRN | I2C_CR2_START | I2C_CR2_AUTOEND;
    for (uint8_t i = 0; i < len; i++) {
        while (!(I2C2->ISR & I2C_ISR_RXNE));
        buf[i] = I2C2->RXDR;
    }
    while (!(I2C2->ISR & I2C_ISR_STOPF));
    I2C2->ICR = I2C_ICR_STOPCF;
}


// HTS221 

// Powers on the sensor and sets it to refresh its reading once per second
void HTS221_PowerOn(void) {
    I2C_Write(CTRL_REG1, PD_ODR_1HZ);
    for (volatile int i = 0; i < 100000; i++);
}

// Reads the two reference points from the sensor needed to convert raw values to Celsius
void HTS221_ReadCalibration(void) {
    uint8_t t0_x8, t1_x8, msb, buf2[2];

    I2C_Read(TEMP_LOW_DEGC_X8,  &t0_x8, 1);
    I2C_Read(TEMP_HIGH_DEGC_X8, &t1_x8, 1);
    I2C_Read(TEMP_LOW_HIGH_MSB, &msb,   1);

    uint16_t t_low  = ((msb & 0x03) << 8) | t0_x8;
    uint16_t t_high = ((msb & 0x0C) << 6) | t1_x8;
    temp_low_degC  = t_low  / 8.0f;
    temp_high_degC = t_high / 8.0f;

    I2C_Read(TEMP_LOW_OUT_L  | 0x80, buf2, 2);   // ? 0x80 required!
    temp_low_out  = (int16_t)((buf2[1] << 8) | buf2[0]);

    I2C_Read(TEMP_HIGH_OUT_L | 0x80, buf2, 2);   // ? 0x80 required!
    temp_high_out = (int16_t)((buf2[1] << 8) | buf2[0]);
}


// Reads the latest raw value and converts it to Celsius using the calibration points
float HTS221_ReadTemperature(void) {
    uint8_t buf[2];
    I2C_Read(TEMP_L | 0x80, buf, 2);
    int16_t raw = (buf[1]<<8) | buf[0];
    float temp = temp_low_degC + ((raw - temp_low_out) * (temp_high_degC - temp_low_degC)) / (temp_high_out - temp_low_out);
		temp += -11.0f;  
		if (temp < -40) temp = -40;
    if (temp > 120) temp = 120;
    return temp;
}

/* ------------------------------------------------------------------ */
/* Main                                                                  */
/* ------------------------------------------------------------------ */
int main(void)
{
    char buf[96];
		static uint8_t evt[EMW_MAX_PAYLOAD];

    Enable_Clocks();
    Configure_Pins();
    USART1_Init();
    USART1_Print("=== Boot OK ===\r\n");

    CS_High();
    SPI2_Init();
		//I2C2_Init();
    //HTS221_PowerOn();
    //HTS221_ReadCalibration();

    /* Power cycle EMW3080 */
    GPIOF->BRR  = (1UL << PIN_Chip_En);
    DELAY_SEC(1u);
    GPIOF->BSRR = (1UL << PIN_Chip_En);
    USART1_Print("Chip_En HIGH - waiting for boot...\r\n");

    uint32_t boot_to = 5000u;
    uint8_t saw_notify = 0u;
    while (boot_to--)
    {
        if (NOTIFY_HasData())
        {
            saw_notify = 1u;
            sprintf(buf, "NOTIFY pulsed at %u ms remaining\r\n", boot_to);
            USART1_Print(buf);
            break;
        }
        DELAY_MS(1u);
    }
    if (!saw_notify) USART1_Print("WARNING: NOTIFY never pulsed during boot\r\n");

    DELAY_SEC(1u);
    USART1_Print("Boot delay done\r\n");

    /* Step 1: confirm module alive */
    USART1_Print("--- VERSION CMD ---\r\n");
    EMW_SendMIPC(MIPC_API_SYS_VERSION_CMD, NULL, 0u);
    DELAY_SEC(1u);

    /* Step 2: connect to WiFi */
    USART1_Print("--- WIFI CONNECT ---\r\n");
		int result = EMW_WiFiConnect("YOUR_SSID", "YOUR_PASSWORD");
		
    /* Declare ip_ok HERE so while(1) can see it */
    int ip_ok = 0;

    if (result == 0)
    {
        USART1_Print("WiFi connected! Waiting for DHCP...\r\n");
        USART1_Print("Waiting for netif up event...\r\n");

        /* Step 1: wait for DHCP event notification from module */
        uint32_t dhcp_wait = 15000u;
        while (dhcp_wait--)
        {
            if (NOTIFY_HasData())
            {
                int n = EMW_Transaction(NULL, 0u, evt, sizeof(evt));
                if (n > (int)MIPC_HDR_SIZE)
                {
                    uint16_t evt_api = (uint16_t)(evt[4] | ((uint16_t)evt[5] << 8u));
                    char dbg[32];
                    sprintf(dbg, "Event api=0x%04X\r\n", evt_api);
                    USART1_Print(dbg);
                    if (evt_api == 0x8101u)
                    {
                        USART1_Print("Netif up!\r\n");
                        break;
                    }
                }
            }
            DELAY_MS(1u);
        }

        /* Step 2: check IP */
        DELAY_MS(500u);
        if (EMW_GetIP() == 0)
        {
            ip_ok = 1;
            USART1_Print("IP ready!\r\n");
        }
        else
        {
            USART1_Print("Netif up but no IP yet, polling...\r\n");
            for (int i = 0; i < 10 && !ip_ok; i++)
            {
                DELAY_SEC(4u);
                if (EMW_GetIP() == 0)
                {
                    ip_ok = 1;
                    USART1_Print("IP ready!\r\n");
                }
                else
                {
                    char dbg[32];
                    sprintf(dbg, "No IP yet... attempt %d/10\r\n", i + 1);
                    USART1_Print(dbg);
                }
            }
        }

        if (!ip_ok)
        {
            USART1_Print("DHCP failed!\r\n");
        }
    }
    else
    {
        USART1_Print("WiFi connect failed\r\n");
    }

    /* Idle loop */
		
		I2C2_Init();
    HTS221_PowerOn();
    HTS221_ReadCalibration();

    for (volatile int i = 0; i < 2000000; i++); // wait for sensor to start
		
    while (1)
    {
        if (ip_ok)
        {
						float temperature = HTS221_ReadTemperature();
            USART1_Print("--- THINGSPEAK POST ---\r\n");
            ThingSpeak_Send(temperature);
						char dbg[32];
						sprintf(dbg, "Temp: %.2f C\r\n", temperature);
            USART1_Print(dbg);
        }
        else
        {
            USART1_Print("No IP - skipping ThingSpeak\r\n");
        }
        DELAY_SEC(60u);
    }
}
