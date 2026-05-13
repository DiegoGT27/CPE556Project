#include "stm32u5xx.h"
#include <stdio.h>

// Pin assignments
#define SCL_PIN     4    // PH4 = I2C2_SCL
#define SDA_PIN     5    // PH5 = I2C2_SDA
#define UART_TX     9    // PA9 = USART1_TX
#define UART_RX     10   // PA10 = USART1_RX

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


//  UART

// Sets up PA9 and PA10 as a serial connection to send text to your PC at 115200 baud
void USART1_Init(void) {
    RCC->APB2ENR  |= RCC_APB2ENR_USART1EN;
    RCC->AHB2ENR1 |= RCC_AHB2ENR1_GPIOAEN;

    GPIOA->MODER &= ~((3 << (2*UART_TX)) | (3 << (2*UART_RX)));
    GPIOA->MODER |=  ((2 << (2*UART_TX)) | (2 << (2*UART_RX)));

    GPIOA->AFR[1] &= ~((0xF << (4*(UART_TX-8))) | (0xF << (4*(UART_RX-8))));
    GPIOA->AFR[1] |=  ((7   << (4*(UART_TX-8))) | (7   << (4*(UART_RX-8))));

    GPIOA->OSPEEDR |=  (3 << (2*UART_TX)) | (3 << (2*UART_RX));
    GPIOA->PUPDR   &= ~((3 << (2*UART_TX)) | (3 << (2*UART_RX)));
    GPIOA->PUPDR   |=  (1 << (2*UART_RX));

    USART1->BRR = 35; // 115200 baud @ 4 MHz
    USART1->CR1 = USART_CR1_TE | USART_CR1_UE;
}

// Sends a string one character at a time, waiting for the hardware to be ready between each
void USART1_SendString(char *s) {
    while (*s) {
        while (!(USART1->ISR & USART_ISR_TXE));
        USART1->TDR = *s++;
    }
}


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


// Main loop: read temperature and send to PC every 1 seconds ish
int main(void) {
    char out[64];

    USART1_Init();
    I2C2_Init();
    HTS221_PowerOn();
    HTS221_ReadCalibration();

    for (volatile int i = 0; i < 2000000; i++); // wait for sensor to start

    while (1) {
        float temperature = HTS221_ReadTemperature();

        snprintf(out, sizeof(out), "Temp: %.2f C\r\n", temperature);
        USART1_SendString(out);

        for (volatile int i = 0; i < 1000000; i++); // about 1s delay
    }
}


//usart1init configures PA9 and PA10 pins as UART, sets baud rate to 115200 so text can be sent to PC 
// I2C2_Init()  configures PH4 and PH5 pins as I2C, sets speed to 100kHz so the i2c chip can talk to the sensor 

// HTS221_PowerOn() calls I2C_Write() to send 0x87 (10000111) to the sensor control register (0x20), which is waking sensor up at 1Hz and then it uses I2C2->CR2 to fire the actual signal down the wire to the sensor 

// HTS221_ReadCalibration() calls I2C_Read() 5 times to fetch the two temperature reference points from sensor memory. It does that first by asking to view the address register, and then collect from that register to send it to the I2C bus. And then when it’s back into readcalibration() loop, it uses the raw bytes from buf2 and we do magic to store its results in temp_low_degC, temp_high_degC, temp_low_out, temp_high_out . ex: temp_low_out = (int16_t)((buf2[1]<<8) | buf2[0]); 

