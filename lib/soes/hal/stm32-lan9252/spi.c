#include "gd32f4xx.h"

#include "spi.h"

void spi_setup(void) {
//    RCC_APB2PeriphClockCmd(ESC_RCC_APB2PERIPH_SPIX, ENABLE);
//    RCC_AHB1PeriphClockCmd(ESC_RCC_APB1PERIPH_GPIOX_CTRL, ENABLE);
//    RCC_AHB1PeriphClockCmd(ESC_RCC_APB1PERIPH_GPIOX_CS, ENABLE);

    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_SPI0);

    /* SPI0 in GPIO is alternate function 5 setup for pins 4 CS, 5 SCK, 6 MISO, 7 MOSI */
    gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_PIN_4);
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLDOWN, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
    gpio_af_set(GPIOA, GPIO_AF_5, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);

    spi_i2s_deinit(SPI0);

    spi_parameter_struct spi_config;
    spi_struct_para_init(&spi_config);
    spi_config.device_mode = SPI_MASTER;
    spi_config.trans_mode = SPI_TRANSMODE_FULLDUPLEX;
    spi_config.frame_size = SPI_FRAMESIZE_8BIT;
    spi_config.nss = SPI_NSS_SOFT;
    spi_config.clock_polarity_phase = SPI_CK_PL_LOW_PH_1EDGE;
    spi_config.prescale = SPI_PSC_2;
    spi_config.endian = SPI_ENDIAN_MSB;
    spi_init(SPI0, &spi_config);
    
    spi_crc_polynomial_set(SPI0, 7);

    spi_enable(SPI0);
}

void spi_select (int8_t board) {
    gpio_bit_reset(GPIOA, GPIO_PIN_4);
}

void spi_unselect (int8_t board) {
    gpio_bit_set(GPIOA, GPIO_PIN_4);
}

inline static uint8_t spi_transfer(uint8_t byte) {
    while ((SPI_STAT(SPI0) & SPI_STAT_TBE) == RESET);
    SPI_DATA(SPI0) = byte;
    while ((SPI_STAT(SPI0) & SPI_STAT_RBNE) == RESET);
    return SPI_DATA(SPI0);
}

void write (int8_t board, uint8_t *data, uint8_t size) {
    for (int i = 0; i < size; ++i) {
        spi_transfer(data[i]);
    }
}

void read (int8_t board, uint8_t *result, uint8_t size) {
	for (int i = 0; i < size; ++i) {
        result[i] = spi_transfer(DUMMY_BYTE);
    }
}

void spi_bidirectionally_transfer (int8_t board, uint8_t *result, uint8_t *data, uint8_t size) {
	for (int i = 0; i < size; ++i) {
        result[i] = spi_transfer(data[i]);
    }
}