#include "gd32f4xx.h"

#include "spi.h"

void spi_gpio_setup(void) {
    rcu_periph_clock_enable(RCU_GPIOA);

	// AF 5 for these pins is SPI0
    gpio_af_set(GPIOA, GPIO_AF_5, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);

	// note 25 MHz instead of 50 here
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_5|GPIO_PIN_6| GPIO_PIN_7);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, GPIO_PIN_5|GPIO_PIN_6| GPIO_PIN_7);

	// no pull up or down
    gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_4);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_4);
}

void spi_setup(void) {
	spi_parameter_struct spi_init_struct;

    spi_i2s_deinit(SPI0);

	rcu_periph_clock_enable(RCU_SPI0);
	
	spi_init_struct.trans_mode           = SPI_TRANSMODE_FULLDUPLEX;
	spi_init_struct.device_mode          = SPI_MASTER;
	spi_init_struct.frame_size           = SPI_FRAMESIZE_8BIT;
	spi_init_struct.clock_polarity_phase = SPI_CK_PL_LOW_PH_1EDGE;
	spi_init_struct.nss                  = SPI_NSS_SOFT;
	// note 16 prescale instead of 2
	spi_init_struct.prescale             = SPI_PSC_16;
	spi_init_struct.endian               = SPI_ENDIAN_MSB;
	spi_init(SPI0, &spi_init_struct);
	
	spi_enable(SPI0);


    //spi_crc_polynomial_set(SPI0, 7);

    //spi_enable(SPI0);
}

void spi_select (int8_t board) {
    gpio_bit_reset(GPIOA, GPIO_PIN_4);
}

void spi_unselect (int8_t board) {
    gpio_bit_set(GPIOA, GPIO_PIN_4);
}

inline static uint8_t spi_transfer(uint8_t byte) {
    while (RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_TBE)) {}

	spi_i2s_data_transmit(SPI0, byte);

	while (RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_RBNE)) {}
    return spi_i2s_data_receive(SPI0);
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
