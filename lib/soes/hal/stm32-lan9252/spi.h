#ifndef __SPI_H__
#define __SPI_H__

#include <stdint.h>
#include "gd32f4xx.h"

#define SCS_LOW                           0
#define SCS_HIGH                          1

#define DUMMY_BYTE 0xFF
#define tout 5000

void spi_setup(void);
void spi_select (int8_t board);
void spi_unselect (int8_t board);
void write (int8_t board, uint8_t *data, uint8_t size);
void read (int8_t board, uint8_t *result, uint8_t size);
void spi_bidirectionally_transfer (int8_t board, uint8_t *result, uint8_t *data, uint8_t size);

#endif /* __SPI_H__ */
