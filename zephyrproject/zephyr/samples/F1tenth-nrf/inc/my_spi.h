uint8_t spi_init(void);

void reset_CSN(void);

uint8_t spi_send_test(void);

uint8_t spi_send(uint8_t mess);

void spi_send_bytes(uint8_t *mess, uint8_t size, uint8_t *res);
