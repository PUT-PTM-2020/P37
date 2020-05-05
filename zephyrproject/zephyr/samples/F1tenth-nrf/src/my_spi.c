#include "my_spi.h"
#include <drivers/spi.h>


uint8_t spi_init(void){
    const char *spi_name = "SPI_1";
    spi_dev = device_get_binding(spi_name);

    if (spi_dev == NULL)
        return 1;
    else
        return 0;
}

/*
	Reset CSN pin
*/
void reset_CSN(void){}

uint8_t spi_send_test(void){
    int err;
	static uint8_t tx_buffer[1] = {0x69};
	static uint8_t rx_buffer[1];

	const struct spi_buf tx_buf = {
        tx_buffer,
		sizeof(tx_buffer)
	};
	const struct spi_buf_set tx = {
		&tx_buf,
		1
	};

	struct spi_buf rx_buf = {
		rx_buffer,
		sizeof(rx_buffer),
	};
	const struct spi_buf_set rx = {
		&rx_buf,
		1
	};

	err = spi_transceive(spi_dev, &config, &tx, &rx);
    
    printk("tutej\n");

	if (err) {
		printk("SPI error: %d\n", err);
	} else {
		/* Connect MISO to MOSI for loopback */
		printk("TX sent: %x\n", tx_buffer[0]);
		printk("RX recv: %x\n", rx_buffer[0]);
		tx_buffer[0]++;
	}
}

/*
	Send byte 
	based on spi_send_test()
*/
uint8_t spi_send(uint8_t mess){
    int err;
	static uint8_t tx_buffer[1];
	tx_buffer[0] = mess;
	static uint8_t rx_buffer[1];

	const struct spi_buf tx_buf = {
        tx_buffer,
		sizeof(tx_buffer)
	};
	const struct spi_buf_set tx = {
		&tx_buf,
		1
	};

	struct spi_buf rx_buf = {
		rx_buffer,
		sizeof(rx_buffer),
	};
	const struct spi_buf_set rx = {
		&rx_buf,
		1
	};

	err = spi_transceive(spi_dev, &config, &tx, &rx);

	if (err) {
		printk("SPI error: %d\n", err);
	}

	return rx_buffer[0];
}

/*
	Send bytes
*/
void spi_send_bytes(uint8_t *mess, uint8_t size, uint8_t *res){
	uint8_t result = 0;
	for(uint8_t i = 0 ; i < size ; i++){
		res[i] = spi_send(mess[i]);

		printk("send: %x\n", mess[i]);
		printk("recv: %x\n", result);
	}
}