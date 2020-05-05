#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>
#include <drivers/spi.h>

#include "test.h"
#include "nRF24L01.h"
#include "my_nrf.h"

// #include "my_spi.h"

#define _BV(bit) (1 << (bit)) 
#define min(x, y) (x<y?x:y) 
#define max(x, y) (x>y?x:y)
#define SLEEP_TIME_MS   1000

#define CSN_PORT "GPIOB"
#define CSN_PIN 6
#define CE_PORT "GPIOC"
#define CE_PIN 7
#define FLAGS 0

struct device *spi_dev;
struct device *gpio_CSN_dev;
struct device *gpio_CE_dev;

bool dynamic_payloads_enabled = false;

struct spi_config config = {
    .frequency = 5000000,
    .operation = (SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPOL | SPI_MODE_CPHA),
    .slave = 0
};

uint8_t spi_init(void){
    const char *spi_name = "SPI_1";
    spi_dev = device_get_binding(spi_name);

    if (spi_dev == NULL)
        return 1;
    else
        return 0;
}

uint8_t gpio_init(void){
	uint8_t ret = 0;

	gpio_CE_dev = device_get_binding(CE_PORT);
	gpio_CSN_dev = device_get_binding(CSN_PORT);

	if (gpio_CSN_dev == NULL || gpio_CE_dev == NULL)
		return 1;
	
	ret = gpio_pin_configure(gpio_CE_dev, CE_PIN, GPIO_OUTPUT_ACTIVE | FLAGS);

	printk("GPIO ret: %d\n", ret);
	ret |= gpio_pin_configure(gpio_CSN_dev, CSN_PIN, GPIO_OUTPUT_ACTIVE | FLAGS);

	printk("GPIO ret: %d\n", ret);

	return ret;
}

void begin_trans(void){
	// set CSN LOW
	gpio_pin_set(gpio_CSN_dev, CSN_PIN, 0);
}

void end_trans(void){
	// set CSN HIGH
	gpio_pin_set(gpio_CSN_dev, CSN_PIN, 1);
}

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
	} 
	else {
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

	begin_trans();
	err = spi_transceive(spi_dev, &config, &tx, &rx);
	end_trans();
	
	if (err) {
		printk("SPI error: %d\n", err);
	}

	return rx_buffer[0];
}

/*
	Send bytes
*/
void spi_send_bytes(uint8_t *mess, uint8_t size, uint8_t *res){
	for(uint8_t i = 0 ; i < size ; i++){
		res[i] = spi_send(mess[i]);

		//printk("send: %x\n", mess[i]);
		//printk("recv: %x\n", res[i]);
	}
}

uint8_t write_register(uint8_t reg, uint8_t val){
	uint8_t status = 0;

	status = spi_send(W_REGISTER | (REGISTER_MASK & reg));
	spi_send(val);

	return status;
}

uint8_t read_register(uint8_t reg){
	uint8_t status = 0;

	spi_send(R_REGISTER | (REGISTER_MASK & reg));
	status = spi_send(0xFF);
}

bool set_data_rate(rf24_datarate_e speed){
    bool result = false;
    uint8_t setup = read_register(RF_SETUP);

    setup &= ~(_BV(RF_DR_LOW) | _BV(RF_DR_HIGH));

    if (speed == RF24_250KBPS) {
		setup |= _BV(RF_DR_LOW);

	}
	else{
		if (speed == RF24_2MBPS) {
            setup |= _BV(RF_DR_HIGH);
		}
	}
	
	write_register(RF_SETUP, setup);
	if (read_register(RF_SETUP) == setup) {
        result = true;
    }
    return result;
}

void set_retries(uint8_t delay, uint8_t count){
	write_register(SETUP_RETR, (delay & 0xf) << ARD | (count & 0xf) << ARC);
}

void toggle_features(void){
	spi_send(ACTIVATE);
	spi_send(0x73);
}

void set_channel(uint8_t channel){
	const uint8_t max_channel = 125;
	write_register(RF_CH, min(channel, max_channel));
}

void flush_rx(){
	spi_send(FLUSH_RX);
}

void flush_tx(){
	spi_send(FLUSH_TX);
}

void power_up(){
	uint8_t cfg = 0;

	cfg = read_register(NRF_CONFIG);

	if (!(cfg & _BV(PWR_UP))) {
		write_register(NRF_CONFIG, cfg | _BV(PWR_UP));

		k_sleep(K_MSEC(5));
	}
}

uint8_t init_nrf(void){
	printk("hej0");
    uint8_t setup = 0;


	write_register(0x00, 0x0C);
	
	printk("hej1");

	set_retries(5, 15);

	printk("hej2\n");

	if(set_data_rate(RF24_250KBPS))
		printk("OK\n");
    
	setup = read_register(RF_SETUP);

	set_data_rate(RF24_1MBPS);

	toggle_features();
    write_register(FEATURE, 0);
    write_register(DYNPD, 0);


    write_register(NRF_STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT));
	set_channel(76);

	flush_rx();
	flush_tx();

	power_up();

    write_register(NRF_CONFIG, (read_register(NRF_CONFIG)) & ~_BV(PRIM_RX));

	printk("setup: %d\n", setup);
    return (setup != 0 && setup != 0xff);
}

void dupa(){
	printk("DUPA\n");
}

void main(void){
	dupa();
	dupa();
    if (spi_init()){
        printk("FAILED to init spi\n");
        printk("exited\n");
        return;
    }
    else{
        printk("SPI ok\n");
    }

	if (gpio_init()){
        printk("FAILED to init spi\n");
        printk("exited\n");
        return;
	}
	else{
		printk("GPIO ok\n");
	}

	printk("witaj\n");

	dupa();
    if(init_nrf()){
		printk("NRF OK\n");
	}
	else{
		printk("NRF FAILED\n");
		return;
	}

	
    while(1){

    }
}
