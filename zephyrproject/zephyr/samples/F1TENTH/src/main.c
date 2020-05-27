#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>
#include <sys/printk.h>
#include <drivers/pwm.h>
#include <drivers/uart.h>
#include <kernel.h>
#include <errno.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <arch/cpu.h>
#include <sys/byteorder.h>
#include <logging/log.h>
#include <sys/util.h>
#include <init.h>
 
#define ENGINE_PORT "GPIOA"
#define IN1_PIN 5
#define IN2_PIN 2
#define IN3_PIN 3
#define IN4_PIN 4
 
#define SONIC_SENSOR_PORT "GPIOE"
#define TRIG1_PIN 1
#define ECHO1_PIN 0
#define TRIG2_PIN 3
#define ECHO2_PIN 2
#define TRIG3_PIN 5
#define ECHO3_PIN 4
 
<<<<<<< HEAD
=======
#define LEDS_PORT "GPIOD"
#define ORANGE_LED 13
#define GREEN_LED 12
#define RED_LED 14
#define BLUE_LED 15
>>>>>>> mts-srs
#define FLAGS 0
 
#define DT_ALIAS_PWM_2_LABEL "PWM_2"
#define DT_ALIAS_PWM_3_LABEL "PWM_3"
 
#ifndef DT_ALIAS_PWM_2_LABEL
#error "PWM_2 device label not defined"
#endif
 
#ifndef DT_ALIAS_PWM_3_LABEL
#error "PWM_3 device label not defined"
#endif
 
#define DT_ALIAS_UART_1_LABEL "UART_1"
 
#ifndef DT_ALIAS_UART_1_LABEL
#error "UART_1 device label not defined"
#endif
 
/* period of servo motor signal ->  2.4ms */
#define PERIOD (USEC_PER_SEC / 490U)
 
#define STACKSIZE 1024
<<<<<<< HEAD
 
struct device *gpio_engine_dev;
struct device *gpio_sonic_sensor_dev;
struct device *uart1_dev;
struct device *pwm2_dev, *pwm3_dev;
int left_dist = 100, front_dist = 100, right_dist = 100;
int *distance = NULL;
unsigned char *recv_data;
=======

extern const k_tid_t main_th_id;

struct device *gpio_engine_dev, *gpio_sonic_sensor_dev, *gpio_leds_dev;
struct device *pwm2_dev, *pwm3_dev;
struct device *uart1_dev;
int left_dist = 100, front_dist = 100, right_dist = 100;
int *distance = NULL;
int stopping_counter = 0; 
u8_t *recv_data = '\0';
u8_t esp_received[10];
>>>>>>> mts-srs
 
const struct uart_config uart_cfg = {
        .baudrate = 115200,
        .parity = UART_CFG_PARITY_NONE,
        .stop_bits = UART_CFG_STOP_BITS_1,
        .data_bits = UART_CFG_DATA_BITS_8,
        .flow_ctrl = UART_CFG_FLOW_CTRL_NONE
    };
 
void insertionSort(int array[], int size) {
  for (int step = 1; step < size; step++) {
    int key = array[step];
    int j = step - 1;
 
    // Compare key with each element on the left of it until an element smaller than
    // it is found.
    // For descending order, change key<array[j] to key>array[j].
    while (key < array[j] && j >= 0) {
      array[j + 1] = array[j];
      --j;
    }
    array[j + 1] = key;
  }
}
 
uint8_t gpio_init(void){
    uint8_t ret = 0;
    gpio_engine_dev = device_get_binding(ENGINE_PORT);
    gpio_sonic_sensor_dev = device_get_binding(SONIC_SENSOR_PORT);
<<<<<<< HEAD
=======
    gpio_leds_dev = device_get_binding(LEDS_PORT);
>>>>>>> mts-srs
    if (gpio_engine_dev == NULL || gpio_sonic_sensor_dev == NULL) {
        return 1;
    }
 
    /* Engines IN1 - IN4 pins cofiguration */
    gpio_pin_configure(gpio_engine_dev, IN2_PIN, GPIO_OUTPUT_INACTIVE | FLAGS);
    gpio_pin_configure(gpio_engine_dev, IN1_PIN, GPIO_OUTPUT_INACTIVE | FLAGS);
    gpio_pin_configure(gpio_engine_dev, IN4_PIN, GPIO_OUTPUT_INACTIVE | FLAGS);
    gpio_pin_configure(gpio_engine_dev, IN3_PIN, GPIO_OUTPUT_INACTIVE | FLAGS);
 
    /* Sonic sensors ECHO and TRIG pins cofiguration */
    gpio_pin_configure(gpio_sonic_sensor_dev, ECHO1_PIN, GPIO_INPUT | FLAGS);
    gpio_pin_configure(gpio_sonic_sensor_dev, ECHO2_PIN, GPIO_INPUT | FLAGS);
    gpio_pin_configure(gpio_sonic_sensor_dev, ECHO3_PIN, GPIO_INPUT | FLAGS);
    gpio_pin_configure(gpio_sonic_sensor_dev, TRIG1_PIN, GPIO_OUTPUT_INACTIVE | FLAGS);
    gpio_pin_configure(gpio_sonic_sensor_dev, TRIG2_PIN, GPIO_OUTPUT_INACTIVE | FLAGS);
    gpio_pin_configure(gpio_sonic_sensor_dev, TRIG3_PIN, GPIO_OUTPUT_INACTIVE | FLAGS);
    
    /* In-board leds configuration */
    gpio_pin_configure(gpio_leds_dev, ORANGE_LED, GPIO_OUTPUT_INACTIVE | FLAGS);
    gpio_pin_configure(gpio_leds_dev, GREEN_LED,  GPIO_OUTPUT_INACTIVE | FLAGS);
    gpio_pin_configure(gpio_leds_dev, RED_LED,    GPIO_OUTPUT_INACTIVE | FLAGS);
    gpio_pin_configure(gpio_leds_dev, BLUE_LED,   GPIO_OUTPUT_INACTIVE | FLAGS);

    return ret;
}
 
<<<<<<< HEAD
    return ret;
}
 
=======
>>>>>>> mts-srs
uint8_t uart_init(void){
    uint8_t ret = 0;
    uart1_dev = device_get_binding(DT_ALIAS_UART_1_LABEL);
    ret = uart_configure(uart1_dev, &uart_cfg);
 
    return ret;
}
 
uint8_t pwm_init(void){
    uint8_t ret = 0;
    pwm2_dev = device_get_binding(DT_ALIAS_PWM_2_LABEL);
    pwm3_dev = device_get_binding(DT_ALIAS_PWM_3_LABEL);
 
    return ret;
}
 
/* Returns distance measured by sensor with given id. */
void get_distance(int id){
        int trig_pin = 0, echo_pin = 0;
        if(id == 1){
            trig_pin = TRIG1_PIN;
            echo_pin = ECHO1_PIN;
            distance = &front_dist;
        }
        else if(id == 2){
            trig_pin = TRIG2_PIN;
            echo_pin = ECHO2_PIN;
            distance = &left_dist;
        }
        else if(id == 3){
            trig_pin = TRIG3_PIN;
            echo_pin = ECHO3_PIN;
            distance = &right_dist;
        }
        else{
            printk("Invalid sensor id.\n");
        }
 
        int data[11];
        int size = 11;
 
        for(int i = 0; i < size; i++){
<<<<<<< HEAD
            int val;
            gpio_pin_set(gpio_sonic_sensor_dev, trig_pin, true);
            k_sleep(K_USEC(11));
            gpio_pin_set(gpio_sonic_sensor_dev, trig_pin, false);
 
            u32_t start_time;
            u32_t stop_time;
            u32_t cycles_spent;
            u32_t nsec_spent;
 
=======
            u32_t start_time;
            u32_t stop_time;
            u32_t cycles_spent;
            u32_t nsec_spent = 0;

            int val;
            gpio_pin_set(gpio_sonic_sensor_dev, trig_pin, true);
            start_time = k_cycle_get_32();
            while(nsec_spent <= 11000){
                stop_time = k_cycle_get_32();
                cycles_spent = stop_time - start_time;
                nsec_spent = SYS_CLOCK_HW_CYCLES_TO_NS(cycles_spent);
            }

            gpio_pin_set(gpio_sonic_sensor_dev, trig_pin, false);
>>>>>>> mts-srs
            while(!gpio_pin_get(gpio_sonic_sensor_dev, echo_pin));
            start_time = k_cycle_get_32();
            while(gpio_pin_get(gpio_sonic_sensor_dev, echo_pin));
            stop_time = k_cycle_get_32();
            cycles_spent = stop_time - start_time;
            nsec_spent = SYS_CLOCK_HW_CYCLES_TO_NS(cycles_spent);
            val = nsec_spent / 58000;
            data[i] = val;
        }
 
        insertionSort(data, size);
        *distance = data[5]; //distance = median of 11 measures
        if(*distance > 1200) *distance = 0; //Distance higher than 1200 means object is closer than 2cm from sensor.
   
<<<<<<< HEAD
}
 
void get_distance_printk(void){
    while(1){
        get_distance(1);
        get_distance(2);
        get_distance(3);
        printk("Sensor left: %d cm\n", left_dist);
        printk("Sensor front: %d cm\n", front_dist);
        printk("Sensor right: %d cm\n\n", right_dist);
        k_msleep(10);
    }
}
 
 
=======
}
 
void get_distance_printk(void){
    while(1){
        get_distance(1);
        get_distance(2);
        get_distance(3);
        printk("Sensor left: %d cm\n", left_dist);
        printk("Sensor front: %d cm\n", front_dist);
        printk("Sensor right: %d cm\n\n", right_dist);
        k_msleep(10);
    }
}

>>>>>>> mts-srs
void engine_test(){
    while(1){
        pwm_pin_set_usec(pwm2_dev, 1, PERIOD, 1000, 0);
        pwm_pin_set_usec(pwm2_dev, 2, PERIOD, 1000, 0);
   
        if(front_dist < 10){
            /* Stop the car */
            gpio_pin_set(gpio_engine_dev, IN1_PIN, true);
            gpio_pin_set(gpio_engine_dev, IN2_PIN, true);
            gpio_pin_set(gpio_engine_dev, IN3_PIN, true);
            gpio_pin_set(gpio_engine_dev, IN4_PIN, true);
            pwm_pin_set_usec(pwm2_dev, 1, PERIOD, 2000, 0);
            pwm_pin_set_usec(pwm2_dev, 2, PERIOD, 2000, 0);
            k_msleep(1000);
 
            /* Turn left/depending on left sensor's value */
            int bool1 = false, bool2 = true;
            if(right_dist < 10){
                bool1 = true;
                bool2 = false;
            }
            pwm_pin_set_usec(pwm2_dev, 1, PERIOD, 1000, 0);
            pwm_pin_set_usec(pwm2_dev, 2, PERIOD, 1000, 0);
            gpio_pin_set(gpio_engine_dev, IN1_PIN, bool1);
            gpio_pin_set(gpio_engine_dev, IN2_PIN, bool2);
            gpio_pin_set(gpio_engine_dev, IN3_PIN, bool2);
            gpio_pin_set(gpio_engine_dev, IN4_PIN, bool1);
 
            /* Move forward */
            gpio_pin_set(gpio_engine_dev, IN2_PIN, true );
            gpio_pin_set(gpio_engine_dev, IN1_PIN, false);
            gpio_pin_set(gpio_engine_dev, IN4_PIN, true );
            gpio_pin_set(gpio_engine_dev, IN3_PIN, false);
        }
    }
}
 
void move_forward(const int velocity) {
    /* Move forward */
<<<<<<< HEAD
    printk("Moving forward!\n");
    pwm_pin_set_usec(pwm2_dev, 1, PERIOD, velocity + 100, 0);
    pwm_pin_set_usec(pwm2_dev, 2, PERIOD, velocity, 0);
    gpio_pin_set(gpio_engine_dev, IN2_PIN, true);
    gpio_pin_set(gpio_engine_dev, IN1_PIN, false);
    gpio_pin_set(gpio_engine_dev, IN4_PIN, true);
    gpio_pin_set(gpio_engine_dev, IN3_PIN, false);
}
 
void stop_car() {
    /* Stop the car */
    printk("Stopping!\n");
=======
    pwm_pin_set_usec(pwm2_dev, 1, PERIOD, velocity, 0);
    pwm_pin_set_usec(pwm2_dev, 2, PERIOD, velocity, 0);
    gpio_pin_set(gpio_engine_dev, IN1_PIN, false);
    gpio_pin_set(gpio_engine_dev, IN2_PIN, true);
    gpio_pin_set(gpio_engine_dev, IN3_PIN, true);
    gpio_pin_set(gpio_engine_dev, IN4_PIN, false);
    printk("Moving forward!\n");
}
void move_backwards(const int velocity) {
    /* Move forward */
    pwm_pin_set_usec(pwm2_dev, 1, PERIOD, velocity, 0);
    pwm_pin_set_usec(pwm2_dev, 2, PERIOD, velocity, 0);
    gpio_pin_set(gpio_engine_dev, IN1_PIN, true);
    gpio_pin_set(gpio_engine_dev, IN2_PIN, false);
    gpio_pin_set(gpio_engine_dev, IN3_PIN, false);
    gpio_pin_set(gpio_engine_dev, IN4_PIN, true);
    printk("Moving backwards!\n");
}
void stop_vehicle() {
    /* Stop the car */
>>>>>>> mts-srs
    gpio_pin_set(gpio_engine_dev, IN1_PIN, true);
    gpio_pin_set(gpio_engine_dev, IN2_PIN, true);
    gpio_pin_set(gpio_engine_dev, IN3_PIN, true);
    gpio_pin_set(gpio_engine_dev, IN4_PIN, true);
    pwm_pin_set_usec(pwm2_dev, 1, PERIOD, 2000, 0);
    pwm_pin_set_usec(pwm2_dev, 2, PERIOD, 2000, 0);
<<<<<<< HEAD
    k_msleep(1000);
}
 
void turn_right() {
    printk("Turning right!\n");
    pwm_pin_set_usec(pwm2_dev, 1, PERIOD, 1000, 0);
    pwm_pin_set_usec(pwm2_dev, 2, PERIOD, 1000, 0);
    gpio_pin_set(gpio_engine_dev, IN1_PIN, false);
    gpio_pin_set(gpio_engine_dev, IN2_PIN, true);
    gpio_pin_set(gpio_engine_dev, IN3_PIN, true);
    gpio_pin_set(gpio_engine_dev, IN4_PIN, false);
    k_msleep(800);
}
 
void turn_left() {
    printk("Turning left!\n");
    pwm_pin_set_usec(pwm2_dev, 1, PERIOD, 1000, 0);
    pwm_pin_set_usec(pwm2_dev, 2, PERIOD, 1000, 0);
    gpio_pin_set(gpio_engine_dev, IN1_PIN, true);
    gpio_pin_set(gpio_engine_dev, IN2_PIN, false);
    gpio_pin_set(gpio_engine_dev, IN3_PIN, false);
    gpio_pin_set(gpio_engine_dev, IN4_PIN, true);
    k_msleep(800);
}
 
void choose_direction(int *direction) {
    printk("dir: %d\n", *direction);
    if(*direction == 0 ) {
        if(right_dist>20){
            turn_right();
            *direction+=90;
        }
        else if(left_dist>20){
            turn_left();
            *direction-=90;
        } else *direction+=1000; //liczba wywolujaca stop jazdy
    }
    //pojazd jest odwrocony w prawo wzgledem celu, wiec bedzie probowac skrecic w lewo
    else if (*direction >= 90) {
        if(left_dist>20){
            turn_left();
            *direction-=90;
        }
    }
    //pojazd jest odwrocony w lewo wzgledem celu, wiec bedzie probowac skrecic w prawo
    else if(*direction <= -90 ) {
        if(right_dist>20){
            turn_right();
            *direction+=90;
        }
    }
 
}
 
void autonomous_test(){
    int dir = 0;
    while(1){
        pwm_pin_set_usec(pwm2_dev, 1, PERIOD, 1000 + 100, 0);
        pwm_pin_set_usec(pwm2_dev, 2, PERIOD, 1000, 0);
   
=======
    printk("Stopping!\n");
}
void turn_right(const int time) {
    pwm_pin_set_usec(pwm2_dev, 1, PERIOD, 500, 0);
    pwm_pin_set_usec(pwm2_dev, 2, PERIOD, 500, 0);
    gpio_pin_set(gpio_engine_dev, IN1_PIN, false);
    gpio_pin_set(gpio_engine_dev, IN2_PIN, true);
    gpio_pin_set(gpio_engine_dev, IN3_PIN, false);
    gpio_pin_set(gpio_engine_dev, IN4_PIN, true);
    if(time) k_msleep(time);
    printk("Turning right!\n");
}
void turn_left(const int time) {
    pwm_pin_set_usec(pwm2_dev, 1, PERIOD, 500, 0);
    pwm_pin_set_usec(pwm2_dev, 2, PERIOD, 500, 0);
    gpio_pin_set(gpio_engine_dev, IN1_PIN, true);
    gpio_pin_set(gpio_engine_dev, IN2_PIN, false);
    gpio_pin_set(gpio_engine_dev, IN3_PIN, true);
    gpio_pin_set(gpio_engine_dev, IN4_PIN, false);
    if(time) k_msleep(time);
    printk("Turning left!\n");
}

void choose_direction(int *direction) {
    printk("dir: %d\n", *direction);
    if(*direction == 0 ) {
        if(right_dist>20){
            turn_right(800);
            *direction+=90;
        }
        else if(left_dist>20){
            turn_left(800);
            *direction-=90;
        } else *direction+=1000; //liczba wywolujaca stop jazdy
    }
    //pojazd jest odwrocony w prawo wzgledem celu, wiec bedzie probowac skrecic w lewo
    else if (*direction >= 90) {
        if(left_dist>20){
            turn_left(800);
            *direction-=90;
        }
    }
    //pojazd jest odwrocony w lewo wzgledem celu, wiec bedzie probowac skrecic w prawo
    else if(*direction <= -90 ) {
        if(right_dist>20){
            turn_right(800);
            *direction+=90;
        }
    }
}
 
K_THREAD_STACK_DEFINE(my_stack, STACKSIZE);
struct k_thread get_distance_th;


void autonomous_mode(){
    int dir = 0;
    k_tid_t get_distance_id = k_thread_create(&get_distance_th, my_stack, STACKSIZE, get_distance_printk, NULL, NULL, NULL, 6, 0, K_NO_WAIT);
    while(*recv_data != 'M'){
        pwm_pin_set_usec(pwm2_dev, 1, PERIOD, 1000, 0);
        pwm_pin_set_usec(pwm2_dev, 2, PERIOD, 1000, 0);
>>>>>>> mts-srs
        if(front_dist < 20){
            choose_direction(&dir);
            if (dir >= 1000)
            {
<<<<<<< HEAD
                stop_car();
                break;
=======
                stop_vehicle();
>>>>>>> mts-srs
            }
            else {
                move_forward(1000);
            }
        }
<<<<<<< HEAD
    }
=======
        k_msleep(10);
    }    
}

void manual_mode(){
    printk("manual modeon\n\n");
    while(*recv_data != 'X' && *recv_data != 'q'){
        if(*recv_data == 'R'){
            turn_right(0);
            stopping_counter = 0;
        }
        else if(*recv_data == 'L'){
            turn_left(0);
            stopping_counter = 0;
        }
        else if(*recv_data == 'F'){
            move_forward(500);
            stopping_counter = 0;
        }
        else if(*recv_data == 'B'){
            move_backwards(500);
            stopping_counter = 0;
        }
        else if(*recv_data == 'A'){
            autonomous_mode();
            stopping_counter = 0;
        }
        else if(stopping_counter < 25){
            stopping_counter++;
        }
        else stop_vehicle();
        //*recv_data = '\0';
        k_msleep(10);
    }
    printk("manual modeoff\n\n");

}

void connection_test(){
    /* Light all in-boards user leds */
    gpio_pin_set(gpio_leds_dev, ORANGE_LED, true);
    k_msleep(500);
    gpio_pin_set(gpio_leds_dev, GREEN_LED, true);
    k_msleep(500);
    gpio_pin_set(gpio_leds_dev, BLUE_LED, true);
    k_msleep(500);
    gpio_pin_set(gpio_leds_dev, RED_LED, true);
    k_msleep(500);

    printk("RECV_DATA: %c\n\n", *recv_data);


    /* Put all in-boards user leds out */
    gpio_pin_set(gpio_leds_dev, ORANGE_LED, false);
    k_msleep(500);
    gpio_pin_set(gpio_leds_dev, GREEN_LED, false);
    k_msleep(500);
    gpio_pin_set(gpio_leds_dev, BLUE_LED, false);
    k_msleep(500);
    gpio_pin_set(gpio_leds_dev, RED_LED, false);
    k_msleep(500);
    *recv_data = '\0';
>>>>>>> mts-srs
}
 
void poll_out(char poll_data[]){
    for (int i = 0; i < strlen(poll_data); i++) {
        uart_poll_out(uart1_dev, poll_data[i]);
    }
}
<<<<<<< HEAD
 
void poll_in(){
    unsigned char recv_char;
    int esp_recv_counter = 0;
    char esp_pattern[5] = {'P','D',',','1',':'};
    //char esp_pattern[] = "PD,1:";
    for(int i = 0; i < sizeof(esp_pattern); i++){
        printk("%d: %c\n", i, esp_pattern[i]);
    }
    while (1) {
        while (uart_poll_in(uart1_dev, &recv_char) < 0) {
        }
        if (recv_char == esp_pattern[esp_recv_counter] || esp_recv_counter == 5) {
            if (esp_recv_counter == 5){
                recv_data = &recv_char;
                //printk("\nrecv_char: %c", recv_char);
                printk("\nrecv_data: %c\n\n", *recv_data);
                esp_recv_counter = 0;
            } else esp_recv_counter++;
        } else esp_recv_counter = 0;
        //printk("%c", recv_char);
    }
}

 
//K_THREAD_DEFINE(sensor1_th_id, STACKSIZE, get_distance, 1, NULL, NULL, 5, 0, 0);
//K_THREAD_DEFINE(sensor2_th_id, STACKSIZE, get_distance, 2, NULL, NULL, 4, 0, 0);
//K_THREAD_DEFINE(sensor3_th_id, STACKSIZE, get_distance, 3, NULL, NULL, 6, 0, 0);
K_THREAD_DEFINE(printk_id, STACKSIZE, get_distance_printk, NULL, NULL, NULL, 6, 0, 0);
K_THREAD_DEFINE(autonomous_id, STACKSIZE, autonomous_test, NULL, NULL, NULL, 7, 0, 0);
K_THREAD_DEFINE(poll_in_id, STACKSIZE, poll_in, NULL, NULL, NULL, 5, 0, 0);

void main(void){
=======

int esp_recv_counter = 0;

void uart_fifo_callback_init(struct device *dev)
{
	u8_t recv_char;
    
	if (!uart_irq_update(dev)) {
		printk("retval should always be 1\n");
		return;
	}

	if (uart_irq_rx_ready(dev)) {
		uart_fifo_read(dev, &recv_char, 7);
		printk("%c", recv_char);
	}
}

void uart_fifo_callback(struct device *dev)
{
	u8_t recv_char;
    
	if (!uart_irq_update(dev)) {
		printk("retval should always be 1\n");
		return;
	}

	if (uart_irq_rx_ready(dev)) {
		uart_fifo_read(dev, &recv_char, 1);
		printk("%c", recv_char);
        esp_received[esp_recv_counter] = recv_char;
        esp_recv_counter++;
        if (recv_char == '\n') {
            esp_recv_counter = 0;
            printk("\n\n");
            for(int i = 0; i < 10; i++) printk("%c", esp_received[i]);
            printk("\n\n");
            *recv_data = esp_received[7];
            //printk("\n\nrecv_data: %c", *recv_data);
        } 
        
	}
    //printk("RECV_DATA: %d\n\n", (int)*recv_data);
}


void fifo_read_init(void)
{
	uart_irq_callback_set(uart1_dev, uart_fifo_callback_init);
	uart_irq_rx_enable(uart1_dev);
}

void fifo_read(void)
{
	uart_irq_callback_set(uart1_dev, uart_fifo_callback);
}

void main_thread(){
    while(1){
        //printk("\nrNiewiem: %c\n\n", *recv_data);
        //if(*recv_data != '\0'){
            if(*recv_data == 'M'){
                manual_mode();
            }
            else if(*recv_data == 'A'){
                autonomous_mode();
            }
            else if(*recv_data == 'C'){
                connection_test();
            }
       // }
        k_msleep(10);
    }
}

K_THREAD_DEFINE(main_th_id, STACKSIZE, main_thread, NULL, NULL, NULL, 4, 0, 0);

void main(void){ 
>>>>>>> mts-srs
    if(gpio_init()) printk("GPIO init failed.\n");
    if(pwm_init())  printk("PWM init failed.\n");
    if(uart_init())  printk("UART init failed.\n");
    
<<<<<<< HEAD
    

    char *poll_data = "AT+RESTORE\r\n";
    // poll_out(poll_data);
    // k_msleep(3000);

    // poll_data = "AT+CWMODE=3\r\n";
    // poll_out(poll_data);
    // k_msleep(3000);

    // poll_data = "AT+CWJAP=\"TP_LINK\",\"65223246\"\r\n";
    // poll_out(poll_data);
    // k_msleep(10000);

    // poll_data = "AT+CIFSR\r\n";
    // poll_out(poll_data);
    // k_msleep(3000);
   
    // poll_data = "AT+CIPSTART=\"TCP\",\"192.168.1.104\",5005\r\n";
    // poll_out(poll_data);
    // k_msleep(3000);

    poll_data = "AT+CIPSEND=1\r\n";
    poll_out(poll_data);
    k_msleep(3000);

    poll_data = "2\n";
    poll_out(poll_data);
    k_msleep(3000);

    poll_data = "AT+CIPSEND=2\r\n";
    poll_out(poll_data);
    k_msleep(3000);

    poll_data = "33\n";
    poll_out(poll_data);
    k_msleep(3000);

    poll_data = "AT+CIPSEND=1\r\n";
    poll_out(poll_data);
    k_msleep(3000);

    poll_data = "@\n";
    poll_out(poll_data);
    k_msleep(3000);

// poll_data = "AT+CIPMUX=1\r\n";
// poll_out(poll_data);
// k_msleep(3000);
// poll_data = "AT+CIPSERVER=1,80\r\n";
// poll_out(poll_data);
=======
    fifo_read_init(); 

    char *poll_data = "AT+RESTORE\r\n";
    poll_out(poll_data);
    k_msleep(3000);

    poll_data = "AT+CWMODE=3\r\n";
    poll_out(poll_data);
    k_msleep(3000);

    poll_data = "AT+CWJAP=\"TP_LINK\",\"65223246\"\r\n";
    poll_out(poll_data);
    k_msleep(10000);

    poll_data = "AT+CIFSR\r\n";
    poll_out(poll_data);
    k_msleep(5000);
   
    poll_data = "AT+CIPSTART=\"TCP\",\"192.168.1.104\",5005\r\n";
    poll_out(poll_data);
    k_msleep(3000);

    fifo_read();

    // poll_data = "AT+CIPSEND=1\r\n";
    // poll_out(poll_data);
    // k_msleep(3000);

    // poll_data = "C\n";
    // poll_out(poll_data);
    // k_msleep(3000);

    // poll_data = "AT+CIPSEND=2\r\n";
    // poll_out(poll_data);
    // k_msleep(3000);

    // poll_data = "33\n";
    // poll_out(poll_data);
    // k_msleep(3000);

    // poll_data = "AT+CIPSEND=1\r\n";
    // poll_out(poll_data);
    // k_msleep(3000);

    // poll_data = "@\n";
    // poll_out(poll_data);
    // k_msleep(3000);

>>>>>>> mts-srs
}