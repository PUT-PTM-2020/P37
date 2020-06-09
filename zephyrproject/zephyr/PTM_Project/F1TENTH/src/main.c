#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>
#include <sys/printk.h>
#include <drivers/pwm.h>
#include <drivers/uart.h>
#include <kernel.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

 
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
 
#define LEDS_PORT "GPIOD"
#define ORANGE_LED 13
#define GREEN_LED 12
#define RED_LED 14
#define BLUE_LED 15
#define FLAGS 0
 
#define DT_ALIAS_PWM_2_LABEL "PWM_2"
#define DT_ALIAS_PWM_3_LABEL "PWM_3" 
#define DT_ALIAS_UART_1_LABEL "UART_1"
 
/* period of servo motor signal ->  2.4ms */
#define PERIOD (USEC_PER_SEC / 490U)
 
#define STACKSIZE 1024

const k_tid_t main_th_id;
k_tid_t get_distance_id;
bool isGetDistanceAlive = false;

const int velocity = 600;

K_SEM_DEFINE(irq_sem, 1, 1);
K_SEM_DEFINE(test_sem, 1, 1);

struct device *gpio_engine_dev, *gpio_sonic_sensor_dev, *gpio_leds_dev;
struct device *pwm2_dev, *pwm3_dev;
struct device *uart1_dev;
int left_dist = 100, front_dist = 100, right_dist = 100;
int *distance = NULL;
int stopping_counter = 0; 
uint8_t recv_data = '\0';
 
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
    gpio_leds_dev = device_get_binding(LEDS_PORT);
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
        
        k_sched_lock();

        for(int i = 0; i < size; i++){
            uint32_t start_time;
            uint32_t stop_time;
            uint32_t cycles_spent;
            uint32_t nsec_spent = 0;

            int val;
            gpio_pin_set(gpio_sonic_sensor_dev, trig_pin, true);
            start_time = k_cycle_get_32();
            while(nsec_spent <= 11000){
                stop_time = k_cycle_get_32();
                cycles_spent = stop_time - start_time;
                nsec_spent = k_cyc_to_ns_ceil32(cycles_spent);
            }

            gpio_pin_set(gpio_sonic_sensor_dev, trig_pin, false);
            while(!gpio_pin_get(gpio_sonic_sensor_dev, echo_pin));
            start_time = k_cycle_get_32();
            while(gpio_pin_get(gpio_sonic_sensor_dev, echo_pin));
            stop_time = k_cycle_get_32();
            cycles_spent = stop_time - start_time;
            nsec_spent = k_cyc_to_ns_ceil32(cycles_spent);
            val = nsec_spent / 58000;
            data[i] = val;
        }
 
        insertionSort(data, size);
        *distance = data[5]; //distance = median of 11 measures
        if(*distance > 400) *distance = 0; //Distance higher than 1200 means object is closer than 2cm from sensor.
        k_sched_unlock();

}
 
void get_distance_printk(){
    while(isGetDistanceAlive){
        get_distance(1);
        get_distance(2);
        get_distance(3);
        // printk("Sensor left: %d cm\n", left_dist);
        // printk("Sensor front: %d cm\n", front_dist);
        // printk("Sensor right: %d cm\n\n", right_dist);
    }
}

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
 
void move_forward() {
    /* Move forward */
    pwm_pin_set_usec(pwm2_dev, 1, PERIOD, velocity, 0);
    pwm_pin_set_usec(pwm2_dev, 2, PERIOD, velocity, 0);
    gpio_pin_set(gpio_engine_dev, IN1_PIN, false);
    gpio_pin_set(gpio_engine_dev, IN2_PIN, true);
    gpio_pin_set(gpio_engine_dev, IN3_PIN, true);
    gpio_pin_set(gpio_engine_dev, IN4_PIN, false);
    //printk("Moving forward!\n");
}
void move_backwards() {
    /* Move backward */
    pwm_pin_set_usec(pwm2_dev, 1, PERIOD, velocity, 0);
    pwm_pin_set_usec(pwm2_dev, 2, PERIOD, velocity, 0);
    gpio_pin_set(gpio_engine_dev, IN1_PIN, true);
    gpio_pin_set(gpio_engine_dev, IN2_PIN, false);
    gpio_pin_set(gpio_engine_dev, IN3_PIN, false);
    gpio_pin_set(gpio_engine_dev, IN4_PIN, true);
    //printk("Moving backwards!\n");
}
void stop_vehicle() {
    /* Stop the car */
    gpio_pin_set(gpio_engine_dev, IN1_PIN, true);
    gpio_pin_set(gpio_engine_dev, IN2_PIN, true);
    gpio_pin_set(gpio_engine_dev, IN3_PIN, true);
    gpio_pin_set(gpio_engine_dev, IN4_PIN, true);
    pwm_pin_set_usec(pwm2_dev, 1, PERIOD, 2000, 0);
    pwm_pin_set_usec(pwm2_dev, 2, PERIOD, 2000, 0);
}
void turn_right(const int time) {
    pwm_pin_set_usec(pwm2_dev, 1, PERIOD, velocity, 0);
    pwm_pin_set_usec(pwm2_dev, 2, PERIOD, velocity, 0);
    gpio_pin_set(gpio_engine_dev, IN1_PIN, false);
    gpio_pin_set(gpio_engine_dev, IN2_PIN, true);
    gpio_pin_set(gpio_engine_dev, IN3_PIN, false);
    gpio_pin_set(gpio_engine_dev, IN4_PIN, true);
    if(time) k_msleep(time);
    //printk("Turning right!\n");
}
void turn_left(const int time) {
    pwm_pin_set_usec(pwm2_dev, 1, PERIOD, velocity, 0);
    pwm_pin_set_usec(pwm2_dev, 2, PERIOD, velocity, 0);
    gpio_pin_set(gpio_engine_dev, IN1_PIN, true);
    gpio_pin_set(gpio_engine_dev, IN2_PIN, false);
    gpio_pin_set(gpio_engine_dev, IN3_PIN, true);
    gpio_pin_set(gpio_engine_dev, IN4_PIN, false);
    if(time) k_msleep(time);
    //printk("Turning left!\n");
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
    printk("Autonomous mode on\n\n");
    int dir = 0;
    isGetDistanceAlive = true;
    get_distance_id = k_thread_create(&get_distance_th, my_stack, STACKSIZE, get_distance_printk, NULL, NULL, NULL, 5, 0, K_NO_WAIT);
    while(recv_data != 'M'){
        move_forward();
        if(front_dist < 20){
            choose_direction(&dir);
            if (dir >= 1000)
            {
                stop_vehicle();
            }
            else {
                move_forward();
            }
        }
        k_msleep(10);
    }
    isGetDistanceAlive = false;
    printk("Autonomous mode off\n\n");
}

void manual_mode(){
    printk("manual modeon\n\n");
    while(recv_data != 'X' && recv_data != 'q'){
        if(recv_data == 'R'){
            turn_right(0);
            stopping_counter = 0;
        }
        else if(recv_data == 'L'){
            turn_left(0);
            stopping_counter = 0;
        }
        else if(recv_data == 'F'){
            move_forward();
            stopping_counter = 0;
        }
        else if(recv_data == 'B'){
            move_backwards();
            stopping_counter = 0;
        }
        else if(recv_data == 'A'){
            autonomous_mode();
            stopping_counter = 0;
        }
        else if(stopping_counter < 25){
            stopping_counter++;
        }
        else stop_vehicle();
        recv_data = '\0';
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

    /* Put all in-boards user leds out */
    gpio_pin_set(gpio_leds_dev, ORANGE_LED, false);
    k_msleep(500);
    gpio_pin_set(gpio_leds_dev, GREEN_LED, false);
    k_msleep(500);
    gpio_pin_set(gpio_leds_dev, BLUE_LED, false);
    k_msleep(500);
    gpio_pin_set(gpio_leds_dev, RED_LED, false);
    k_msleep(500);
    recv_data = '\0';
}
 
void poll_out(char poll_data[]){
    for (int i = 0; i < strlen(poll_data); i++) {
        uart_poll_out(uart1_dev, poll_data[i]);
    }
}

int esp_recv_counter = 0;

void uart_fifo_callback_init(struct device *dev)
{
	uint8_t recv_char;
    
	if (!uart_irq_update(dev)) {
		printk("retval should always be 1\n");
		return;
	}

	if (uart_irq_rx_ready(dev)) {
		uart_fifo_read(dev, &recv_char, 1);
		printk("%c", recv_char);
	}
}

void uart_fifo_callback(struct device *dev)
{
    uint8_t recv_char;
    const char esp_pattern[7] = {'+', 'I', 'P', 'D', ',' ,'1' ,':'};
   
    if (!uart_irq_update(dev)) {
        printk("retval should always be 1\n");
        return;
    }
 
    if (uart_irq_rx_ready(dev)) {
        uart_fifo_read(dev, &recv_char, 1);
        if (recv_char == esp_pattern[esp_recv_counter] || esp_recv_counter == 7) {
            if (esp_recv_counter == 7){
                if(k_sem_count_get(&irq_sem) == 1){
                    k_sem_take(&irq_sem, K_NO_WAIT);
                    recv_data = recv_char;
                    esp_recv_counter = 0;
                    k_sem_give(&irq_sem);   
                }
            } else esp_recv_counter++;
        } else esp_recv_counter = 0;
    }
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
            if(recv_data == 'M'){
                manual_mode();
            }
            else if(recv_data == 'A'){
                autonomous_mode();
            }
            else if(recv_data == 'C'){
                connection_test();
            }
    }
}

K_THREAD_DEFINE(main_th_id, STACKSIZE, main_thread, NULL, NULL, NULL, 4, 0, 0);

void main(void){ 
    if(gpio_init()) printk("GPIO init failed.\n");
    if(pwm_init())  printk("PWM init failed.\n");
    if(uart_init())  printk("UART init failed.\n");
    
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
   
    poll_data = "AT+CIPSTART=\"TCP\",\"192.168.1.102\",5005\r\n";
    poll_out(poll_data);
    k_msleep(3000);

    fifo_read();
}
