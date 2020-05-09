#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>
#include <sys/printk.h>
#include <drivers/pwm.h>
#include <kernel.h>
 
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
 
#define TRANSCEIVER_PORT "GPIOB"
#define CE_PIN 0
#define SPI_SCK_PIN 3
#define MOSI_PIN 5
#define MISO_PIN 4
#define IRQ_PIN 1
 
#define FLAGS 0
 
#define DT_ALIAS_PWM_2_LABEL "PWM_2"
#define DT_ALIAS_PWM_3_LABEL "PWM_3"
 
#ifndef DT_ALIAS_PWM_2_LABEL
#error "PWM_2 device label not defined"
#endif
 
#ifndef DT_ALIAS_PWM_3_LABEL
#error "PWM_3 device label not defined"
#endif
 
/* period of servo motor signal ->  2.4ms */
#define PERIOD (USEC_PER_SEC / 490U)
 
/* all in micro second */
#define STEP 1
//#define MINPULSEWIDTH 50 /* Servo 0 degrees */
//#define MIDDLEPULSEWIDTH 1500 /* Servo 90 degrees */
//#define MAXPULSEWIDTH 255 /* Servo 180 degrees */
 
#define STACKSIZE 1024
 
struct device *gpio_engine_dev;
struct device *gpio_sonic_sensor_dev;
struct device *gpio_transceiver_dev;
struct device *pwm2_dev, *pwm3_dev;
int left_dist, front_dist, right_dist;
int *distance = NULL;

uint8_t gpio_init(void){
    uint8_t ret = 0;
    gpio_engine_dev = device_get_binding(ENGINE_PORT);
    gpio_sonic_sensor_dev = device_get_binding(SONIC_SENSOR_PORT);
    gpio_transceiver_dev = device_get_binding(TRANSCEIVER_PORT);
    if (gpio_engine_dev == NULL || gpio_sonic_sensor_dev == NULL || gpio_transceiver_dev == NULL) {
        return 1;
    }
 
    /* Engines IN1 - IN4 pins cofiguration */
    gpio_pin_configure(gpio_engine_dev, IN2_PIN, GPIO_OUTPUT_ACTIVE   | FLAGS);
    gpio_pin_configure(gpio_engine_dev, IN1_PIN, GPIO_OUTPUT_INACTIVE | FLAGS);
    gpio_pin_configure(gpio_engine_dev, IN4_PIN, GPIO_OUTPUT_ACTIVE   | FLAGS);
    gpio_pin_configure(gpio_engine_dev, IN3_PIN, GPIO_OUTPUT_INACTIVE | FLAGS);
 
    /* Sonic sensors ECHO and TRIG pins cofiguration */
    gpio_pin_configure(gpio_sonic_sensor_dev, ECHO1_PIN, GPIO_INPUT | FLAGS);
    gpio_pin_configure(gpio_sonic_sensor_dev, ECHO2_PIN, GPIO_INPUT | FLAGS);
    gpio_pin_configure(gpio_sonic_sensor_dev, ECHO3_PIN, GPIO_INPUT | FLAGS);
    gpio_pin_configure(gpio_sonic_sensor_dev, TRIG1_PIN, GPIO_OUTPUT_INACTIVE | FLAGS);
    gpio_pin_configure(gpio_sonic_sensor_dev, TRIG2_PIN, GPIO_OUTPUT_INACTIVE | FLAGS);
    gpio_pin_configure(gpio_sonic_sensor_dev, TRIG3_PIN, GPIO_OUTPUT_INACTIVE | FLAGS);
 
    /* Wireless radio component pin configuration */
    gpio_pin_configure(gpio_transceiver_dev, CE_PIN,   GPIO_OUTPUT_ACTIVE | FLAGS);
    gpio_pin_configure(gpio_transceiver_dev, MOSI_PIN, GPIO_OUTPUT_ACTIVE | FLAGS);
    gpio_pin_configure(gpio_transceiver_dev, MISO_PIN, GPIO_OUTPUT_ACTIVE | FLAGS);
    gpio_pin_configure(gpio_transceiver_dev, IRQ_PIN,  GPIO_OUTPUT_ACTIVE | FLAGS);
 
    return ret;
}
 
uint8_t pwm_init(void){
    uint8_t ret = 0;
    pwm2_dev = device_get_binding(DT_ALIAS_PWM_2_LABEL);
    pwm3_dev = device_get_binding(DT_ALIAS_PWM_3_LABEL);
    return ret;
}
 
void print1(void){
    while(1){
        printk("It's thread 1./n");
        k_msleep(1000);
    }
}
 
void print2(void){
    while(1){
        printk("It's thread 2, should appear twice per every thread 1./n");
        k_msleep(500);
    }
}
 
/* Returns distance measured by sensor with given id. */
void get_distance(int id){
    while(1){
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
 
        gpio_pin_set(gpio_sonic_sensor_dev, trig_pin, true);
        k_sleep(K_USEC(11));
        gpio_pin_set(gpio_sonic_sensor_dev, trig_pin, false);
 
        u32_t start_time;
        u32_t stop_time;
        u32_t cycles_spent;
        u32_t usec_spent;

        while(!gpio_pin_get(gpio_sonic_sensor_dev, echo_pin));
        start_time = k_cycle_get_32();
        while(gpio_pin_get(gpio_sonic_sensor_dev, echo_pin));
        stop_time = k_cycle_get_32();
        cycles_spent = stop_time - start_time;
        usec_spent = SYS_CLOCK_HW_CYCLES_TO_NS(cycles_spent);
        *distance = usec_spent / 58000;
        printk("Sensor %d: %d cm\n", id, *distance);
        if(id == 3){
            printk("\n\n");
        }
        k_msleep(10);
        if(*distance > 1200) *distance = 0; //Distance higher than 1200 means object is closer than 2cm from sensor.
    }
}
 
void engine_test(){
    while(1){
        pwm_pin_set_usec(pwm2_dev, 1, PERIOD, 1000, 0);
        pwm_pin_set_usec(pwm2_dev, 2, PERIOD, 1000, 0);
    
        if(front_dist < 20){
            /* Stop the car */
            gpio_pin_set(gpio_engine_dev, IN1_PIN, true);
            gpio_pin_set(gpio_engine_dev, IN2_PIN, true);
            gpio_pin_set(gpio_engine_dev, IN3_PIN, true);
            gpio_pin_set(gpio_engine_dev, IN4_PIN, true);
            pwm_pin_set_usec(pwm2_dev, 1, PERIOD, 2000, 0);
            pwm_pin_set_usec(pwm2_dev, 2, PERIOD, 2000, 0);
            k_msleep(100);
    
            /* Turn right */
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
            k_msleep(500);

            /* Move forward */
            gpio_pin_set(gpio_engine_dev, IN2_PIN, true);
            gpio_pin_set(gpio_engine_dev, IN1_PIN, false);
            gpio_pin_set(gpio_engine_dev, IN4_PIN, true);
            gpio_pin_set(gpio_engine_dev, IN3_PIN, false);
        }
    }
}
 
K_THREAD_DEFINE(sensor1_th_id, STACKSIZE, get_distance, 1, NULL, NULL, 4, 0, 0);
K_THREAD_DEFINE(sensor2_th_id, STACKSIZE, get_distance, 2, NULL, NULL, 5, 0, 0);
K_THREAD_DEFINE(sensor3_th_id, STACKSIZE, get_distance, 3, NULL, NULL, 6, 0, 0);
K_THREAD_DEFINE(engines_id, STACKSIZE, engine_test, NULL, NULL, NULL, 7, 0, 0);
 
void main(void){
    if(gpio_init()) printk("GPIO init failed.\n");
    if(pwm_init())  printk("PWM init failed.\n");
}
 
// void main(void)
// {
//  gpio_init();
//  pwm_init();
//  int fill = 500;
   
//  // while (1) {
//  //  pwm_pin_set_usec(pwm2_dev, 1, PERIOD, fill, 0);
//  //  pwm_pin_set_usec(pwm2_dev, 2, PERIOD, fill, 0);
//  //  if (fill < 2040) {
//  //      fill += STEP;
//  //  } else {
//  //      pwm_pin_set_usec(pwm2_dev, 1, PERIOD, 0, 0);
//  //      k_sleep(K_MSEC(800));
//  //      fill = 300;
//  //  }
//  //  k_sleep(K_MSEC(10));
//  // }
//  u32_t start_time;
//  u32_t stop_time;
//  u32_t cycles_spent;
//  u32_t usec_spent;
//  float distance;
 
//  while(1){
//      gpio_pin_set(gpio_sonic_sensor_dev, TRIG1_PIN, GPIO_OUTPUT_ACTIVE);
//      k_sleep(K_USEC(11));
//      gpio_pin_set(gpio_sonic_sensor_dev, TRIG1_PIN, GPIO_OUTPUT_INACTIVE);
 
 
//      start_time = k_cycle_get_32();
//      while(gpio_pin_get(gpio_sonic_sensor_dev, ECHO1_PIN)){}
//      stop_time = k_cycle_get_32();
//      cycles_spent = stop_time - start_time;
//      usec_spent = SYS_CLOCK_HW_CYCLES_TO_US(cycles_spent);
//      distance = usec_spent / 58.;
//      printk("%f",distance);
//  }
// }