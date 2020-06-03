#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc_ulp.h"

    .data

    .global adc_1_4r  // r - reminder from devision
adc_1_4r:
    .long 0
    .global adc_1_4q  // q - quotient from devision
adc_1_4q:
    .long 0

    .global adc_1_7r
adc_1_7r:
    .long 0
    .global adc_1_7q
adc_1_7q:
    .long 0
    
    .global adc_2_4r
adc_2_4r:
    .long 0
    .global adc_2_4q
adc_2_4q:
    .long 0
    
    .global adc_2_6r
adc_2_6r:
    .long 0
    .global adc_2_6q
adc_2_6q:
    .long 0
	
    .global analog_measurements_taken
analog_measurements_taken:
    .long 1
	
    .global hibernation_flag
hibernation_flag:
    .long 1
	
    .global reset_flag
reset_flag:
    .long 1

    // ADC1 channel 4, GPIO 32
    .set adc_channel_1_4, 4
    // ADC1 channel 7, GPIO 35
    .set adc_channel_1_7, 7
    // ADC2 channel 4, GPIO 13
    .set adc_channel_2_4, 4
    // ADC2 channel 6, GPIO 14
    .set adc_channel_2_6, 6
    
/* Code goes into .text section */
    .text
    .global entry

entry:

  move r1, hibernation_flag         // put address to r1
  ld r0, r1, 0                      // load hibernation_flag value to r0
  jumpr prog_start, 14, ge          // if r0 >= 14 begin program  (these 2 line actually represent "if r0 == 13 else")
  jumpr hibernation_loop, 13, ge    // if r0 >= 13 begin loop

prog_start:
  WRITE_RTC_REG(RTC_GPIO_OUT_REG, RTC_GPIO_OUT_DATA_S + 8, 1, 1) // rtc_gpio_8 is output
  WRITE_RTC_REG(RTC_GPIO_ENABLE_W1TC_REG, RTC_GPIO_ENABLE_W1TC_S + 8, 1, 1) // set rtc_gpio_8 LOW

  move  r0, 450            // r0 = 450
  move  r1, 1000           // r1 = 1000
decr1:                     // wait 1 hour = 450 * 1000 times per 64000 clock ticks
  wait  64000              // wait 64000 clock ticks at 8MHz -> 8ms
  sub   r1, r1, 1          // decrement r1
  jump  decr0, eq          // if r0 is zero then decrease r1
  jump  decr1              // else continue to wait
decr0:
  sub   r0, r0, 1          // decrement r0
  jump  start_measure, eq  // if r0 is zero then start main program
  move  r1, 1000           // else r1 = 1000
  jump  decr1              // and continue countdown

start_measure:             // begin analog sensors measurement

  // set rtc_gpio_8 HIGH to lock the sensors key
  WRITE_RTC_REG(RTC_GPIO_ENABLE_W1TS_REG, RTC_GPIO_ENABLE_W1TS_S + 8, 1, 1)
  move  r0, 900            // wait in ms
turn_sensors:              // wait for sensors to power on
  wait  8000               // wait 8000 clock ticks at 8MHz -> 1ms
  sub   r0, r0, 1          // decrement ms count
  jump  sensors_on, eq     // if ms count is zero then quit
  jump  turn_sensors       // else continue to wait

sensors_on:
  stage_rst                // stage counter = 0
  move r3, 0               // r0 = 0
measure1:
  adc r1, 0, adc_channel_1_4 + 1   // humidity sensor

  move  r0, 64             // wait in ms
delay1:
  wait  8000               // wait 8000 clock ticks at 8MHz -> 1ms
  sub   r0, r0, 1          // decrement ms count
  jump  after_delay1, eq   // if ms count is zero then quit
  jump  delay1             // else continue to wait

after_delay1:
  add r3, r3, r1           // (r3)sum += result
  stage_inc 1              // stage counter += 1
  jumps measure1, 16, LT   // if stage counter < 16 measure again
  move r2, 0               // quotient = 0
division1:
  sub r1, r3, 16           // r1 = (r3)dividend - (16)divisor
  jump finish_div1, ov     // if dividend < divisor (overflow flag was set)
  move r3, r1              // (r3)dividend = (r1)dividend - (16)divisor
  add r2, r2, 1            // (r2)quotient = quotient + 1
  jump division1           // cycle
finish_div1:
  move r0, adc_1_4r        // put address to r0
  st r3, r0, 0             // store final reminder to mem
  move r0, adc_1_4q        // put address to r0
  st r2, r0, 0             // store final quotient to mem

  stage_rst                // stage counter = 0
  move r3, 0
measure2:
  adc r1, 0, adc_channel_1_7 + 1   // fertility sensor

  move  r0, 64             // wait in ms
delay2:
  wait  8000               // wait 8000 clock ticks at 8MHz -> 1ms
  sub   r0, r0, 1          // decrement ms count
  jump  after_delay2, eq   // if ms count is zero then quit
  jump  delay2             // else continue to wait

after_delay2:
  add r3, r3, r1           // (r3)sum += result
  stage_inc 1              // stage counter += 1
  jumps measure2, 16, LT   // if stage counter < 16 measure again
  move r2, 0               // quotient = 0
division2:
  sub r1, r3, 16           // r1 = (r3)dividend - (16)divisor
  jump finish_div2, ov     // if dividend < divisor (overflow flag was set)
  move r3, r1              // (r3)dividend = (r1)dividend - (16)divisor
  add r2, r2, 1            // (r2)quotient = quotient + 1
  jump division2           // cycle
finish_div2:
  move r0, adc_1_7r        // put address to r0
  st r3, r0, 0             // store final reminder to mem
  move r0, adc_1_7q        // put address to r0
  st r2, r0, 0             // store final quotient to mem

  stage_rst                // stage counter = 0
  move r3, 0
measure3:
  adc r1, 1, adc_channel_2_4 + 1   // water level sensor

  move  r0, 64             // wait in ms
delay3:
  wait  8000               // wait 8000 clock ticks at 8MHz -> 1ms
  sub   r0, r0, 1          // decrement ms count
  jump  after_delay3, eq   // if ms count is zero then quit
  jump  delay3             // else continue to wait

after_delay3:

  move r2, 4095
  sub r1, r2, r1           // r1 = not(r1)
  add r3, r3, r1           // (r3)sum += result
  stage_inc 1              // stage counter += 1
  jumps measure3, 16, LT   // if stage counter < 16 measure again
  move r2, 0               // quotient = 0
division3:
  sub r1, r3, 16           // r1 = (r3)dividend - (16)divisor
  jump finish_div3, ov     // if dividend < divisor (overflow flag was set)
  move r3, r1              // (r3)dividend = (r1)dividend - (16)divisor
  add r2, r2, 1            // (r2)quotient = quotient + 1
  jump division3           // cycle
finish_div3:
  move r0, adc_2_4r        // put address to r0
  st r3, r0, 0             // store final reminder to mem
  move r0, adc_2_4q        // put address to r0
  st r2, r0, 0             // store final quotient to mem

  stage_rst                // stage counter = 0
  move r3, 0
measure4:
  adc r1, 1, adc_channel_2_6 + 1   // battery charge sensor

  move  r0, 64             // wait in ms
delay4:
  wait  8000               // wait 8000 clock ticks at 8MHz -> 1ms
  sub   r0, r0, 1          // decrement ms count
  jump  after_delay4, eq   // if ms count is zero then quit
  jump  delay4             // else continue to wait

after_delay4:

  move r2, 4095
  sub r1, r2, r1           // r1 = not(r1)
  add r3, r3, r1           // (r3)sum += result
  stage_inc 1              // stage counter += 1
  jumps measure4, 16, LT   // if stage counter < 16 measure again
  move r2, 0               // quotient = 0
division4:
  sub r1, r3, 16           // r1 = (r3)dividend - (16)divisor
  jump finish_div4, ov     // if dividend < divisor (overflow flag was set)
  move r3, r1              // (r3)dividend = (r1)dividend - (16)divisor
  add r2, r2, 1            // (r2)quotient = quotient + 1
  jump division4           // cycle
finish_div4:
  move r0, adc_2_6r        // put address to r0
  st r3, r0, 0             // store final reminder to mem
  move r0, adc_2_6q        // put address to r0
  st r2, r0, 0             // store final quotient to mem
  
  // set rtc_gpio_8 LOW to unlock the sensors key
  WRITE_RTC_REG(RTC_GPIO_ENABLE_W1TC_REG, RTC_GPIO_ENABLE_W1TC_S + 8, 1, 1)
  move r0, analog_measurements_taken        // put address to r0
  move r1, 2905            // set flag
  st r1, r0, 0             // save flag to mem

  jump wake_up

wake_up:
  // check if the SoC can be woken up
  READ_RTC_FIELD(RTC_CNTL_LOW_POWER_ST_REG, RTC_CNTL_RDY_FOR_WAKEUP)
  and r0, r0, 1
  jump wake_up, eq         // retry until the bit is set
  // wake up the SoC and stop ULP program
  wake
  // stop the wakeup timer so it does not restart ULP
  WRITE_RTC_FIELD(RTC_CNTL_STATE0_REG, RTC_CNTL_ULP_CP_SLP_TIMER_EN, 0)
  halt

hibernation_loop:           // do nothing in hibernation mode (device is off)
   jump hibernation_loop    // infinite loop