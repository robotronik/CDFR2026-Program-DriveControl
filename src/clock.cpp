#include "clock.h"
#include "uart.h"

#ifdef SIMULATION
    #include <hardware_interface.h>
#else
    #include <libopencm3/cm3/systick.h>
    #include <libopencm3/stm32/rcc.h>
    #include <libopencm3/cm3/nvic.h>
    #include <libopencm3/stm32/timer.h>
#endif

void clock_setup(){
  rcc_clock_setup_pll(&rcc_hsi_configs[RCC_CLOCK_3V3_84MHZ]);

  systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
  // Interrupts each millisec
  systick_set_reload(SYSTICK_PERIOD - 1);
  // clear counter so it starts right away
  systick_clear();
  systick_counter_enable();
  systick_interrupt_enable();

  //enable global iterrupts
  nvic_enable_irq(NVIC_SYSTICK_IRQ);

  rcc_periph_clock_enable(RCC_TIM2);
  //timer_set_prescaler(TIM2, 83); // 1 tick = 1 µs
  timer_set_period(TIM2, 0xFFFFFFFF);
  timer_enable_counter(TIM2);

}

volatile uint32_t systicks = 0;

void sys_tick_handler(){
  systicks++;
}


uint32_t get_uptime_ms(){
  return systicks;
}

uint32_t get_uptime_us(){
  // TODO Do no use, this is not accurate, use micros() instead
  return systicks*1000 + systick_get_value()/MICROS_SYSTICK_RATIO;
}

uint32_t micros(void) {
    return timer_get_counter(TIM2) / 84;
}

void delay_ms(uint32_t ms) {
  uint32_t count_max = systicks + ms;
  while(systicks < count_max) {}
}

// const uint32_t millisec = 1600;
// const uint32_t second = millisec*1000;
