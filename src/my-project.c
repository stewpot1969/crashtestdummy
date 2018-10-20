#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

/* Do stuff with a Blue Pill.

Current setup:
    No LEDs or anything. Toggle all I/Os to see if they all work 18 hi, 19-20 lo, 12-14 lo
*/

uint32_t ctr1;

static void gpio_setup(void)
{
	/* Enable GPIOB clock. */
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);

	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO0|GPIO1|GPIO2|GPIO3|GPIO4|GPIO5|GPIO6|GPIO7|GPIO8|GPIO9|GPIO10|GPIO11|GPIO12|GPIO15);
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO0|GPIO1|GPIO2|GPIO3|GPIO4|GPIO5|GPIO6|GPIO7|GPIO10|GPIO11|GPIO12|GPIO13|GPIO14|GPIO15);
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO13|GPIO14|GPIO15);
}

void sys_tick_handler(void)
{

	if (ctr1++ == 2000) {
  	gpio_toggle(GPIOA, GPIO0|GPIO1|GPIO2|GPIO3|GPIO4|GPIO5|GPIO6|GPIO7|GPIO8|GPIO9|GPIO10|GPIO11|GPIO12|GPIO15);
  	gpio_toggle(GPIOB, GPIO0|GPIO1|GPIO2|GPIO3|GPIO4|GPIO5|GPIO6|GPIO7|GPIO10|GPIO11|GPIO12|GPIO13|GPIO14|GPIO15);
  	gpio_toggle(GPIOC, GPIO13|GPIO14|GPIO15);
		ctr1 = 0;
	}

}

int main(void) {
	rcc_clock_setup_in_hse_16mhz_out_72mhz();
	gpio_setup();

	gpio_clear(GPIOA, GPIO0|GPIO1|GPIO2|GPIO3|GPIO4|GPIO5|GPIO6|GPIO7|GPIO8|GPIO9|GPIO10|GPIO11|GPIO12|GPIO15);
	gpio_clear(GPIOB, GPIO0|GPIO1|GPIO2|GPIO3|GPIO4|GPIO5|GPIO6|GPIO7|GPIO10|GPIO11|GPIO12|GPIO13|GPIO14|GPIO15);
	gpio_clear(GPIOC, GPIO13|GPIO14|GPIO15);

	ctr1 = 0;

	/* 72MHz / 8 => 9000000 counts per second */
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);

	/* 9000000/9000 = 1000 overflows per second - every 1ms one interrupt */
	/* SysTick interrupt every N clock pulses: set reload to N-1 */
	systick_set_reload(8999);

	systick_interrupt_enable();

	/* Start counting. */
	systick_counter_enable();

	while (1); /* Halt. */

  //while (1) {
  //	gpio_toggle(GPIOA, GPIO0|GPIO1|GPIO2|GPIO3|GPIO4|GPIO5|GPIO6|GPIO7|GPIO8|GPIO9|GPIO10|GPIO11|GPIO12|GPIO15);
  //	gpio_toggle(GPIOB, GPIO0|GPIO1|GPIO2|GPIO3|GPIO4|GPIO5|GPIO6|GPIO7|GPIO10|GPIO11|GPIO12|GPIO13|GPIO14|GPIO15);
  //	gpio_toggle(GPIOC, GPIO13|GPIO14|GPIO15);
//} 
	return 0;
}
