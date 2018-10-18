#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

/* Do stuff with a Blue Pill.

Current setup:
  Red LED:      pin 17    (PB5 active low)
  Orange LED:   pin 15    (PB7 active low)
  On board LED:           (PB1 active high)
*/

uint32_t ctr1;
uint32_t ctr2;
uint32_t ctr3;

static void gpio_setup(void)
{
	/* Enable GPIOB clock. */
	rcc_periph_clock_enable(RCC_GPIOB);

	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO1);
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO5);
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO7);
}

void sys_tick_handler(void)
{

	if (ctr1++ == 79) {
		gpio_toggle(GPIOB, GPIO1);
		ctr1 = 0;
	}

	if (ctr2++ == 37) {
		gpio_toggle(GPIOB, GPIO5);
		ctr2 = 0;
	}

	if (ctr3++ == 53) {
		gpio_toggle(GPIOB, GPIO7);
		ctr3 = 0;
	}

}

int main(void) {
	rcc_clock_setup_in_hse_16mhz_out_72mhz();
	gpio_setup();

	gpio_set(GPIOB, GPIO1);
	gpio_clear(GPIOB, GPIO5);
	gpio_clear(GPIOB, GPIO7);

	ctr1 = ctr2 = ctr3 = 0;

	/* 72MHz / 8 => 9000000 counts per second */
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);

	/* 9000000/9000 = 1000 overflows per second - every 1ms one interrupt */
	/* SysTick interrupt every N clock pulses: set reload to N-1 */
	systick_set_reload(8999);

	systick_interrupt_enable();

	/* Start counting. */
	systick_counter_enable();

	while (1); /* Halt. */

	return 0;
}
