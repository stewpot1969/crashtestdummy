#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <stdio.h>
#include <errno.h>


/* Do stuff with a Blue Pill.

Current setup:
    Serial port on USART3
    
    Receive chars on USART3
    Echo hex of chars out
    PB1 high=on
*/
/* ==================================================================== */
/*   RINGBUFFER                                                         */
/*
struct ring {
	uint8_t *data;
	uint32_t size;
	uint32_t begin;
	uint32_t end;
};

static void ring_init(struct ring *ring, uint8_t *buf, uint32_t size)
{
	ring->data = buf;
	ring->size = size;
	ring->begin = 0;
	ring->end = 0;
}

static int32_t ring_write_ch(struct ring *ring, uint8_t ch)
{
	if (((ring->end + 1) % ring->size) != ring->begin) {
		ring->data[ring->end++] = ch;
		ring->end %= ring->size;
		return (uint32_t)ch;
	}

	return -1;
}

static int32_t ring_write(struct ring *ring, uint8_t *data, uint32_t size)
{
	int32_t i;

	for (i = 0; i < (int32_t)size; i++) {
		if (ring_write_ch(ring, data[i]) < 0)
			return -i;
	}

	return i;
}

static int32_t ring_read_ch(struct ring *ring, uint8_t *ch)
{
	int32_t ret = -1;

	if (ring->begin != ring->end) {
		ret = ring->data[ring->begin++];
		ring->begin %= ring->size;
		if (ch)
			*ch = ret;
	}

	return ret;
}
*/
/* ==================================================================== */

static void clock_setup(void)
{
	rcc_clock_setup_in_hse_8mhz_out_72mhz();

	/* Enable GPIOC clock. */
	rcc_periph_clock_enable(RCC_GPIOB);

	/* Enable clocks for GPIO port B (for GPIO_USART3_TX) and USART3. */
	rcc_periph_clock_enable(RCC_USART3);
}
static void usart_setup(void)
{
	/* Setup GPIO pin GPIO_USART3_TX. */
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART3_TX);

	/* Setup UART parameters. */
	usart_set_baudrate(USART3, 9600);
	usart_set_databits(USART3, 8);
	usart_set_stopbits(USART3, USART_STOPBITS_1);
	usart_set_mode(USART3, USART_MODE_TX_RX);
	usart_set_parity(USART3, USART_PARITY_NONE);
	usart_set_flow_control(USART3, USART_FLOWCONTROL_NONE);

	/* Finally enable the USART. */
	usart_enable(USART3);
}

static void gpio_setup(void)
{
	/* Set GPIO1 (in GPIO port B) to 'output push-pull'. */
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO1);
}

uint8_t hexchar(uint8_t c) {
  c+=48;   /* convert 0-9 to '0'-'9' */
  if (c>'9') {
    c+=7;
  }
  return c;
}


int main(void)
{
  uint8_t x;
  
  clock_setup();
  gpio_setup();
  usart_setup();
  
  while(1) {
    x=usart_recv_blocking(USART3);
    usart_send_blocking(USART3, hexchar((x>>4) & 0x0f ));
    usart_send_blocking(USART3, hexchar(x & 0x0f ));
    usart_send_blocking(USART3, '\r');
    usart_send_blocking(USART3, '\n');
  }    
}

