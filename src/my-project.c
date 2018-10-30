#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <stdio.h>
#include <errno.h>


/* Do stuff with a Blue Pill.

Current setup:
    Serial port on USART3
*/
/* ==================================================================== */
/*   RINGBUFFER                                                         */

struct ring {
	uint8_t *data;
	uint32_t size;
	uint32_t begin;
	uint32_t end;
};

#define RING_SIZE(RING)  ((RING)->size - 1)
#define RING_DATA(RING)  (RING)->data
#define RING_EMPTY(RING) ((RING)->begin == (RING)->end)

int _write(int file, char *ptr, int len);

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

/* ==================================================================== */


#define BUFFER_SIZE 256
uint8_t  serbuf[BUFFER_SIZE];
struct ring out_ring;

static void clock_setup(void)
{
	rcc_clock_setup_in_hse_8mhz_out_72mhz();

	/* Enable GPIOB clock (for LED GPIOs). */
	rcc_periph_clock_enable(RCC_GPIOB);

	/* Enable clocks for GPIO port B (for GPIO_USART3_TX) and USART3. */
	rcc_periph_clock_enable(RCC_AFIO);
	rcc_periph_clock_enable(RCC_USART3);
}

static void usart_setup(void)
{
	/* Enable the USART3 interrupt. */
	nvic_enable_irq(NVIC_USART3_IRQ);

  /* Initialise the ring */
  ring_init(&out_ring,serbuf,BUFFER_SIZE);
  
  
	/* Setup GPIO pin GPIO_USART3_TX on GPIO port B for transmit. */
	gpio_set_mode(GPIO_BANK_USART3_TX, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART3_TX);

	/* Setup GPIO pin GPIO_USART3_RX on GPIO port B for receive. */
	gpio_set_mode(GPIO_BANK_USART3_RX, GPIO_MODE_INPUT,
		      GPIO_CNF_INPUT_FLOAT, GPIO_USART3_RX);

	/* Setup UART parameters. */
	usart_set_baudrate(USART3, 115200);
	usart_set_databits(USART3, 8);
	usart_set_stopbits(USART3, USART_STOPBITS_1);
	usart_set_parity(USART3, USART_PARITY_NONE);
	usart_set_flow_control(USART3, USART_FLOWCONTROL_NONE);
	usart_set_mode(USART3, USART_MODE_TX_RX);

	/* Enable USART2 Receive interrupt. */
	USART_CR1(USART3) |= USART_CR1_RXNEIE;

	/* Finally enable the USART. */
	usart_enable(USART3);
}

static void gpio_setup(void)
{

	/* Setup PB1 for LED use. */
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO1);
	
	gpio_clear(GPIOB, GPIO1);
}

void usart3_isr(void)
{
	/* Check if we were called because of RXNE. */
	if (((USART_CR1(USART3) & USART_CR1_RXNEIE) != 0) &&
	    ((USART_SR(USART3) & USART_SR_RXNE) != 0)) {

		/* Indicate that we got data. */
		gpio_toggle(GPIOB, GPIO1);

		/* Retrieve the data from the peripheral. */
		ring_write_ch(&out_ring, usart_recv(USART3));

		/* Enable transmit interrupt so it sends back the data. */
		USART_CR1(USART3) |= USART_CR1_TXEIE;
	}

	/* Check if we were called because of TXE. */
	if (((USART_CR1(USART3) & USART_CR1_TXEIE) != 0) &&
	    ((USART_SR(USART3) & USART_SR_TXE) != 0)) {

		/* Indicate that we are sending out data. */
		//gpio_toggle(GPIOB, GPIO1);
    int32_t data;
    data = ring_read_ch(&out_ring, NULL);
	  if (data == -1) {
			/* Disable the TXE interrupt, it's no longer needed. */
			USART_CR1(USART3) &= ~USART_CR1_TXEIE;
		} else {
			/* Put data into the transmit register. */
			usart_send(USART3, data);
		}
	}
}

int _write(int file, char *ptr, int len)
{
	int ret;

	if (file == 1) {
		ret = ring_write(&out_ring, (uint8_t *)ptr, len);

		if (ret < 0)
			ret = -ret;

		USART_CR1(USART3) |= USART_CR1_TXEIE;

		return ret;
	}

	errno = EIO;
	return -1;
}

int main(void)
{
	clock_setup();
	gpio_setup();
	usart_setup();

	/* Wait forever and do nothing. */
	while (1) {
    printf("Hello World! %i %f %f\r\n", 9678, 3.1415926,
		       2.71828);  }
	return 0;
}
