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


#define BUFFER_SIZE 16
uint8_t  serbuf[BUFFER_SIZE];
struct ring myring;
uint8_t x;
uint32_t ret;

int main(void)
{
    while (1) {
      ring_init( &myring, serbuf, BUFFER_SIZE );
      ring_write_ch( &myring, 'A');
      ring_write_ch( &myring, 'B');
      ring_write_ch( &myring, 'C');
      ret=ring_read_ch( &myring, &x);
    }

}

