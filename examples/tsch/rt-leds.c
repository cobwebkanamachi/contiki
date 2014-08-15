
#include "contiki.h"
#include "sys/rtimer.h"
#include "lib/random.h"

#include "dev/leds.h"

void uart0_writeb(unsigned char c);
#undef putchar
#define putchar uart0_writeb
#define DEBUG 0
#undef PUTCHAR
#if DEBUG
#define PUTCHAR(X) do { putchar(X); putchar('\n'); } while(0);
int dbg_printf(const char *fmt, ...);
#define PRINTF(...) do { dbg_printf(__VA_ARGS__); } while(0)
#else
#define PRINTF(...) do {} while(0)
#define PUTCHAR(X)
#endif /* DEBUG */
/*---------------------------------------------------------------------------*/
PROCESS(test_rtimer_process, "Test RT");
AUTOSTART_PROCESSES(&test_rtimer_process);

#define NUM_LEDS 3
unsigned char leds[] = { LEDS_GREEN, LEDS_YELLOW, LEDS_RED };

struct fade {
  struct rtimer rt;
  struct pt pt;
  int led;
  rtimer_clock_t ontime, offtime;
  int addend;
};
/*---------------------------------------------------------------------------*/
static char
fade(struct rtimer *t, void *ptr)
{
  struct fade *f = ptr;

  PT_BEGIN(&f->pt);

  while(1) {
    PUTCHAR('1');
    leds_on(leds[++(f->led) % NUM_LEDS]);
    rtimer_set(t, RTIMER_TIME(t) + f->ontime * 1000, 1,
               (rtimer_callback_t)fade, ptr);
    PT_YIELD(&f->pt);

    PUTCHAR('0');
    leds_off(leds[f->led % NUM_LEDS]);
    rtimer_set(t, RTIMER_TIME(t) + f->offtime * 1000, 1,
               (rtimer_callback_t)fade, ptr);

    f->ontime += f->addend;
    f->offtime -= f->addend;
    if(f->offtime <= 4 || f->offtime >= 100) {
      f->addend = -f->addend;
    }
    PT_YIELD(&f->pt);
  }

  PT_END(&f->pt);
}
/*---------------------------------------------------------------------------*/
static void
init_fade(struct fade *f, int led)
{
  f->led = led;
  f->addend = 4;
  f->ontime = 4;
  f->offtime = 100;
  PT_INIT(&f->pt);
  rtimer_set(&f->rt, RTIMER_NOW() + 1000 * (random_rand() & 0xf), 1, (rtimer_callback_t)fade, f);
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(test_rtimer_process, ev, data)
{
  static struct fade red;
  PROCESS_BEGIN();
  PUTCHAR('N');
  rtimer_init();

  init_fade(&red, 0);

  while(1) {
    PROCESS_WAIT_EVENT();
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
