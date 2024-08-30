#include <stdint.h>
#include <math.h>

#include "gd32f4xx.h"

#include "esc.h"

extern "C" {
#include "ecat_slv.h"
#include "utypes.h"
#include "systick.h"
}

#include <CircularBuffer.hpp>


/* CANopen Object Dictionary */
_Objects    Obj;


/* Application hook declaration */
void ecatapp();


/* SOES configuration */
static esc_cfg_t config = { 
    .user_arg = NULL,
    .use_interrupt = 0,
    .watchdog_cnt = 150,
    .set_defaults_hook = NULL,
    .pre_state_change_hook = NULL,
    .post_state_change_hook = NULL,
    .application_hook = NULL,
    .safeoutput_override = NULL,
    .pre_object_download_hook = NULL,
    .post_object_download_hook = NULL,
    .rxpdo_override = NULL,
    .txpdo_override = NULL,
    .esc_hw_interrupt_enable = NULL,
    .esc_hw_interrupt_disable = NULL,
    .esc_hw_eep_handler = NULL,
    .esc_check_dc_handler = NULL,
};

#if 0
void esc_pdi_debug()
{
    // Read few core CSR registers to verify PDI is working
    uint8_t value = 0;
    //printf("\r\n[ESC debug] ");
        
    ESC_read(0x0000, (void*) &value, sizeof(uint8_t));
    //printf("Type [0x0000]: %s, ", STR5(value));
    ESC_read(0x0001, (void*) &value, sizeof(uint8_t));
    //printf("Revision [0x0001]: %s, ", STR5(value));
    ESC_read(0x0004, (void*) &value, sizeof(uint8_t));
    //printf("FMMU count [0x0004]: %s, ", STR5(value));
    ESC_read(0x0005, (void*) &value, sizeof(uint8_t));
    //printf("Sync Managers count [0x0005]: %s, ", STR5(value));
    ESC_read(0x0006, (void*) &value, sizeof(uint8_t));
    //printf("RAM size [0x0006]: %s, \r\n", STR5(value));
    
    //printf("\r\n");
}
#endif

//---- user application ------------------------------------------------------------------------------

void indexPulse(void);
double posScaleRes = 1.0;
uint32_t curPosScale = 1;
uint8_t oldLatchCEnable = 0;
volatile uint8_t indexPulseDetected = 0;
uint32_t numIndexPulsesDetected = 0;
volatile uint8_t counterReset = 0;
uint32_t prevTime = 0, prev2Time = 0;
int64_t previousEncoderCounterValue = 0;

#define RINGBUFFERLEN 101
CircularBuffer<double, RINGBUFFERLEN> Pos;
CircularBuffer<uint32_t, RINGBUFFERLEN> TDelta;

#define ONE_PERIOD 65536
#define HALF_PERIOD 32768

int64_t unwrap_encoder(uint16_t in, int64_t *prev) {
    int64_t c64 = (int32_t)in - HALF_PERIOD; // remove half period to determine (+/-) sign of the wrap
    int64_t dif = (c64 - *prev);             // core concept: prev + (current - prev) = current
    
    // wrap difference from -HALF_PERIOD to HALF_PERIOD. modulo prevents differences after the wrap from having an incorrect result
    int64_t mod_dif = ((dif + HALF_PERIOD) % ONE_PERIOD) - HALF_PERIOD;
    if (dif < -HALF_PERIOD)
        mod_dif += ONE_PERIOD; // account for mod of negative number behavior in C
        
    int64_t unwrapped = *prev + mod_dif;
    *prev = unwrapped; // load previous value
    
    return unwrapped + HALF_PERIOD; // remove the shift we applied at the beginning, and return
}

void indexPulse(void) {
}

void cb_get_inputs() {
    Obj.IndexStatus = 0;
    if (indexPulseDetected) {
        Obj.IndexStatus = 1;
        indexPulseDetected = 0;
        numIndexPulsesDetected++;
        previousEncoderCounterValue = 0;
   }
   uint64_t now = micros();
   Obj.DiffT = now - prev2Time;
   prev2Time = prevTime;
   prevTime = now;

   //int64_t pos = unwrap_encoder(TIM2->CNT, &previousEncoderCounterValue);
   int64_t pos = unwrap_encoder(42, &previousEncoderCounterValue);
   double curPos = pos * posScaleRes;
   Obj.EncPos = curPos;

   double diffT = 0;
   double diffPos = 0;
   TDelta.push(ESCvar.Time); // Running average over the length of the circular buffer
   Pos.push(curPos);
   if (Pos.size() >= 2)
   {
      diffT = 1.0e-9 * (TDelta.last() - TDelta.first()); // Time is in nanoseconds
      diffPos = std::fabs(Pos.last() - Pos.first());
   }
   Obj.EncFrequency = diffT != 0 ? diffPos / diffT : 0.0; // Revolutions per second

   Obj.IndexByte = gpio_input_bit_get(GPIOE, GPIO_PIN_1);
}

void cb_set_outputs() {
    if (Obj.IndexLatchEnable && !oldLatchCEnable) {
        counterReset = 1;
    }
    oldLatchCEnable = Obj.IndexLatchEnable;
    
    if (curPosScale != Obj.EncPosScale && Obj.EncPosScale != 0) {
        curPosScale = Obj.EncPosScale;
        posScaleRes = 1.0 / double(curPosScale);
    }
}

void gpio_init(void) {
	rcu_periph_clock_enable(RCU_GPIOB);
	rcu_periph_clock_enable(RCU_GPIOC);
	rcu_periph_clock_enable(RCU_GPIOE);
	rcu_periph_clock_enable(RCU_GPIOG);

	// the switches on the GD32 board
	gpio_mode_set(GPIOE, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_0 | GPIO_PIN_1);
	gpio_mode_set(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_8 | GPIO_PIN_9);
	gpio_mode_set(GPIOG, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_14 | GPIO_PIN_15);
	// the LEDs on the GD32 board
	gpio_mode_set(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);
	gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);

	gpio_mode_set(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6);
	gpio_output_options_set(GPIOE, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6);
}

int main(void) {
	systick_config();
	gpio_init();
	gpio_bit_set(GPIOC, GPIO_PIN_15);

	//APP_USART_Init();
	//printf("\r\n[ESC Setup] %s \r\n", "Started");
	//
	ecat_slv_init(&config);
	gpio_bit_set(GPIOC, GPIO_PIN_14);
	//printf("\r\n[ESC Setup] Done, ready \r\n\n");
	//
//	esc_pdi_debug();

    int i = 0;
	while (1) {
        if (i++ % 2 == 1)
	        gpio_bit_reset(GPIOC, GPIO_PIN_13);
        else
	        gpio_bit_set(GPIOC, GPIO_PIN_13);
        ESCvar.PrevTime = ESCvar.Time;
		ecat_slv();
		if (ESCvar.ALerror) {
			//printf("\rAL Error %d\r\n", ESCvar.ALerror);
		}
	}
}

