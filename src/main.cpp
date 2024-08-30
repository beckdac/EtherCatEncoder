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

//---- user application ------------------------------------------------------------------------------

void indexPulse(void);
void encoderAPulse(void);
void encoderBPulse(void);
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
    if (counterReset) {
        //TIM2->CNT = 0;
        indexPulseDetected = 1;
        Pos.clear();
        TDelta.clear();
        counterReset = 0;
    }
}

void encoderAPulse(void){

}

void encoderBPulse(void){

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


#define EXTI_NUMS   (16)

typedef struct {
    IRQn_Type irqNum;
    void (*callback)(void);
} extiConf_t;

extiConf_t gpio_exti_infor[EXTI_NUMS] = {
    {EXTI0_IRQn,      NULL},
    {EXTI1_IRQn,      NULL},
    {EXTI2_IRQn,      NULL},
    {EXTI3_IRQn,      NULL},
    {EXTI4_IRQn,      NULL},
    {EXTI5_9_IRQn,    NULL},
    {EXTI5_9_IRQn,    NULL},
    {EXTI5_9_IRQn,    NULL},
    {EXTI5_9_IRQn,    NULL},
    {EXTI5_9_IRQn,    NULL},
    {EXTI10_15_IRQn,  NULL},
    {EXTI10_15_IRQn,  NULL},
    {EXTI10_15_IRQn,  NULL},
    {EXTI10_15_IRQn,  NULL},
    {EXTI10_15_IRQn,  NULL},
    {EXTI10_15_IRQn,  NULL}
};

void gpio_interrupt_enable(uint32_t portNum, uint32_t pinNum, void (*callback)(void), exti_trig_type_enum mode) {
    exti_line_enum exti_line = (exti_line_enum)BIT(pinNum);
    exti_mode_enum exti_mode = EXTI_INTERRUPT;
    exti_trig_type_enum  trig_type = mode;
    gpio_exti_infor[pinNum].callback = callback;

#define EXTI_IRQ_PRIO   3
#define EXTI_IRQ_SUBPRIO    0

    nvic_irq_enable(gpio_exti_infor[pinNum].irqNum, EXTI_IRQ_PRIO, EXTI_IRQ_SUBPRIO);
    // call the below above so we can be sure we are using the right defines, not these
    //syscfg_exti_line_config((uint8_t)portNum, (uint8_t)pinNum);

    exti_init(exti_line, exti_mode, trig_type);
    exti_interrupt_flag_clear(exti_line);
    exti_interrupt_enable(exti_line);
}

void gpio_init(void) {
	rcu_periph_clock_enable(RCU_GPIOA);
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

    // interupts for the encoders on the switches
    exti_deinit();
    syscfg_exti_line_config(EXTI_SOURCE_GPIOE, EXTI_SOURCE_PIN1);
    gpio_interrupt_enable(GPIOE, GPIO_PIN_1, &indexPulse, EXTI_TRIG_RISING);
    syscfg_exti_line_config(EXTI_SOURCE_GPIOE, EXTI_SOURCE_PIN0);
    gpio_interrupt_enable(GPIOE, GPIO_PIN_0, &encoderAPulse, EXTI_TRIG_RISING);
    syscfg_exti_line_config(EXTI_SOURCE_GPIOB, EXTI_SOURCE_PIN9);
    gpio_interrupt_enable(GPIOB, GPIO_PIN_9, &encoderBPulse, EXTI_TRIG_RISING);
    //syscfg_exti_line_config(EXTI_SOURCE_GPIOB, EXTI_SOURCE_PIN6);
    //syscfg_exti_line_config(EXTI_SOURCE_GPIOB, EXTI_SOURCE_PIN8);
    //syscfg_exti_line_config(EXTI_SOURCE_GPIOB, EXTI_SOURCE_PIN9);
    //syscfg_exti_line_config(EXTI_SOURCE_GPIOG, EXTI_SOURCE_PIN14);
    //syscfg_exti_line_config(EXTI_SOURCE_GPIOG, EXTI_SOURCE_PIN15);
    
}

void exti_callbackHandler(uint32_t pinNum)
{
    exti_line_enum linex = (exti_line_enum)BIT(pinNum);
    if (exti_interrupt_flag_get(linex) != RESET) {
        exti_interrupt_flag_clear(linex);
        if (gpio_exti_infor[pinNum].callback != NULL) {
            gpio_exti_infor[pinNum].callback();
        }
    }
}

void EXTI0_IRQHandler(void){
    exti_callbackHandler(0);
}

void EXTI1_IRQHandler(void)
{
    exti_callbackHandler(1);
}

void EXTI2_IRQHandler(void)
{
    exti_callbackHandler(2);
}

void EXTI3_IRQHandler(void)
{
    exti_callbackHandler(3);
}

void EXTI4_IRQHandler(void)
{
    exti_callbackHandler(4);
}

void EXTI5_9_IRQHandler(void)
{
    uint32_t i;
    for (i = 5; i < 10; i++) {
        exti_callbackHandler(i);
    }

}

void EXTI10_15_IRQHandler(void)
{
    uint32_t i;
    for (i = 10; i < 16; i++) {
        exti_callbackHandler(i);
    }
}


int main(void) {
	systick_config();
	gpio_init();
	gpio_bit_set(GPIOC, GPIO_PIN_15);

	ecat_slv_init(&config);
	gpio_bit_set(GPIOC, GPIO_PIN_14);

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

