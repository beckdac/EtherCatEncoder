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
        timer_counter_value_config(TIMER1, 0);
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

   int64_t pos = unwrap_encoder(timer_counter_read(TIMER1), &previousEncoderCounterValue);
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

//    exti_interrupt_enable(exti_line);

void gpio_init(void) {
	rcu_periph_clock_enable(RCU_GPIOA);
	rcu_periph_clock_enable(RCU_GPIOB);
	rcu_periph_clock_enable(RCU_GPIOC);
	rcu_periph_clock_enable(RCU_GPIOE);
	rcu_periph_clock_enable(RCU_GPIOG);

	// the switches on the GD32 board
	//gpio_mode_set(GPIOE, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_0 | GPIO_PIN_1);
    // 3 of the four below are used for the encoder, see below
	//gpio_mode_set(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_8 | GPIO_PIN_9);
	//gpio_mode_set(GPIOG, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_14 | GPIO_PIN_15);

	// the LEDs on the GD32 board
	gpio_mode_set(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);
	gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);
	gpio_mode_set(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6);
	gpio_output_options_set(GPIOE, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6);
    // turn these off
    gpio_bit_set(GPIOC, GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);
    gpio_bit_set(GPIOE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6);

    rcu_periph_clock_enable(RCU_SYSCFG);
    // LAN9252 IRQ pin will have EXTI0
    gpio_mode_set(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_0);
    nvic_irq_enable(EXTI0_IRQn, 0U, 0U);
    syscfg_exti_line_config(EXTI_SOURCE_GPIOC, EXTI_SOURCE_PIN0);
    exti_init(EXTI_0, EXTI_INTERRUPT, EXTI_TRIG_FALLING);
    exti_interrupt_flag_clear(EXTI_0);
    exti_interrupt_enable(EXTI_0);
    // LAN9252 SYNC0 pin will have EXTI3
    gpio_mode_set(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_3);
    nvic_irq_enable(EXTI3_IRQn, 1U, 1U);
    syscfg_exti_line_config(EXTI_SOURCE_GPIOC, EXTI_SOURCE_PIN3);
    exti_init(EXTI_3, EXTI_INTERRUPT, EXTI_TRIG_FALLING);
    exti_interrupt_flag_clear(EXTI_3);
    exti_interrupt_enable(EXTI_3);
    // LAN9252 SYNC1 pin will have EXTI1
    gpio_mode_set(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_1);
    nvic_irq_enable(EXTI1_IRQn, 1U, 2U);
    syscfg_exti_line_config(EXTI_SOURCE_GPIOC, EXTI_SOURCE_PIN1);
    exti_init(EXTI_1, EXTI_INTERRUPT, EXTI_TRIG_FALLING);
    exti_interrupt_flag_clear(EXTI_1);
    exti_interrupt_enable(EXTI_1);

    // the encoder will live on portb 6, 8 and 9; 9 and 8 are using timer1
    // and 6 will use an IRQ and needs to be setup here
    gpio_mode_set(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_6 | GPIO_PIN_8 | GPIO_PIN_9);
    // interrupt on 6 for index
    nvic_irq_enable(EXTI5_9_IRQn, 2U, 0U);
    syscfg_exti_line_config(EXTI_SOURCE_GPIOB, EXTI_SOURCE_PIN6);
    exti_init(EXTI_6, EXTI_INTERRUPT, EXTI_TRIG_RISING);
    exti_interrupt_flag_clear(EXTI_6);
    exti_interrupt_enable(EXTI_6);
    // setup timer alternate functions for 8 and 9 (ch0 and ch1 on timer1)
    // for encoder
    gpio_af_set(GPIOB, GPIO_AF_1, GPIO_PIN_8 | GPIO_PIN_9);
}

// the encoder will sit on the pins b8 and b9 which are
// switches on the lan9252 gd32 dev board and timer1 ch0 and ch1
void timer1_init(void) {
    timer_parameter_struct timer_init_param;

    timer_deinit(TIMER1);

    rcu_periph_clock_enable(RCU_TIMER1);

    timer_init_param.prescaler = 0;
    timer_init_param.alignedmode = TIMER_COUNTER_EDGE;
    timer_init_param.counterdirection = TIMER_COUNTER_UP;
    timer_init_param.period = 65535;
    timer_init_param.clockdivision = TIMER_CKDIV_DIV1;
    timer_init_param.repetitioncounter = 0;
    timer_init(TIMER1, &timer_init_param);              /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER1);

    timer_slave_mode_select(TIMER1, TIMER_ENCODER_MODE2);

    //timer_interrupt_enable(TIMER1, TIMER_INT_UP);
    //nvic_irq_enable(TIMER1_IRQn, 3, 1);
    timer_enable(TIMER1);
}

// LAN9252 IRQ
void EXTI0_IRQHandler(void){
    //PDI_Isr();
    exti_interrupt_flag_clear(EXTI_0);
}

// LAN9252 SYNC1
void EXTI1_IRQHandler(void) {
    NVIC_DisableIRQ(EXTI0_IRQn);
    //Sync1_Isr();
    exti_interrupt_flag_clear(EXTI_1);
    NVIC_EnableIRQ(EXTI0_IRQn);
}

// LAN9252 SYNC0
void EXTI3_IRQHandler(void) {
    //Sync0_Isr();
    exti_interrupt_flag_clear(EXTI_3);
}

// index Pulse, encoder A pulse, and encoder B pulse
// all sit on this ISR
void EXTI5_9_IRQHandler(void) {
    gpio_bit_toggle(GPIOE, GPIO_PIN_3);
    // Switch 5 (GPIO 6 on Port B) is index
    if (exti_interrupt_flag_get(EXTI_6) != RESET) {
        encoderBPulse(); 
        exti_interrupt_flag_clear(EXTI_6);
    }
}

void watchdog_config(void) {
    fwdgt_config(0x0FFF, FWDGT_PSC_DIV32);
    // the above prescale results in a pretty generous timer 
    // so we can enable here and it won't time out before 
    // the first reload in the main loop
    fwdgt_enable();
}

int main(void) {
	systick_config();
    watchdog_config();

	gpio_init();
	gpio_bit_toggle(GPIOC, GPIO_PIN_15);

    timer1_init();
	gpio_bit_toggle(GPIOC, GPIO_PIN_14);

	ecat_slv_init(&config);
    gpio_bit_toggle(GPIOC, GPIO_PIN_13);

    uint32_t i = 0;
	while (1) {
        if (++i % 100 == 0) {
	        gpio_bit_toggle(GPIOE, GPIO_PIN_6); 
        }
        ESCvar.PrevTime = ESCvar.Time;
		ecat_slv();
		if (ESCvar.ALerror) {
			//printf("\rAL Error %d\r\n", ESCvar.ALerror);
		}
        fwdgt_counter_reload();
	}
}

