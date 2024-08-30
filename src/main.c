#include <stdint.h>
#include "gd32f4xx.h"

#include "esc.h"
#include "ecat_slv.h"
#include "utypes.h"


/* CANopen Object Dictionary */
_Objects    Obj;


/* Application hook declaration */
void ecatapp();


/* SOES configuration */
static esc_cfg_t config = { 
    .user_arg = "/dev/lan9252",
    .use_interrupt = 0,
    .watchdog_cnt = 500,
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


//---- user application ------------------------------------------------------------------------------

void cb_get_inputs()
{
    // write to slave TxPDO  
    // dummy value, so that sawtooth value profile will be seen constantly changing by ecat master 
    Obj.Counter++;
}

void cb_set_outputs()
{
    // read slave RxPDO  
    if (Obj.LedIn)
	{
		//STM_EVAL_LEDOn(outPins[0]);
	}
	else 
	{
		//STM_EVAL_LEDOff(outPins[0]);
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

void systick_init(void)
{
    // Setup systick timer for 1000Hz interrupts.
    if (SysTick_Config(SystemCoreClock / 1000U)){
        // infinite loop on error
        while (1) {
        }
    }
    // set the systick handler priority
    NVIC_SetPriority(SysTick_IRQn, 0x00U);
}


int main(void) {
	systick_init();
	gpio_init();
	gpio_bit_set(GPIOC, GPIO_PIN_14);
	gpio_bit_set(GPIOE, GPIO_PIN_6);

	//APP_USART_Init();
	//printf("\r\n[ESC Setup] %s \r\n", "Started");
	//
	ecat_slv_init(&config);
	//printf("\r\n[ESC Setup] Done, ready \r\n\n");
	//
	esc_pdi_debug();

	while (1) {
		ecat_slv();

		if (ESCvar.ALerror) {
			//printf("\rAL Error %d\r\n", ESCvar.ALerror);
		}
	}
}
