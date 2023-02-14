#include "ARMCM0.h"
#include <stdint.h>
#include <string.h>

#ifdef __GNUC__
	#define ALIGN4_U8 _Alignas(4) uint8_t
#else
	#define ALIGN4_U8 __align(4) uint8_t
#endif

#define write_reg(addr,data)             (*(volatile unsigned int*)(addr)=(unsigned int)(data))
#define read_reg(addr)                   (*(volatile unsigned int*)(addr))
#define subWriteReg(addr,high,low,value) write_reg(addr,read_reg(addr)&\
										 (~((((unsigned int)1<<((high)-(low)+1))-1)<<(low)))|\
										 ((unsigned int)(value)<<(low)))

#define  XTAL            ( 5000000U)      /* Oscillator frequency             */
#define  SYSTEM_CLOCK    (5 * XTAL)

#define JUMPTABLE_BASE_ADDR 0x1fff0000
#define CONFIG_BASE_ADDR 0x1fff0400
#define AP_APB0_BASE (0x40000000UL)
#define AP_AON_BASE (AP_APB0_BASE + 0xF000)/*aon*/
#define AP_AON ((AP_AON_TypeDef  *) AP_AON_BASE)
#define AP_IOMUX_BASE (AP_APB0_BASE + 0x3800)/*iomux*/
#define AP_IOMUX ((IOMUX_TypeDef *) AP_IOMUX_BASE)
#define AP_GPIOA_BASE (AP_APB0_BASE + 0x8000)/*gpio*/
#define AP_GPIO ((AP_GPIO_TypeDef *) AP_GPIOA_BASE)

#define __NVIC_PRIO_BITS  2U /* Number of Bits used for Priority Levels */
#define IRQ_PRIO_REALTIME 0
#define IRQ_PRIO_HIGH     1
#define RTC_IRQn          6

#define BIT(n) (1ul << (n))
#define RET_SRAM0 BIT(0)  /*32K, 0x1fff0000~0x1fff7fff*/
#define RET_SRAM1 BIT(1)  /*16K, 0x1fff8000~0x1fffbfff*/
#define RET_SRAM2 BIT(2)  /*16K, 0x1fffc000~0x1fffffff*/

#define DCDC_CONFIG_SETTING(x) subWriteReg(0x4000f014,18,15, (0x0f&(x)))
#define DCDC_REF_CLK_SETTING(x) subWriteReg(0x4000f014,25,25, (0x01&(x)))
#define DIG_LDO_CURRENT_SETTING(x) subWriteReg(0x4000f014,22,21, (0x03&(x)))
#define HAL_PWRMGR_RAM_RETENTION_SET(x) subWriteReg(0x4000f01c,21,17, (((x) & 0xffffffe0) ? 0x00 : (x)))
#define HAL_PWRMGR_LOWCURRENTLDO_ENABLE(x) subWriteReg(0x4000f014,26,26, (x));
#define ENABLE_SOFTWARE_CONTROL(x) subWriteReg(&(AP_AON->PMCTL2_0),6,6, (x));
#define RTC_TICK_CURRENT (AP_AON->RTCCNT)
#define SET_SLEEP_FLAG (AP_AON->REG_S11 |= 1)
#define UNSET_SLEEP_FLAG (AP_AON->REG_S11 &= ~1)
#define ENTER_SYSTEM_SLEEP_MODE (AP_AON->PWRSLP = 0xA5A55A5A)
#define ENTER_SYSTEM_OFF_MODE (AP_AON->PWROFF = 0x5A5AA5A5)

#define SOFT_PARAMETER_NUM 256
#define NUMBER_OF_PINS 23
#define MSEC 32 // 32k RTCCounts is about a second (32kHz clock right?)
#define SEC 32768


typedef void (*pFunc)(void);

typedef enum {
	GPIO_P00   =   0,    P0  =  GPIO_P00,
	GPIO_P01   =   1,    P1  =  GPIO_P01,
	GPIO_P02   =   2,    P2  =  GPIO_P02,
	GPIO_P03   =   3,    P3  =  GPIO_P03,
	GPIO_P07   =   4,    P7  =  GPIO_P07,
	GPIO_P09   =   5,    P9  =  GPIO_P09,
	GPIO_P10   =   6,    P10  =  GPIO_P10,
	GPIO_P11   =   7,    P11  =  GPIO_P11,   Analog_IO_0 = GPIO_P11,
	GPIO_P14   =   8,    P14  =  GPIO_P14,   Analog_IO_1 = GPIO_P14,
	GPIO_P15   =   9,    P15  =  GPIO_P15,   Analog_IO_2 = GPIO_P15,
	GPIO_P16   =   10,   P16  =  GPIO_P16,   Analog_IO_3 = GPIO_P16,XTALI = GPIO_P16,
	GPIO_P17   =   11,   P17  =  GPIO_P17,   Analog_IO_4 = GPIO_P17,XTALO = GPIO_P17,
	GPIO_P18   =   12,   P18  =  GPIO_P18,   Analog_IO_5 = GPIO_P18,
	GPIO_P20   =   13,   P20  =  GPIO_P20,   Analog_IO_6 = GPIO_P20,
	GPIO_P23   =   14,   P23  =  GPIO_P23,   Analog_IO_7 = GPIO_P23,
	GPIO_P24   =   15,   P24  =  GPIO_P24,   Analog_IO_8 = GPIO_P24,
	GPIO_P25   =   16,   P25  =  GPIO_P25,   Analog_IO_9 = GPIO_P25,
	GPIO_P26   =   17,   P26  =  GPIO_P26,
	GPIO_P27   =   18,   P27  =  GPIO_P27,
	GPIO_P31   =   19,   P31  =  GPIO_P31,
	GPIO_P32   =   20,   P32  =  GPIO_P32,
	GPIO_P33   =   21,   P33  =  GPIO_P33,
	GPIO_P34   =   22,   P34  =  GPIO_P34,
	GPIO_NUM   =   23,
	GPIO_DUMMY =  0xff,
} gpio_pin_e;

enum {
	GPIO_PIN_ASSI_NONE = 0,
	GPIO_PIN_ASSI_OUT,
	GPIO_PIN_ASSI_IN,
};

typedef enum {
	Bit_DISABLE = 0,
	Bit_ENABLE,
} bit_action_e;

typedef enum {
	GPIO_INPUT  = 0,
	GPIO_OUTPUT = 1
} gpio_dir_t;

typedef struct {
	int           enable;
	uint8_t       pin_state;
} gpioin_Ctx_t;

typedef struct {
	int           state;
	uint8_t       pin_assignments[NUMBER_OF_PINS];
	gpioin_Ctx_t  irq_ctx[NUMBER_OF_PINS];
} gpio_Ctx_t;

typedef enum {
	GPIO_FLOATING   = 0x00,     //no pull
	GPIO_PULL_UP_S  = 0x01,     //pull up strong
	GPIO_PULL_UP    = 0x02,     //pull up weak
	GPIO_PULL_DOWN  = 0x03,
} gpio_pupd_e;

typedef struct {
	uint8_t    reg_i;
	uint8_t    bit_h;
	uint8_t    bit_l;
} PULL_TypeDef;

const PULL_TypeDef c_gpio_pull[GPIO_NUM] = {
	{0,2,1},  //p0
	{0,5,4},  //p1
	{0,8,7},  //p2
	{0,11,10},//p3
	{0,23,22},//p7
	{0,29,28},//p9
	{1,2,1},  //p10
	{1,5,4},  //p11
	{1,14,13},//p14
	{1,17,16},//p15
	{1,20,19},//p16
	{1,23,22},//p17
	{1,26,25},//p18
	{2,2,1},  //p20
	{2,11,10},//p23
	{2,14,13},//p24
	{2,17,16},//p25
	{2,20,19},//p26
	{2,23,22},//p27
	{3,5,4},  //p31
	{3,8,7},  //p32
	{3,11,10},//p33
	{3,14,13},//p34
};

typedef struct {
	gpio_pin_e pin;
	gpio_pupd_e type;
} ioinit_cfg_t;

typedef enum  _SYSCLK_SEL {
	SYS_CLK_RC_32M      = 0,
	SYS_CLK_DBL_32M     = 1,
	SYS_CLK_XTAL_16M    = 2,
	SYS_CLK_DLL_48M     = 3,
	SYS_CLK_DLL_64M     = 4,
	SYS_CLK_DLL_96M     = 5,
	SYS_CLK_8M          = 6,
	SYS_CLK_4M          = 7,
	SYS_CLK_NUM         = 8,
} sysclk_t;

typedef enum {
	CLK_32K_XTAL        = 0,
	CLK_32K_RCOSC       = 1,
} CLK32K_e;

typedef enum {
	MCU_SLEEP_MODE,
	SYSTEM_SLEEP_MODE,
	SYSTEM_OFF_MODE
} Sleep_Mode;

typedef struct {
	volatile uint32_t PWROFF;        //0x00
	volatile uint32_t PWRSLP;        //0x04
	volatile uint32_t IOCTL[3];      //0x08 0x0c 0x10
	volatile uint32_t PMCTL0;        //0x14
	volatile uint32_t PMCTL1;        //0x18
	volatile uint32_t PMCTL2_0;      //0x1c
	volatile uint32_t PMCTL2_1;      //0x20
	volatile uint32_t RTCCTL;        //0x24
	volatile uint32_t RTCCNT;        //0x28
	volatile uint32_t RTCCC0;        //0x2c
	volatile uint32_t RTCCC1;        //0x30
	volatile uint32_t RTCCC2;        //0x34
	volatile uint32_t RTCFLAG;       //0x38
	volatile uint32_t reserved[25];
	volatile uint32_t REG_S9;        //0xa0
	volatile uint32_t REG_S10;       //0xa4
	volatile uint32_t REG_S11;       //0xa8
	volatile uint32_t IDLE_REG;      //0xac
	volatile uint32_t GPIO_WAKEUP_SRC[2]; //0xb0 b4
	volatile uint32_t PCLK_CLK_GATE; //0xb8
	volatile uint32_t XTAL_16M_CTRL; //0xbc
	volatile uint32_t SLEEP_R[4];    //0xc0 c4 c8 cc
} AP_AON_TypeDef;

typedef struct {
	volatile uint32_t Analog_IO_en;  //0x00
	volatile uint32_t SPI_debug_en;  //0x04
	volatile uint32_t debug_mux_en;  //0x08
	volatile uint32_t full_mux0_en;  //0x0c
	volatile uint32_t full_mux1_en;  //0x10 reserved in some soc
	volatile uint32_t gpio_pad_en;   //0x14
	volatile uint32_t gpio_sel[9];   //0x18
	volatile uint32_t pad_pe0;       //0x3c
	volatile uint32_t pad_pe1;       //0x40
	volatile uint32_t pad_ps0;       //0x44
	volatile uint32_t pad_ps1;       //0x48
	volatile uint32_t keyscan_in_en; //0x4c
	volatile uint32_t keyscan_out_en;  //0x50
} IOMUX_TypeDef;

typedef struct {
	volatile uint32_t swporta_dr;    //0x00
	volatile uint32_t swporta_ddr;   //0x04
	volatile uint32_t swporta_ctl;   //0x08
	uint32_t reserved1[9];           //0x18-0x2c portC&D
	volatile uint32_t inten;         //0x30
	volatile uint32_t intmask;       //0x34
	volatile uint32_t inttype_level; //0x38
	volatile uint32_t int_polarity;  //0x3c
	volatile uint32_t int_status;   //0x40
	volatile uint32_t raw_instatus;  //0x44
	volatile uint32_t debounce;      //0x48
	volatile uint32_t porta_eoi;    //0x4c
	volatile uint32_t ext_porta;    //0x50
	uint32_t reserved2[3];           //0x58 0x5c
	volatile uint32_t ls_sync;       //0x60
	volatile uint32_t id_code;      //0x64
	uint32_t reserved3[1];          //0x68
	volatile uint32_t ver_id_code;  //0x6c
	volatile uint32_t config_reg2;  //0x70
	volatile uint32_t config_reg1;  //0x74
} AP_GPIO_TypeDef;



/*********************************************************************
	OSAL LARGE HEAP CONFIG
*/
typedef uint8_t halDataAlign_t; //!< Used for byte alignment
typedef struct {
	// The 15 LSB's of 'val' indicate the total item size, including the header, in 8-bit bytes.
	unsigned short len : 15;   // unsigned short len : 15;
	// The 1 MSB of 'val' is used as a boolean to indicate in-use or freed.
	unsigned short inUse : 1;  // unsigned short inUse : 1;
} osalMemHdrHdr_t;

typedef union {
	/*  Dummy variable so compiler forces structure to alignment of largest element while not wasting
		space on targets when the halDataAlign_t is smaller than a UINT16.
	*/
	halDataAlign_t alignDummy;
	uint32_t val;            // uint16    // TODO: maybe due to 4 byte alignment requirement in M0, this union should be 4 byte, change from uint16 to uint32, investigate more later -  04-25
	osalMemHdrHdr_t hdr;
} osalMemHdr_t;

#define LARGE_HEAP_SIZE (1*1024)
#define __STACK_SIZE 0x00000400
#define __HEAP_SIZE 0x00000C00

ALIGN4_U8 g_largeHeap[LARGE_HEAP_SIZE];
static uint8_t stack[__STACK_SIZE] __attribute__((aligned(8), used, section(".stack")));
static uint8_t heap[__HEAP_SIZE] __attribute__((aligned(8), used, section(".heap")));

/*********************************************************************
	GLOBAL VARIABLES
*/
uint32_t SystemCoreClock = SYSTEM_CLOCK;  /* System Core Clock Frequency      */
volatile uint8_t g_clk32K_config;
volatile sysclk_t g_spif_clk_config;
static gpio_Ctx_t m_gpioCtx = {
	.state = 0,
	.pin_assignments = {0,},
};

/*********************************************************************
	EXTERNAL VARIABLES
*/
extern uint32_t __etext;
extern uint32_t __data_start__;
extern uint32_t __data_end__;
extern uint32_t __bss_start__;
extern uint32_t __bss_end__;
extern uint32_t __StackTop;

extern void _start(void) __attribute__((noreturn)); /* PreeMain (C library entry point) */
extern void Reset_Handler(void);
extern volatile sysclk_t g_system_clk;
extern int clk_init(sysclk_t h_system_clk_sel);
extern void osal_mem_set_heap(osalMemHdr_t* hdr, uint32_t size);
extern uint8_t osal_init_system( void );
extern void osal_start_system( void );
extern void enableSleep(void);
extern void setSleepMode(Sleep_Mode mode);
extern void WaitRTCCount(uint32_t rtcDelyCnt);

const pFunc __initial_sp = (pFunc)(&__StackTop);

void Default_Handler(void) __attribute__((noreturn));
void Reset_Handler(void) __attribute__((noreturn));

/*----------------------------------------------------------------------------
	Exception / Interrupt Vector table
 *----------------------------------------------------------------------------*/
extern const pFunc __Vectors[48];
const pFunc __Vectors[48] __attribute__((used, section(".vectors"))) = {
	(pFunc)(&__StackTop), /*     Initial Stack Pointer */
	Reset_Handler,        /*     Reset Handler */
	0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,
};

void Reset_Handler(void) {
	uint32_t *pSrc, *pDest;
	
	SystemCoreClock = SYSTEM_CLOCK;
	
	pSrc = &__etext;
	pDest = &__data_start__;
	for (; pDest < &__data_end__;) {
			*pDest++ = *pSrc++;
	}

	pDest = &__bss_start__;
	for (; pDest < &__bss_end__;) {
			*pDest++ = 0UL;
	}

	_start(); /* Enter PreeMain (C library entry point) */
}

void _exit(int status) {
	(void)status;
	while (1);
}


void hal_gpio_pull_set(gpio_pin_e pin, gpio_pupd_e type) {
	uint8_t i = c_gpio_pull[pin].reg_i;
	uint8_t h = c_gpio_pull[pin].bit_h;
	uint8_t l = c_gpio_pull[pin].bit_l;

	if(pin < P31) {
		subWriteReg(&(AP_AON->IOCTL[i]),h,l,type);
	}
	else {
		subWriteReg(&(AP_AON->PMCTL0),h,l,type);
	}
}

static void hal_low_power_io_init(void) {
	//========= pull all io to gnd by default
	ioinit_cfg_t ioInit[] = {
		//TSOP6252 10 IO
		{GPIO_P02, GPIO_FLOATING},/*SWD*/
		{GPIO_P03, GPIO_FLOATING},/*SWD,LED*/
		{GPIO_P09, GPIO_FLOATING},/*UART TX*/
		{GPIO_P10, GPIO_FLOATING},/*UART RX*/
		{GPIO_P11, GPIO_FLOATING},
		{GPIO_P14, GPIO_FLOATING},
		{GPIO_P15, GPIO_FLOATING},
		{GPIO_P16, GPIO_FLOATING},
		{GPIO_P18, GPIO_FLOATING},
		{GPIO_P20, GPIO_FLOATING},
	};

	for(uint8_t i=0; i<sizeof(ioInit)/sizeof(ioinit_cfg_t); i++) {
		hal_gpio_pull_set(ioInit[i].pin,ioInit[i].type);
	}

	DCDC_CONFIG_SETTING(0x0a);
	DCDC_REF_CLK_SETTING(1);
	DIG_LDO_CURRENT_SETTING(0x01);
	HAL_PWRMGR_RAM_RETENTION_SET(RET_SRAM0);
	HAL_PWRMGR_LOWCURRENTLDO_ENABLE(1);
}

void hal_rtc_clock_config(CLK32K_e clk32Mode) {
	if(clk32Mode == CLK_32K_RCOSC) {
		subWriteReg(&(AP_AON->PMCTL0),31,27,0x05);
		subWriteReg(&(AP_AON->PMCTL2_0),16,7,0x3fb);
		subWriteReg(&(AP_AON->PMCTL2_0),6,6,0x01);
	}
	else if(clk32Mode == CLK_32K_XTAL) {
		// P16 P17 for 32K XTAL input
		hal_gpio_pull_set(GPIO_P16, GPIO_FLOATING);
		hal_gpio_pull_set(GPIO_P17, GPIO_FLOATING);
		subWriteReg(&(AP_AON->PMCTL2_0),9,8,0x03);   //software control 32k_clk
		subWriteReg(&(AP_AON->PMCTL2_0),6,6,0x00);   //disable software control
		subWriteReg(&(AP_AON->PMCTL0),31,27,0x16);
	}
}

static void hal_init(void) {
	hal_low_power_io_init();
	clk_init(g_system_clk); //system init
	hal_rtc_clock_config((CLK32K_e)g_clk32K_config);
	enableSleep();
	setSleepMode(SYSTEM_SLEEP_MODE);
}

void hal_gpio_init(void) {
	memset(&m_gpioCtx, 0, sizeof(m_gpioCtx));
	m_gpioCtx.state = 1;
}

void hal_gpio_fmux(gpio_pin_e pin, bit_action_e value) {
	if(value) {
		AP_IOMUX->full_mux0_en |= BIT(pin);
	}
	else {
		AP_IOMUX->full_mux0_en &= ~BIT(pin);
	}
}

void hal_gpio_pin2pin3_control(gpio_pin_e pin, uint8_t en) {//0:sw,1:other func
	if(en) {
		AP_IOMUX->gpio_pad_en |= BIT(pin-2);
	}
	else {
		AP_IOMUX->gpio_pad_en &= ~BIT(pin-2);
	}
}

void hal_gpio_cfg_analog_io(gpio_pin_e pin, bit_action_e value) {
	if(value) {
		hal_gpio_pull_set(pin,GPIO_FLOATING);
		AP_IOMUX->Analog_IO_en |= BIT(pin - P11);
	}
	else {
		AP_IOMUX->Analog_IO_en &= ~BIT(pin - P11);
	}
}

void hal_gpio_pin_init(gpio_pin_e pin, gpio_dir_t type) {
	hal_gpio_fmux(pin,Bit_DISABLE);

	if((pin == P2) || (pin == P3)) {
		hal_gpio_pin2pin3_control(pin,1);
	}

	hal_gpio_cfg_analog_io(pin,Bit_DISABLE);

	if(type == GPIO_OUTPUT) {
		AP_GPIO->swporta_ddr |= BIT(pin);
	}
	else {
		AP_GPIO->swporta_ddr &= ~BIT(pin);
		m_gpioCtx.pin_assignments[pin] = GPIO_PIN_ASSI_IN;
	}
}

void hal_gpioretention_register(gpio_pin_e pin) {
	m_gpioCtx.pin_assignments[pin] = GPIO_PIN_ASSI_OUT;
	hal_gpio_pin_init(pin, GPIO_OUTPUT);
}

void hal_gpio_write(gpio_pin_e pin, uint8_t en) {
	if(en) {
		AP_GPIO->swporta_dr |= BIT(pin);
	}
	else {
		AP_GPIO->swporta_dr &= ~BIT(pin);
	}
}

void debug_blink(uint32_t nblink) {
	gpio_pin_e pin = P3; // LED is on pin 3
	hal_gpioretention_register(pin);
	while(nblink--) {
		hal_gpio_write(pin, 1);
		WaitRTCCount(5*MSEC);
		hal_gpio_write(pin, 0);
		if(nblink) {
			WaitRTCCount(195*MSEC);
		}
	}
}
void debug_blink3(void) { debug_blink(3); }
void debug_blink5(void) { debug_blink(5); }

void config_RTC(uint32_t time) {
	uint32_t tick_curr = RTC_TICK_CURRENT;
	WaitRTCCount(1); //align to rtc clock edge
	AP_AON->RTCCC0 = tick_curr + time;  //set RTC comparator0 value
	//enable (20) comparator0 event, (18) counter overflow interrupt, (15) comparator0 interrupt
	AP_AON->RTCCTL |= (BIT(15) | BIT(18) | BIT(20));
}

void Blink_IRQHandler() {
	gpio_pin_e pin = P3; // LED is on pin 3
	hal_gpioretention_register(pin);
	hal_gpio_write(pin, 1);
	WaitRTCCount(5*MSEC);
	hal_gpio_write(pin, 0);
	config_RTC(5*SEC);
	ENTER_SYSTEM_SLEEP_MODE;
}


///////////////////////////////////////////////////////////////////////////////////
int main(void) {
	uint32_t* pJump_table = (uint32_t*)(JUMPTABLE_BASE_ADDR);
	uint32_t* pGlobal_config = (uint32_t*)(CONFIG_BASE_ADDR);

	for(int i = 0; i < 256; i++) { // global config and jump table are both 256 elements
		switch(i) {
		case 1: // tasks init, should have an entry or osal crashes. Here abused as WAKEUP_PROCESS and sleep again
			pJump_table[i] = (uint32_t)Blink_IRQHandler;
			break;
		case 2: // task array
		case 3: // task count
		case 4: // task events
		case 5: // osal mem init, on wakeup
		case 58: // LL_DIRECT_TEST_TX_TEST, called on wakeup and blocks
		case 59: // LL_DIRECT_TEST_RX_TEST, called on wakeup and blocks
		case 60:  // OSAL_POWER_CONSERVE called by osal_pwrmgr_powerconserve()
		case 61:  // ENTER_SLEEP_PROCESS called by enterSleepProcess()
		case 62:  // WAKEUP_PROCESS, called on wakeup and blocks
		case 63:  // CONFIG_RTC called by enterSleepProcess()
		case 64:  // ENTER_SLEEP_OFF_MODE called by enterSleepProcess()
		case 100: // APP_SLEEP_PROCESS called by enterSleepProcess()
		case 212: // called by drv_irq_init()
		case 213: // called by drv_enable_irq()
		case 214: // called by osal_pwrmgr_powerconserve()
		case 228: // V4_IRQ_HANDLER
		case 230: // called when RTC comparator0 is reached (event?)
			pJump_table[i] = 0;
			break;
		default:
			pJump_table[i] = (uint32_t)debug_blink3;
			break;
		}

		pGlobal_config[i] = 0;
	}
	
	pGlobal_config[34] = (uint32_t)&__initial_sp; // INITIAL_STACK_PTR
	// sleep times, in us
	pGlobal_config[5] = 30000000; // MAX_SLEEP_TIME
	pGlobal_config[6] = 1600; // MIN_SLEEP_TIME
	pGlobal_config[35] = 55;// 30.5 per tick // ALLOW_TO_SLEEP_TICK_RC32K

	g_system_clk = SYS_CLK_XTAL_16M;//SYS_CLK_XTAL_16M;//SYS_CLK_DLL_64M;
	g_clk32K_config = CLK_32K_RCOSC;//CLK_32K_XTAL;//CLK_32K_XTAL,CLK_32K_RCOSC
	
	osal_mem_set_heap((osalMemHdr_t*)g_largeHeap, LARGE_HEAP_SIZE);
	hal_init();
	hal_gpio_init();

	// CMSIS enable interrupts (device specific interrupt flags also need to be set!)
	__enable_irq();
	NVIC_SetPriority(RTC_IRQn, IRQ_PRIO_HIGH);
	NVIC_EnableIRQ(RTC_IRQn);
	
	config_RTC(3); // fire RTC irq as fast as you can
	ENABLE_SOFTWARE_CONTROL(0x00); // software control disable
	SET_SLEEP_FLAG;
	AP_AON->SLEEP_R[0] = 4; //RSTC_WAKE_RTC;
	
	/* Initialize the operating system */
	osal_init_system();
	/* Start OSAL */
	osal_start_system(); // No Return from here

	return 0; // should never reach
}
/////////////////////////////////////  end  ///////////////////////////////////////
