#include <stdint.h>
#include <string.h>

#ifdef __GCC
	#define ALIGN4_U8 _Alignas(4) uint8_t
#else
	#define ALIGN4_U8 __align(4) uint8_t
#endif

#define write_reg(addr,data)             (*(volatile unsigned int*)(addr)=(unsigned int)(data))
#define read_reg(addr)                   (*(volatile unsigned int*)(addr))
#define subWriteReg(addr,high,low,value) write_reg(addr,read_reg(addr)&\
										 ((~((((unsigned int)1<<((high)-(low)+1))-1)<<(low)))|\
										 ((unsigned int)(value)<<(low))))

#define JUMP_BASE_ADDR 0x1fff0000
#define JUMP_FUNCTION(x) (*(uint32_t *)(JUMP_BASE_ADDR + (x << 2)))
#define V4_IRQ_HANDLER 228

#define AP_APB0_BASE (0x40000000UL)
#define AP_AON_BASE (AP_APB0_BASE + 0xF000)/*aon*/
#define AP_AON ((AP_AON_TypeDef  *) AP_AON_BASE)
#define AP_IOMUX_BASE (AP_APB0_BASE + 0x3800)/*iomux*/
#define AP_IOMUX ((IOMUX_TypeDef *) AP_IOMUX_BASE)
#define AP_GPIOA_BASE (AP_APB0_BASE + 0x8000)/*gpio*/
#define AP_GPIO ((AP_GPIO_TypeDef *) AP_GPIOA_BASE)
#define AP_PCR_BASE (AP_APB0_BASE + 0x0000)/*pcr*//* APB0 peripherals   */
#define AP_PCR ((AP_PCR_TypeDef *) AP_PCR_BASE)

#define XTAL16M_CAP_SETTING(x) subWriteReg(0x4000f0bc, 4, 0, (0x1f&(x)))
#define XTAL16M_CURRENT_SETTING(x) subWriteReg(0x4000f0bc, 6, 5, (0x03&(x)))

//SW_CLK -->0x4000f008
#define _CLK_DMA         (BIT(3))
#define _CLK_IOMUX       (BIT(7))
#define _CLK_UART0       (BIT(8))
#define _CLK_GPIO        (BIT(13))
#define _CLK_SPIF        (BIT(19))
//SW_CLK1 -->0x4000f014
#define _CLK_M0_CPU      (BIT(0))
#define _CLK_BB          (BIT(3))
#define _CLK_TIMER       (BIT(4))
#define _CLK_COM         (BIT(6))
#define _CLK_BBREG       (BIT(9))
#define _CLK_TIMER1      (BIT(21))
#define _CLK_TIMER2      (BIT(22))
#define _CLK_TIMER3      (BIT(23))
#define _CLK_TIMER4      (BIT(24))
#define DEF_CLKG_CONFIG_0 (_CLK_DMA | _CLK_IOMUX | _CLK_UART0 | _CLK_GPIO | _CLK_SPIF)
#define DEF_CLKG_CONFIG_1 (_CLK_M0_CPU | _CLK_BB |_CLK_TIMER | _CLK_COM | _CLK_BBREG | _CLK_TIMER1 | _CLK_TIMER2 | _CLK_TIMER3 | _CLK_TIMER4)

#define __NVIC_PRIO_BITS  2U /* Number of Bits used for Priority Levels */
#define IRQ_PRIO_REALTIME 0
#define IRQ_PRIO_HIGH     1

#define BIT(n) (1ul << (n))
#define RET_SRAM0 BIT(0)  /*32K, 0x1fff0000~0x1fff7fff*/
#define RET_SRAM1 BIT(1)  /*16K, 0x1fff8000~0x1fffbfff*/
#define RET_SRAM2 BIT(2)  /*16K, 0x1fffc000~0x1fffffff*/

#define PWRMGR_ALWAYS_ON  0
#define PWRMGR_BATTERY    1

#define TASKID  0
#define TASKEVT 0

#define SOFT_PARAMETER_NUM 256
#define NUMBER_OF_PINS 23

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

typedef enum IRQn {
	/* -----  Cortex-M0 Processor Exceptions Numbers  ----- */
	NonMaskableInt_IRQn = -14, /*  2 Non Maskable Interrupt */
	HardFault_IRQn      = -13, /*  3 HardFault Interrupt */
	SVCall_IRQn         =  -5, /* 11 SV Call Interrupt */
	PendSV_IRQn         =  -2, /* 14 Pend SV Interrupt */
	SysTick_IRQn        =  -1, /* 15 System Tick Interrupt */
	/* -----  PHY BUMBEE M0 Interrupt Numbers  ----- */
	BB_IRQn             =   4, /* Base band Interrupt */
	KSCAN_IRQn          =   5, /* Key scan Interrupt */
	RTC_IRQn            =   6, /* RTC Timer Interrupt */
	WDT_IRQn            =  10, /* Watchdog Timer Interrupt */
	UART0_IRQn          =  11, /* UART0 Interrupt */
	I2C0_IRQn           =  12, /* I2C0 Interrupt */
	I2C1_IRQn           =  13, /* I2C1 Interrupt */
	SPI0_IRQn           =  14, /* SPI0 Interrupt */
	SPI1_IRQn           =  15, /* SPI1 Interrupt */
	GPIO_IRQn           =  16, /* GPIO Interrupt */
	UART1_IRQn          =  17, /* UART1 Interrupt */
	SPIF_IRQn           =  18, /* SPIF Interrupt */
	DMAC_IRQn           =  19, /* DMAC Interrupt */
	TIM1_IRQn           =  20, /* Timer1 Interrupt */
	TIM2_IRQn           =  21, /* Timer2 Interrupt */
	TIM3_IRQn           =  22, /* Timer3 Interrupt */
	TIM4_IRQn           =  23, /* Timer4 Interrupt */
	TIM5_IRQn           =  24, /* Timer5 Interrupt */
	TIM6_IRQn           =  25, /* Timer6 Interrupt */
	AES_IRQn            =  28, /* AES Interrupt */
	ADCC_IRQn           =  29, /* ADC Interrupt */
	QDEC_IRQn           =  30, /* QDEC Interrupt */
	RNG_IRQn            =  31  /* RNG Interrupt */
} IRQn_Type;
#include "core_cm0.h" // hacky

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
	volatile uint32_t keyscan_out_en;//0x50
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
	volatile uint32_t int_status;    //0x40
	volatile uint32_t raw_instatus;  //0x44
	volatile uint32_t debounce;      //0x48
	volatile uint32_t porta_eoi;     //0x4c
	volatile uint32_t ext_porta;     //0x50
	uint32_t reserved2[3];           //0x58 0x5c
	volatile uint32_t ls_sync;       //0x60
	volatile uint32_t id_code;       //0x64
	uint32_t reserved3[1];           //0x68
	volatile uint32_t ver_id_code;   //0x6c
	volatile uint32_t config_reg2;   //0x70
	volatile uint32_t config_reg1;   //0x74
} AP_GPIO_TypeDef;

typedef struct {
	volatile uint32_t SW_RESET0;     //0x0
	volatile uint32_t SW_RESET1;     //0x4
	volatile uint32_t SW_CLK;        //0x8
	volatile uint32_t SW_RESET2;     //0xc
	volatile uint32_t SW_RESET3;     //0x10
	volatile uint32_t SW_CLK1;       //0x14
	volatile uint32_t APB_CLK;       //0x18
	volatile uint32_t APB_CLK_UPDATE;//0x1c
	volatile uint32_t CACHE_CLOCK_GATE;//0x20
	volatile uint32_t CACHE_RST;     //0x24
	volatile uint32_t CACHE_BYPASS;  //0x28
} AP_PCR_TypeDef;


/*********************************************************************
	OSAL LARGE HEAP CONFIG
*/
#define LARGE_HEAP_SIZE (1*1024)
ALIGN4_U8 g_largeHeap[LARGE_HEAP_SIZE];
typedef struct {
	// The 15 LSB's of 'val' indicate the total item size, including the header, in 8-bit bytes.
	unsigned short len : 15;   // unsigned short len : 15;
	// The 1 MSB of 'val' is used as a boolean to indicate in-use or freed.
	unsigned short inUse : 1;  // unsigned short inUse : 1;
} osalMemHdrHdr_t;

typedef uint8_t halDataAlign_t;
typedef union {
	/*  Dummy variable so compiler forces structure to alignment of largest element while not wasting
		space on targets when the halDataAlign_t is smaller than a UINT16.
	*/
	halDataAlign_t alignDummy;
	uint32_t val;            // uint16    // TODO: maybe due to 4 byte alignment requirement in M0, this union should be 4 byte, change from uint16 to uint32, investigate more later -  04-25
	osalMemHdrHdr_t hdr;
} osalMemHdr_t;

/*********************************************************************
	GLOBAL VARIABLES
*/
uint32_t global_config[SOFT_PARAMETER_NUM] __attribute__((section("global_config_area"))) = {0};
volatile uint8_t g_clk32K_config;
volatile sysclk_t g_spif_clk_config;
static gpio_Ctx_t m_gpioCtx = {
	.state = 0,
	.pin_assignments = {0,},
};

// LED is on pin 3
gpio_pin_e g_led = P3;
void tasksInit(void);
uint16_t ProcessEvent( uint8_t task_id, uint16_t events );
typedef unsigned short (*pTaskEventHandlerFn)( unsigned char task_id, unsigned short event );
const pTaskEventHandlerFn tasksArr[] = { ProcessEvent, };
const uint8_t tasksCnt = sizeof( tasksArr ) / sizeof( tasksArr[0] );
uint16_t* tasksEvents;

int hal_pwrmgr_sleep_process(void) __attribute__((weak));
int hal_pwrmgr_wakeup_process(void) __attribute__((weak));
static uint32_t s_config_swClk0 = DEF_CLKG_CONFIG_0;
static uint32_t s_config_swClk1 = DEF_CLKG_CONFIG_1;
uint32_t s_gpio_wakeup_src_group1;
uint32_t s_gpio_wakeup_src_group2;

/*********************************************************************
	EXTERNAL VARIABLES
*/
extern volatile sysclk_t g_system_clk;
extern int clk_init(sysclk_t h_system_clk_sel);
extern uint32_t read_current_fine_time(void);
extern void enableSleep(void);
extern void setSleepMode(Sleep_Mode mode);
extern void WaitRTCCount(uint32_t rtcDelyCnt);
extern void spif_release_deep_sleep(void);
extern void spif_set_deep_sleep(void);

extern uint8_t osal_init_system( void );
extern void* osal_mem_alloc( uint16_t size );
extern void* osal_memset( void* dest, uint8_t value, int len );
extern void osal_mem_set_heap(osalMemHdr_t* hdr, uint32_t size);
extern void osal_pwrmgr_device( uint8_t pwrmgr_device );
extern void osal_start_system( void );
extern uint8_t osal_set_event( uint8_t task_id, uint16_t event_flag );
extern uint8_t osal_start_timerEx( uint8_t task_id, uint16_t event_id, uint32_t timeout_value );

extern int drv_disable_irq(void);
extern void ll_hw_clr_irq(void);
extern int drv_enable_irq(void);
extern void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority);


void (*trap_c_callback)(void);
void _hard_fault(uint32_t* arg) {
	if (trap_c_callback) {
		trap_c_callback();
	}
	while (1);
}
void hard_fault(void) {
	uint32_t arg = 0;
	_hard_fault(&arg);
}
// jump table, this table save the function entry which will be called by ROM code
// item 1 - 4 for OSAL task entry
// item 224 - 255 for ISR(Interrupt Service Routine) entry
// others are reserved by ROM code
const uint32_t* const jump_table_base[256] __attribute__((section("jump_table_mem_area"))) = {
	(const uint32_t*)0,               // 0. write Log
	(const uint32_t*)tasksInit,      // 1. init entry of app
	(const uint32_t*)tasksArr,        // 2. task list
	(const uint32_t*)& tasksCnt,      // 3. task count
	(const uint32_t*)& tasksEvents,   // 4. task events
	0, 0, 0, 0, 0,                    // 5 - 9, reserved for rom patch
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,     // 10 - 19, reserved for rom patch
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,     // 20 - 29, reserved for rom patch
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,     // 30 - 39, reserved for rom patch
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,     // 40 - 49, reserved for rom patch
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,     // 50 - 59, reserved for rom patch
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,     // 60 - 69, reserved for rom patch
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,     // 70 - 79, reserved for rom patch
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,     // 80 - 89, reserved for rom patch
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,     // 90 - 99, reserved for rom patch
	(const uint32_t*)hal_pwrmgr_sleep_process,  // 100
	(const uint32_t*)hal_pwrmgr_wakeup_process, // 101
	0,                                // 102 (const uint32_t*)rf_phy_ini,
	0, 0, 0, 0, 0, 0, 0,              // 103 - 109, reserved for rom patch
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,     // 110 - 119, reserved for rom patch
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,     // 120 - 129, reserved for rom patch
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,     // 130 - 139, reserved for rom patch
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,     // 140 - 149, reserved for rom patch
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,     // 150 - 159, reserved for rom patch
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,     // 160 - 169, reserved for rom patch
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,     // 170 - 179, reserved for rom patch
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,     // 180 - 189, reserved for rom patch
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,     // 190 - 199, reserved for rom patch
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,     // 200 - 209, reserved for rom patch
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,     // 210 - 219, reserved for rom patch
	(const uint32_t*)hard_fault,      // 220 hard fault
	0, 0, 0, 0, 0, 0, 0, 0, 0,        // 221 - 229
	0, 0, 0, 0, 0,                    // 230 - 234
	0,                                // 235 uart irq handler
	0, 0, 0, 0, 0,                    // 236 - 240
	0, 0, 0, 0, 0, 0, 0, 0, 0,        // 241 - 249, for ISR entry
	0, 0, 0, 0, 0, 0                  // 250 - 255, for ISR entry
};


void hal_pwrmgr_RAM_retention_set(uint32_t sram) {
	uint32_t sramRet_config = sramRet_config;
	if(sram & 0xffffffe0) {
		sramRet_config = 0x00;
	}
	subWriteReg(0x4000f01c,21,17,sramRet_config);
}


void hal_pwrmgr_LowCurrentLdo_enable(void) {
	subWriteReg(0x4000f014,26,26, 1);
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

	#define DCDC_CONFIG_SETTING(x) subWriteReg(0x4000f014,18,15, (0x0f&(x)))
	DCDC_CONFIG_SETTING(0x0a);

	#define DCDC_REF_CLK_SETTING(x) subWriteReg(0x4000f014,25,25, (0x01&(x)))
	DCDC_REF_CLK_SETTING(1);

	#define DIG_LDO_CURRENT_SETTING(x) subWriteReg(0x4000f014,22,21, (0x03&(x)))
	DIG_LDO_CURRENT_SETTING(0x01);

	hal_pwrmgr_RAM_retention_set(0x00);
	hal_pwrmgr_LowCurrentLdo_enable();
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

	//ZQ 20200812 for rc32k wakeup
	subWriteReg(&(AP_AON->PMCTL0),28,28,0x1);//turn on 32kxtal
	subWriteReg(&(AP_AON->PMCTL1),18,17,0x0);// reduce 32kxtl bias current
}

static void hal_init(void) {
	XTAL16M_CAP_SETTING(0x09);
	XTAL16M_CURRENT_SETTING(0x01);

	typedef void (*my_function)(void);
	my_function pFunc = (my_function)(0xa2e1); // WTF? _rom_sec_boot_init
	pFunc();

	NVIC_SetPriority((IRQn_Type)BB_IRQn,    IRQ_PRIO_REALTIME);
	NVIC_SetPriority((IRQn_Type)TIM1_IRQn,  IRQ_PRIO_HIGH);     //ll_EVT
	NVIC_SetPriority((IRQn_Type)TIM2_IRQn,  IRQ_PRIO_HIGH);     //OSAL_TICK
	NVIC_SetPriority((IRQn_Type)TIM4_IRQn,  IRQ_PRIO_HIGH);     //LL_EXA_ADV
	
	osal_mem_set_heap((osalMemHdr_t*)g_largeHeap, LARGE_HEAP_SIZE);

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

int __attribute__((used)) hal_pwrmgr_wakeup_process(void) {
	spif_release_deep_sleep();
	WaitRTCCount(8);

	AP_PCR->SW_CLK  = s_config_swClk0;
	AP_PCR->SW_CLK1 = s_config_swClk1|0x01;//force set M0 CPU
	s_gpio_wakeup_src_group1 = AP_AON->GPIO_WAKEUP_SRC[0];
	s_gpio_wakeup_src_group2 = AP_AON->GPIO_WAKEUP_SRC[1];
	//restore BB TIMER IRQ_PRIO
	NVIC_SetPriority((IRQn_Type)BB_IRQn,    IRQ_PRIO_REALTIME);
	NVIC_SetPriority((IRQn_Type)TIM1_IRQn,  IRQ_PRIO_HIGH);     //ll_EVT
	NVIC_SetPriority((IRQn_Type)TIM2_IRQn,  IRQ_PRIO_HIGH);     //OSAL_TICK
	NVIC_SetPriority((IRQn_Type)TIM4_IRQn,  IRQ_PRIO_HIGH);     //LL_EXA_ADV

	return 0;
}

int __attribute__((used)) hal_pwrmgr_sleep_process(void) {
	spif_set_deep_sleep();
	return 0;
}

void IRQHandler(void) {
	drv_disable_irq();
	osal_set_event(TASKID,TASKEVT);
	ll_hw_clr_irq(); // post ISR process
	drv_enable_irq();
}

void tasksInit(void) {
	tasksEvents = (uint16_t*)osal_mem_alloc( sizeof( uint16_t ) * tasksCnt);
	osal_memset( tasksEvents, 0, (sizeof( uint16_t ) * tasksCnt));
	JUMP_FUNCTION(V4_IRQ_HANDLER) = (uint32_t)&IRQHandler;
}

uint16_t ProcessEvent(uint8_t task_id, uint16_t events) {
	hal_gpio_write(g_led,0);
	return 0;
}


void app_main() {
	hal_gpioretention_register(g_led);
	hal_gpio_write(g_led,1);

	// init osal tasks (is this necessary? it's in the jump table as well)
	tasksInit();
	osal_start_timerEx(TASKID,TASKEVT, 500); // ms

	/* Initialize the operating system */
	osal_init_system();
	osal_pwrmgr_device(PWRMGR_BATTERY);
	/* Start OSAL */
	osal_start_system(); // No Return from here
}

///////////////////////////////////////////////////////////////////////////////////
int main(void) {
	g_system_clk = SYS_CLK_XTAL_16M;//SYS_CLK_XTAL_16M;//SYS_CLK_DLL_64M;
	g_clk32K_config = CLK_32K_RCOSC;//CLK_32K_XTAL;//CLK_32K_XTAL,CLK_32K_RCOSC
	hal_init();
	
	hal_gpio_init();
	
	app_main();

	return 0; // should never reach
}
/////////////////////////////////////  end  ///////////////////////////////////////
