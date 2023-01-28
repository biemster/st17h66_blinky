#include <stdint.h>

#ifdef __GCC
	#define ALIGN4_U8 _Alignas(4) uint8_t
#else
	#define ALIGN4_U8 __align(4) uint8_t
#endif

#define write_reg(addr,data)             (*(volatile unsigned int*)(addr)=(unsigned int)(data))
#define read_reg(addr)                   (*(volatile unsigned int*)(addr))
#define subWriteReg(addr,high,low,value) write_reg(addr,read_reg(addr)&\
										 (~((((unsigned int)1<<((high)-(low)+1))-1)<<(low)))|\
										 ((unsigned int)(value)<<(low)))
																				 
#define AP_APB0_BASE (0x40000000UL)
#define AP_AON_BASE (AP_APB0_BASE + 0xF000)/*aon*/
#define AP_AON ((AP_AON_TypeDef  *) AP_AON_BASE)

#define BIT(n) (1ul << (n))
#define RET_SRAM0 BIT(0)  /*32K, 0x1fff0000~0x1fff7fff*/
#define RET_SRAM1 BIT(1)  /*16K, 0x1fff8000~0x1fffbfff*/
#define RET_SRAM2 BIT(2)  /*16K, 0x1fffc000~0x1fffffff*/
																				 

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

typedef enum
{
	CLK_32K_XTAL        = 0,
	CLK_32K_RCOSC       = 1,

} CLK32K_e;

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



/*********************************************************************
	OSAL LARGE HEAP CONFIG
*/
#define LARGE_HEAP_SIZE (1*1024)
ALIGN4_U8 g_largeHeap[LARGE_HEAP_SIZE];

/*********************************************************************
	GLOBAL VARIABLES
*/
volatile uint8_t g_clk32K_config;
volatile sysclk_t g_spif_clk_config;


/*********************************************************************
	EXTERNAL VARIABLES
*/
extern uint32_t  __initial_sp;
extern volatile sysclk_t g_system_clk;
extern int clk_init(sysclk_t h_system_clk_sel);
extern void enableSleep(void);


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
	hal_low_power_io_init();
	clk_init(g_system_clk); //system init
	hal_rtc_clock_config((CLK32K_e)g_clk32K_config);
	enableSleep();
}

void app_main() {
	// TOGGLE LED
	while(1) {}
}

///////////////////////////////////////////////////////////////////////////////////
int main(void) {
	g_system_clk = SYS_CLK_XTAL_16M;//SYS_CLK_XTAL_16M;//SYS_CLK_DLL_64M;
	g_clk32K_config = CLK_32K_RCOSC;//CLK_32K_XTAL;//CLK_32K_XTAL,CLK_32K_RCOSC
	hal_init();
	app_main();

	return 0; // should never reach
}
/////////////////////////////////////  end  ///////////////////////////////////////
