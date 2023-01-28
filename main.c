
/*********************************************************************
    OSAL LARGE HEAP CONFIG
*/
#define LARGE_HEAP_SIZE  (1*1024)
ALIGN4_U8 g_largeHeap[LARGE_HEAP_SIZE];

/*********************************************************************
    GLOBAL VARIABLES
*/
volatile uint8 g_clk32K_config;
volatile sysclk_t g_spif_clk_config;


/*********************************************************************
    EXTERNAL VARIABLES
*/
extern uint32_t  __initial_sp;


static void hal_low_power_io_init(void) {
    //========= pull all io to gnd by default
    ioinit_cfg_t ioInit[] = {
        //TSOP6252 10 IO
        {GPIO_P02,   GPIO_FLOATING   },/*SWD*/
        {GPIO_P03,   GPIO_FLOATING   },/*SWD,LED*/
        {GPIO_P09,   GPIO_FLOATING   },/*UART TX*/
        {GPIO_P10,   GPIO_FLOATING   },/*UART RX*/
        {GPIO_P11,   GPIO_FLOATING   },
        {GPIO_P14,   GPIO_FLOATING   },
        {GPIO_P15,   GPIO_FLOATING   },
        {GPIO_P16,   GPIO_FLOATING   },
        {GPIO_P18,   GPIO_FLOATING   },
        {GPIO_P20,   GPIO_FLOATING   },
    };

    for(uint8_t i=0; i<sizeof(ioInit)/sizeof(ioinit_cfg_t); i++) {
        hal_gpio_pull_set(ioInit[i].pin,ioInit[i].type);
		}

    DCDC_CONFIG_SETTING(0x0a);
    DCDC_REF_CLK_SETTING(1);
    DIG_LDO_CURRENT_SETTING(0x01);
    hal_pwrmgr_RAM_retention();
    hal_pwrmgr_RAM_retention_set();
    hal_pwrmgr_LowCurrentLdo_enable();
}


static void hal_init(void) {
    hal_low_power_io_init();
    clk_init(g_system_clk); //system init
    hal_rtc_clock_config((CLK32K_e)g_clk32K_config);
    hal_pwrmgr_init();
	
    xflash_Ctx_t cfg = {
        .spif_ref_clk   =   SYS_CLK_RC_32M,
        .rd_instr       =   XFRD_FCMD_READ_DUAL
    };
    hal_spif_cache_init(cfg);
		
    hal_gpio_init();
}

void drv_irq_init() {
}
void init_config() {
}
void app_main() {
}

///////////////////////////////////////////////////////////////////////////////////
int main(void) {
    g_system_clk = SYS_CLK_XTAL_16M;//SYS_CLK_XTAL_16M;//SYS_CLK_DLL_64M;
    g_clk32K_config = CLK_32K_RCOSC;//CLK_32K_XTAL;//CLK_32K_XTAL,CLK_32K_RCOSC
    drv_irq_init();
    init_config();
    hal_init();
    app_main();
}


/////////////////////////////////////  end  ///////////////////////////////////////
