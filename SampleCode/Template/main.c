/*_____ I N C L U D E S ____________________________________________________*/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#include "project_config.h"

#include "DataFlashProg.h"
/*_____ D E C L A R A T I O N S ____________________________________________*/

struct flag_32bit flag_MISC_CTL;
#define FLAG_MISC_ERROR                                 (flag_MISC_CTL.bit0)
#define FLAG_MISC_TIMER_PERIOD_1000MS                   (flag_MISC_CTL.bit1)
#define FLAG_MISC_SYSTICK_PERIOD_1000MS                 (flag_MISC_CTL.bit2)
#define FLAG_MISC_REVERSE1                              (flag_MISC_CTL.bit3)
#define FLAG_MISC_REVERSE2                              (flag_MISC_CTL.bit4)
#define FLAG_MISC_REVERSE3                              (flag_MISC_CTL.bit5)
#define FLAG_MISC_REVERSE4                              (flag_MISC_CTL.bit6)
#define FLAG_MISC_REVERSE5                              (flag_MISC_CTL.bit7)


/*_____ D E F I N I T I O N S ______________________________________________*/
// volatile uint32_t BitFlag = 0;
volatile uint32_t counter_tick = 0;
volatile uint32_t counter_systick = 0;

#if defined (ENABLE_TICK_EVENT)
typedef void (*sys_pvTimeFunPtr)(void);   /* function pointer */
typedef struct timeEvent_t
{
    uint8_t             active;
    unsigned long       initTick;
    unsigned long       curTick;
    sys_pvTimeFunPtr    funPtr;
} TimeEvent_T;

#define TICKEVENTCOUNT                                 (8)                   
volatile  TimeEvent_T tTimerEvent[TICKEVENTCOUNT];
volatile uint8_t _sys_uTimerEventCount = 0;             /* Speed up interrupt reponse time if no callback function */
#endif

uint32_t Storage_Block[BUFFER_PAGE_SIZE / 4] = {0};
/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/

int IsDebugFifoEmpty(void);


uint32_t get_systick(void)
{
	return (counter_systick);
}

void set_systick(uint32_t t)
{
	counter_systick = t;
}

void systick_counter(void)
{
	counter_systick++;
}

uint32_t get_tick(void)
{
	return (counter_tick);
}

void set_tick(uint32_t t)
{
	counter_tick = t;
}

void tick_counter(void)
{
	counter_tick++;
    if (get_tick() >= 60000)
    {
        set_tick(0);
    }
}

void compare_buffer(uint8_t *src, uint8_t *des, int nBytes)
{
    uint16_t i = 0;	
	
    #if 1
    for (i = 0; i < nBytes; i++)
    {
        if (src[i] != des[i])
        {
            printf("error idx : %4d : 0x%2X , 0x%2X\r\n", i , src[i],des[i]);
			FLAG_MISC_ERROR = 1;//set_flag(flag_error , ENABLE);
        }
    }

	if (!FLAG_MISC_ERROR)//(!is_flag_set(flag_error))
	{
    	printf("%s finish \r\n" , __FUNCTION__);	
		FLAG_MISC_ERROR = 0;//set_flag(flag_error , DISABLE);
	}
    #else
    if (memcmp(src, des, nBytes))
    {
        printf("\nMismatch!! - %d\n", nBytes);
        for (i = 0; i < nBytes; i++)
            printf("0x%02x    0x%02x\n", src[i], des[i]);
        return -1;
    }
    #endif

}

void reset_buffer(void *dest, unsigned int val, unsigned int size)
{
    uint8_t *pu8Dest;
//    unsigned int i;
    
    pu8Dest = (uint8_t *)dest;

	#if 1
	while (size-- > 0)
		*pu8Dest++ = val;
	#else
	memset(pu8Dest, val, size * (sizeof(pu8Dest[0]) ));
	#endif
	
}

void copy_buffer(void *dest, void *src, unsigned int size)
{
    uint8_t *pu8Src, *pu8Dest;
    unsigned int i;
    
    pu8Dest = (uint8_t *)dest;
    pu8Src  = (uint8_t *)src;


	#if 0
	  while (size--)
	    *pu8Dest++ = *pu8Src++;
	#else
    for (i = 0; i < size; i++)
        pu8Dest[i] = pu8Src[i];
	#endif
}

void dump_buffer(uint8_t *pucBuff, int nBytes)
{
    uint16_t i = 0;
    
    printf("dump_buffer : %2d\r\n" , nBytes);    
    for (i = 0 ; i < nBytes ; i++)
    {
        printf("0x%2X," , pucBuff[i]);
        if ((i+1)%8 ==0)
        {
            printf("\r\n");
        }            
    }
    printf("\r\n\r\n");
}

void dump_buffer_hex(uint8_t *pucBuff, int nBytes)
{
    int     nIdx, i;

    nIdx = 0;
    while (nBytes > 0)
    {
        printf("0x%04X  ", nIdx);
        for (i = 0; i < 16; i++)
            printf("%02X ", pucBuff[nIdx + i]);
        printf("  ");
        for (i = 0; i < 16; i++)
        {
            if ((pucBuff[nIdx + i] >= 0x20) && (pucBuff[nIdx + i] < 127))
                printf("%c", pucBuff[nIdx + i]);
            else
                printf(".");
            nBytes--;
        }
        nIdx += 16;
        printf("\n");
    }
    printf("\n");
}

// void delay_ms(uint16_t ms)
// {
// 	TIMER_Delay(TIMER0, 1000*ms);
// }


#if defined (ENABLE_TICK_EVENT)
void TickCallback_processB(void)
{
    printf("%s test \r\n" , __FUNCTION__);
}

void TickCallback_processA(void)
{
    printf("%s test \r\n" , __FUNCTION__);
}

void TickClearTickEvent(uint8_t u8TimeEventID)
{
    if (u8TimeEventID > TICKEVENTCOUNT)
        return;

    if (tTimerEvent[u8TimeEventID].active == TRUE)
    {
        tTimerEvent[u8TimeEventID].active = FALSE;
        _sys_uTimerEventCount--;
    }
}

signed char TickSetTickEvent(unsigned long uTimeTick, void *pvFun)
{
    int  i;
    int u8TimeEventID = 0;

    for (i = 0; i < TICKEVENTCOUNT; i++)
    {
        if (tTimerEvent[i].active == FALSE)
        {
            tTimerEvent[i].active = TRUE;
            tTimerEvent[i].initTick = uTimeTick;
            tTimerEvent[i].curTick = uTimeTick;
            tTimerEvent[i].funPtr = (sys_pvTimeFunPtr)pvFun;
            u8TimeEventID = i;
            _sys_uTimerEventCount += 1;
            break;
        }
    }

    if (i == TICKEVENTCOUNT)
    {
        return -1;    /* -1 means invalid channel */
    }
    else
    {
        return u8TimeEventID;    /* Event ID start from 0*/
    }
}

void TickCheckTickEvent(void)
{
    uint8_t i = 0;

    if (_sys_uTimerEventCount)
    {
        for (i = 0; i < TICKEVENTCOUNT; i++)
        {
            if (tTimerEvent[i].active)
            {
                tTimerEvent[i].curTick--;

                if (tTimerEvent[i].curTick == 0)
                {
                    (*tTimerEvent[i].funPtr)();
                    tTimerEvent[i].curTick = tTimerEvent[i].initTick;
                }
            }
        }
    }
}

void TickInitTickEvent(void)
{
    uint8_t i = 0;

    _sys_uTimerEventCount = 0;

    /* Remove all callback function */
    for (i = 0; i < TICKEVENTCOUNT; i++)
        TickClearTickEvent(i);

    _sys_uTimerEventCount = 0;
}
#endif 

void SysTick_Handler(void)
{

    systick_counter();

    if (get_systick() >= 0xFFFFFFFF)
    {
        set_systick(0);      
    }

    // if ((get_systick() % 1000) == 0)
    // {
       
    // }

    #if defined (ENABLE_TICK_EVENT)
    TickCheckTickEvent();
    #endif    
}

void SysTick_delay(unsigned long delay)
{  
    
    uint32_t tickstart = get_systick(); 
    uint32_t wait = delay; 

    while((get_systick() - tickstart) < wait) 
    { 
    } 

}

void SysTick_enable(int ticks_per_second)
{
    set_systick(0);
    if (SysTick_Config(SystemCoreClock / ticks_per_second))
    {
        /* Setup SysTick Timer for 1 second interrupts  */
        printf("Set system tick error!!\n");
        while (1);
    }

    #if defined (ENABLE_TICK_EVENT)
    TickInitTickEvent();
    #endif
}

void Data_flash_loop(void)
{
	uint32_t idx = 0;
	uint32_t i = 0;

		idx = 0xF0;
		DataFlashRead(idx , BUFFER_PAGE_SIZE , (uint32_t) &Storage_Block[0]);
		printf("0x%2X : 0x%4X \r\n" , idx , Storage_Block[0] );
		Storage_Block[0] += 2;
		DataFlashWrite(idx , BUFFER_PAGE_SIZE , (uint32_t) &Storage_Block[0]);		

		idx = 0x10;
		DataFlashRead(idx , BUFFER_PAGE_SIZE , (uint32_t) &Storage_Block[0]);
		printf("0x%2X : 0x%4X \r\n" , idx , Storage_Block[0] );
		Storage_Block[0] += 1;
		DataFlashWrite(idx , BUFFER_PAGE_SIZE , (uint32_t) &Storage_Block[0]);		
		
		idx = 0x20;
		DataFlashRead(idx , BUFFER_PAGE_SIZE , (uint32_t) &Storage_Block[0]);
		printf("0x%2X : 0x%4X \r\n" , idx , Storage_Block[0] );
		Storage_Block[0] += 1;
		DataFlashWrite(idx , BUFFER_PAGE_SIZE , (uint32_t) &Storage_Block[0]);	

		// need to align uint8_t to uint32_t  
		for (i = 0 ; i < 16 ; i++)
		{
			idx = 0x30 + i*4;
			DataFlashRead(idx , BUFFER_PAGE_SIZE , (uint32_t) &Storage_Block[0] );
			printf("0x%2X : 0x%4X  , " , idx , Storage_Block[0]);
			Storage_Block[0] += 1;
			DataFlashWrite(idx , BUFFER_PAGE_SIZE , (uint32_t) &Storage_Block[0] );
	        if ((i+1)%4 ==0)
	        {
	            printf("\r\n");
	        }  			
		}
		printf("\r\n");    
}

void Data_flash_test(void)
{

    uint32_t i = 0;
	uint32_t idx = 0;


    idx = 0xF0;
	Storage_Block[0] = 0x0A;	
	DataFlashWrite(idx , BUFFER_PAGE_SIZE , (uint32_t)&Storage_Block[0] );

    idx = 0x10;
	Storage_Block[0] = 0x1A;	
	DataFlashWrite(idx , BUFFER_PAGE_SIZE , (uint32_t)&Storage_Block[0] );
	
    idx = 0x20;    
	Storage_Block[0] = 0x2A;
	DataFlashWrite(idx , BUFFER_PAGE_SIZE , (uint32_t)&Storage_Block[0] );

	for ( i = 0 ; i < 16 ; i++)
	{
		Storage_Block[0] = 0x3A + i;
		DataFlashWrite(0x30 + i*4 , BUFFER_PAGE_SIZE , (uint32_t)&Storage_Block[0] );
	}
				

    idx = 0xF0;
    DataFlashRead(idx , BUFFER_PAGE_SIZE , (uint32_t) &Storage_Block[0]);
    printf("0x%2X : 0x%4X \r\n" , idx , Storage_Block[0] );
    Storage_Block[0] += 2;
    DataFlashWrite(idx , BUFFER_PAGE_SIZE , (uint32_t) &Storage_Block[0]);		

    idx = 0x10;
    DataFlashRead(idx , BUFFER_PAGE_SIZE , (uint32_t) &Storage_Block[0]);
    printf("0x%2X : 0x%4X \r\n" , idx , Storage_Block[0] );
    Storage_Block[0] += 1;
    DataFlashWrite(idx , BUFFER_PAGE_SIZE , (uint32_t) &Storage_Block[0]);		

    idx = 0x20;
    DataFlashRead(idx , BUFFER_PAGE_SIZE , (uint32_t) &Storage_Block[0]);
    printf("0x%2X : 0x%4X \r\n" , idx , Storage_Block[0] );
    Storage_Block[0] += 1;
    DataFlashWrite(idx , BUFFER_PAGE_SIZE , (uint32_t) &Storage_Block[0]);	    
		    

}

void TMR1_IRQHandler(void)
{
	
    if(TIMER_GetIntFlag(TIMER1) == 1)
    {
        TIMER_ClearIntFlag(TIMER1);
		tick_counter();

		if ((get_tick() % 1000) == 0)
		{
            FLAG_MISC_TIMER_PERIOD_1000MS = 1;//set_flag(flag_timer_period_1000ms ,ENABLE);
		}

		if ((get_tick() % 50) == 0)
		{

		}	
    }
}

void TIMER1_Init(void)
{
    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, 1000);
    TIMER_EnableInt(TIMER1);
    NVIC_EnableIRQ(TMR1_IRQn);	
    TIMER_Start(TIMER1);
}

void loop(void)
{
	// static uint32_t LOG1 = 0;
	// static uint32_t LOG2 = 0;

    if ((get_systick() % 1000) == 0)
    {
        // printf("%s(systick) : %4d\r\n",__FUNCTION__,LOG2++);    
    }

    if (FLAG_MISC_TIMER_PERIOD_1000MS)//(is_flag_set(flag_timer_period_1000ms))
    {
        FLAG_MISC_TIMER_PERIOD_1000MS = 0;//set_flag(flag_timer_period_1000ms ,DISABLE);

        // printf("%s(timer) : %4d\r\n",__FUNCTION__,LOG1++);
        PB14 ^= 1;        
        Data_flash_loop();
    }
}

void UARTx_Process(void)
{
	uint8_t res = 0;
	res = UART_READ(UART0);

	if (res > 0x7F)
	{
		printf("invalid command\r\n");
	}
	else
	{
		switch(res)
		{
			case '1':
				break;

			case 'X':
			case 'x':
			case 'Z':
			case 'z':
				NVIC_SystemReset();		
				break;
		}
	}
}

void UART02_IRQHandler(void)
{
    if(UART_GET_INT_FLAG(UART0, UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk))     /* UART receive data available flag */
    {
        while(UART_GET_RX_EMPTY(UART0) == 0)
        {
			UARTx_Process();
        }
    }

    if(UART0->FIFOSTS & (UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_RXOVIF_Msk))
    {
        UART_ClearIntFlag(UART0, (UART_INTSTS_RLSINT_Msk| UART_INTSTS_BUFERRINT_Msk));
    }	
}

void UART0_Init(void)
{
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
    UART_EnableInt(UART0, UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk);
    NVIC_EnableIRQ(UART02_IRQn);
	
	#if (_debug_log_UART_ == 1)	//debug
	printf("\r\nCLK_GetCPUFreq : %8d\r\n",CLK_GetCPUFreq());
	printf("CLK_GetHCLKFreq : %8d\r\n",CLK_GetHCLKFreq());
	printf("CLK_GetHXTFreq : %8d\r\n",CLK_GetHXTFreq());
	printf("CLK_GetLXTFreq : %8d\r\n",CLK_GetLXTFreq());	
	printf("CLK_GetPCLK0Freq : %8d\r\n",CLK_GetPCLK0Freq());
	printf("CLK_GetPCLK1Freq : %8d\r\n",CLK_GetPCLK1Freq());	
	#endif	

    #if 0
    printf("FLAG_MISC_ERROR : 0x%2X\r\n",FLAG_MISC_ERROR);
    printf("FLAG_MISC_TIMER_PERIOD_1000MS : 0x%2X\r\n",FLAG_MISC_TIMER_PERIOD_1000MS);
    printf("FLAG_MISC_SYSTICK_PERIOD_1000MS : 0x%2X\r\n",FLAG_MISC_SYSTICK_PERIOD_1000MS);
    printf("FLAG_MISC_REVERSE1 : 0x%2X\r\n",FLAG_MISC_REVERSE1);
    printf("FLAG_MISC_REVERSE2 : 0x%2X\r\n",FLAG_MISC_REVERSE2);
    printf("FLAG_MISC_REVERSE3 : 0x%2X\r\n",FLAG_MISC_REVERSE3);
    printf("FLAG_MISC_REVERSE4 : 0x%2X\r\n",FLAG_MISC_REVERSE4);
    printf("FLAG_MISC_REVERSE5 : 0x%2X\r\n",FLAG_MISC_REVERSE5);
    #endif

}

void GPIO_Init (void)
{
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB14MFP_Msk)) | (SYS_GPB_MFPH_PB14MFP_GPIO);
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB15MFP_Msk)) | (SYS_GPB_MFPH_PB15MFP_GPIO);

    GPIO_SetMode(PB, BIT14, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PB, BIT15, GPIO_MODE_OUTPUT);	

}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);
    
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

//    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);
//    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

//    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);
//    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk);	

//    CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);
//    CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk);	

    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    CLK_EnableModuleClock(UART0_MODULE);
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    // CLK_EnableModuleClock(TMR0_MODULE);
  	// CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HIRC, 0);

    CLK_EnableModuleClock(TMR1_MODULE);
  	CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_HIRC, 0);

    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk)) |
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

   /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Lock protected registers */
    SYS_LockReg();
}

/*
 * This is a template project for M031 series MCU. Users could based on this project to create their
 * own application without worry about the IAR/Keil project settings.
 *
 * This template application uses external crystal as HCLK source and configures UART0 to print out
 * "Hello World", users may need to do extra system configuration based on their system design.
 */

int main()
{
    SYS_Init();

	GPIO_Init();
	UART0_Init();
	TIMER1_Init();

    SysTick_enable(1000);
    #if defined (ENABLE_TICK_EVENT)
    TickSetTickEvent(1000, TickCallback_processA);  // 1000 ms
    TickSetTickEvent(5000, TickCallback_processB);  // 5000 ms
    #endif

    DataFlashInit();
    Data_flash_test();

    /* Got no where to go, just loop forever */
    while(1)
    {
        loop();

    }
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
