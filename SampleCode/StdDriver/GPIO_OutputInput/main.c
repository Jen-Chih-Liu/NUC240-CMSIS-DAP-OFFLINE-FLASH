/**************************************************************************//**
 * @file     main.c
 * @version  V2.00
 * $Revision: 3 $
 * $Date: 15/04/10 10:26a $
 * @brief    NUC230_240 Series GPIO Driver Sample Code
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NUC230_240.h"
#include "SWD_host.h"
#include "SWD_flash.h"
#include "algo/M031_AP_32.c"

uint32_t Flash_Page_Size = 512;
uint32_t Flash_Start_Addr = 0x08000000;

uint8_t demo_code[1024];

uint8_t buff[512] = {0};

#define PLL_CLOCK           72000000


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 22.1184MHz clock */
    CLK_EnableXtalRC(CLK_PWRCON_OSC22M_EN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_OSC22M_STB_Msk);

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HIRC, CLK_CLKDIV_HCLK(1));

    /* Enable external XTAL 12MHz clock */
    CLK_EnableXtalRC(CLK_PWRCON_XTL12M_EN_Msk);

    /* Waiting for external XTAL clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_XTL12M_STB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_CLKDIV_UART(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFP &= ~(SYS_GPB_MFP_PB0_Msk | SYS_GPB_MFP_PB1_Msk);
    SYS->GPB_MFP |= (SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD);

}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    unsigned int j;
	for(j=0;j<1024;j++)
	{
	demo_code[j]=j&0xff;
	}

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
  swd_init_debug();
	
	target_flash_init(Flash_Start_Addr);
	
	for(uint32_t addr = 0; addr < sizeof(demo_code); addr += Flash_Page_Size)
	{
		target_flash_erase_sector(addr);
	}
	
	for(uint32_t addr = 0; addr < sizeof(demo_code); addr += Flash_Page_Size)
	{
		swd_read_memory(addr, buff, Flash_Page_Size);
		for(uint32_t i = 0; i < Flash_Page_Size; i++) 
		printf("%02X ", buff[i]);
		printf("\r\n\r\n\r\n");
	}
	
	for(uint32_t addr = 0; addr < sizeof(demo_code); addr += Flash_Page_Size)
	{
		target_flash_program_page(addr, &demo_code[addr], Flash_Page_Size);
	}
	
	for(uint32_t addr = 0; addr < sizeof(demo_code); addr += Flash_Page_Size)
	{
		swd_read_memory(addr, buff, Flash_Page_Size);
		for(uint32_t i = 0; i < Flash_Page_Size; i++) 
		printf("%02X ", buff[i]);
		printf("\r\n\r\n\r\n");
	}
	
	//swd_set_target_state_hw(RUN);

    while(1);
}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
