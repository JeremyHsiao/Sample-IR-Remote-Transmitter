/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * $Revision: 1 $
 * $Date: 14/07/10 10:14a $
 * @brief    Template project for ISD9300 MCU
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
//#include "ISD9100.h"
#include "N575.h"
#include "NVTTypes.h"
#include "bod.h"
#include "gpio.h"
#include "fmc.h"
#include "FMCRW.h"

#define ISP_PIN ((PB->PIN)&BIT0)	//SW 1003

//#define UART_ONLY
#define HID_MAX_PACKET_SIZE_EP1		64//8
#define E_SUCCESS		0
#define S_OK			E_SUCCESS
#define FW_VERSION		0x23

//#define SUPPORT_WRITECKSUM
#define USING_AUTODETECT //using autodetect for UART download
//#define USING_RS485 //default using GPD13 act as Rx/Tx switch for RS485 tranceiver


#define CMD_UPDATE_APROM	0x000000A0
#define CMD_UPDATE_CONFIG	0x000000A1
#define CMD_READ_CONFIG		0x000000A2
#define CMD_ERASE_ALL		0x000000A3
#define CMD_SYNC_PACKNO		0x000000A4
#define CMD_GET_FWVER		0x000000A6
#define CMD_RUN_APROM		0x000000AB
#define CMD_RUN_LDROM		0x000000AC
#define CMD_RESET			0x000000AD
#define CMD_CONNECT			0x000000AE
#define CMD_DISCONNECT		0x000000AF

#define CMD_GET_DEVICEID	0x000000B1

#define CMD_UPDATE_DATAFLASH 	0x000000C3
#define CMD_WRITE_CHECKSUM 	 	0x000000C9
#define CMD_GET_FLASHMODE 	 	0x000000CA

#define CMD_RESEND_PACKET       0x000000FF

#define	V6M_AIRCR_VECTKEY_DATA	0x05FA0000UL
#define V6M_AIRCR_SYSRESETREQ	0x00000004UL

#define DISCONNECTED	0
#define CONNECTING		1
#define CONNECTED		2

static uint8_t volatile	bufhead;
static uint8_t volatile	g_connStatus;
__align(4) static uint8_t uart_rcvbuf[64];
__align(4) static uint8_t uart_sendbuf[64];
__align(4) static uint8_t aprom_buf[FMC_FLASH_PAGE_SIZE];//PAGE_SIZE];
BOOL bUsbDataReady, bUartDataReady, bUsingUART;
BOOL bUsbInReady, bUpdateApromCmd;
uint32_t g_apromSize, g_dataFlashAddr, g_dataFlashSize;
volatile uint32_t g_pdid, g_timecnt;
#ifdef SUPPORT_WRITECKSUM
static uint32_t g_ckbase = (0x20000 - 8);
static void CheckCksumBase(void);
#endif

void my_memcpy(void *dest, void *src, int32_t size);
extern void WordsCpy(void *dest, void *src, int32_t size);
void SysTimerDelay(uint32_t us);



void UART0_IRQHandler(void)//uart->INTSTS
{
	//if (u32IntStatus & 0x11) //RDA FIFO interrupt & RDA timeout interrupt
	if( UART0->INTSTS & 0x11) //RDA FIFO interrupt & RDA timeout interrupt		
  {
   		while(((inpw(&UART0->FIFOSTS) & (0x4000)) == 0) && (bufhead<64))//RX fifo not empty
			uart_rcvbuf[bufhead++] = inpw(&UART0->DAT);
	}

	if(bufhead == 64)
	{
		bUartDataReady = TRUE;
		bufhead = 0;
	}
}


/****** send data for UART*******/
static __inline void PutString()
{
	int i;

	for(i = 0; i<HID_MAX_PACKET_SIZE_EP1; i++)
	{
		while(((UART0->FIFOSTS & UART_FIFOSTS_TXPTR_Msk)>>UART_FIFOSTS_TXPTR_Pos)/*g_pUART->FSR.TX_POINTER*/ >= 6);
		//while(g_pUART->FSR.TX_EMPTY == 0);
		UART0 -> DAT/*g_pUART->DATA*/ = uart_sendbuf[i];
	}
}

static uint16_t Checksum (unsigned char *buf, int len)
{
    int i;
    uint16_t c;

    for (c=0, i=0; i < len; i++) {
        c += buf[i];
    }
    return (c);
}

static uint16_t CalCheckSum(uint32_t start, uint32_t len)
{
	int i;
	uint16_t lcksum = 0;

	for(i = 0; i < len; i+=FMC_FLASH_PAGE_SIZE)//PAGE_SIZE)
	{
		ReadData(start + i, start + i + FMC_FLASH_PAGE_SIZE/*PAGE_SIZE*/, (uint32_t*)aprom_buf);
		if(len - i >= FMC_FLASH_PAGE_SIZE/*PAGE_SIZE*/)
			lcksum += Checksum(aprom_buf, FMC_FLASH_PAGE_SIZE/*PAGE_SIZE*/);
		else
			lcksum += Checksum(aprom_buf, len - i);
	}

    return lcksum;

}

static int ParseCmd(unsigned char *buffer, uint8_t len, BOOL bUSB)
{
	static uint32_t StartAddress, StartAddress_bak, TotalLen, TotalLen_bak, LastDataLen, g_packno = 1;
	uint8_t *response;
	uint16_t cksum, lcksum;
	uint32_t	lcmd, packno, srclen, i, regcnf0, security;
	unsigned char *pSrc;
	static uint32_t	gcmd;

	response = uart_sendbuf;

	pSrc = buffer;
	srclen = len;

	lcmd = inpw(pSrc);
	packno = inpw(pSrc + 4);
	outpw(response+4, 0);

	pSrc += 8;
	srclen -= 8;

	ReadData(Config0, Config0 + 8, (uint32_t*)(response+8));//read config
	regcnf0 = *(uint32_t*)(response + 8);
	security = regcnf0 & 0x2;

	if(lcmd == CMD_SYNC_PACKNO)
	{
		g_packno = inpw(pSrc);
	}
	if((packno != g_packno) && ((lcmd != CMD_CONNECT))) //Connection state, packno always == 1
		goto out;


	if((lcmd)&&(lcmd!=CMD_RESEND_PACKET))
		gcmd = lcmd;

	if(lcmd == CMD_GET_FWVER)
		response[8] = FW_VERSION;//version 2.3
	else if(lcmd == CMD_READ_CONFIG)
	{
		*(response+8)=FMC_Read(FMC_CONFIG_BASE);//DrvFMC_Read(Config0, (uint32_t*)(response+8));
		*(response+12)=FMC_Read(FMC_CONFIG_BASE+4);//DrvFMC_Read(Config1, (uint32_t*)(response+12));
	}
	else if(lcmd == CMD_GET_DEVICEID)
	{
		//outpw(response+8, 0x00012009);
		outpw(response+8, SYS->PDID);			//Need to modify Windows AP to accept ISD9160
		goto out;
	}
	else if(lcmd == CMD_RUN_APROM || lcmd == CMD_RUN_LDROM || lcmd == CMD_RESET)
	{
		outpw(&SYS->RSTSTS, 3);//clear bit
		/* Set BS */
		if(lcmd == CMD_RUN_APROM)
		{
			i = (inpw(&FMC->ISPCTL) & 0xFFFFFFFC);
		}
		else if(lcmd == CMD_RUN_LDROM)
		{
			i = (inpw(&FMC->ISPCTL) & 0xFFFFFFFC);
			i |= 0x00000002;
		}
		else
		{
			i = (inpw(&FMC->ISPCTL) & 0xFFFFFFFE);//ISP disable
		}

		outpw(&FMC->ISPCTL, i);
		outpw(&SCB->AIRCR, (V6M_AIRCR_VECTKEY_DATA | V6M_AIRCR_SYSRESETREQ));

		/* Trap the CPU */
		while(1);
	}
	else if(lcmd == CMD_CONNECT)
	{
		//g_connStatus = CONNECTED;
		g_packno = 1;
		goto out;
	}
	//else if(lcmd == CMD_DISCONNECT)
	//{
		//g_connStatus = DISCONNECTED;
	//}

	else if((lcmd == CMD_UPDATE_APROM) || (lcmd == CMD_ERASE_ALL))
	{

		if((regcnf0 & 0x4) && ((security == 0)||(lcmd == CMD_ERASE_ALL)))//erase APROM + data flash
		{
			EraseAP(FALSE, 0, (g_apromSize < 0x20000)?0x20000:g_apromSize);//erase all aprom including data flash
		}
		else
			EraseAP(TRUE, 0, g_apromSize);//don't erase data flash


		bUpdateApromCmd = TRUE;
	}


#ifdef SUPPORT_WRITECKSUM
	else if(lcmd == CMD_WRITE_CHECKSUM)//write cksum to aprom last
	{
		cktotallen = inpw(pSrc);
		lcksum = inpw(pSrc+4);
		CheckCksumBase();
		ReadData(g_ckbase & 0xFFE00, g_ckbase, (uint32_t*)aprom_buf);
		outpw(aprom_buf+FMC_FLASH_PAGE_SIZE/*PAGE_SIZE*/ - 8, cktotallen);
		outpw(aprom_buf+FMC_FLASH_PAGE_SIZE/*PAGE_SIZE*/ - 4, lcksum);
		FMC_Erase/*DrvFMC_Erase*/(g_ckbase & 0xFFE00);
		WriteData(g_ckbase & 0xFFE00, g_ckbase + 8, (uint32_t*)aprom_buf);
	}
#endif
	else if(lcmd == CMD_GET_FLASHMODE)
	{
		//return 1: APROM, 2: LDROM
		outpw(response+8, (inpw(&FMC->ISPCTL)&0x2)? 2 : 1);
	}


	if((lcmd == CMD_UPDATE_APROM) || (lcmd == CMD_UPDATE_DATAFLASH))
	{
		if(lcmd == CMD_UPDATE_DATAFLASH)
		{
			StartAddress = g_dataFlashAddr;

			if(g_dataFlashSize)//g_dataFlashAddr
				EraseAP(FALSE, g_dataFlashAddr, g_dataFlashAddr + g_dataFlashSize);
			else goto out;
		}
		else
		{
			StartAddress = 0;
		}

		//StartAddress = inpw(pSrc);
		TotalLen = inpw(pSrc+4);
		pSrc += 8;
		srclen -= 8;
		StartAddress_bak = StartAddress;
		TotalLen_bak = TotalLen;
		//printf("StartAddress=%x,TotalPadLen=%d\n",StartAddress, TotalPadLen);
		//return 0;
	}
	else if(lcmd == CMD_UPDATE_CONFIG)			//Not suppot
	{
		//if((security == 0) && (!bUpdateApromCmd))//security lock
			//goto out;

		//UpdateConfig((uint32_t*)(pSrc), (uint32_t*)(response+8));
		//GetDataFlashInfo(&g_dataFlashAddr, &g_dataFlashSize);

		goto out;
	}
	else if(lcmd == CMD_RESEND_PACKET)//for APROM&Data flash only
	{
		StartAddress -= LastDataLen;
		TotalLen += LastDataLen;
		if((StartAddress & 0xFFE00) >= Config0)
			goto out;
		ReadData(StartAddress & 0xFFE00, StartAddress, (uint32_t*)aprom_buf);
		FMC_Erase/*DrvFMC_Erase*/(StartAddress & 0xFFE00);
		WriteData(StartAddress & 0xFFE00, StartAddress, (uint32_t*)aprom_buf);
		if((StartAddress%FMC_FLASH_PAGE_SIZE/*PAGE_SIZE*/) >= (FMC_FLASH_PAGE_SIZE/*PAGE_SIZE*/-LastDataLen))
	    	FMC_Erase/*DrvFMC_Erase*/((StartAddress & 0xFFE00)+FMC_FLASH_PAGE_SIZE/*PAGE_SIZE*/);
		goto out;

	}

	if((gcmd == CMD_UPDATE_APROM) || (gcmd == CMD_UPDATE_DATAFLASH))
	{
		if(TotalLen >= srclen)
			TotalLen -= srclen;
		else{
			srclen = TotalLen;//prevent last package from over writing
			TotalLen = 0;
	    }

		WriteData(StartAddress, StartAddress + srclen, (uint32_t*)pSrc);
		//memset(pSrc, 0, srclen);
		{
			uint32_t *p = (uint32_t*)pSrc;
			for(i = 0; i<srclen/4; i++)
				p[i] = 0;
		}
		ReadData(StartAddress, StartAddress + srclen, (uint32_t*)pSrc);
		StartAddress += srclen;
		LastDataLen =  srclen;
		if(TotalLen == 0)
		{
			lcksum = CalCheckSum(StartAddress_bak, TotalLen_bak);
			outps(response + 8, lcksum);
		}
	}
out:
	cksum = Checksum(buffer, len);
	//NVIC_DisableIRQ(USBD_IRQn);
	outps(response, cksum);
	++g_packno;
	outpw(response+4, g_packno);
	g_packno++;
	//NVIC_EnableIRQ(USBD_IRQn);
	//TotalLen -= srclen;

	return 0;
}


#define SYSTICK_ENABLE              0                                          /* Config-Bit to start or stop the SysTick Timer                         */
#define SYSTICK_TICKINT             1                                          /* Config-Bit to enable or disable the SysTick interrupt                 */
#define SYSTICK_CLKSOURCE           2                                          /* Clocksource has the offset 2 in SysTick Control and Status Register   */
#define SYSTICK_MAXCOUNT       ((1<<24) -1)                                    /* SysTick MaxCount                                                      */
void SysTimerDelay(uint32_t us)
{
    SysTick->LOAD = us * 49; /* Assume the internal 49MHz RC used */
    SysTick->VAL  =  (0x00);
    SysTick->CTRL = (1 << SYSTICK_CLKSOURCE) | (1<<SYSTICK_ENABLE);

    /* Waiting for down-count to zero */
    while((SysTick->CTRL & (1 << 16)) == 0);
}


#ifdef SUPPORT_WRITECKSUM
static void CheckCksumBase()
{
	//skip data flash
	unsigned int aprom_end = g_apromSize, data;
	int result;

	result = DrvFMC_Read(0x1F000 - 8, &data);
	if(result == 0)//128K flash
    {
    	DrvFMC_Read(Config0, &data);
    	if((data&0x01)==0)//DFEN enable
    	{
    		DrvFMC_Read(Config1, &aprom_end);
    	}
    	g_ckbase = aprom_end - 8;
    	return;
    }
    else// less than 128K
    {
    	aprom_end = 0x10000;//64K
    	do{
		    result = DrvFMC_Read(aprom_end - 8, &data);
		    if(result == 0)
		    	break;
    		aprom_end = aprom_end/2;
    	}while(aprom_end > 4096);

		g_ckbase = aprom_end - 8;
	}
}
#endif


void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable External OSC49M */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
	
    /* Switch HCLK clock source to OSC48M */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKSEL0_HIRCFSEL_48M, CLK_CLKDIV0_HCLK(1));//, FALSE);

	/* Set ADC clock from HCLK */
    CLK_SetModuleClock(ADC_MODULE, MODULE_NoMsk, CLK_CLKDIV0_ADC(1));
	
	/* Set DPWM clock from HIRC2X */
	CLK_SetModuleClock(DPWM_MODULE, CLK_CLKSEL1_DPWMSEL_HIRC2X, MODULE_NoMsk);
	
	/* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();
#ifndef DEBUG_ENABLE_SEMIHOST
    /* Set GPG multi-function pins for UART0 RXD and TXD */
	SYS->GPA_MFP  = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA8MFP_Msk) ) | SYS_GPA_MFP_PA8MFP_UART_TX;
	SYS->GPA_MFP  = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA9MFP_Msk) ) | SYS_GPA_MFP_PA9MFP_UART_RX;
#endif

	SYS->GPB_MFP  = (SYS->GPB_MFP & (~SYS_GPB_MFP_PB0MFP_Msk) ) | SYS_GPB_MFP_PB0MFP_GPIO;

    GPIO_SetMode(PB, BIT0, GPIO_MODE_QUASI);

    /* Lock protected registers */
    SYS_LockReg();
}

void UART_Init(void)
{
    /* Reset IP */
	CLK_EnableModuleClock(UART_MODULE);
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate(115200) */
    UART_Open( UART0,115200 );//UART_Open( UART0,115200 );//109714
	NVIC_EnableIRQ(UART0_IRQn);
	UART_ENABLE_INT(UART0,UART_INTEN_RDAIEN_Msk);
}







/*---------------------------------------------------------------------------------------------------------*/
/* Main function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{	
	volatile uint32_t pinst;
    uint8_t volatile	bufhead_bak=0;
//	uint32_t u32Config[4] = {0xffffffff,0xffffffff,0xffffffff,0xffffffff};//SW 1003
#ifdef SUPPORT_WRITECKSUM
  uint32_t totallen, cksum;
#endif

	/* Lock protected registers */
    if(SYS->REGLCTL == 1) // In end of main function, program issued CPU reset and write-protection will be disabled.
		SYS_LockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init(); //In the end of SYS_Init() will issue SYS_LockReg() to lock protected register. If user want to write protected register, please issue SYS_UnlockReg() to unlock protected register.


	SYS_UnlockReg();
	FMC_Open();

	g_apromSize = 141 * 0x400;	//141KB				//Need to modify if IC changed
	GetDataFlashInfo(&g_dataFlashAddr, &g_dataFlashSize);

	//g_pUART = UART0;	//SW 0115 should be not necessary
	
//#ifndef DEBUG_ENABLE_SEMIHOST
    /* Init UART for printf */
    UART_Init();
//#endif

    // if(ISP_PIN) //SW 1003
	if((ISP_PIN)&&(!SYS_IS_CPU_RST()))      // Not return to APROM (i.e. enter ISP mode) when (PB0 is high) and boot due to CPU RESET (from FMC) is low OR was boot from APROM
		goto _APROMBOOT;

#if defined(USING_AUTODETECT)
    //timeout 30ms
    SysTick->LOAD = 30000 * 48; /* using 48MHz cpu clock*/
    SysTick->VAL   =  (0x00);
    SysTick->CTRL = (1 << SysTick_CTRL_CLKSOURCE_Pos)
					| (1<<SysTick_CTRL_ENABLE_Pos);//using cpu clock


    while(1)
    {
		uint32_t	lcmd;

		if((bufhead >= 4) || (bUartDataReady == TRUE))
		{
			lcmd = inpw(uart_rcvbuf);
			if(lcmd == CMD_CONNECT)
			{
				//pinst = 0;
				//bUsingUART = TRUE;
				goto _ISP;
				//break;
			}
			else
			{
				bUartDataReady = FALSE;
				bufhead = 0;
			}
		}


    }

#endif


#ifdef SUPPORT_WRITECKSUM
	CheckCksumBase();

    totallen=FMC_Read(g_ckbase);//DrvFMC_Read(g_ckbase, &totallen);
	cksum=FMC_Read(g_ckbase+4);//DrvFMC_Read(g_ckbase+4, &cksum);

    if((pinst == 0) || ((inpw(&SYS->RSTSRC)&0x3) == 0x00)
    	|| (totallen > g_apromSize) || (CalCheckSum(0x0, totallen) != cksum))//if GPIO low or SYSRESETREQ reset or checksum error, run ISP
//#else
	//if((inpw(&SYS->RSTSRC)&0x3) == 0x00)//if GPIO low or SYSRESETREQ reset, run ISP
//	{
//	}
#endif

_ISP:
    while(1)
    {
    		if(bUartDataReady == TRUE)
	   		{
    			bUartDataReady = FALSE;
	    		g_timecnt = 0;
    			ParseCmd(uart_rcvbuf, HID_MAX_PACKET_SIZE_EP1, FALSE);
	   			PutString();
	    	}
    		//timeout happen; but byte is less than 64 bytes; host goes wrong
			if(bufhead > 0)
			{
				if(g_timecnt == 0)
					bufhead_bak = bufhead;
				SysTimerDelay(1);//0.5us
				g_timecnt++;
				if(g_timecnt > 2000)//1ms
				{
					g_timecnt = 0;
					if(bufhead_bak == bufhead)
					{
						bufhead = 0;
					}
				}
			}

	}
		
/*SW 1003*/

_APROMBOOT:	
//	SYS_UnlockReg();
//	FMC_EnableConfigUpdate();
//	FMC_WriteConfig(u32Config,4);	
//	SYS_ResetChip();
       	//FMC_Open();
        //FMC_SetBootSource(0);   // Boot from APROM     
        //FMC->ISPCTL |= FMC_ISPCTL_SWRST_Msk;    // Software RESET
        //SYS_LockReg();

        SYS_UnlockReg();
        //SYS_ClearResetSrc(SYS_RSTSTS_CPURF_Msk);
		outpw(&SYS->RSTSTS, 3);//clear bit
		outpw(&FMC->ISPCTL, (inpw(&FMC->ISPCTL) & 0xFFFFFFFC));
		outpw(&SCB->AIRCR, (V6M_AIRCR_VECTKEY_DATA | V6M_AIRCR_SYSRESETREQ));
        SYS_LockReg();
        
		/* Trap the CPU */
		while(1);



}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/

