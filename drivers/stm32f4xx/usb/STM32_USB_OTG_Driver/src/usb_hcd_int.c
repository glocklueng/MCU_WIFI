/**
 ******************************************************************************
 * @file    usb_hcd_int.c
 * @author  MCD Application Team
 * @version V2.0.0
 * @date    22-July-2011
 * @brief   Host driver interrupt subroutines
 ******************************************************************************
 * @attention
 *
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
 * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
 * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
 * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
 * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#define DEBUG
#include "usb_core.h"
#include "usb_defines.h"
#include "usb_hcd_int.h"
#include "usbh_core.h"
#include "debug.h"

#if defined   (__CC_ARM) /*!< ARM Compiler */
#pragma O0
#elif defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma O0
#elif defined (__GNUC__) /*!< GNU Compiler */
#pragma GCC optimize ("O0")
#elif defined  (__TASKING__) /*!< TASKING Compiler */
#pragma optimize=0

#endif /* __CC_ARM */

/** @addtogroup USB_OTG_DRIVER
 * @{
 */

/** @defgroup USB_HCD_INT
 * @brief This file contains the interrupt subroutines for the Host mode.
 * @{
 */


/** @defgroup USB_HCD_INT_Private_Defines
 * @{
 */
/**
 * @}
 */


/** @defgroup USB_HCD_INT_Private_TypesDefinitions
 * @{
 */
/**
 * @}
 */



/** @defgroup USB_HCD_INT_Private_Macros
 * @{
 */
/**
 * @}
 */


/** @defgroup USB_HCD_INT_Private_Variables
 * @{
 */
/**
 * @}
 */


/** @defgroup USB_HCD_INT_Private_FunctionPrototypes
 * @{
 */

static uint32_t USB_OTG_USBH_handle_sof_ISR(USB_OTG_CORE_HANDLE *pdev);
static uint32_t USB_OTG_USBH_handle_port_ISR(USB_OTG_CORE_HANDLE *pdev);
static uint32_t USB_OTG_USBH_handle_hc_ISR(USB_OTG_CORE_HANDLE *pdev);
static uint32_t USB_OTG_USBH_handle_hc_n_In_ISR(USB_OTG_CORE_HANDLE *pdev,
	uint32_t num);
static uint32_t USB_OTG_USBH_handle_hc_n_Out_ISR(USB_OTG_CORE_HANDLE *pdev,
	uint32_t num);
static uint32_t USB_OTG_USBH_handle_rx_qlvl_ISR(USB_OTG_CORE_HANDLE *pdev);
static uint32_t USB_OTG_USBH_handle_nptxfempty_ISR(USB_OTG_CORE_HANDLE *pdev);
static uint32_t USB_OTG_USBH_handle_ptxfempty_ISR(USB_OTG_CORE_HANDLE *pdev);
static uint32_t USB_OTG_USBH_handle_Disconnect_ISR(USB_OTG_CORE_HANDLE *pdev);
static uint32_t USB_OTG_USBH_handle_IncompletePeriodicXfer_ISR
	(USB_OTG_CORE_HANDLE *pdev);

/**
 * @}
 */


/** @defgroup USB_HCD_INT_Private_Functions
 * @{
 */

/**
 * @brief  HOST_Handle_ISR
 *         This function handles all USB Host Interrupts
 * @param  pdev: Selected device
 * @retval status
 */
extern USB_OTG_CORE_HANDLE USB_OTG_Core;


void USBH_OTG_ISR_Handler()
{
	USB_OTG_GINTSTS_TypeDef gintsts;
	uint32_t retval = 0;
	USB_OTG_CORE_HANDLE *pdev = &USB_OTG_Core;

	enter_interrupt();

	gintsts.d32 = USB_OTG_READ_REG32(&pdev->regs.GREGS->GINTSTS);
	gintsts.d32 &= USB_OTG_READ_REG32(&pdev->regs.GREGS->GINTMSK);
	if (!gintsts.d32)
	{
		exit_interrupt(0);
		return ;
	}

	if (gintsts.b.rxstsqlvl)
	{
		gintsts.b.rxstsqlvl = 0;
		USB_OTG_USBH_handle_rx_qlvl_ISR(pdev);
	}

	if (gintsts.b.nptxfempty)
	{
		gintsts.b.nptxfempty = 0;
		USB_OTG_USBH_handle_nptxfempty_ISR(pdev);
	}

	if (gintsts.b.hcintr)
	{
		gintsts.b.hcintr = 0;
		retval = USB_OTG_USBH_handle_hc_ISR(pdev);
	}

	if (!gintsts.d32)
		goto end;

	if (gintsts.b.sofintr)
	{
		USB_OTG_USBH_handle_sof_ISR(pdev);
	}

	if (gintsts.b.ptxfempty)
	{
		USB_OTG_USBH_handle_ptxfempty_ISR(pdev);
	}

	if (gintsts.b.portintr)
	{
		USB_OTG_USBH_handle_port_ISR(pdev);
	}

	if (gintsts.b.disconnect)
	{
		USB_OTG_USBH_handle_Disconnect_ISR(pdev);

	}

	if (gintsts.b.incomplisoout)
	{
		USB_OTG_USBH_handle_IncompletePeriodicXfer_ISR(pdev);
	}

	if (gintsts.b.datafsusp)
	{

	}

end: 
	exit_interrupt(retval);
}

/**
 * @brief  USB_OTG_USBH_handle_hc_ISR
 *         This function indicates that one or more host channels has a pending
 * @param  pdev: Selected device
 * @retval status
 */
extern USBH_HOST USB_Host;


//根据可能发送的概率排列优先级
const unsigned short chint_msk_tb[10][3] =
{
	{
		(1 << 2), 2, 1
	}
	,  //bulk in0
	{
		(1 << 3), 3, 0
	}
	,  //bulk out0
	{
		(1 << 0), 0, 1
	}
	,  //ctrol in
	{
		(1 << 1), 1, 0
	}
	,  //ctrol out
	{
		(1 << 4), 4, 0
	}
	,
	{
		(1 << 5), 5, 0
	}
	,
	{
		(1 << 6), 6, 0
	}
	,
	{
		(1 << 7), 7, 0
	}
	,
	{
		(1 << 8), 8, 0
	}
	,
	{
		(1 << 9), 9, 0
	}
};

static uint32_t inline USB_OTG_USBH_handle_hc_ISR(USB_OTG_CORE_HANDLE *pdev)
{
	USB_OTG_HAINT_TypeDef haint;
	USB_OTG_HCCHAR_TypeDef hcchar;
	uint32_t i = 0;
	uint32_t retval = 0, need_wake_up = 0;

	/* Clear appropriate bits in HCINTn to clear the interrupt bit in
	 * GINTSTS */
	//  p_dbg_enter;
	haint.d32 = USB_OTG_READ_REG32(&pdev->regs.HREGS->HAINT);

#if 0
	if (haint.b.chint & (1 << 0)){

		hcchar.d32 = USB_OTG_READ_REG32(&pdev->regs.HC_REGS[0]
->HCCHAR);
		if ( hcchar.b.epdir)
			retval = USB_OTG_USBH_handle_hc_n_In_ISR(pdev, 0);
		else
			retval = USB_OTG_USBH_handle_hc_n_Out_ISR(pdev, 0);
		if (retval){
			need_wake_up = 1;
			wake_up_urb(pdev, 0);
		}
	}

	if (haint.b.chint & (1 << 1)){

		hcchar.d32 = USB_OTG_READ_REG32(&pdev->regs.HC_REGS[1]
->HCCHAR);
		if ( hcchar.b.epdir)
			retval = USB_OTG_USBH_handle_hc_n_In_ISR(pdev, 1);
		else
			retval = USB_OTG_USBH_handle_hc_n_Out_ISR(pdev, 1);
		if (retval){
			need_wake_up = 1;
			wake_up_urb(pdev, 1);
		}
	}

	if (haint.b.chint & (1 << 2)){

		hcchar.d32 = USB_OTG_READ_REG32(&pdev->regs.HC_REGS[2]
->HCCHAR);
		if ( hcchar.b.epdir)
			retval = USB_OTG_USBH_handle_hc_n_In_ISR(pdev, 2);
		else
			retval = USB_OTG_USBH_handle_hc_n_Out_ISR(pdev, 2);
		if (retval){
			need_wake_up = 1;
			wake_up_urb(pdev, 2);
		}
	}

	if (haint.b.chint & (1 << 3)){

		hcchar.d32 = USB_OTG_READ_REG32(&pdev->regs.HC_REGS[3]
->HCCHAR);
		if ( hcchar.b.epdir)
			retval = USB_OTG_USBH_handle_hc_n_In_ISR(pdev, 3);
		else
			retval = USB_OTG_USBH_handle_hc_n_Out_ISR(pdev, 3);
		if (retval){
			need_wake_up = 1;
			wake_up_urb(pdev, 3);
		}
	}

	if (haint.b.chint & (1 << 3)){

		hcchar.d32 = USB_OTG_READ_REG32(&pdev->regs.HC_REGS[3]
->HCCHAR);
		if ( hcchar.b.epdir)
			retval = USB_OTG_USBH_handle_hc_n_In_ISR(pdev, 3);
		else
			retval = USB_OTG_USBH_handle_hc_n_Out_ISR(pdev, 3);
		if (retval){
			need_wake_up = 1;
			wake_up_urb(pdev, 3);
		}
	}
#endif
	for(i = 0;i < 8; i++)
	{
		if (haint.b.chint & (1 << i)){

			hcchar.d32 = USB_OTG_READ_REG32(&pdev->regs.HC_REGS[i]
	->HCCHAR);
			if ( hcchar.b.epdir)
				retval = USB_OTG_USBH_handle_hc_n_In_ISR(pdev, i);
			else
				retval = USB_OTG_USBH_handle_hc_n_Out_ISR(pdev, i);
			if (retval){
				need_wake_up = 1;
				wake_up_urb(pdev, i);
			}
		}
	}
	return need_wake_up;
}

/**
 * @brief  USB_OTG_otg_hcd_handle_sof_intr
 *         Handles the start-of-frame interrupt in host mode.
 * @param  pdev: Selected device
 * @retval status
 */
static uint32_t USB_OTG_USBH_handle_sof_ISR(USB_OTG_CORE_HANDLE *pdev)
{
	USB_OTG_GINTSTS_TypeDef gintsts;


	gintsts.d32 = 0;
	/* Clear interrupt */
	gintsts.b.sofintr = 1;
	USB_OTG_WRITE_REG32(&pdev->regs.GREGS->GINTSTS, gintsts.d32);

	return 1;
}

/**
 * @brief  USB_OTG_USBH_handle_Disconnect_ISR
 *         Handles disconnect event.
 * @param  pdev: Selected device
 * @retval status
 */
static uint32_t USB_OTG_USBH_handle_Disconnect_ISR(USB_OTG_CORE_HANDLE *pdev)
{
	USB_OTG_GINTSTS_TypeDef gintsts;

	pdev->host.ConnSts = 0;
	gintsts.d32 = 0;

	//  pdev->host.port_cb->Disconnect(pdev);

	/* Clear interrupt */
	gintsts.b.disconnect = 1;
	USB_OTG_WRITE_REG32(&pdev->regs.GREGS->GINTSTS, gintsts.d32);

	return 1;
}

/**
 * @brief  USB_OTG_USBH_handle_nptxfempty_ISR
 *         Handles non periodic tx fifo empty.
 * @param  pdev: Selected device
 * @retval status
 */
#if 0
static uint32_t USB_OTG_USBH_handle_nptxfempty_ISR(USB_OTG_CORE_HANDLE*pdev)
{
	USB_OTG_GINTMSK_TypeDef intmsk;
	USB_OTG_HNPTXSTS_TypeDef hnptxsts;
	uint16_t len_words, len;

	hnptxsts.d32 = USB_OTG_READ_REG32(&pdev->regs.GREGS->HNPTXSTS);
	
	#ifdef DEBUG
	if (hnptxsts.b.chnum > 0)
		p_err("nptxfempty:%d\n", hnptxsts.b.nptxfspcavail);

	if (pdev->host.hc[hnptxsts.b.chnum].xfer_len == 0)
	{
		p_err("%s xfer_len == 0, %d\n", __FUNCTION__, hnptxsts.b.chnum);
		USB_OTG_HC_Halt(pdev, hnptxsts.b.chnum); //test
		return 0;
	}
	#endif

	

	len_words = (pdev->host.hc[hnptxsts.b.chnum].xfer_len + 3) / 4;

	while ((hnptxsts.b.nptxfspcavail > len_words) && (pdev
	->host.hc[hnptxsts.b.chnum].xfer_len != 0))
	{

		len = hnptxsts.b.nptxfspcavail * 4;

		if (len > pdev->host.hc[hnptxsts.b.chnum].xfer_len)
		{
			/* Last packet */
			len = pdev->host.hc[hnptxsts.b.chnum].xfer_len;

			intmsk.d32 = 0;
			intmsk.b.nptxfempty = 1;
			USB_OTG_MODIFY_REG32(&pdev->regs.GREGS->GINTMSK, intmsk.d32, 0);
		}

		len_words = (pdev->host.hc[hnptxsts.b.chnum].xfer_len + 3) / 4;

		USB_OTG_WritePacket(pdev, pdev->host.hc[hnptxsts.b.chnum].xfer_buff,
	hnptxsts.b.chnum, len);

		pdev->host.hc[hnptxsts.b.chnum].xfer_buff += len;
		pdev->host.hc[hnptxsts.b.chnum].xfer_len -= len;
		pdev->host.hc[hnptxsts.b.chnum].xfer_count += len;
		hnptxsts.d32 = USB_OTG_READ_REG32(&pdev->regs.GREGS->HNPTXSTS);
	}

	return 1;
}

#else

static uint32_t USB_OTG_USBH_handle_nptxfempty_ISR(USB_OTG_CORE_HANDLE*pdev)
{
	int free_space;
	USB_OTG_HNPTXSTS_TypeDef hnptxsts;
	uint32_t write_len;
	char *write_buff;

	hnptxsts.d32 = USB_OTG_READ_REG32(&pdev->regs.GREGS->HNPTXSTS);
	hnptxsts.b.chnum = pdev->cur_bulk_out_ch;


	write_len = (pdev->host.hc[hnptxsts.b.chnum].xfer_len + 3) / 4;

	free_space = hnptxsts.b.nptxfspcavail - 16;

	if (free_space < write_len)
	{
		write_len = free_space * 4;
		write_buff = (char*)pdev->host.hc[hnptxsts.b.chnum].xfer_buff;
		pdev->host.hc[hnptxsts.b.chnum].xfer_len -= write_len;
		pdev->host.hc[hnptxsts.b.chnum].xfer_buff += write_len;
	}
	else
	{
		USB_OTG_GINTMSK_TypeDef intmsk;
		intmsk.d32 = 0;
		intmsk.b.nptxfempty = 1;
		USB_OTG_MODIFY_REG32(&pdev->regs.GREGS->GINTMSK, intmsk.d32, 0);

		write_buff = (char*)pdev->host.hc[hnptxsts.b.chnum].xfer_buff;
		write_len = write_len * 4;
		pdev->host.hc[hnptxsts.b.chnum].xfer_len = 0;
		pdev->bulk_out_irq_pending = 0;
	}
	USB_OTG_WritePacket(pdev, (uint8_t*)write_buff, hnptxsts.b.chnum, write_len);

	if (pdev->host.hc[hnptxsts.b.chnum].xfer_len < 0)
	{
		pdev->host.hc[hnptxsts.b.chnum].xfer_len = 0;
	}

	return 1;
}

#endif

/**
 * @brief  USB_OTG_USBH_handle_ptxfempty_ISR
 *         Handles periodic tx fifo empty
 * @param  pdev: Selected device
 * @retval status
 */
#if 0
static uint32_t USB_OTG_USBH_handle_ptxfempty_ISR(USB_OTG_CORE_HANDLE *pdev)
{
	USB_OTG_GINTMSK_TypeDef intmsk;
	USB_OTG_HPTXSTS_TypeDef hptxsts;
	uint16_t len_words, len;

	hptxsts.d32 = USB_OTG_READ_REG32(&pdev->regs.HREGS->HPTXSTS);

	len_words = (pdev->host.hc[hptxsts.b.chnum].xfer_len + 3) / 4;

	while ((hptxsts.b.ptxfspcavail > len_words) && (pdev
	->host.hc[hptxsts.b.chnum].xfer_len != 0))
	{

		len = hptxsts.b.ptxfspcavail * 4;

		if (len > pdev->host.hc[hptxsts.b.chnum].xfer_len)
		{
			len = pdev->host.hc[hptxsts.b.chnum].xfer_len;
			/* Last packet */
			intmsk.d32 = 0;
			intmsk.b.ptxfempty = 1;
			USB_OTG_MODIFY_REG32(&pdev->regs.GREGS->GINTMSK, intmsk.d32, 0);
		}

		len_words = (pdev->host.hc[hptxsts.b.chnum].xfer_len + 3) / 4;

		USB_OTG_WritePacket(pdev, pdev->host.hc[hptxsts.b.chnum].xfer_buff,
	hptxsts.b.chnum, len);

		pdev->host.hc[hptxsts.b.chnum].xfer_buff += len;
		pdev->host.hc[hptxsts.b.chnum].xfer_len -= len;
		pdev->host.hc[hptxsts.b.chnum].xfer_count += len;

		hptxsts.d32 = USB_OTG_READ_REG32(&pdev->regs.HREGS->HPTXSTS);
	}

	return 1;
}

#endif

static uint32_t USB_OTG_USBH_handle_ptxfempty_ISR(USB_OTG_CORE_HANDLE *pdev)
{

	USB_OTG_HPTXSTS_TypeDef hptxsts;
	uint32_t write_len;
	char *write_buff;

	hptxsts.d32 = USB_OTG_READ_REG32(&pdev->regs.HREGS->HPTXSTS);


	write_len = (pdev->host.hc[hptxsts.b.chnum].xfer_len + 3) / 4;

	if (hptxsts.b.ptxfspcavail <= write_len)
	{
		write_len = hptxsts.b.ptxfspcavail * 4;
		write_buff = (char*)pdev->host.hc[hptxsts.b.chnum].xfer_buff;
		pdev->host.hc[hptxsts.b.chnum].xfer_len -= write_len;
		pdev->host.hc[hptxsts.b.chnum].xfer_buff += write_len;
	}
	else
	{
		USB_OTG_GINTMSK_TypeDef intmsk;
		intmsk.d32 = 0;
		intmsk.b.ptxfempty = 1;
		USB_OTG_MODIFY_REG32(&pdev->regs.GREGS->GINTMSK, intmsk.d32, 0);

		write_buff = (char*)pdev->host.hc[hptxsts.b.chnum].xfer_buff;
		write_len = pdev->host.hc[hptxsts.b.chnum].xfer_len;

		pdev->host.hc[hptxsts.b.chnum].xfer_len = 0;
		pdev->host.hc[hptxsts.b.chnum].xfer_buff += write_len;
	}


	USB_OTG_WritePacket(pdev, (uint8_t*)write_buff, hptxsts.b.chnum, write_len);

	if (pdev->host.hc[hptxsts.b.chnum].xfer_len < 0)
	{
		pdev->host.hc[hptxsts.b.chnum].xfer_len = 0;
	}

	return 1;
}



/**
 * @brief  USB_OTG_USBH_handle_port_ISR
 *         This function determines which interrupt conditions have occurred
 * @param  pdev: Selected device
 * @retval status
 */
static uint32_t USB_OTG_USBH_handle_port_ISR(USB_OTG_CORE_HANDLE *pdev)
{
	USB_OTG_HPRT0_TypeDef hprt0;
	USB_OTG_HPRT0_TypeDef hprt0_dup;
	USB_OTG_HCFG_TypeDef hcfg;
	uint32_t do_reset = 0;
	uint32_t retval = 0;

	hcfg.d32 = 0;
	hprt0.d32 = 0;
	hprt0_dup.d32 = 0;

	hprt0.d32 = USB_OTG_READ_REG32(pdev->regs.HPRT0);
	hprt0_dup.d32 = USB_OTG_READ_REG32(pdev->regs.HPRT0);

	/* Clear the interrupt bits in GINTSTS */

	hprt0_dup.b.prtena = 0;
	hprt0_dup.b.prtconndet = 0;
	hprt0_dup.b.prtenchng = 0;
	hprt0_dup.b.prtovrcurrchng = 0;

	/* Port Connect Detected */
	if (hprt0.b.prtconndet)
	{
		//    pdev->host.port_cb->Connect(pdev);
		hprt0_dup.b.prtconndet = 1;
		do_reset = 1;
		retval |= 1;
	}

	/* Port Enable Changed */
	if (hprt0.b.prtenchng)
	{
		hprt0_dup.b.prtenchng = 1;
		if (hprt0.b.prtena == 1)
		{
			pdev->host.ConnSts = 1;

			if ((hprt0.b.prtspd == HPRT0_PRTSPD_LOW_SPEED) || (hprt0.b.prtspd ==
	HPRT0_PRTSPD_FULL_SPEED))
			{

				hcfg.d32 = USB_OTG_READ_REG32(&pdev->regs.HREGS->HCFG);

				if (hprt0.b.prtspd == HPRT0_PRTSPD_LOW_SPEED)
				{
					USB_OTG_WRITE_REG32(&pdev->regs.HREGS->HFIR, 6000);
					if (hcfg.b.fslspclksel != HCFG_6_MHZ)
					{
						if (pdev->cfg.coreID == USB_OTG_FS_CORE_ID)
						{
							USB_OTG_InitFSLSPClkSel(pdev, HCFG_6_MHZ);
						}
						do_reset = 1;
					}
				}
				else
				{

					USB_OTG_WRITE_REG32(&pdev->regs.HREGS->HFIR, 48000);
					if (hcfg.b.fslspclksel != HCFG_48_MHZ)
					{
						USB_OTG_InitFSLSPClkSel(pdev, HCFG_48_MHZ);
						do_reset = 1;
					}
				}
			}
			else
			{
				do_reset = 1;
			}
		}
	}
	/* Overcurrent Change Interrupt */
	if (hprt0.b.prtovrcurrchng)
	{
		hprt0_dup.b.prtovrcurrchng = 1;
		retval |= 1;
	}
	if (do_reset)
	{
		USB_OTG_ResetPort(pdev);

	}
	/* Clear Port Interrupts */
	USB_OTG_WRITE_REG32(pdev->regs.HPRT0, hprt0_dup.d32);

	return retval;
}

/**
 * @brief  USB_OTG_USBH_handle_hc_n_Out_ISR
 *         Handles interrupt for a specific Host Channel
 * @param  pdev: Selected device
 * @param  hc_num: Channel number
 * @retval status
 */


#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED


uint32_t USB_OTG_USBH_handle_hc_n_Out_ISR(USB_OTG_CORE_HANDLE *pdev, uint32_t
	num)
{

	USB_OTG_HCINTn_TypeDef hcint;
	USB_OTG_HCGINTMSK_TypeDef hcintmsk;
	USB_OTG_HC_REGS *hcreg;
	USB_OTG_HCCHAR_TypeDef hcchar;
	int ret = 0;
	hcreg = pdev->regs.HC_REGS[num];
	hcint.d32 = USB_OTG_READ_REG32(&hcreg->HCINT);
	hcintmsk.d32 = USB_OTG_READ_REG32(&hcreg->HCGINTMSK);
	hcint.d32 = hcint.d32 &hcintmsk.d32;
	hcchar.d32 = USB_OTG_READ_REG32(&hcreg->HCCHAR);

	//	USB_OTG_WRITE_REG32(&hcreg->HCINT, hcint.d32|0x7f);	//clear 0-10bit

	if (hcint.b.ahberr)
	{
		CLEAR_HC_INT(hcreg, ahberr);
		UNMASK_HOST_INT_CHH(num);
	}
	else if (hcint.b.ack)
	{
		CLEAR_HC_INT(hcreg, ack);
	}

	else if (hcint.b.xfercompl)
	{
		pdev->host.ErrCnt[num] = 0;
		UNMASK_HOST_INT_CHH(num);
		USB_OTG_HC_Halt(pdev, num);
		CLEAR_HC_INT(hcreg, xfercompl);
		pdev->host.HC_Status[num] = HC_XFRC;
	}

	else if (hcint.b.stall)
	{
		CLEAR_HC_INT(hcreg, stall);
		UNMASK_HOST_INT_CHH(num);
		USB_OTG_HC_Halt(pdev, num);
		pdev->host.HC_Status[num] = HC_STALL;
	}

	else if (hcint.b.nak)
	{
		pdev->host.ErrCnt[num] = 0;
		UNMASK_HOST_INT_CHH(num);
		USB_OTG_HC_Halt(pdev, num);
		CLEAR_HC_INT(hcreg, nak);
		pdev->host.HC_Status[num] = HC_NAK;
	}

	else if (hcint.b.xacterr)
	{
		UNMASK_HOST_INT_CHH(num);
		USB_OTG_HC_Halt(pdev, num);
		pdev->host.ErrCnt[num]++;
		pdev->host.HC_Status[num] = HC_XACTERR;
		CLEAR_HC_INT(hcreg, xacterr);
	}
	else if (hcint.b.nyet)
	{
		pdev->host.ErrCnt[num] = 0;
		UNMASK_HOST_INT_CHH(num);
		USB_OTG_HC_Halt(pdev, num);
		CLEAR_HC_INT(hcreg, nyet);
		pdev->host.HC_Status[num] = HC_NYET;
	}
	else if (hcint.b.datatglerr)
	{
		UNMASK_HOST_INT_CHH(num);
		USB_OTG_HC_Halt(pdev, num);
		CLEAR_HC_INT(hcreg, nak);
		pdev->host.HC_Status[num] = HC_DATATGLERR;

		CLEAR_HC_INT(hcreg, datatglerr);
	}
	else if (hcint.b.chhltd)
	{
		MASK_HOST_INT_CHH(num);
		ret = 1;
		if (pdev->host.HC_Status[num] == HC_XFRC)
		{
			pdev->host.URB_State[num] = URB_DONE;
			if (hcchar.b.eptype == EP_TYPE_BULK)
			{
				
				if (pdev->host.hc[num].packet_count & 1)
					pdev->host.hc[num].toggle_out ^= 1;
			}
		}
		else if (pdev->host.HC_Status[num] == HC_NAK)
		{
			pdev->host.URB_State[num] = URB_NOTREADY;
		}
		else if (pdev->host.HC_Status[num] == HC_NYET)
		{
			if (pdev->host.hc[num].do_ping == 1)
			{
				USB_OTG_HC_DoPing(pdev, num);
			}
			pdev->host.URB_State[num] = URB_NOTREADY;
		}
		else if (pdev->host.HC_Status[num] == HC_STALL)
		{
			pdev->host.URB_State[num] = URB_STALL;
		}
		else if (pdev->host.HC_Status[num] == HC_XACTERR)
		{
			if (pdev->host.ErrCnt[num] == 3)
			{
				pdev->host.URB_State[num] = URB_ERROR;
				pdev->host.ErrCnt[num] = 0;
			}
		}else
			ret = 0;
		
		CLEAR_HC_INT(hcreg, chhltd);
	}
	else
	{
		p_err("unkown out isr:%08x\n", hcint.d32);
	}


	return ret;
}



#else

uint32_t USB_OTG_USBH_handle_hc_n_Out_ISR(USB_OTG_CORE_HANDLE *pdev, uint32_t
	num)
{

	USB_OTG_HCINTn_TypeDef hcint;
	USB_OTG_HCGINTMSK_TypeDef hcintmsk;
	USB_OTG_HC_REGS *hcreg;
	USB_OTG_HCCHAR_TypeDef hcchar;
	int ret = 0;
	hcreg = pdev->regs.HC_REGS[num];
	hcint.d32 = USB_OTG_READ_REG32(&hcreg->HCINT);
	hcintmsk.d32 = USB_OTG_READ_REG32(&hcreg->HCGINTMSK);
	hcint.d32 = hcint.d32 &hcintmsk.d32;
	hcchar.d32 = USB_OTG_READ_REG32(&hcreg->HCCHAR);

	USB_OTG_WRITE_REG32(&hcreg->HCINT, hcint.d32 | 0x7f); //clear 0-10bit

	if (hcint.b.chhltd)
	{
		hcint.b.chhltd = 0;
		ret = 1;
		if (pdev->host.HC_Status[num] == HC_XFRC)
		{
			pdev->host.URB_State[num] = URB_DONE;
			if (hcchar.b.eptype == EP_TYPE_BULK)
			{
				
				if (pdev->host.hc[num].packet_count &1)
					pdev->host.hc[num].toggle_out ^= 1;

			}
		}
		else if (pdev->host.HC_Status[num] == HC_NAK)
		{
			pdev->host.URB_State[num] = URB_NOTREADY;
		}
		else if (pdev->host.HC_Status[num] == HC_NYET)
		{
			if (pdev->host.hc[num].do_ping == 1)
			{
				USB_OTG_HC_DoPing(pdev, num);
			}
			pdev->host.URB_State[num] = URB_NOTREADY;
		}
		else if (pdev->host.HC_Status[num] == HC_STALL)
		{
			pdev->host.URB_State[num] = URB_STALL;
		}
		else if (pdev->host.HC_Status[num] == HC_XACTERR)
		{
			if (pdev->host.ErrCnt[num] == 3)
			{
				pdev->host.URB_State[num] = URB_ERROR;
				pdev->host.ErrCnt[num] = 0;
			}
		}
		else if (pdev->host.HC_Status[num] == HC_HALTED)
		{
			pdev->host.URB_State[num] = URB_ERROR;
			pdev->host.ErrCnt[num] = 0;
		}
		else{
			ret = 0;
			p_err("unkown out chhltd:%d\n", pdev->host.HC_Status[num]);
		}
		
	}

	if (hcint.b.xfercompl)
	{
		hcint.b.xfercompl = 0;

		if (num > 0)
			pdev->host.ErrCnt[num] = 0;
		USB_OTG_HC_Halt(pdev, num);
		pdev->host.HC_Status[num] = HC_XFRC;
	}

	if(hcint.d32 == 0)
		goto end;


	if (hcint.b.ahberr)
	{
		p_err("ahberr in:%08x\n", USB_OTG_READ_REG32(&pdev->regs.HC_REGS[num]->HCDMA))
	;
	}

	if (hcint.b.ack)
	{
		p_err("ack\n");
	}

	if (hcint.b.stall)
	{
		USB_OTG_HC_Halt(pdev, num);
		pdev->host.HC_Status[num] = HC_STALL;
	}

	if (hcint.b.nak)
	{
		pdev->host.ErrCnt[num] = 0;
		USB_OTG_HC_Halt(pdev, num);
		pdev->host.HC_Status[num] = HC_NAK;
	}

	if (hcint.b.xacterr)
	{
		USB_OTG_HC_Halt(pdev, num);
		pdev->host.ErrCnt[num]++;
		pdev->host.HC_Status[num] = HC_XACTERR;
	}
	if (hcint.b.nyet)
	{
		pdev->host.ErrCnt[num] = 0;
		USB_OTG_HC_Halt(pdev, num);
		pdev->host.HC_Status[num] = HC_NYET;
	}
	if (hcint.b.datatglerr)
	{
		USB_OTG_HC_Halt(pdev, num);
		pdev->host.HC_Status[num] = HC_DATATGLERR;
	}

end:
	return ret;
}

#endif


/**
 * @brief  USB_OTG_USBH_handle_hc_n_In_ISR
 *         Handles interrupt for a specific Host Channel
 * @param  pdev: Selected device
 * @param  hc_num: Channel number
 * @retval status
 */


#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED

uint32_t USB_OTG_USBH_handle_hc_n_In_ISR(USB_OTG_CORE_HANDLE *pdev, uint32_t
	num)
{
	USB_OTG_HCINTn_TypeDef hcint;
	USB_OTG_HCGINTMSK_TypeDef hcintmsk;
	USB_OTG_HCCHAR_TypeDef hcchar;
	USB_OTG_HCTSIZn_TypeDef hctsiz;
	USB_OTG_HC_REGS *hcreg;
	int ret = 0;

	hcreg = pdev->regs.HC_REGS[num];
	hcint.d32 = USB_OTG_READ_REG32(&hcreg->HCINT);
	hcintmsk.d32 = USB_OTG_READ_REG32(&hcreg->HCGINTMSK);
	hcint.d32 = hcint.d32 &hcintmsk.d32;
	hcchar.d32 = USB_OTG_READ_REG32(&pdev->regs.HC_REGS[num]->HCCHAR);
	hcintmsk.d32 = 0;


	if (hcint.b.ahberr)
	{
		CLEAR_HC_INT(hcreg, ahberr);
		UNMASK_HOST_INT_CHH(num);
	}
	else if (hcint.b.ack)
	{
		CLEAR_HC_INT(hcreg, ack);
	}

	else if (hcint.b.stall)
	{
		UNMASK_HOST_INT_CHH(num);
		pdev->host.HC_Status[num] = HC_STALL;
		CLEAR_HC_INT(hcreg, nak); /* Clear the NAK Condition */
		CLEAR_HC_INT(hcreg, stall); /* Clear the STALL Condition */
		hcint.b.nak = 0; /* NOTE: When there is a 'stall', reset also nak,
		else, the pdev->host.HC_Status = HC_STALL
		will be overwritten by 'nak' in code below */
		USB_OTG_HC_Halt(pdev, num);
	}
	else if (hcint.b.datatglerr)
	{
		UNMASK_HOST_INT_CHH(num);
		USB_OTG_HC_Halt(pdev, num);
		CLEAR_HC_INT(hcreg, nak);
		pdev->host.HC_Status[num] = HC_DATATGLERR;
		CLEAR_HC_INT(hcreg, datatglerr);
	}

	if (hcint.b.frmovrun)
	{
		UNMASK_HOST_INT_CHH(num);
		USB_OTG_HC_Halt(pdev, num);
		CLEAR_HC_INT(hcreg, frmovrun);
	}

	else if (hcint.b.xfercompl)
	{
		pdev->host.HC_Status[num] = HC_XFRC;
		pdev->host.ErrCnt[num] = 0;
		CLEAR_HC_INT(hcreg, xfercompl);
		CLEAR_HC_INT(hcreg , nak);

		if (hcchar.b.eptype == EP_TYPE_BULK)
		{
			if (pdev->cfg.dma_enable == 1)
			{
				hctsiz.d32 = USB_OTG_READ_REG32(&pdev->regs.HC_REGS[num]->HCTSIZ);
				pdev->host.XferCnt[num] = pdev->host.hc[num].xfer_len - hctsiz.b.xfersize;

				pdev->host.hc[num].packet_count = (pdev->host.XferCnt[num] + 63) / 64;
			}
		
			if (pdev->host.hc[num].packet_count & 1)
				pdev->host.hc[num].toggle_in ^= 1;
		}
		else if (hcchar.b.eptype == EP_TYPE_CTRL)
		{
			pdev->host.hc[num].toggle_in ^= 1;
		}
		else if (hcchar.b.eptype == EP_TYPE_INTR)
		{
			hcchar.b.oddfrm = 1;
			USB_OTG_WRITE_REG32(&pdev->regs.HC_REGS[num]->HCCHAR, hcchar.d32);
			pdev->host.URB_State[num] = URB_DONE;
		}

		UNMASK_HOST_INT_CHH(num);
		USB_OTG_HC_Halt(pdev, num);

	}
	else if (hcint.b.chhltd)
	{
		ret = 1;
		MASK_HOST_INT_CHH(num);

		if (pdev->host.HC_Status[num] == HC_XFRC)
		{
			pdev->host.URB_State[num] = URB_DONE;
		}
		else if (pdev->host.HC_Status[num] == HC_STALL)
		{
			pdev->host.URB_State[num] = URB_STALL;
		}
		else if ((pdev->host.HC_Status[num] == HC_XACTERR) || (pdev
	->host.HC_Status[num] == HC_DATATGLERR))
		{
			pdev->host.ErrCnt[num] = 0;
			pdev->host.URB_State[num] = URB_ERROR;
		}else
			ret = 0;
		/*
		else if (pdev->host.HC_Status[num] == HC_NAK)
		{
		pdev->host.URB_State[num] = URB_NAK;
		need_wake_up = 1;
		} */
		

		CLEAR_HC_INT(hcreg, chhltd);

	}
	else if (hcint.b.xacterr)
	{
		UNMASK_HOST_INT_CHH(num);
		pdev->host.ErrCnt[num]++;
		pdev->host.HC_Status[num] = HC_XACTERR;
		USB_OTG_HC_Halt(pdev, num);
		CLEAR_HC_INT(hcreg, xacterr);

	}
	else if (hcint.b.nak)
	{
		if (hcchar.b.eptype == EP_TYPE_CTRL)
		{
			CLEAR_HC_INT(hcreg, nak);
			hcchar.b.chen = 1;
			hcchar.b.chdis = 0;
			USB_OTG_WRITE_REG32(&pdev->regs.HC_REGS[num]->HCCHAR, hcchar.d32);
		}
		else
		{
			UNMASK_HOST_INT_CHH(num);
			USB_OTG_HC_Halt(pdev, num);
			CLEAR_HC_INT(hcreg, nak);
		}
		pdev->host.HC_Status[num] = HC_NAK;
	}
	else
	{
		p_err("unkown in isr:%08x\n", hcint.d32);
	}


	return ret;

}


#else
uint32_t USB_OTG_USBH_handle_hc_n_In_ISR(USB_OTG_CORE_HANDLE *pdev, uint32_t
	num)
{
	USB_OTG_HCINTn_TypeDef hcint;
	USB_OTG_HCGINTMSK_TypeDef hcintmsk;
	USB_OTG_HCCHAR_TypeDef hcchar;
	USB_OTG_HCTSIZn_TypeDef hctsiz;
	USB_OTG_HC_REGS *hcreg;
	int ret = 0;

	hcreg = pdev->regs.HC_REGS[num];
	hcint.d32 = USB_OTG_READ_REG32(&hcreg->HCINT);
	hcintmsk.d32 = USB_OTG_READ_REG32(&hcreg->HCGINTMSK);
	hcint.d32 = hcint.d32 &hcintmsk.d32;
	hcchar.d32 = USB_OTG_READ_REG32(&pdev->regs.HC_REGS[num]->HCCHAR);
	hcintmsk.d32 = 0;

	USB_OTG_WRITE_REG32(&hcreg->HCINT, hcint.d32 | 0x7f); //clear 0-10bit

	if (hcint.b.chhltd)
	{
		//      MASK_HOST_INT_CHH(num);
		hcint.b.chhltd = 0;
		
		ret = 1;
		if (pdev->host.HC_Status[num] == HC_XFRC)
		{
			pdev->host.URB_State[num] = URB_DONE;
		}
		else if (pdev->host.HC_Status[num] == HC_NAK)
		//mark nak作为正常的接收
		{
			pdev->host.URB_State[num] = URB_DONE;
		}
		else if (pdev->host.HC_Status[num] == HC_STALL)
		{
			pdev->host.URB_State[num] = URB_STALL;
		}

		else if ((pdev->host.HC_Status[num] == HC_XACTERR) || (pdev
	->host.HC_Status[num] == HC_DATATGLERR))
		{
			pdev->host.ErrCnt[num] = 0;
			pdev->host.URB_State[num] = URB_ERROR;

		}
		else if (pdev->host.HC_Status[num] == HC_HALTED)
		{
			pdev->host.URB_State[num] = URB_ERROR;
			pdev->host.ErrCnt[num] = 0;
		}
		else{
			ret = 0;
			p_err("unkown hltd:%d\n", pdev->host.HC_Status[num]);
		}
		

	}

	if (hcint.b.xfercompl)
	{
		hcint.b.xfercompl = 0;

		pdev->host.HC_Status[num] = HC_XFRC;
		pdev->host.ErrCnt[num] = 0;

		if (hcchar.b.eptype == EP_TYPE_BULK)
		{
			if (pdev->cfg.dma_enable == 1)
			{
				hctsiz.d32 = USB_OTG_READ_REG32(&pdev->regs.HC_REGS[num]->HCTSIZ);
				pdev->host.XferCnt[num] = pdev->host.hc[num].xfer_len - hctsiz.b.xfersize;

				pdev->host.hc[num].packet_count = (pdev->host.XferCnt[num] + pdev
	->host.hc[num].max_packet - 1) / pdev->host.hc[num].max_packet;
			}
			if (pdev->host.hc[num].packet_count &1)
				pdev->host.hc[num].toggle_in ^= 1;
			USB_OTG_HC_Halt(pdev, num);
		}
		else if (hcchar.b.eptype == EP_TYPE_CTRL)
		{
			pdev->host.hc[num].toggle_in ^= 1;
			USB_OTG_HC_Halt(pdev, num);
		}
		else if (hcchar.b.eptype == EP_TYPE_INTR)
		{
			hcchar.b.oddfrm = 1;
			USB_OTG_WRITE_REG32(&pdev->regs.HC_REGS[num]->HCCHAR, hcchar.d32);
			pdev->host.URB_State[num] = URB_DONE;
		}

	}

	if (hcint.b.nak)
	{
		hcint.b.nak = 0;

		pdev->host.HC_Status[num] = HC_NAK;

		if (hcchar.b.eptype == EP_TYPE_CTRL)
		{
			hcchar.b.chen = 1;
			hcchar.b.chdis = 0;
			USB_OTG_WRITE_REG32(&pdev->regs.HC_REGS[num]->HCCHAR, hcchar.d32);
		}
		else
			USB_OTG_HC_Halt(pdev, num);
	}

	if(hcint.d32 == 0)
		goto end;


	if (hcint.b.ahberr)
	{
		p_err("ahberr\n");
	}

	if (hcint.b.ack)
	{
		p_err("ack\n");
	}

	if (hcint.b.stall)
	{
		pdev->host.HC_Status[num] = HC_STALL;
		hcint.b.nak = 0; /* NOTE: When there is a 'stall', reset also nak,
		else, the pdev->host.HC_Status = HC_STALL
		will be overwritten by 'nak' in code below */
		USB_OTG_HC_Halt(pdev, num);
	}

	if (hcint.b.datatglerr)
	{
		pdev->host.HC_Status[num] = HC_DATATGLERR;
		USB_OTG_HC_Halt(pdev, num);
	}

	if (hcint.b.frmovrun)
	{
		USB_OTG_HC_Halt(pdev, num);
	}

	if (hcint.b.xacterr)
	{
		pdev->host.ErrCnt[num]++;
		pdev->host.HC_Status[num] = HC_XACTERR;
		USB_OTG_HC_Halt(pdev, num);
	}
	
end:
	return ret;

}
#endif

/**
 * @brief  USB_OTG_USBH_handle_rx_qlvl_ISR
 *         Handles the Rx Status Queue Level Interrupt
 * @param  pdev: Selected device
 * @retval status
 */
static uint32_t USB_OTG_USBH_handle_rx_qlvl_ISR(USB_OTG_CORE_HANDLE *pdev)
{
	USB_OTG_GRXFSTS_TypeDef grxsts;
	USB_OTG_GINTMSK_TypeDef intmsk;
	USB_OTG_HCTSIZn_TypeDef hctsiz;
	USB_OTG_HCCHAR_TypeDef hcchar;
	__IO uint8_t channelnum = 0;

	/* Disable the Rx Status Queue Level interrupt */
	intmsk.d32 = 0;
	intmsk.b.rxstsqlvl = 1;
	USB_OTG_MODIFY_REG32(&pdev->regs.GREGS->GINTMSK, intmsk.d32, 0);

	grxsts.d32 = USB_OTG_READ_REG32(&pdev->regs.GREGS->GRXSTSP);
	channelnum = grxsts.b.chnum;
	hcchar.d32 = USB_OTG_READ_REG32(&pdev->regs.HC_REGS[channelnum]->HCCHAR);

	switch (grxsts.b.pktsts)
	{
		case GRXSTS_PKTSTS_IN:
			/* Read the data into the host buffer. */
			if ((grxsts.b.bcnt > 0) && (pdev->host.hc[channelnum].xfer_buff != (void*)0))
			{

				USB_OTG_ReadPacket(pdev, pdev->host.hc[channelnum].xfer_buff, channelnum,
				grxsts.b.bcnt);

				/*manage multiple Xfer */
				pdev->host.hc[grxsts.b.chnum].xfer_buff += grxsts.b.bcnt;
				pdev->host.hc[grxsts.b.chnum].xfer_count += grxsts.b.bcnt;
				pdev->host.hc[grxsts.b.chnum].packet_count += 1;

				pdev->host.XferCnt[channelnum] = pdev->host.hc[channelnum].xfer_count;

				hctsiz.d32 = USB_OTG_READ_REG32(&pdev->regs.HC_REGS[channelnum]->HCTSIZ);



				if (hctsiz.b.pktcnt > 0)
				{
					/* re-activate the channel when more packets are expected */
					hcchar.b.chen = 1;
					hcchar.b.chdis = 0;
					USB_OTG_WRITE_REG32(&pdev->regs.HC_REGS[channelnum]->HCCHAR, hcchar.d32);
				}
			}
			break;

		case GRXSTS_PKTSTS_IN_XFER_COMP:

		case GRXSTS_PKTSTS_DATA_TOGGLE_ERR:
		case GRXSTS_PKTSTS_CH_HALTED:
		default:
			break;
	}

	/* Enable the Rx Status Queue Level interrupt */
	intmsk.b.rxstsqlvl = 1;
	USB_OTG_MODIFY_REG32(&pdev->regs.GREGS->GINTMSK, 0, intmsk.d32);
	return 1;
}

/**
 * @brief  USB_OTG_USBH_handle_IncompletePeriodicXfer_ISR
 *         Handles the incomplete Periodic transfer Interrupt
 * @param  pdev: Selected device
 * @retval status
 */
static uint32_t USB_OTG_USBH_handle_IncompletePeriodicXfer_ISR
	(USB_OTG_CORE_HANDLE *pdev)
{

	USB_OTG_GINTSTS_TypeDef gintsts;
	USB_OTG_HCCHAR_TypeDef hcchar;




	hcchar.d32 = USB_OTG_READ_REG32(&pdev->regs.HC_REGS[0]->HCCHAR);
	hcchar.b.chen = 1;
	hcchar.b.chdis = 1;
	USB_OTG_WRITE_REG32(&pdev->regs.HC_REGS[0]->HCCHAR, hcchar.d32);

	gintsts.d32 = 0;
	/* Clear interrupt */
	gintsts.b.incomplisoout = 1;
	USB_OTG_WRITE_REG32(&pdev->regs.GREGS->GINTSTS, gintsts.d32);

	return 1;
}

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF
      FILE****/
