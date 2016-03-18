/**
 ******************************************************************************
 * @file    usbh_ioreq.c
 * @author  MCD Application Team
 * @version V2.0.0
 * @date    22-July-2011
 * @brief   This file handles the issuing of the USB transactions
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
//#define DEBUG
#include "usbh_ioreq.h"
#include "app_cfg.h"
#include "usbh_core.h"
/** @addtogroup USBH_LIB
 * @{
 */

/** @addtogroup USBH_LIB_CORE
 * @{
 */

/** @defgroup USBH_IOREQ
 * @brief This file handles the standard protocol processing (USB v2.0)
 * @{
 */


/** @defgroup USBH_IOREQ_Private_Defines
 * @{
 */
/**
 * @}
 */


/** @defgroup USBH_IOREQ_Private_TypesDefinitions
 * @{
 */
/**
 * @}
 */



/** @defgroup USBH_IOREQ_Private_Macros
 * @{
 */
/**
 * @}
 */


/** @defgroup USBH_IOREQ_Private_Variables
 * @{
 */
/**
 * @}
 */


/** @defgroup USBH_IOREQ_Private_FunctionPrototypes
 * @{
 */
static USBH_Status USBH_SubmitSetupRequest(USB_OTG_CORE_HANDLE *phost,
    USB_Setup_TypeDef *setup, uint8_t *buff, uint16_t length);

/**
 * @}
 */


/** @defgroup USBH_IOREQ_Private_Functions
 * @{
 */


/**
 * @brief  USBH_CtlReq
 *         USBH_CtlReq sends a control request and provide the status after
 *            completion of the request
 * @param  pdev: Selected device
 * @param  req: Setup Request Structure
 * @param  buff: data buffer address to store the response
 * @param  length: length of the response
 * @retval Status
 */
USBH_Status USBH_CtlReq(USB_OTG_CORE_HANDLE *pdev, USBH_HOST *phost, uint8_t
    *buff, uint16_t length)
{
    USBH_Status status;
//    URB_STATE URB_Status = URB_IDLE;
    //	p_dbg_enter;
    //	URB_Status = HCD_GetURB_State(pdev, );

    //	p_dbg("USBH_Machine.RequestState %d\n", phost->RequestState);
    status = USBH_SubmitSetupRequest(pdev, &phost->Control.setup, buff, length);

	if (status != USBH_OK)
	{
		p_err("USBH_SubmitSetupRequest err, len:%d\n", length);
	}
	return status;
}


/**
 * @brief  USBH_CtlSendSetup
 *         Sends the Setup Packet to the Device
 * @param  pdev: Selected device
 * @param  buff: Buffer pointer from which the Data will be send to Device
 * @param  hc_num: Host channel Number
 * @retval Status
 */
USBH_Status USBH_CtlSendSetup(USB_OTG_CORE_HANDLE *pdev, uint8_t *buff, uint8_t
    hc_num)
{
    //  p_dbg_enter;
    pdev->host.hc[hc_num].ep_is_in = 0;
    pdev->host.hc[hc_num].data_pid = HC_PID_SETUP;
    pdev->host.hc[hc_num].xfer_buff = buff;
    pdev->host.hc[hc_num].xfer_len = USBH_SETUP_PKT_SIZE;

    /* DATA PID initialized as DATA0 */
    pdev->host.hc[hc_num].toggle_out = 0;

    if (HCD_SubmitRequest((void*)0, pdev, hc_num) != URB_DONE)
        return USBH_FAIL;

    return USBH_OK;
}


/**
 * @brief  USBH_CtlSendData
 *         Sends a data Packet to the Device
 * @param  pdev: Selected device
 * @param  buff: Buffer pointer from which the Data will be sent to Device
 * @param  length: Length of the data to be sent
 * @param  hc_num: Host channel Number
 * @retval Status
 */
USBH_Status USBH_CtlSendData(USB_OTG_CORE_HANDLE *pdev, uint8_t *buff, uint8_t
    length, uint8_t hc_num)
{
    //	p_dbg_enter;
    pdev->host.hc[hc_num].ep_is_in = 0;
    pdev->host.hc[hc_num].xfer_buff = buff;
    pdev->host.hc[hc_num].xfer_len = length;


    if (length == 0)
    {
         /* For Status OUT stage, Length==0, Status Out PID = 1 */
        pdev->host.hc[hc_num].toggle_out = 1;
    }

    /* Set the Data Toggle bit as per the Flag */
    if (pdev->host.hc[hc_num].toggle_out == 0)
    {
         /* Put the PID 0 */
        pdev->host.hc[hc_num].data_pid = HC_PID_DATA0;
    }
    else
    {
         /* Put the PID 1 */
        pdev->host.hc[hc_num].data_pid = HC_PID_DATA1;
    }

    if (HCD_SubmitRequest((void*)0, pdev, hc_num) != URB_DONE)
        return USBH_FAIL;

    return USBH_OK;
}


/**
 * @brief  USBH_CtlReceiveData
 *         Receives the Device Response to the Setup Packet
 * @param  pdev: Selected device
 * @param  buff: Buffer pointer in which the response needs to be copied
 * @param  length: Length of the data to be received
 * @param  hc_num: Host channel Number
 * @retval Status.
 */
USBH_Status USBH_CtlReceiveData(USB_OTG_CORE_HANDLE *pdev, uint8_t *buff,
    uint8_t length, uint8_t hc_num)
{
    //	p_dbg_enter;
    pdev->host.hc[hc_num].ep_is_in = 1;
    pdev->host.hc[hc_num].data_pid = HC_PID_DATA1;
    pdev->host.hc[hc_num].xfer_buff = buff;
    pdev->host.hc[hc_num].xfer_len = length;
 //   pdev->host.re_active_timeout[hc_num] = USB_RE_ACTIVE_TIMEOUT;
    if (HCD_SubmitRequest((void*)0, pdev, hc_num) != URB_DONE)
        return USBH_FAIL;


    return USBH_OK;

}


/**
 * @brief  USBH_BulkSendData
 *         Sends the Bulk Packet to the device
 * @param  pdev: Selected device
 * @param  buff: Buffer pointer from which the Data will be sent to Device
 * @param  length: Length of the data to be sent
 * @param  hc_num: Host channel Number
 * @retval Status
 */
USBH_Status USBH_BulkSendData(struct urb *urb, USB_OTG_CORE_HANDLE *pdev,
    uint8_t *buff, uint16_t length, uint8_t hc_num)
{

	mutex_lock(pdev->bulk_out_urb_lock);

	length = (length + 0x003f) & (~0x003f);

    pdev->host.hc[hc_num].ep_is_in = 0;
    pdev->host.hc[hc_num].xfer_buff = buff;
    pdev->host.hc[hc_num].xfer_len = length;

   
    /* Set the Data Toggle bit as per the Flag */
    if (pdev->host.hc[hc_num].toggle_out == 0)
    {
         /* Put the PID 0 */
        pdev->host.hc[hc_num].data_pid = HC_PID_DATA0;
    }
    else
    {
         /* Put the PID 1 */
        pdev->host.hc[hc_num].data_pid = HC_PID_DATA1;
    }
    if (HCD_SubmitRequest(urb, pdev, hc_num) != URB_DONE)
    {
    	mutex_unlock(pdev->bulk_out_urb_lock);
        return USBH_FAIL;
    }

    return USBH_OK;
}




/**
 * @brief  USBH_BulkReceiveData
 *         Receives IN bulk packet from device
 * @param  pdev: Selected device
 * @param  buff: Buffer pointer in which the received data packet to be copied
 * @param  length: Length of the data to be received
 * @param  hc_num: Host channel Number
 * @retval Status.
 */
USBH_Status USBH_BulkReceiveData(struct urb *urb, USB_OTG_CORE_HANDLE *pdev,
    uint8_t *buff, uint16_t length, uint8_t hc_num)
{
    //	p_dbg_enter;
    pdev->host.hc[hc_num].ep_is_in = 1;
    pdev->host.hc[hc_num].xfer_buff = buff;
    pdev->host.hc[hc_num].xfer_len = length;
  //  pdev->host.re_active_timeout[hc_num] = USB_RE_ACTIVE_TIMEOUT;

    if (pdev->host.hc[hc_num].toggle_in == 0)
    {
        pdev->host.hc[hc_num].data_pid = HC_PID_DATA0;
    }
    else
    {
        pdev->host.hc[hc_num].data_pid = HC_PID_DATA1;
    }

//    p_dbg("bulk receive,tg:%d\n", pdev->host.hc[hc_num].toggle_in);

    if (HCD_SubmitRequest(urb, pdev, hc_num) != URB_DONE)
        return USBH_FAIL;
    //p_dbg_exit;
    return USBH_OK;
}


/**
 * @brief  USBH_InterruptReceiveData
 *         Receives the Device Response to the Interrupt IN token
 * @param  pdev: Selected device
 * @param  buff: Buffer pointer in which the response needs to be copied
 * @param  length: Length of the data to be received
 * @param  hc_num: Host channel Number
 * @retval Status.
 */
USBH_Status USBH_InterruptReceiveData(USB_OTG_CORE_HANDLE *pdev, uint8_t *buff,
    uint8_t length, uint8_t hc_num)
{

    pdev->host.hc[hc_num].ep_is_in = 1;
    pdev->host.hc[hc_num].xfer_buff = buff;
    pdev->host.hc[hc_num].xfer_len = length;



    if (pdev->host.hc[hc_num].toggle_in == 0)
    {
        pdev->host.hc[hc_num].data_pid = HC_PID_DATA0;
    }
    else
    {
        pdev->host.hc[hc_num].data_pid = HC_PID_DATA1;
    }

    /* toggle DATA PID */
    pdev->host.hc[hc_num].toggle_in ^= 1;

    if (HCD_SubmitRequest((void*)0, pdev, hc_num) != URB_DONE)
        return USBH_FAIL;

    return USBH_OK;
}

/**
 * @brief  USBH_InterruptSendData
 *         Sends the data on Interrupt OUT Endpoint
 * @param  pdev: Selected device
 * @param  buff: Buffer pointer from where the data needs to be copied
 * @param  length: Length of the data to be sent
 * @param  hc_num: Host channel Number
 * @retval Status.
 */
USBH_Status USBH_InterruptSendData(USB_OTG_CORE_HANDLE *pdev, uint8_t *buff,
    uint8_t length, uint8_t hc_num)
{

    pdev->host.hc[hc_num].ep_is_in = 0;
    pdev->host.hc[hc_num].xfer_buff = buff;
    pdev->host.hc[hc_num].xfer_len = length;

    if (pdev->host.hc[hc_num].toggle_in == 0)
    {
        pdev->host.hc[hc_num].data_pid = HC_PID_DATA0;
    }
    else
    {
        pdev->host.hc[hc_num].data_pid = HC_PID_DATA1;
    }

    pdev->host.hc[hc_num].toggle_in ^= 1;

    if (HCD_SubmitRequest((void*)0, pdev, hc_num) != URB_DONE)
        return USBH_FAIL;

    return USBH_OK;
}


/**
 * @brief  USBH_SubmitSetupRequest
 *         Start a setup transfer by changing the state-machine and
 *         initializing  the required variables needed for the Control Transfer
 * @param  pdev: Selected device
 * @param  setup: Setup Request Structure
 * @param  buff: Buffer used for setup request
 * @param  length: Length of the data
 * @retval Status.
 */
static USBH_Status USBH_SubmitSetupRequest(USB_OTG_CORE_HANDLE *pdev,
    USB_Setup_TypeDef *setup, uint8_t *buff, uint16_t length)
{
    USBH_Status status;
    //	p_dbg_enter;

    /* Save Global State */

    USB_Host.Control.setup =  *setup;
    USB_Host.Control.buff = buff;
    USB_Host.Control.length = length;

    //	p_dbg("usb_control_msg   pipe:%08x, request:%02x, type:%02x, value:%04x, off:%d, buffer_len:%d, timeout:%d\n",
    //	       0, setup->b.bRequest, setup->b.bmRequestType, setup->b.wValue.w, setup->b.wIndex.w, length, 0);

	mutex_lock(pdev->bulk_out_urb_lock);
    status = USBH_HandleControl(&USB_OTG_Core, &USB_Host);
	mutex_unlock(pdev->bulk_out_urb_lock);

    return status;
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

/**
 * @}
 */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
