/**
 ******************************************************************************
 * @file    usbh_usr.c
 * @author  MCD Application Team
 * @version V1.0.0
 * @date    22/07/2011
 * @brief   This file includes the usb host user callbacks
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
#include "usb.h"
#include "app.h"
#include "debug.h"

struct usb_device usb_dev;
struct usb_host_config usb_config;
struct usb_interface usb_intfc;
struct usb_host_interface host_intfc;



void init_usb_struct(USB_OTG_CORE_HANDLE *pdev, USBH_HOST *phost)
{
    int i;
    memset(&usb_dev, 0, sizeof(struct usb_device));
    memset(&usb_config, 0, sizeof(struct usb_host_config));
    memset(&usb_intfc, 0, sizeof(struct usb_interface));
    memset(&host_intfc, 0, sizeof(struct usb_host_interface));

    usb_dev.descriptor = &phost->device_prop.Dev_Desc;
    usb_dev.config = &usb_config;

    usb_config.desc = &phost->device_prop.Cfg_Desc;
    pdev->dev = &usb_dev;
    usb_config.interface[0] = &usb_intfc;

    usb_intfc.altsetting = usb_intfc.cur_altsetting = &host_intfc;

    host_intfc.desc = phost->device_prop.Itf_Desc;

    host_intfc.endpoint = usb_dev.bulk_ep;

    for (i = 0; i < USBH_MAX_NUM_ENDPOINTS; i++)
    {
        usb_dev.bulk_ep[i].desc = &phost->device_prop.Ep_Desc[0][i];
    }

	//sys_cfg.usb_dev = &usb_dev;
}


extern USB_OTG_CORE_HANDLE USB_OTG_Core;
extern USBH_HOST USB_Host;

USB_OTG_CORE_HANDLE *p_usb_dev = &USB_OTG_Core;

int usb_control_msg(unsigned int pipe, u8 request, u8 requesttype, u16 value,
    u16 index, void *data, u16 size, int timeout)
{
	int ret;

//	mutex_lock(USB_Host.Control.setup_urb_mutex);

	USB_Host.Control.setup.b.bmRequestType = requesttype;
	USB_Host.Control.setup.b.bRequest = request;
	USB_Host.Control.setup.b.wValue.w = value;
	USB_Host.Control.setup.b.wIndex.w = index;
	USB_Host.Control.setup.b.wLength.w = size;


	ret = USBH_CtlReq(&USB_OTG_Core, &USB_Host, data, size);
	if (ret != USBH_OK)
		ret = -1;
	else
		ret = 0;
//	mutex_unlock(USB_Host.Control.setup_urb_mutex);

	return ret;
}

#if 0   //use inline
void usb_fill_bulk_urb(struct urb *urb, struct usb_device *dev, unsigned int
    pipe, void *transfer_buffer, int buffer_length, void *complete_fn, void
    *context)
{
    urb->dev = dev;
    urb->pipe = pipe;
    urb->transfer_buffer = transfer_buffer;
    urb->transfer_buffer_length = buffer_length;
    urb->complete = complete_fn;
    urb->context = context;
 }
#endif

DECLARE_MONITOR_ITEM("bulk out cnt", bulk_out_cnt) ;
DECLARE_MONITOR_ITEM("bulk out complite", bulk_out_complite) ;
DECLARE_MONITOR_ITEM("bulk in cnt", bulk_in_cnt) ;
DECLARE_MONITOR_ITEM("bulk in complite", bulk_in_complite) ;

int usb_submit_urb(struct urb *urb, gfp_t mem_flags)
{
    struct usb_device *dev;
    USB_OTG_HC *hc;
    USBH_Status stat;

    if (!urb /* || urb->hcpriv*/ || !urb->complete)
        return  - EINVAL;
    dev = urb->dev;
    if ((!dev))
        return  - ENODEV;

    hc = &USB_OTG_Core.host.hc[urb->pipe];

    if (!hc->ep_is_in)
    {
    	INC_MONITOR_ITEM_VALUE(bulk_out_cnt);
        stat = USBH_BulkSendData(urb, &USB_OTG_Core, urb->transfer_buffer, 
        	urb->transfer_buffer_length, urb->pipe);
    }
    else
    {
    	INC_MONITOR_ITEM_VALUE(bulk_in_cnt);
        stat = USBH_BulkReceiveData(urb, &USB_OTG_Core, urb->transfer_buffer,
            urb->transfer_buffer_length, urb->pipe);
    }
    return stat;
}


int usb_enumeration(USB_OTG_CORE_HANDLE *p_dev, USBH_HOST	*p_host)
{
    int ret =  - 1;
    int i = 0;
    p_dbg_enter;
again:
    if (HCD_ResetPort(p_dev) != 0)
    {
        p_err("HCD_ResetPort err\n");
        goto end;
    }	
    sleep(200);
    if (!HCD_IsDeviceConnected(p_dev))
    {
        p_err("device not conneted, try again %d\n", i++);
	sleep(1000);
        goto again;
    }

    USBH_DeAllocate_AllChannel(p_dev);

    p_host->Control.hc_num_out = USBH_Alloc_Channel(p_dev, 0x00);

    p_host->Control.hc_num_in = USBH_Alloc_Channel(p_dev, 0x80);

    /* Reset USB Device */
    if (HCD_ResetPort(p_dev) == 0)
    {
        sleep(200);
        p_host->device_prop.speed = HCD_GetCurrentSpeed(p_dev);
        p_dbg("speed %d\n", p_host->device_prop.speed);

            /* Open Control pipes */
        USBH_Open_Channel(p_dev, p_host->Control.hc_num_in,
                p_host->device_prop.address, p_host->device_prop.speed,
                EP_TYPE_CTRL, p_host->Control.ep0size);

            /* Open Control pipes */
        USBH_Open_Channel(p_dev, p_host->Control.hc_num_out,
                p_host->device_prop.address, p_host->device_prop.speed,
                EP_TYPE_CTRL, p_host->Control.ep0size);
    }
    else
    {
        p_err("HCD_ResetPort err\n");
				goto end;
    }
    if (USBH_HandleEnum(p_dev, p_host) != USBH_OK)
    {
        p_err("USBH_HandleEnum faild\n");
        goto end;
    }

    for (i = 0; i < p_host->device_prop.Itf_Desc[0].bNumEndpoints; i++)
    {
        if (p_host->device_prop.Ep_Desc[0][i].bEndpointAddress &0x80)
        {
            int hc_num_in;
            hc_num_in = USBH_Alloc_Channel(p_dev,
                p_host->device_prop.Ep_Desc[0][i].bEndpointAddress);

            /* Open channel for IN endpoint */
            USBH_Open_Channel(p_dev, hc_num_in,
                p_host->device_prop.address, p_host->device_prop.speed,
                EP_TYPE_BULK, p_host->device_prop.Ep_Desc[0][i].wMaxPacketSize)
                ;
        }
        else
        {
            int hc_num_out;
            hc_num_out = USBH_Alloc_Channel(p_dev,
                p_host->device_prop.Ep_Desc[0][i].bEndpointAddress);

            /* Open channel for IN endpoint */
            USBH_Open_Channel(p_dev, hc_num_out,
                p_host->device_prop.address, p_host->device_prop.speed,
                EP_TYPE_BULK, p_host->device_prop.Ep_Desc[0][i].wMaxPacketSize)
                ;
        }

    }
  
    ret = ERR_OK;
    end: 
	if (ret != ERR_OK)
        p_err("usb_enumeration faild\n");
    return ret;
}

int init_usb()
{
	int ret;
	
	USBH_Init(&USB_OTG_Core, 
            USB_OTG_HS_CORE_ID,
            &USB_Host,
            0, 
            0);

	ret = usb_enumeration(&USB_OTG_Core, &USB_Host);

	return ret;
}

 /******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF
     FILE****/
