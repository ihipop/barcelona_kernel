/*
 * drivers/misc/ce_mailbox.c
 *
 *  GPL LICENSE SUMMARY
 *
 *  Copyright(c) 2012 Intel Corporation. All rights reserved.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of version 2 of the GNU General Public License as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *  The full GNU General Public License is included in this distribution
 *  in the file called LICENSE.GPL.
 *
 *    Intel Corporation
 *    2200 Mission College Blvd.
 *    Santa Clara, CA  97052
 *
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <asm/uaccess.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/jiffies.h>
#include <linux/proc_fs.h>
#include <linux/hw_mutex.h> 
#include <linux/pci.h>
#include <linux/seq_file.h>
#include <linux/types.h>
#include <linux/socket.h>
#include <linux/in.h>

#include <linux/ce_mailbox.h>

static struct proc_dir_entry	*mbx_proc_dir;
static struct npcpu_appcpu_mbx_user	mbx_usr_info;

#define MAX_PARAMETER_SIZE  6
#define AVALANCHE_SRAM_BASE 0xC8000000	/* APPCPU SRAM base address */

volatile void __iomem *npcpu_appcpu_mbx_base_addr = NULL;
bool glob_mbx_is_initialized = false;

#define NPCPU_APPCPU_MBX_STRUCT_PHY_ADDR		(AVALANCHE_SRAM_BASE + 0x00001E00)

#define NPCPU_MBX_STRUCT_BASE_ADDR     (npcpu_appcpu_mbx_base_addr)
#define APPCPU_MBX_STRUCT_BASE_ADDR  (NPCPU_MBX_STRUCT_BASE_ADDR + 4)

#define NPCPU_MBX_STRUCT_EVENT_MASK_ADDR  (NPCPU_MBX_STRUCT_BASE_ADDR)
#define NPCPU_MBX_STRUCT_ACK_MASK_ADDR    (NPCPU_MBX_STRUCT_BASE_ADDR + 2)


#define APPCPU_MBX_STRUCT_EVENT_MASK_ADDR  (APPCPU_MBX_STRUCT_BASE_ADDR)
#define APPCPU_MBX_STRUCT_ACK_MASK_ADDR    (APPCPU_MBX_STRUCT_BASE_ADDR + 2)

/* RPC-IF INFO */
#define NPCPU_RPC_IPV4_ADDR				(NPCPU_MBX_STRUCT_BASE_ADDR + 0x8)
#define APPCPU_RPC_IPV4_ADDR				(NPCPU_MBX_STRUCT_BASE_ADDR + 0xC)
#define RPC_NETWORK_MASK				(NPCPU_MBX_STRUCT_BASE_ADDR + 0x10)
#define VLAN_ID							(NPCPU_MBX_STRUCT_BASE_ADDR + 0x14)


/* 30 seconds max delay */
#define MBX_MAX_POLLING_DELAY		(msecs_to_jiffies(30000))

/* Do this parameter check to avoid wild operations */
#define APPCPU_MBX_FUNC_PARAM_CHECK(base_addr, eventId)	((base_addr)&&\
		((eventId == APPCPU_EVENT_SPI_ADVANCE_EXIT)||\
		 (eventId == APPCPU_EVENT_EMMC_ADVANCE_EXIT)))
#define NPCPU_MBX_FUNC_PARAM_CHECK(base_addr, eventId)	((base_addr)&&\
		((eventId == NPCPU_EVENT_SPI_INIT_EXIT)||\
		 (eventId == NPCPU_EVENT_EMMC_INIT_EXIT)||\
		 (eventId == NPCPU_EVENT_GPIO_INIT_EXIT)||\
		 (eventId == NPCPU_EVENT_EMMC_ADVANCE_INIT_EXIT)||\
		 (eventId == NPCPU_EVENT_RPC_IF_OBTAIN_ADDR)))

#define reg_write_32(addr, data) (__raw_writel(data, (volatile void *)addr))
#define reg_read_32(addr)        ( __raw_readl((volatile void *)addr))
#define reg_write_16(addr, data) ( __raw_writel((unsigned long)data,(volatile void *)addr))
#define reg_read_16(addr)        ((unsigned short)__raw_readl((volatile void *)addr))

static unsigned int ref = 0;

/* Polloing on specivec ACK/Event bit - max timeout is 4 sec */ 
static int mbx_wait_till_ready(volatile unsigned int regAddressToPoll, unsigned short eventId)
{
	unsigned long timeout = jiffies + MBX_MAX_POLLING_DELAY;

	do{
		if( (le16_to_cpu( reg_read_16(regAddressToPoll) ) & eventId) == 0 ) {
			continue;
		} else {
			return 0;  /* Normal exit */
		}

	}while(time_after(timeout, jiffies));

	printk( "*****************************************************************\n" );
	printk( "*** mbx_wait_till_ready Wait for ACK/EVENT Fail on timeout     **\n" );
	printk( "*****************************************************************\n" );

	return 1;
}

static int mbx_open(struct inode *inode, struct file *filp)
{

	printk(KERN_DEBUG "npcpu_appcpu_mbx driver open ref %d\n", ++ref);
	return 0;
}

static int mbx_close(struct inode *inode, struct file *filp)
{
	printk(KERN_DEBUG "npcpu_appcpu_mbx driver close ref %d\n", --ref);
	return 0;
}

static long mbx_unlocked_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int err = 0;

	/*
	 * Extract the type and number bitfields, and don't decode
	 * wrong cmds
	 */
	if (_IOC_TYPE(cmd) != MBX_MODULE_ID) return -ENOTTY;
	if (_IOC_NR(cmd) > MBX_IOC_MAXNR) return -ENOTTY;

	/*
	 * Verify the user space pointer
	 */
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err =  !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	if (err) return -EFAULT;

	/* copy from the user the event ID and if need to copy parameter */
	if (copy_from_user(&mbx_usr_info, (struct npcpu_appcpu_mbx_user *)arg,
				sizeof(struct npcpu_appcpu_mbx_user)) ) {
		return -EFAULT;
	}

	switch (cmd) {
		case MBX_SEND_EVENT_CMD: 
			err = npcpu_appcpu_mbx_send_notification(mbx_usr_info.eventId,(unsigned int *)&mbx_usr_info.parameter);
			break;

		case MBX_GET_EVENT_CMD:
			err = npcpu_appcpu_mbx_receive_event_notification(mbx_usr_info.eventId,(unsigned int *)&mbx_usr_info.parameter);
			if(mbx_usr_info.isParamRequired) {
				if (copy_to_user((struct npcpu_appcpu_mbx_user *)arg,&mbx_usr_info,sizeof(struct npcpu_appcpu_mbx_user))) 
					return -EFAULT;
			}
			break;

		case MBX_SEND_ACK_CMD:
			err = npcpu_appcpu_mbx_send_ack(mbx_usr_info.eventId);
			break;
		case MBX_RECEIVE_ACK_CMD:
			err = npcpu_appcpu_mbx_receive_specific_ack(mbx_usr_info.eventId);
			break;
		default:
			printk(KERN_ERR "NPCPU/APPCPU Mailbox driver receive Wrong IOCTL command = 0x%x \n",cmd);
			return -EFAULT;
	}

	return err;
}
/*
 * npcpu_appcpu_mbx_send_notification 
 * @eventId: Event need send to NPCPU
 * @ParamPtr - pointer to parameter for this event (optional)
 * 
 * Send Specific event notification to the NPCPU. 
 * If Parameter is requiered update the corresponding parameter 
 * Returns 0 if success, negative if error / timeout
 */
long npcpu_appcpu_mbx_send_notification(unsigned short eventID, unsigned int *paramPtr)
{
	volatile unsigned short eventMask;
	if (!glob_mbx_is_initialized) {
		printk("ERROR : Intel(R) CE Mailbox driver is not installed \n");
		return -ENODEV;
	}
	if (!APPCPU_MBX_FUNC_PARAM_CHECK(APPCPU_MBX_STRUCT_BASE_ADDR,eventID))
		return -EINVAL;
	/* Take the HW mutex */
	if (hw_mutex_lock_interruptible(HW_MUTEX_ATOM_MBX) != 0) {
		printk("ERROR : npcpu_appcpu_mbx_send_notification- Can't lock HW mutex\n");
		return -ENOSYS;
	}
	/* Get the event mask */
	eventMask = le16_to_cpu( reg_read_16(APPCPU_MBX_STRUCT_EVENT_MASK_ADDR) );
	eventMask |= eventID;

	switch(eventID) {
		case APPCPU_EVENT_SPI_ADVANCE_EXIT:
		case APPCPU_EVENT_EMMC_ADVANCE_EXIT: 
			/* Event w/o parameters */
			break;
		default: 
			/* Release the HW Mutex */
			hw_mutex_unlock(HW_MUTEX_ATOM_MBX);		
			printk( "ERROR : npcpu_appcpu_mbx_send_notification Event=0x%x is invalid \n", eventID );
			return -ENOSYS;
	}

	/* set the appropiated bit of the event mask on NPCPU structur */
	reg_write_16(APPCPU_MBX_STRUCT_EVENT_MASK_ADDR, cpu_to_le16(eventMask));
	printk("npcpu_appcpu_mbx_send_notification event id %d, 0x%x\n",eventID,reg_read_32(APPCPU_MBX_STRUCT_EVENT_MASK_ADDR));

	/* Release the HW Mutex */
	hw_mutex_unlock(HW_MUTEX_ATOM_MBX);
	return 0;
}
/*
 * npcpu_appcpu_mbx_send_ack 
 * @eventId: Event need to ACK on
 *  
 * Send ACK to NPCPU on specific event received by APPCPU
 * Set the apropriate ACK bit in the NPCPU mbox struct indicates APPCPU was processing this event
 * Returns 0 if success, negative if error / timeout
 */
long npcpu_appcpu_mbx_send_ack(unsigned short eventID)
{
	volatile unsigned short ackMask;
	if (!glob_mbx_is_initialized) {
		printk("ERROR : Intel(R) CE Mailbox driver is not installed \n");
		return -ENODEV;
	}
	if (!NPCPU_MBX_FUNC_PARAM_CHECK(NPCPU_MBX_STRUCT_BASE_ADDR,eventID))
		return -EINVAL;
	/* Take the HW mutex */
	if (hw_mutex_lock_interruptible(HW_MUTEX_ARM_MBX) != 0) {
		printk("npcpu_appcpu_mbx_send_ack- Can't lock HW mutex\n");
		return -ENOSYS;
	}
	ackMask = le16_to_cpu( reg_read_16(NPCPU_MBX_STRUCT_ACK_MASK_ADDR) );
	printk(KERN_DEBUG "npcpu_appcpu_mbx_send_ack- ack Mask = 0x%x\n",ackMask);

	ackMask |= eventID;
	/* Set the ACK bit on NPCPU ACK bit mask structure*/
	reg_write_16(NPCPU_MBX_STRUCT_ACK_MASK_ADDR,cpu_to_le16(ackMask));
	printk("npcpu_appcpu_mbx_send_ack- Writing ack Mask = 0x%x for event = 0x%x\n",ackMask,eventID);

	/* Release the HW Mutex */
	hw_mutex_unlock(HW_MUTEX_ARM_MBX);

	return 0;
}
/*
 * npcpu_appcpu_mbx_receive_specific_ack 
 * @eventId: Wait for ack on Event Sent earlier by the APPCPU
 * 
 * Wait for specific  ACK from the NPCPU - indicates NPCPU got this event
 * polling the APPCPU Mbx structure (ACK bit Mask)- DO not wait forever exit on timeout
 * Returns 0 if success, negative if error / timeout
 */
long npcpu_appcpu_mbx_receive_specific_ack(unsigned short eventId)
{
	volatile unsigned short ackMask;
	volatile unsigned short eventMask;
	if (!glob_mbx_is_initialized) {
		printk("ERROR : Intel(R) CE Mailbox driver is not installed \n");
		return -ENODEV;
	}
	if (!APPCPU_MBX_FUNC_PARAM_CHECK(APPCPU_MBX_STRUCT_BASE_ADDR,eventId))
		return -EINVAL;

	if (mbx_wait_till_ready((unsigned int)APPCPU_MBX_STRUCT_ACK_MASK_ADDR, eventId) ) {
		printk(KERN_DEBUG "APPCPU-NPCPU MBX is stuck - ACK from NPCPU on eventId=0x%x NOT arrived \n", eventId );
		return -ENOSYS;
	}
	printk( "\nGOT ACK from NPCPU on eventId=0x%x arrived \n", eventId );
	/* ACK was received - Need to Clear the event and ACK bit*/
	/* Take the HW mutex */
	if (hw_mutex_lock_interruptible(HW_MUTEX_ATOM_MBX) != 0) {
		printk("npcpu_appcpu_mbx_send_ack- Can't lock HW mutex\n");
		return -ENOSYS;
	}
	printk( "Clear the ACK Vector from NPCPU on eventId=0x%x \n", eventId );

	/* Clear APPCPU ACK vector */
	ackMask = le16_to_cpu( reg_read_16(APPCPU_MBX_STRUCT_ACK_MASK_ADDR) );
	ackMask = (ackMask & ~(eventId));
	reg_write_16(APPCPU_MBX_STRUCT_ACK_MASK_ADDR,cpu_to_le16(ackMask));

	/* Clear APPCPU event vector */
	eventMask = le16_to_cpu( reg_read_16(APPCPU_MBX_STRUCT_EVENT_MASK_ADDR) );
	eventMask = (eventMask & ~(eventId));
	reg_write_16(APPCPU_MBX_STRUCT_EVENT_MASK_ADDR,cpu_to_le16(eventMask));
	/* Release the HW Mutex */
	hw_mutex_unlock(HW_MUTEX_ATOM_MBX);
	return 0;
}

/*
 * npcpu_appcpu_mbx_receive_event_notification
 * @eventId: Wait for this event
 * param - if the event requiered parameter - this is the output parameter
 * 
 * Wait for specific event from the NPCPU - 
 * polling the NPCPU Mbx structure - DO not wait forever exit on timeout
 * Returns 0 if success, negative if error / timeout
 */
long npcpu_appcpu_mbx_receive_event_notification(unsigned short eventId, unsigned int *param)
{
	volatile unsigned short ackMask;
	bool ackRequiered = 0;
	struct npcpu_rpc_info *rpc_info = (struct npcpu_rpc_info *)param;
	if (!glob_mbx_is_initialized) {
		printk("ERROR : Intel(R) CE Mailbox driver is not installed \n");
		return -ENODEV;
	}
	if (!NPCPU_MBX_FUNC_PARAM_CHECK(NPCPU_MBX_STRUCT_BASE_ADDR,eventId))
		return -EINVAL;

	if( mbx_wait_till_ready((unsigned int)NPCPU_MBX_STRUCT_EVENT_MASK_ADDR, eventId) ) {
		printk(KERN_DEBUG "APPCPU-NPCPU MBX is stuck - Wait for Event=0x%x from NPCPU fail on timeout \n", eventId );
		return -ENOSYS;
	}

	switch(eventId)
	{
		case NPCPU_EVENT_RPC_IF_OBTAIN_ADDR: 
			BUG_ON(!rpc_info);	
			rpc_info->npcpu_ipv4_addr  = 	le32_to_cpu( reg_read_32(NPCPU_RPC_IPV4_ADDR));
			rpc_info->appcpu_ipv4_addr =  	le32_to_cpu( reg_read_32(APPCPU_RPC_IPV4_ADDR));
			rpc_info->netmask 	   =  	le32_to_cpu( reg_read_32(RPC_NETWORK_MASK));
			rpc_info->vlan_id	   = 	le32_to_cpu( reg_read_32(VLAN_ID));
			ackRequiered = 1;
			break; 

		case NPCPU_EVENT_GPIO_INIT_EXIT:
		case NPCPU_EVENT_SPI_INIT_EXIT:
		case NPCPU_EVENT_EMMC_INIT_EXIT:  
		case NPCPU_EVENT_EMMC_ADVANCE_INIT_EXIT:
			break;

		default:
			printk( "ERROR : receive_specific_event_notification Event=0x%x is invalid \n", eventId );
			return -ENOSYS;

	}

//	printk( "Receive_specific_event_notification Event=0x%x \n", eventId);

	if (ackRequiered) {

		/* Take the HW mutex */
		if (hw_mutex_lock_interruptible(HW_MUTEX_ARM_MBX) != 0) {
			printk("ERROR - npcpu_appcpu_mbx_receive_event_notification- Can't lock HW mutex\n");
			return -ENOSYS;
		}
		ackMask = le16_to_cpu(reg_read_16(NPCPU_MBX_STRUCT_ACK_MASK_ADDR)) | eventId ;
		reg_write_16(NPCPU_MBX_STRUCT_ACK_MASK_ADDR,cpu_to_le16(ackMask));

		/* Set the appropiated ACK bit  */
		/* Release the HW Mutex */
		hw_mutex_unlock(HW_MUTEX_ARM_MBX);
	}

	return 0;
}

static struct file_operations mbx_fops = {
	.owner   	= THIS_MODULE,
	.unlocked_ioctl   = mbx_unlocked_ioctl,
	.open 		= mbx_open,
	.release 	= mbx_close,
};

        
/* Translate IP address */
#define NIPQUAD(addr) \
        ((unsigned char*)&addr)[0], \
        ((unsigned char*)&addr)[1], \
        ((unsigned char*)&addr)[2], \
        ((unsigned char*)&addr)[3]

static int npcpu_ip_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%u.%u.%u.%u\n",NIPQUAD(mbx_usr_info.parameter.npcpu_ipv4_addr));
	return 0;
}
static int appcpu_ip_proc_show(struct seq_file *m, void *v)
{	
	seq_printf(m, "%u.%u.%u.%u\n",NIPQUAD(mbx_usr_info.parameter.appcpu_ipv4_addr));
	return 0;
}
static int netmask_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%u.%u.%u.%u\n",NIPQUAD(mbx_usr_info.parameter.netmask));
	return 0;
}
static int vlan_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%u\n",mbx_usr_info.parameter.vlan_id);
	return 0;
}

static int npcpu_ip_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, npcpu_ip_proc_show, PDE(inode)->data);
}
static int appcpu_ip_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, appcpu_ip_proc_show, PDE(inode)->data);
}

static int netmask_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, netmask_proc_show, PDE(inode)->data);
}
static int vlan_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, vlan_proc_show, PDE(inode)->data);
}


static struct file_operations npcpu_ip_fops = {
	.owner   	= THIS_MODULE,
	.open		= npcpu_ip_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,		
};
static struct file_operations appcpu_ip_fops = {
	.owner   	= THIS_MODULE,
	.open		= appcpu_ip_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,		
};
static struct file_operations netmask_fops = {
	.owner   	= THIS_MODULE,
	.open		= netmask_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,		
};
static struct file_operations vlan_fops = {
	.owner  	= THIS_MODULE,
	.open		= vlan_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,		
};

static int __exit remove_mbx_proc(struct proc_dir_entry *mbx_dir)
{
	if (!mbx_dir)
		return -EIO;
	remove_proc_entry(CE_MAILBOX_DEVICE_NAME, mbx_dir);
	remove_proc_entry("npcpu_ip", mbx_dir);	
	remove_proc_entry("appcpu_ip", mbx_dir);	
	remove_proc_entry("netmask", mbx_dir);	
	remove_proc_entry("vlan_id", mbx_dir);	
	remove_proc_entry(CE_MAILBOX_DEVICE_NAME,NULL);
	return 0;
}

static struct proc_dir_entry * __init create_mbx_proc(void)
{
	/* create /proc/ce_mailbox */
	struct proc_dir_entry *mbx_dir = proc_mkdir(CE_MAILBOX_DEVICE_NAME, NULL);
	if (!mbx_dir)
		return NULL;

	/* create /proc/ce_mailbox/ce_mailbox */
	if (!proc_create_data(CE_MAILBOX_DEVICE_NAME, S_IRUSR|S_IWUSR | S_IRGRP |S_IWGRP |S_IROTH |S_IWOTH, mbx_dir,
				&mbx_fops, NULL))
		return NULL;
	/* create /proc/ce_mailbox/npcpu_ipv4_addr */
	if (!proc_create_data("npcpu_ip", S_IRUSR|S_IRGRP|S_IROTH, mbx_dir,
				&npcpu_ip_fops, NULL))
		return NULL;

	/* create /proc/ce_mailbox/appcpu_ipv4_addr */
	if (!proc_create_data("appcpu_ip", S_IRUSR|S_IRGRP|S_IROTH, mbx_dir,
				&appcpu_ip_fops, NULL))
		return NULL;

	/* create /proc/ce_mailbox/network_mask */
	if (!proc_create_data("netmask", S_IRUSR|S_IRGRP|S_IROTH, mbx_dir,
				&netmask_fops, NULL))
		return NULL;
	/* create /proc/ce_mailbox/vlan_id */
	if (!proc_create_data("vlan_id", S_IRUSR|S_IRGRP|S_IROTH, mbx_dir,
				&vlan_fops, NULL))
		return NULL;

	return mbx_dir;
}
/* Init the module */
static int __init npcpu_appcpu_mbx_init(void)
{
	struct pci_dev *hwmutex_dev = 0;

	/*  Mailbox driver is only installed when there's HW MUTEX device existing */
	hwmutex_dev = pci_get_device(0x8086, HW_MUTEX_DEV_ID,NULL);
	if (!hwmutex_dev) 
		return -ENODEV;
	else
		pci_dev_put(hwmutex_dev);
	mbx_proc_dir = create_mbx_proc();
	if (!mbx_proc_dir)
		printk("ERROR - %s initialization- can not create proc fs\n",CE_MAILBOX_DEVICE_NAME);

	npcpu_appcpu_mbx_base_addr = (void __iomem *)ioremap_nocache(NPCPU_APPCPU_MBX_STRUCT_PHY_ADDR,(MAX_PARAMETER_SIZE+2)*sizeof(unsigned int));
	if (!npcpu_appcpu_mbx_base_addr) {
		printk("ERROR - %s initialization- can not access SRAM\n", CE_MAILBOX_DEVICE_NAME);
		remove_mbx_proc(mbx_proc_dir);
		return -ENOMEM;
	}
	printk(KERN_INFO "Intel(R) NPCPU <-> APPCPU Event Mailbox Device Driver built on %s @ %s\n", __DATE__, __TIME__);
	glob_mbx_is_initialized = true;
	return 0;
}

/* remove the module */
static void __exit npcpu_appcpu_mbx_exit(void)
{
	if (npcpu_appcpu_mbx_base_addr) {
		iounmap(npcpu_appcpu_mbx_base_addr);
		remove_mbx_proc(mbx_proc_dir);
	}
	glob_mbx_is_initialized = false;
	printk(KERN_INFO "Intel(R) NPCPU <-> APPCPU Event Mailbox Device Driver removed\n");

}
subsys_initcall_sync(npcpu_appcpu_mbx_init);
module_exit(npcpu_appcpu_mbx_exit); 


EXPORT_SYMBOL(npcpu_appcpu_mbx_send_notification);
EXPORT_SYMBOL(npcpu_appcpu_mbx_send_ack);
EXPORT_SYMBOL(npcpu_appcpu_mbx_receive_specific_ack);
EXPORT_SYMBOL(npcpu_appcpu_mbx_receive_event_notification);

MODULE_DESCRIPTION("NPCPU <-> APPCPU Event Mailbox Device Driver");
MODULE_LICENSE("GPL"); 


