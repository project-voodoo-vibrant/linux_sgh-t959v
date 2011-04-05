/*  $Date: 2009/11/10 17:37:35 $
 *  $Revision: 1.0 $ 
 */

/************************* kr3dh *************************************

Application description       : kr3dh Linux driver

                              : STMicroelectronics

Date                          : 08/12/2009

Revision                      : 1-0-0

Changed Features              : First Release

Bug fixes                     : First Release

MEMS platform                 : digital output KR3DH_ACC

S/W platform                  : gcc 4.2.1

Application Details           : kr3dh Linux driver
                                

Copyright (c) 2009 STMicroelectronics.

THIS PROGRAM IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER
EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTY
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. THE ENTIRE RISK
AS TO THE QUALITY AND PERFORMANCE OF THE PROGRAM IS WITH YOU. SHOULD THE
PROGRAM PROVE DEFECTIVE, YOU ASSUME THE COST OF ALL NECESSARY SERVICING,
REPAIR OR CORRECTION.

THIS DOCUMENT CONTAINS PROPRIETARY AND CONFIDENTIAL INFORMATION OF THE
STMICROELECTRONICS GROUP.
INFORMATION FURNISHED IS BELIEVED TO BE ACCURATE AND RELIABLE. HOWEVER, 
STMICROELECTRONICS ASSUMES NO RESPONSIBILITY FOR THE CONSEQUENCES OF USE
OF SUCH INFORMATION.
SPECIFICATIONS MENTIONED IN THIS PUBLICATION ARE SUBJECT TO CHANGE WITHOUT NOTICE.
THIS PUBLICATION SUPERSEDES AND REPLACES ALL INFORMATION PREVIOUSLY SUPPLIED.
STMICROELECTRONICS PRODUCTS ARE NOT AUTHORIZED FOR USE AS CRITICAL COMPONENTS IN LIFE
SUPPORT DEVICES OR SYSTEMS WITHOUT EXPRESS WRITTEN APPROVAL OF STMICROELECTRONICS.

STMicroelectronics GROUP OF COMPANIES

Australia - Belgium - Brazil - Canada - China - France - Germany - Italy - Japan - Korea -
Malaysia - Malta - Morocco - The Netherlands - Singapore - Spain - Sweden - Switzerland -
Taiwan - Thailand - United Kingdom - U.S.A.
STMicroelectronics Limited is a member of the STMicroelectronics Group.
********************************************************************************
Version History.

Revision 1-0-0 19/11/09
First Release

Revision 1-0-0 08/12/2009
First Release for KR3DH based on LIS331DLH
*******************************************************************************/


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <asm/uaccess.h>
#include <linux/unistd.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <mach/gpio.h>
#include <linux/timer.h>
#include <linux/jiffies.h>


//#define DEBUG 1	
#define STM_DEBUG 0

#define ACC_DEV_MAJOR 241

/* KR3DH I2C Slave Address */
#define KR3DH_I2C_ADDR    0x18

/* ST KR3DX Chip ID */
#define KR3DM_ID                    0x12
#define KR3DH_ID                    0x32

#define KR3DH_IOCTL_BASE 'a'
/** The following define the IOCTL command values via the ioctl macros */
#define KR3DH_IOCTL_SET_DELAY       _IOW(KR3DH_IOCTL_BASE, 0, int)
#define KR3DH_IOCTL_GET_DELAY       _IOR(KR3DH_IOCTL_BASE, 1, int)
#define KR3DH_IOCTL_SET_ENABLE      _IOW(KR3DH_IOCTL_BASE, 2, int)
#define KR3DH_IOCTL_GET_ENABLE      _IOR(KR3DH_IOCTL_BASE, 3, int)
#define KR3DH_IOCTL_SET_G_RANGE     _IOW(KR3DH_IOCTL_BASE, 4, int)
#define KR3DH_IOCTL_READ_ACCEL_XYZ  _IOW(KR3DH_IOCTL_BASE, 8, int)

//kr3dh registers
#define WHO_AM_I    0x0F

/* ctrl 1: pm2 pm1 pm0 dr1 dr0 zenable yenable zenable */
#define CTRL_REG1       0x20    /* power control reg */
#define CTRL_REG2       0x21    /* power control reg */
#define CTRL_REG3       0x22    /* power control reg */
#define CTRL_REG4       0x23    /* interrupt control reg */
#define CTRL_REG5       0x24    /* interrupt control reg */
#define AXISDATA_REG    0x28

#define KR3DH_G_2G    0x00
#define KR3DH_G_4G    0x10
#define KR3DH_G_8G    0x30

#define PM_OFF            0x00
#define PM_NORMAL         0x20
#define ENABLE_ALL_AXES   0x07

#define ODRHALF           0x40  /* 0.5Hz output data rate */
#define ODR1              0x60  /* 1Hz output data rate */
#define ODR2              0x80  /* 2Hz output data rate */
#define ODR5              0xA0  /* 5Hz output data rate */
#define ODR10             0xC0  /* 10Hz output data rate */
#define ODR50             0x00  /* 50Hz output data rate */
#define ODR100            0x08  /* 100Hz output data rate */
#define ODR400            0x10  /* 400Hz output data rate */

/** KR3DH acceleration data 
  \brief Structure containing acceleration values for x,y and z-axis in signed short

*/

typedef struct  
{
  short x, /**< holds x-axis acceleration data sign extended. Range -2048 to +2047. */
		y, /**< holds y-axis acceleration data sign extended. Range -2048 to +2047. */
		z; /**< holds z-axis acceleration data sign extended. Range -2048 to +2047. */

} kr3dh_acc_t;

static struct i2c_client *kr3dh_client = NULL;
static int chipID = 0x0;

struct kr3dh_data{
  struct i2c_client client;
};

static int open_count;

static struct class *kr3d_dev_class;

static char kr3dh_i2c_write(unsigned char reg_addr, unsigned char *data, unsigned char len);
static char kr3dh_i2c_read(unsigned char reg_addr, unsigned char *data, unsigned char len);

extern unsigned int HWREV;

/*************************************************************************/
/*		KR3DH Functions	  				         */
/*************************************************************************/
/* Device Initialization  */
int device_init(void/*kr3dh_t *pLis*/) 
{
	int res;
	unsigned char buf[5];
	buf[0]=ENABLE_ALL_AXES;			
	buf[1]=0x00;
	buf[2]=0x00;
	buf[3]=KR3DH_G_2G;
	buf[4]=0x00;
	res = kr3dh_i2c_write(CTRL_REG1, &buf[0], 5);
	return res;
}

int kr3dh_set_range(char range) 
{     
	int res = 0;	
	unsigned char data = 0;
	
	res = i2c_smbus_read_word_data(kr3dh_client, CTRL_REG4);
	if( res >=0 )
		data = res & 0x6f;
		
	data = data + range;	
	
	res = kr3dh_i2c_write(CTRL_REG4, &data, 1);
	return res;
}

// Data rate selection
int kr3dh_set_bandwidth(char bw) 
{
	int res = 0;
	unsigned char data = 0;
	
	res = i2c_smbus_read_word_data(kr3dh_client, CTRL_REG1);
	if (res>=0){         
		data = res & 0x00e7;
	}
	data = data + bw;
	
	res = kr3dh_i2c_write(CTRL_REG1, &data, 1);
	return res;
}

// read selected bandwidth from kr3dh 
int kr3dh_get_bandwidth(unsigned char *bw) {
	int res = 0;
	//TO DO 	
	res = i2c_smbus_read_word_data(kr3dh_client, CTRL_REG4);
	if( res >=0 )
		*bw = res & 0x30;
	
	return res;
}

// power mode selection
int kr3dh_set_enable(char mode)
{
	int res = 0;
	unsigned char data;

	res = i2c_smbus_read_word_data(kr3dh_client, CTRL_REG1);
	if (res>=0){         
		data = res & 0x001f;
	}
	data = mode + data;
	res = kr3dh_i2c_write(CTRL_REG1, &data, 1);
	return res;
}

/** X,Y and Z-axis acceleration data readout 
  \param *acc pointer to \ref kr3dh_acc_t structure for x,y,z data readout
  \note data will be read by multi-byte protocol into a 6 byte structure 
*/
int kr3dh_read_accel_xyz(kr3dh_acc_t * acc)
{
	int res;
	unsigned char acc_data[6];

	res = kr3dh_i2c_read(AXISDATA_REG,&acc_data[0], 6);
  
	/*
	acc->x = *(s16 *)&acc_data[0];
	acc->y = *(s16 *)&acc_data[2];
	acc->z = *(s16 *)&acc_data[4];
	*/	
	
	acc->x = (short) (((acc_data[1]) << 8) | acc_data[0]);
	acc->y = (short) (((acc_data[3]) << 8) | acc_data[2]);
	acc->z = (short) (((acc_data[5]) << 8) | acc_data[4]);
	
	/* mask low 4bit to remove dummy 4bit */
	acc->x = (acc->x) >> 4;
	acc->y = (acc->y) >> 4;
	acc->z = (acc->z) >> 4;
	

#if STM_DEBUG
	printk("[kr3dh_read_accel_xyz] acc_data[0] = %d, acc_data[1] = %d, acc_data[2] = %d, acc_data[3] = %d, acc_data[4] = %d, acc_data[5] = %d\n", 
		acc_data[0], acc_data[1], acc_data[2], acc_data[3], acc_data[4],  acc_data[5]);
	printk("[%s] x = %d, y = %d, z = %d\n", __func__, acc->x, acc->y, acc->z);
#endif	
	
	return res; 
}


/*************************************************************************/
/*		KR3DH I2C_API	  				         */
/*************************************************************************/

/*  i2c write routine for kr3dh */
static char kr3dh_i2c_write(unsigned char reg_addr, unsigned char *data, unsigned char len)
{
	int dummy;
	int i; 
	//printk(KERN_INFO"%s\n", __FUNCTION__);
	if( kr3dh_client == NULL )  /*  No global client pointer? */
		return -1;
	for(i=0;i<len;i++)
	{ 
		dummy = i2c_smbus_write_byte_data(kr3dh_client, reg_addr++, data[i]);
		if(dummy)
		{
			printk(KERN_INFO"i2c write error\n");
			return dummy; 
		}
	}
	return 0;
}

/*  i2c read routine for kr3dh  */
static char kr3dh_i2c_read(unsigned char reg_addr, unsigned char *data, unsigned char len) 
{
	int dummy=0;
	int i=0;
	//printk(KERN_INFO"%s\n", __FUNCTION__);
	if( kr3dh_client == NULL )  /*  No global client pointer? */
		return -1;
	while(i<len)
	{        
		dummy = i2c_smbus_read_word_data(kr3dh_client, reg_addr++);
		if (dummy>=0)
		{         
			data[i] = dummy & 0x00ff;
			i++;
		} 
		else
		{
			printk(KERN_INFO" i2c read error\n "); 
			return dummy;
		}
		dummy = len;
	}
	return dummy;
}


/*************************************************************************/
/*		KR3DH Sysfs	  				         */
/*************************************************************************/
//TEST
static ssize_t kr3dh_fs_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	int count;
	kr3dh_acc_t acc;
	kr3dh_read_accel_xyz(&acc);

	printk("x: %d,y: %d,z: %d\n", acc.x, acc.y, acc.z);
	count = sprintf(buf,"%d,%d,%d\n", acc.x, acc.y, acc.z );

	return count;
}

static ssize_t kr3dh_fs_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	//buf[size]=0;
	printk("input data --> %s\n", buf);

	return size;
}

static DEVICE_ATTR(acc_file, S_IRUGO | S_IWUSR | S_IWOTH | S_IXOTH, kr3dh_fs_read, kr3dh_fs_write);

/*************************************************************************/
/*		KR3DH operation functions			         */
/*************************************************************************/
/*  read command for kr3dh device file  */
static ssize_t kr3dh_read(struct file *file, char __user *buf, size_t count, loff_t *offset)
{
	int ret = 0;
	kr3dh_acc_t acc;  
	if( kr3dh_client == NULL )
		return -1;
	kr3dh_read_accel_xyz(&acc);

#if STM_DEBUG
	printk(KERN_INFO "KR3DH Accel: X/Y/Z axis: %-8d %-8d %-8d\n" ,
		(int)acc.x, (int)acc.y, (int)acc.z);  
#endif

	if( count != sizeof(acc) )
	{
		return -1;
	}
	ret = copy_to_user(buf,&acc, sizeof(acc));
	if( ret != 0 )
	{
#if STM_DEBUG
	printk(KERN_INFO "KR3DH Accel: copy_to_user result: %d\n", ret);
#endif
	}
	return sizeof(acc);
}

/*  write command for KR3DH device file */
static ssize_t kr3dh_write(struct file *file, const char __user *buf, size_t count, loff_t *offset)
{
	if( kr3dh_client == NULL )
		return -1;
#if STM_DEBUG
	printk(KERN_INFO "KR3DH Accel should be accessed with ioctl command\n");
#endif
	return 0;
}


/*  open command for KR3DH device file  */
static int kr3dh_open(struct inode *inode, struct file *file)
{
#if STM_DEBUG
		printk(KERN_INFO "%s\n",__FUNCTION__); 
#endif
	if( kr3dh_client == NULL)
	{
#if STM_DEBUG
		printk(KERN_INFO "I2C driver not install\n"); 
#endif
		return -1;
	}
	
	device_init();
	
	kr3dh_set_enable(PM_NORMAL);
	
	open_count++;
	
#if STM_DEBUG
	printk(KERN_INFO "KR3DH Accel has been opened\n");
#endif
	return 0;
}


/*  release command for KR3DH device file */
static int kr3dh_close(struct inode *inode, struct file *file)
{
#if STM_DEBUG
	printk(KERN_INFO "KR3DH Accel has been closed\n");
#endif
	
	open_count--;
	
	if(open_count == 0)
		kr3dh_set_enable(PM_OFF);

	return 0;
}


/*  ioctl command for KR3DH device file */
static int kr3dh_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	unsigned char data[6];
#if STM_DEBUG
	printk("kr3dh_ioctl\n");  
#endif
	
	// check kr3dh_client 
	if( kr3dh_client == NULL)
	{
#if STM_DEBUG
		printk("I2C driver not install\n"); 
#endif
		return -EFAULT;
	}
	
	/* cmd mapping */
	
	switch(cmd)
	{		
		/*case KR3DH_SELFTEST:
		//TO DO
		return err;*/
		
		case KR3DH_IOCTL_SET_G_RANGE:
			if(copy_from_user(data,(unsigned char*)arg,1)!=0)
			{
#if STM_DEBUG
				printk("copy_from_user error\n");
#endif
				return -EFAULT;
			}
			err = kr3dh_set_range(*data);
			return err;
		
		case KR3DH_IOCTL_SET_ENABLE:
			if(copy_from_user(data,(unsigned char*)arg,1)!=0){
#if STM_DEBUG
				printk("copy_to_user error\n");
#endif
				return -EFAULT;
			}
			err = kr3dh_set_enable(*data);
			return err;
		
		case KR3DH_IOCTL_SET_DELAY:
			if(copy_from_user(data,(unsigned char*)arg,1)!=0)
			{
#if STM_DEBUG
				printk("copy_from_user error\n");
#endif
				return -EFAULT;
			}
			err = kr3dh_set_bandwidth(*data);
			return err;
		
		case KR3DH_IOCTL_GET_DELAY:
			err = kr3dh_get_bandwidth(data);
			if(copy_to_user((unsigned char*)arg,data,1)!=0)
			{
#if STM_DEBUG
				printk("copy_to_user error\n");
#endif
				return -EFAULT;
			}
			return err;
		
		case KR3DH_IOCTL_READ_ACCEL_XYZ:
#if STM_DEBUG
			printk("[KR3DH] KR3DH_IOCTL_READ_ACCEL_XYZ !!!\r\n");
#endif
			err = kr3dh_read_accel_xyz((kr3dh_acc_t*)data);
			if(copy_to_user((kr3dh_acc_t*)arg,(kr3dh_acc_t*)data,6)!=0)
			{
#if STM_DEBUG
				printk("copy_to error\n");
#endif
				return -EFAULT;
			}
			return err;
		
		default:
			return 0;
	}
}

static const struct file_operations kr3dh_fops = {
	.owner = THIS_MODULE,
	.read = kr3dh_read,
	.write = kr3dh_write,
	.open = kr3dh_open,
	.release = kr3dh_close,
	.ioctl = kr3dh_ioctl,
};

static struct miscdevice kr3dh_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "kr3dh_accel",
	.fops = &kr3dh_fops,
};

static int kr3dh_detect(struct i2c_client *client, int kind,
			  struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;
#if STM_DEBUG
	printk(KERN_INFO "%s\n", __FUNCTION__);
#endif
	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -ENODEV;

	strlcpy(info->type, "kr3dh", I2C_NAME_SIZE);

	return 0;
}

static int kr3dh_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int err = 0;
	int tempvalue;
	struct kr3dh_data *data;
#if STM_DEBUG
	printk(KERN_INFO "%s\n",__FUNCTION__);
#endif

#if 1
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		printk(KERN_INFO "[KR3DH] i2c_check_functionality error\n");
		err = -ENODEV;
		goto exit;
	}
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_I2C_BLOCK))
	{
		err = -ENODEV;
		goto exit;
	}

	/* OK. For now, we presume we have a valid client. We now create the
	client structure, even though we cannot fill it completely yet. */
	if (!(data = kmalloc(sizeof(struct kr3dh_data), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	
	memset(data, 0, sizeof(struct kr3dh_data));
	i2c_set_clientdata(client, data);
	kr3dh_client = client;

#if 1
	if ( (err = i2c_smbus_read_byte(client)) < 0)
	{
		printk(KERN_ERR "[KR3DH] i2c_smbus_read_byte error!!\n");
		goto exit_kfree;
	}
	else
	{
		printk("KR3DH Device detected!\n");
	}
#endif


	printk("[%s] slave addr = %x\n", __func__, client->addr);

#if 0
	/* read chip id */
	tempvalue = 0;
	tempvalue = i2c_smbus_read_word_data(client,WHO_AM_I);
	
	chipID = KR3DH_ID;
	if(HWREV >= 0x01)	// from real rev0.0
		chipID += 1;	// 0x33
	
	if((tempvalue & 0x00FF) == chipID)  // changed for KR3DH.
	{
		printk("KR3DH I2C driver registered!\n");
		kr3dh_client = client;
	}
	else
	{
		printk(KERN_ERR "KR3DH I2C driver not registered 0x%x!\n", (tempvalue& 0xFF));
		kr3dh_client = NULL;
		err = -ENODEV;
		goto exit_kfree;
	}
#else
	/* read chip id */
	tempvalue = 0;
	tempvalue = i2c_smbus_read_word_data(client,WHO_AM_I);
	
	chipID = tempvalue & 0x00FF;
	printk("KR3DH I2C driver registered!. Chip ID = %x\n", chipID);
	kr3dh_client = client;
#endif

#if 1
	device_init();
	kr3dh_set_enable( PM_NORMAL );	
#endif	
	
//	i2c_set_clientdata(kr3dh_client, data);
  	err = misc_register(&kr3dh_device);
	if (err) {
		printk(KERN_ERR "KR3DH accel device register failed\n");
		goto exit_kfree;
	}

#endif
	printk(KERN_INFO "KR3DH accel device create ok\n");	

	return 0;

exit_kfree:
	kfree(data);
exit:
	return err;
}


static int kr3dh_remove(struct i2c_client *client)
{
	struct kr3dh_data *data = i2c_get_clientdata(client);
#if STM_DEBUG
	printk(KERN_INFO "%s\n",__FUNCTION__);
#endif	
	kr3dh_set_enable(PM_OFF);

	misc_deregister(&kr3dh_device);

	kfree(data);
	kr3dh_client = NULL;
	return 0;
}


#ifdef CONFIG_PM
static int kr3dh_suspend(struct i2c_client *client, pm_message_t mesg)
{	
	kr3dh_set_enable(PM_OFF);
	
#if STM_DEBUG
	printk(KERN_INFO "[%s] KR3DH !!suspend mode!!\n",__FUNCTION__);
#endif

	return 0;
}

static int kr3dh_resume(struct i2c_client *client)
{
	kr3dh_set_enable(PM_NORMAL);
	
#if STM_DEBUG
	printk(KERN_INFO "[%s] KR3DH !!resume mode!!\n",__FUNCTION__);
#endif

	return 0;
}
#else
#define kr3dh_suspend NULL
#define kr3dh_resume NULL
#endif


static unsigned short normal_i2c[] = { I2C_CLIENT_END};
I2C_CLIENT_INSMOD_1(kr3dh_accel);

static const struct i2c_device_id kr3dh_id[] = {
	{ "kr3dh_accel", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, kr3dh_id);

static struct i2c_driver kr3dh_accel_driver = {	
	.driver = {
		.owner	= THIS_MODULE,	
		.name	= "kr3dh_accel",
	},
	.class		= I2C_CLASS_HWMON,
	.id_table	= kr3dh_id,
	.address_data	= &addr_data,
	.probe		= kr3dh_probe,
	.remove		= kr3dh_remove,
	.detect		= kr3dh_detect,
	.suspend	= kr3dh_suspend,
	.resume		= kr3dh_resume,
};

static int __init kr3dh_init(void)
{
	struct device *dev_t;

#if STM_DEBUG
	printk(KERN_INFO "%s\n",__FUNCTION__);
#endif

	kr3d_dev_class = class_create(THIS_MODULE, "accelerometer");

	if (IS_ERR(kr3d_dev_class)) 
		return PTR_ERR( kr3d_dev_class );

	dev_t = device_create( kr3d_dev_class, NULL, 0, "%s", "accelerometer");
	
	if (IS_ERR(dev_t)) 
	{
		return PTR_ERR(dev_t);
	}

	if (device_create_file(dev_t, &dev_attr_acc_file) < 0)
		printk("Failed to create device file(%s)!\n", dev_attr_acc_file.attr.name);
	
	return i2c_add_driver(&kr3dh_accel_driver);
}

static void __exit kr3dh_exit(void)
{
	i2c_del_driver(&kr3dh_accel_driver);
	printk(KERN_ERR "KR3DH_ACCEL exit\n");

	class_destroy( kr3d_dev_class );
}

module_init(kr3dh_init);
module_exit(kr3dh_exit);

MODULE_DESCRIPTION("kr3dh accelerometer driver");
MODULE_AUTHOR("STMicroelectronics");
MODULE_LICENSE("GPL");

                     