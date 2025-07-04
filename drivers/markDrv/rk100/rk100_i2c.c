#include <linux/module.h>
//#include <linux/config.h>
#include <linux/errno.h>
#include <linux/miscdevice.h>
#include <linux/fcntl.h>
#include <linux/init.h>
#include <linux/delay.h>
//#include <linux/proc_fs.h>
//#include <linux/workqueue.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
//#include <asm/system.h>
#include <asm/io.h>
#include <linux/platform_device.h>

#include <linux/init.h>
#include <linux/delay.h>
#include <linux/sched.h>   //wake_up_process()

#include <linux/kthread.h> //kthread_create()、kthread_run()


#include <linux/fs.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/atomic.h>
#include <linux/module.h>
//#include <linux/config.h>
#include <linux/errno.h>
#include <linux/miscdevice.h>
#include <linux/fcntl.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/delay.h>
//#include <linux/proc_fs.h>
//#include <linux/workqueue.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <asm/io.h>

#include <linux/random.h>

#include <linux/sched.h>
#include <linux/timer.h>
////////////////////////////////////////////////////////////////////////

#define ZTL_RK100_I2C_NAMEE "video_map"

#define NAME "video_map1"
#define DBUG true
#define DEBUG(x...) if(DBUG){printk(x);}


#define ONESECOND 3
#define TWOSECOND 6
#define THREESECOND 10

struct timer_list mtimer;//定义一个定时器
static int second = 1;
static int times = 1;

#include <linux/kthread.h> //kthread_create()、kthread_run()
 #include <linux/reboot.h>

#include <linux/fs.h>
#include <asm/uaccess.h>
#include <asm/io.h>
static int isTimerEnable = 1;
static int i_system_err  = 1;
static int i_boot_count  = 1;
#define ENABLE_BOOT_TIME_LIMIT 0
#define BOOT_TIME_MAX_LIMIT 180//180 // 180s

//static user_gpio_set_t  gpio_int_info[1];
static struct i2c_client *gi2c_client = NULL;
#define SET_IIC_ADDR 0x30>>1
typedef unsigned char uchar;
static const unsigned short normal_i2c[] = {
    SET_IIC_ADDR, I2C_CLIENT_END};
static const struct i2c_device_id i2c_encode_id[] = {
{ZTL_RK100_I2C_NAMEE, 0 },
{}
};
MODULE_DEVICE_TABLE(i2c, i2c_encode_id);

char driver_status[32];



/*读一个寄存器的接口*/
inline static int read_reg(struct i2c_client *client, char *reg,char *buf,int length,int datalen)
{
    //    int ret = 0;
    DEBUG("shx : send %d \n",datalen);
    i2c_master_send(client, reg, datalen);  // 发送寄存器地址
    msleep(10);
    return i2c_master_recv(client, buf, length);  // 接收寄存器的值
}

/********************************************
 *   设置i2c的功能
*******************************************/
inline static int write_i2c(struct i2c_client *client,uchar addr,uchar* buf,int length){
    char write_buf[257] ;
    write_buf[0] = addr;
    memcpy(write_buf+1,buf,length);
    return i2c_master_send(client,write_buf,length+1);  // 发送寄存器地址 ,值
}

#if 0
/* Return 0 if detection is successful, -ENODEV otherwise */
static int i2c_encode_detect(struct i2c_client *new_client,
			     struct i2c_board_info *info){
    struct i2c_adapter *adapter = new_client->adapter;
    //static __u32 twi_id = 3;// which i2c bus
    __u32 twi_id = 1;// which i2c bus
    //    DEBUG("lm90 detect::::: %d\n",adapter->nr);
    if(twi_id == adapter->nr)
    {
	DEBUG("%s: Detected :chip %s at adapter %d, address 0x%02x\n",
	       __func__, ZTL_RK100_I2C_NAMEE, i2c_adapter_id(adapter), new_client->addr);

	strlcpy(info->type, ZTL_RK100_I2C_NAMEE, I2C_NAME_SIZE);
	return 0;
    }else{
	//		DEBUG("err  %s: Detected :chip %s at adapter %d, address 0x%02x\n",
	//             __func__, ZTL_RK100_I2C_NAMEE, i2c_adapter_id(adapter), new_client->addr);
	return -ENODEV;
    }
}
#endif
#if 0
static int ding_reg_open(struct inode *inode, struct file *file)
{
    DEBUG("My device opened===================.\n");
    return 0;
}

static ssize_t ding_reg_write(struct file *file,const  char __user *buffer, size_t size, loff_t *l)
{
    char bufTemp[257] = {0};
   // unsigned char iMapAddr = 0;
   // unsigned int iDataValue  = 0;
  //  volatile unsigned int *reserve_virt_addr;
    int ret;
    int mSize = size;
    //    DEBUG("============ding_reg_write========================\n" );

    ret = copy_from_user(bufTemp, buffer, size);
    DEBUG("My de-vice ding_reg_write.iOffset:%d,0x%x,0x%x\n",mSize,bufTemp[0],bufTemp[1]);
//    iMapAddr = *(unsigned char *)bufTemp;

    ret = i2c_master_send(gi2c_client,bufTemp,size);
    if(ret > 0 ){
//	ret = copy_to_user(buffer,bufTemp , ret);
//	DEBUG("read addr:0x%X :0x%X\n",bufTemp[0],bufTemp[1]);
	return ret;
    }

    //  /*  if(iMapAddr > 0xF000000){

    //	return -1;
    //    }*/

    //    iDataValue = *(int *)(bufTemp+4);
    //    DEBUG("My device ding_reg_write.iOffset:0x%x\n",iDataValue);

    //    DEBUG("My device ding_reg_write.\n");
    //    if(reserve_virt_addr!=NULL){
    //	DEBUG("reserve_virt_addr = 0x%x\n", *(unsigned int*)(reserve_virt_addr));

    ////	copy_to_user(buffer,reserve_virt_addr , 4);
    //	*reserve_virt_addr = iDataValue;
    //	ret = copy_to_user((void __user *)buffer,(void*)reserve_virt_addr , 4);

    //    }else{
    //	return -1;
    //    }

    return 0;
}


// buffer : addr offset length
static ssize_t ding_reg_read(struct file *file, char __user *buffer, size_t count, loff_t *ppos){
   // char bufTemp[257] = {0};
    char mcmd[32] = {0};
    char cbuf[32] = {0};
//    unsigned char iMapAddr = 0;
    int ret;
    //    unsigned int iOffset  = 0;
//    char *reserve_virt_addr = NULL;
    DEBUG("============ding_reg_read========================\n" );

	ret = copy_from_user(&mcmd[0], buffer, count);
	if(ret < 0){
		DEBUG("%s(%d) : value err!\n",__func__,__LINE__);
		return -EINVAL;
	}
//    iMapAddr = *(unsigned char *)bufTemp;
	
	ret = read_reg(gi2c_client, mcmd,cbuf,12,13);
	if(ret < 0){
		DEBUG("%s(%d) : I2C read_reg err!\n",__func__,__LINE__);
		msleep(300);
		ret = read_reg(gi2c_client, mcmd,cbuf,12,13);
		if(ret < 0){
			DEBUG("shx : I2C read_reg err(2)!\n");
			return -EAGAIN;
		}
	}
	
	DEBUG("chip return buf:0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x\n",cbuf[0],cbuf[1],cbuf[2],cbuf[3],cbuf[4],cbuf[5],cbuf[6],cbuf[7],cbuf[8],cbuf[9],cbuf[10],cbuf[11]);

	ret = copy_to_user(buffer,cbuf,ret);



//	DEBUG("My device ding_reg_read.iMapAddr:%x\n",iMapAddr);
/*    DEBUG("My de-vice ding_reg_write.iOffset:%d,0x%x,0x%x\n",count,bufTemp[0],bufTemp[1]);
	ret = i2c_master_recv(gi2c_client, bufTemp, count);
	if(ret > 0 ){
	    ret = copy_to_user(buffer,bufTemp , ret);
	    DEBUG("read addr:0x%X :0x%X\n",bufTemp[0],bufTemp[1]);
	    return ret;
	}
*/

    /* if(iMapAddr > 0xF000000){

	return -1;
    }*/

    //    iOffset = *(int *)(bufTemp+4);
    //    DEBUG("My device ding_reg_read.iOffset:%x\n",iOffset);
    //    if(iOffset > 0xF000000){

    //	return -1;
    //    }

    //    reserve_virt_addr = ioremap(iMapAddr,16);
    //    if(reserve_virt_addr!=NULL){
    //	DEBUG("reserve_virt_addr = 0x%x\n", *(unsigned int*)(reserve_virt_addr));

    //	ret = copy_to_user(buffer,reserve_virt_addr , 4);

    //	iounmap((void *)reserve_virt_addr);
    //    }else{
    //	return -1;
    //    }
    //    char *reserve_virt_addr = ioremap(0x01F02C00,1024);


    return ret;
}
/*
static struct file_operations ding_reg_fops =
{
    .owner = THIS_MODULE,
    .open = ding_reg_open,
    .read = ding_reg_read,
    .write = ding_reg_write,
//    .close =
};
*/
/*
static struct miscdevice reg_misc =
{
    .minor = MISC_DYNAMIC_MINOR,
    .name = NAME,
    .fops = &ding_reg_fops,
};
*/
#endif
int check_data_first(char *src,char *tmp){
    int i;
	char uCRC=driver_status[0]^driver_status[7];//校验初始值 
	if(uCRC==0){
		uCRC = driver_status[9]|0x1;
	}
	DEBUG("uCRC : 0x%02x\n",uCRC);
	DEBUG("shx : first check !\n");
//	DEBUG("-------------------first check--------------------\n");
	for(i=0;i<12;i++) {
	//	uCRC^=src[i]; 
		DEBUG("%d:0x%02x  ",i,(src[i]^uCRC));
		if(tmp[i] != (src[i]^uCRC)){
//			DEBUG("\n");
			DEBUG("\n%s(%d) : check err\n",__func__,__LINE__);
			return -1;
		}
	}
	DEBUG("\n");
//	DEBUG("\nshx : check succeed return 0\n");
//	DEBUG("\n\n");
	return 0;
}



static int i2c_encode_probe(struct i2c_client *new_client,
			    const struct i2c_device_id *id){
    DEBUG( "i2c network test  probe... \n");
  //  printk("RK100 I2C Address: 0x%02x\n", new_client->addr);
    //printk( "shx :i2c network test  probe... \n");

	DEBUG("susan now is rk100<<%s %s %d\n",__func__,__FILE__,__LINE__);
    gi2c_client = new_client;
    return 0;
}


/********************************************

********************************************/
static int check_crc(int a){
    char cbuf[200]={0};
    char mcmd[32];
    int ret = 0;
    int i = 0;
    int crc;
  //  char *por = NULL;
    char rand_buf[32];
    DEBUG( "i2c network test  probe... \n");
   // printk( "shx :i2c network test  probe... \n");

//    gi2c_client = new_client;

	//get CHIP_ID
	memset(cbuf,0,sizeof(cbuf));
	mcmd[0] = 0xa1;
	ret = read_reg(gi2c_client, mcmd,cbuf,12,1);
	if(ret < 0){
		DEBUG("shx : get chip id failed 1!\n");
	}
	DEBUG("shx : get chip id \n");
	DEBUG("gi2c_client buf:0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x\n",cbuf[0],cbuf[1],cbuf[2],cbuf[3],cbuf[4],cbuf[5],cbuf[6],cbuf[7],cbuf[8],cbuf[9],cbuf[10],cbuf[11]);
	memcpy(driver_status,cbuf,12);
	
	crc=0;
	//chip check1
	for(i = 0 ; i < 12 ; i++)
	{
		get_random_bytes(&rand_buf[i],sizeof(rand_buf[i]));
		DEBUG("0x%x  ",rand_buf[i]);
	}
	DEBUG("\n");
	rand_buf[4] = driver_status[4]^rand_buf[3];// flag 1
	rand_buf[7] = driver_status[2]^rand_buf[6];// flag 2
	
	for(i = 0 ; i < 3 ; i++){
		mcmd[0] = 0xb1;
		memcpy(mcmd+1,rand_buf,12);
		ret = read_reg(gi2c_client, mcmd,cbuf,12,13);
		if(ret < 0){
			DEBUG("shx : check chip id failed 1 !\n");
		}else{
			ret = check_data_first(rand_buf,cbuf);
			if(ret < 0){
				DEBUG("shx : check_data_first failed !\n");
			}else{
				DEBUG("shx : check_data_first succeed!\n");
				//isTimerEnable = 0;
				i_system_err = 0;
				crc = 1;
				a = 3;
			}
			
		}
		DEBUG("chip return buf:0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x\n",cbuf[0],cbuf[1],cbuf[2],cbuf[3],cbuf[4],cbuf[5],cbuf[6],cbuf[7],cbuf[8],cbuf[9],cbuf[10],cbuf[11]);
		
		
	}
	if(crc == 0 && a==2){
		DEBUG("shx : set timer to die \n");
		isTimerEnable = 1;	//disable timer
		i_system_err = 1;
	//	por[1] = 0;
	}
	DEBUG("shx : probe end!\n");

    DEBUG(  "==============ret :%d============\n",ret);
    return crc;
  /*  
err1:
	return ret;
  */
}

static int i2c_encode_remove(struct i2c_client *client)
{
    DEBUG(  " i2c network test _remove \n");
    //    kobject_del(&cvbs_object);
   // misc_deregister(&reg_misc);
    return 0;
}


 static struct of_device_id ztl_rk100_ids[] = {
   {.compatible = "ztl,rk100"},
   {}   
 };

 static const struct i2c_device_id ztl_rk100_id[] = { 
    {ZTL_RK100_I2C_NAMEE, 0}, 
    {}   
 };
 MODULE_DEVICE_TABLE(i2c, ztl_rk100_id);

 

static struct i2c_driver i2c_encode_driver = {
//    .class		= I2C_CLASS_HWMON,
    .driver = {
	.name	= ZTL_RK100_I2C_NAMEE,
	.owner = THIS_MODULE, 
     	.of_match_table = of_match_ptr(ztl_rk100_ids), 
    },
    .probe		= i2c_encode_probe,
    .remove		= i2c_encode_remove,
    .id_table	= ztl_rk100_id,
//    .id_table	= i2c_encode_id,
//    .detect		= i2c_encode_detect,
//    .address_list	= normal_i2c,
};


int i2c_init(void)
{
    DEBUG(  "ztl:rkdev:chip_i2c_init...\n");
 //   printk(  "shx:rkdev:chip_i2c_init...\n");
    return i2c_add_driver(&i2c_encode_driver);
}
//chip_i2c_exit
void i2c_exit(void)
{
    i2c_del_driver(&i2c_encode_driver);
//    printk(  "shx:rkdev:chip_i2c_exist...\n");
}

void  mtimer_function(unsigned long arg)
{
		int i;
		char *por = NULL;	
		DEBUG("times = %d \n",times);
        //	DEBUG("receive data from timer: %d\n",arg);
		if(isTimerEnable){
			if(i_system_err){
				DEBUG("System die!!\n");
				printk("CPU failure to wait for data\n");
				for(i = 0; i < 32 ; i++)
				{	
					por[i] = i;
				}
			}else{
				

			}
			
				
			//isTimerEnable = 0;
	
	/*   		if(times <= ONESECOND)
  	 		{
				printk("shx bef exist\n");
			//	i2c_exit();
				printk("shx bef sleep\n");
			//	msleep(10);
				printk("shx bef init\n");
			//	i2c_init();	
			}else if(times > ONESECOND && times < THREESECOND){
				second = 1;
				DEBUG("delay 700ms!!\n");
			//	mdelay(700);
			}else if(times >= THREESECOND){
				DEBUG("System die!!\n");
				printk("CPU failure to wait for data！\n");
				for(i = 0; i < 32 ; i++)
				{
			//		por[i] = i;
				}
				
	
				isTimerEnable = 0;
	
			}
	*/	
			if(i_boot_count>=0)
   				mod_timer(&mtimer, jiffies + (second*HZ));//重新设置定时器，每隔5秒执行一次
		}
		times++;
		
}


static ssize_t ztlcfg_att_store(struct device *dev, 
					struct device_attribute *attr, 
					const char *buf, size_t count) 
{
	if(count>0){
		printk("ztlcfg_att_store2 buf:%s,count:%d\n",buf ,(int)count);

		if(strcmp(buf,"51953\n")==0){
			printk("ztlcfg_att_store need reboot\n" );
			//msleep(1000); // wait 1s
			//kernel_restart(NULL);// system restart
			i_boot_count = BOOT_TIME_MAX_LIMIT-1 ;
			return count;
		}else if(strcmp(buf,"654987\n")==0){
			i_boot_count = -100;// stop restart
			return count;
		}else if(strcmp(buf,"519531\n")==0){
			printk("ztlcfg_att_store need reboot\n" );
			msleep(1000); // wait 1s
			kernel_restart(NULL);// system restart
			return count;
		}
	}	

	return count;
}

static ssize_t ztlcfg_att_show(struct device *dev,
                 struct device_attribute *attr,
                 char *buf)
{
	printk("cat debug %d\n",i_boot_count);
	sprintf(buf,"%d\n",i_boot_count);
	return 2;
}

static DEVICE_ATTR(ztlcfg,0777,ztlcfg_att_show,ztlcfg_att_store);
// static struct att_dev *dev = NULL;
static struct kobject *ztlcfg_kobj=NULL;

// int get_system_type(int iTemp){
// 	return i_enable_touch ;
// }
// EXPORT_SYMBOL(get_system_type);
static struct task_struct * reboot_tsk;  
static int thread_function(void *data)  
{  
	i_boot_count = 0;
    do {
    	printk( "zts thread_function :%d\n" ,i_boot_count);
		#if ENABLE_BOOT_TIME_LIMIT
		if(i_boot_count>=0){
			if(i_boot_count++>BOOT_TIME_MAX_LIMIT){//180 s
				printk("wait boot end too long..let us reboot..\n");
				kernel_restart(NULL);
			}
			if(i_boot_count>100){
				printk("wait boot end too long...%d.\n",i_boot_count);
			}
		}else{
			return 0;
		}
		#else
			return 0;
		#endif

       schedule_timeout_interruptible(msecs_to_jiffies(1000));// 1s

    }while(!kthread_should_stop());  

    return 0;  
}  

static __init int chip_i2c_init(void)
{

    int ret = 0;
    int i = 0;
        i2c_init();
    for(i = 0 ;i < 3 ;i++){
        i2c_init();
	msleep(1000);
	ret=check_crc(i);
	if(ret == 0){ 
	     i2c_exit();
	     printk("wait for data again\n");
	}else{ 
	     i = 3;
	}
//    }
  }
//	printk("shx is enable timer %d\n",isTimerEnable);
    if(isTimerEnable){
        DEBUG("add timer\n");
        init_timer(&mtimer);     //初始化定时器
        mtimer.expires = jiffies+(7*HZ);//设定超时时间，25秒
        mtimer.data = 5;    //传递给定时器超时函数的值
        mtimer.function = mtimer_function;//设置定时器超时函数
        add_timer(&mtimer); //添加定时器，定时器开始生效
    }else{
		
    }
    if(i_system_err==0){
    	printk("CPU wait for data succeed,%d\n",isTimerEnable);
    	if(ztlcfg_kobj == NULL){//zts add
			ztlcfg_kobj = kobject_create_and_add("ztlcfg", NULL); 
			if(ztlcfg_kobj == NULL){
	  		}else{
				ret = sysfs_create_file(ztlcfg_kobj,&dev_attr_ztlcfg.attr);
				reboot_tsk =kthread_run(thread_function,NULL,"reboot_thread");
			}
			
		}
    }
    

	DEBUG(  " i2c network test init \n");
    return 0;	
}

static void __exit   chip_i2c_exit(void)
{
    //    unregister_chrdev(major, "hello_world");

    //    device_destroy(hello_world_class,  MKDEV(major, 0));
    //    class_destroy(hello_world_class);
    DEBUG(  "goodbye world!");
    del_timer(&mtimer);
    i2c_exit();
    if(ztlcfg_kobj != NULL){
	    sysfs_remove_file(ztlcfg_kobj,&dev_attr_ztlcfg.attr);
	    kobject_del(ztlcfg_kobj);
	}
		if(reboot_tsk)
	{
		printk("stop reboot_tsk\n");
		kthread_stop(reboot_tsk);
	}
}



MODULE_INFO(intree,"Y");
MODULE_AUTHOR("ztl");
module_init(chip_i2c_init);
module_exit(chip_i2c_exit);


MODULE_LICENSE("GPL");


