#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/rpmsg.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/dma-mapping.h>
#include <linux/miscdevice.h>
#include <linux/eventfd.h>
#include <linux/of_platform.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#define MSG	"h"
#define RPMSG_SDB_DRIVER_VERSION "1.0"
#define MSG_RECEP 100  

static const char rpmsg_sdb_driver_name[] = "stm32-rpmsg-sdb";
struct mutex mymutex;
struct miscdevice mydev;
static int i =0, n=0;
static uint8_t recv_full[496*MSG_RECEP];
unsigned long start_time, end_time, tot_time;

static wait_queue_head_t my_wait_queue;
static int wait_queue_flag = 0;

static int dev_open(struct inode *inode, struct file *file){
	printk("Open fops user");	
	return 0;
}

static ssize_t dev_read(struct file *fil, char *buff, size_t len, loff_t *off){
	int k=0;

	wait_event_interruptible(my_wait_queue, wait_queue_flag != 0);
	wait_queue_flag = 0;

	k=copy_to_user(buff, recv_full, sizeof(recv_full));
	i=0;

	return sizeof(recv_full);
}

static int dev_close(struct inode *inode, struct file *file){
	printk("Close fops user");
	return 0;
}

static const struct file_operations user_fops = {
	.owner		= THIS_MODULE,
	.read 		= dev_read,
	.open           = dev_open,
	.release        = dev_close,
};

static int rpmsg_sdb_drv_cb(struct rpmsg_device *rpdev, void *data, int len,
			void *priv, u32 src){
	int k;
	uint8_t *buf;
 	buf= (uint8_t*)data;

	for(k=0; k<496;k++){
		recv_full[k+496*i]=buf[k];
	}

	i++;
	n++;

	if(i==MSG_RECEP){
	       	wait_queue_flag = 1 ;
		wake_up_interruptible(&my_wait_queue);
	}
	rpmsg_send(rpdev->ept, MSG, strlen(MSG));

	return len;
}

static int rpmsg_sdb_drv_probe(struct rpmsg_device *rpdev){

	int reg =0;
	
	rpmsg_send(rpdev->ept, MSG, strlen(MSG));
	start_time = jiffies;

	return reg;
}

static void rpmsg_sdb_drv_remove(struct rpmsg_device *rpmsgdev){
	unsigned int total=0;
	end_time=jiffies;
	tot_time=end_time-start_time;
	total=jiffies_to_msecs(tot_time);
	printk("Message received %d \n",n);
	printk("End Time [%lu] - Start Time [%lu]: \n",end_time,start_time);
	printk("Totlal Time : %d ms\n",total);	
}

static struct rpmsg_device_id rpmsg_driver_sdb_id_table[] = {
	{ .name	= "rpmsg-sdb-channel" },
	{ },
};
MODULE_DEVICE_TABLE(rpmsg, rpmsg_driver_sdb_id_table);

static struct rpmsg_driver rpmsg_sdb_rmpsg_drv = {
	.drv.name	= KBUILD_MODNAME,
	.drv.owner	= THIS_MODULE,
	.id_table	= rpmsg_driver_sdb_id_table,
	.probe		= rpmsg_sdb_drv_probe,
	.callback	= rpmsg_sdb_drv_cb,
	.remove		= rpmsg_sdb_drv_remove,
};

static int __init rpmsg_sdb_drv_init(void){
	int ret = 0;
	printk("We are in the init part\n");
	pr_info("%s(rpmsg_sdb): Init done\n", __func__);
	/* Register rpmsg device */
	ret = register_rpmsg_driver(&rpmsg_sdb_rmpsg_drv);
	mutex_init(&mymutex);
	if (ret) {
		pr_err("%s(rpmsg_sdb): Failed to register device\n", __func__);
		return ret;
	}
	mydev.name = "rpmsg-client-user";
	mydev.minor = MISC_DYNAMIC_MINOR;
	mydev.fops = &user_fops;
	printk("Name : %s\r\n", mydev.name  );
	printk("MISC_DYNAMIC_MINOR : %d\r\n", mydev.minor  );
	/* Register misc device */
	ret = misc_register(&mydev);

	if (ret) {
		printk("Erreur du misc_register : user_fops\r\n");
	}
	
	// -- initialize the WAIT QUEUE head
	init_waitqueue_head(& my_wait_queue);

	return ret;
}

static void __exit rpmsg_sdb_drv_exit(void){
	misc_deregister(&mydev);
	unregister_rpmsg_driver(&rpmsg_sdb_rmpsg_drv);
	pr_info("%s(rpmsg_sdb): Exit\n", __func__);
	printk("We are in the exit\n");
}

module_init(rpmsg_sdb_drv_init);
module_exit(rpmsg_sdb_drv_exit);


MODULE_AUTHOR("GABRY Aurélien");
MODULE_DESCRIPTION("Data Buffer over RPMSG");
MODULE_LICENSE("GPL v2");
