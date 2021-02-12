/*
 * Copyright (c) 2020, STMICROLECTRONICS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <string.h>
#include <sys/printk.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <drivers/ipm.h>
#include <openamp/open_amp.h>
#include <metal/device.h>
#include <resource_table.h>
#include <sys_clock.h>
#include <logging/log.h>
#include <drivers/spi.h>
#include <drivers/gpio.h>
LOG_MODULE_REGISTER(openamp_rsc_table, LOG_LEVEL_DBG);

#define RPMSG_CHAN_NAME	"rpmsg-sdb-channel"
#define SHM_DEVICE_NAME	"shm"
#if !DT_HAS_CHOSEN(zephyr_ipc_shm)
#error "Sample requires definition of shared memory for rpmsg"
#endif

/* Constants derived from device tree */
#define SHM_NODE		DT_CHOSEN(zephyr_ipc_shm)
#define SHM_START_ADDR	DT_REG_ADDR(SHM_NODE)
#define SHM_SIZE		DT_REG_SIZE(SHM_NODE)
#define SPI DT_NODELABEL(spi4)


#define APP_TASK_STACK_SIZE (512)

static const struct device *ipm_handle;
static metal_phys_addr_t shm_physmap = SHM_START_ADDR;

struct metal_device shm_device = {
	.name = SHM_DEVICE_NAME,
	.num_regions = 2,
	.regions = {
		{.virt = NULL}, /* shared memory */
		{.virt = NULL}, /* rsc_table memory */
	},
	.node = { NULL },
	.irq_num = 0,
	.irq_info = NULL
};

static struct metal_io_region *shm_io;
static struct rpmsg_virtio_shm_pool shpool;

static struct metal_io_region *rsc_io;
static struct rpmsg_virtio_device rvdev;

static void *rsc_table;

static char rcv_msg[20]; 
static uint8_t  buffer_msg[496]; 
static uint8_t  buffer_msg2[496]; 
static unsigned int rcv_len;
static struct rpmsg_endpoint rcv_ept;

static uint8_t flag_1=0;
static uint8_t flag_2=0;

static unsigned char *msg;
static unsigned int len;

static K_SEM_DEFINE(data_sem, 0, 1);
static K_SEM_DEFINE(data_rx_sem, 0, 1);
static K_SEM_DEFINE(data_buff, 0, 1);

uint64_t start_time;
uint64_t time_spent;

static struct device *spi; 

static struct spi_config in_read_conf={
	.frequency=50000000U,
	.operation=(SPI_OP_MODE_MASTER
			|SPI_MODE_CPHA
			|SPI_WORD_SET(16)
			|SPI_LINES_SINGLE), 
	.cs=NULL,	
};
static struct spi_config *read_conf=&in_read_conf; 

static void platform_ipm_callback(const struct device *device, void *context,
				  uint32_t id, volatile void *data)
{
	//printk("%s: msg received from mb %d\n", __func__, id);
	k_sem_give(&data_sem);
}

static int rpmsg_recv_callback(struct rpmsg_endpoint *ept, void *data,
			       size_t len, uint32_t src, void *priv)
{
	memcpy(rcv_msg, data, len);
	rcv_len = len;
	k_sem_give(&data_rx_sem);

	return RPMSG_SUCCESS;
}

static void receive_message(unsigned char **msg, unsigned int *len)
{
	while (k_sem_take(&data_rx_sem, K_NO_WAIT) != 0) {
		int status = k_sem_take(&data_sem, K_FOREVER);

		if (status == 0) {
			rproc_virtio_notified(rvdev.vdev, VRING1_ID);
		}
	}
	*len = rcv_len;
	*msg = rcv_msg;
}

static void new_service_cb(struct rpmsg_device *rdev, const char *name,
			   uint32_t src)
{
	printk("%s: unexpected ns service receive for name %s\n",
		__func__, name);
}

int mailbox_notify(void *priv, uint32_t id)
{
	ARG_UNUSED(priv);

	//printk("%s: msg received\n", __func__);
	ipm_send(ipm_handle, 0, id, NULL, 0);

	return 0;
}

int platform_init(void)
{
	void *rsc_tab_addr;
	int rsc_size;
	struct metal_device *device;
	struct metal_init_params metal_params = METAL_INIT_DEFAULTS;
	int status;

	status = metal_init(&metal_params);
	if (status) {
		LOG_DBG("metal_init: failed: %d\n", status);
		return -1;
	}

	status = metal_register_generic_device(&shm_device);
	if (status) {
		LOG_DBG("Couldn't register shared memory: %d\n", status);
		return -1;
	}

	status = metal_device_open("generic", SHM_DEVICE_NAME, &device);
	if (status) {
		LOG_DBG("metal_device_open failed: %d\n", status);
		return -1;
	}

	/* declare shared memory region */
	metal_io_init(&device->regions[0], (void *)SHM_START_ADDR, &shm_physmap,
		      SHM_SIZE, -1, 0, NULL);

	shm_io = metal_device_io_region(device, 0);
	if (!shm_io) {
		LOG_DBG("Failed to get shm_io region\n");
		return -1;
	}

	/* declare resource table region */
	rsc_table_get(&rsc_tab_addr, &rsc_size);
	rsc_table = (struct st_resource_table *)rsc_tab_addr;

	metal_io_init(&device->regions[1], rsc_table,
		      (metal_phys_addr_t *)rsc_table, rsc_size, -1, 0, NULL);

	rsc_io = metal_device_io_region(device, 1);
	if (!rsc_io) {
		LOG_DBG("Failed to get rsc_io region\n");
		return -1;
	}

	/* setup IPM */
	ipm_handle = device_get_binding(CONFIG_OPENAMP_IPC_DEV_NAME);
	if (!ipm_handle) {
		LOG_DBG("Failed to find ipm device\n");
		return -1;
	}

	ipm_register_callback(ipm_handle, platform_ipm_callback, NULL);

	status = ipm_set_enabled(ipm_handle, 1);
	if (status) {
		LOG_DBG("ipm_set_enabled failed\n");
		return -1;
	}

	return 0;
}

static void cleanup_system(void)
{
	ipm_set_enabled(ipm_handle, 0);
	rpmsg_deinit_vdev(&rvdev);
	metal_finish();
}

struct  rpmsg_device *
platform_create_rpmsg_vdev(unsigned int vdev_index,
			   unsigned int role,
			   void (*rst_cb)(struct virtio_device *vdev),
			   rpmsg_ns_bind_cb ns_cb)
{
	struct fw_rsc_vdev_vring *vring_rsc;
	struct virtio_device *vdev;
	int ret;

	vdev = rproc_virtio_create_vdev(VIRTIO_DEV_SLAVE,VDEV_ID,
					rsc_table_to_vdev(rsc_table),
					rsc_io, NULL, mailbox_notify, NULL);

	if (!vdev) {
		LOG_DBG("failed to create vdev\r\n");
		return NULL;
	}

	/* wait master rpmsg init completion */
	rproc_virtio_wait_remote_ready(vdev);

	vring_rsc = rsc_table_get_vring0(rsc_table);
	ret = rproc_virtio_init_vring(vdev, 0, vring_rsc->notifyid,
				      (void *)vring_rsc->da, rsc_io,
				      vring_rsc->num, vring_rsc->align);
	if (ret) {
		LOG_DBG("failed to init vring 0\r\n");
		goto failed;
	}

	vring_rsc = rsc_table_get_vring1(rsc_table);
	ret = rproc_virtio_init_vring(vdev, 1, vring_rsc->notifyid,
				      (void *)vring_rsc->da, rsc_io,
				      vring_rsc->num, vring_rsc->align);
	if (ret) {
		LOG_DBG("failed to init vring 1\r\n");
		goto failed;
	}

	rpmsg_virtio_init_shm_pool(&shpool, NULL, SHM_SIZE);
	ret =  rpmsg_init_vdev(&rvdev, vdev, ns_cb, shm_io, &shpool);

	if (ret) {
		LOG_DBG("failed rpmsg_init_vdev\r\n");
		goto failed;
	}

	return rpmsg_virtio_get_rpmsg_device(&rvdev);

failed:
	rproc_virtio_remove_vdev(vdev);

	return NULL;
}


int my_spi_read(struct device *my_dev, struct spi_config *my_conf, uint8_t *buf2fill, uint8_t len_buff){

const struct spi_buf rx_buf={
	.buf=buf2fill,
	.len=len_buff
	};
const struct spi_buf_set spi_rx={
	.buffers=&rx_buf,
	.count=1
};

	if(spi_transceive(my_dev, my_conf, NULL, &spi_rx)){ //Read/write the specified amount of data from the SPI driver
		return 10;}
	else {return 0;}
}

void insert_buff(void *arg1, void *arg2, void *arg3){
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);
	int k; uint8_t djcb_rx[4];

	
	for(k=0;k<495;k+=2){
		if(my_spi_read(spi, read_conf, djcb_rx, sizeof(djcb_rx))){
			printk("Read error \n");
		}
		else {				
			buffer_msg[k]=djcb_rx[2];
			buffer_msg[k+1]=djcb_rx[3]&0x3F;
		}
	}
	flag_1=1;

	for(k=0;k<495;k+=2){
		if(my_spi_read(spi, read_conf, djcb_rx, sizeof(djcb_rx))){
			printk("Read error \n");
		}
		else {				
			buffer_msg2[k]=djcb_rx[2];
			buffer_msg2[k+1]=djcb_rx[3]&0x3F;
		}
	}
	flag_2=1;
		
	k_sem_give(&data_buff);
	return;	
}

void send_buff(void *arg1, void *arg2, void *arg3){
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);		
		k_sem_take(&data_buff, K_FOREVER);

		if(flag_1==1){
			rpmsg_send(&rcv_ept, buffer_msg, sizeof(buffer_msg));	
			flag_1=0;	
		}
		if(flag_2==1){
			rpmsg_send(&rcv_ept, buffer_msg2, sizeof(buffer_msg2));
			flag_2=0;	
		}

		receive_message(&msg,&len);		
	return;
}

void main(void){

	printk("Starting application !\n");	
	struct rpmsg_device *rpdev;
	int ret = 0; 

	spi=device_get_binding(DT_LABEL(SPI));
	if(!spi){ 	
		printk("Could not find SPI driver\n");
		return;
	}


	printk("\r\nOpenAMP[remote]  linux responder demo started\r\n");

	/* Initialize platform */
	ret = platform_init();
	if (ret) {
		printk("Failed to initialize platform\n");
		ret = -1;
		goto task_end;
	}

	rpdev = platform_create_rpmsg_vdev(0, VIRTIO_DEV_SLAVE, NULL,
					   new_service_cb);
	if (!rpdev) {
		printk("Failed to create rpmsg virtio device\n");
		ret = -1;
		goto task_end;
	}

	ret = rpmsg_create_ept(&rcv_ept, rpdev, RPMSG_CHAN_NAME,
			       RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
			      rpmsg_recv_callback, NULL);
	if (ret != 0)
		printk("error while creating endpoint(%d)\n", ret);
	printk("We are going to send information \n");
	
	start_time = k_uptime_get();
	
	
	receive_message(&msg,&len);	
	while(1){
		insert_buff(NULL,NULL,NULL);
		send_buff(NULL,NULL,NULL);
	}

	//k_timer_start(&my_timer, K_MSEC(100), K_MSEC(100));
        time_spent = k_uptime_delta(&start_time);
	printk("Temps  écoulé (en ms): %" PRIu64 "\n",time_spent);
	rpmsg_destroy_ept(&rcv_ept);

task_end:
	cleanup_system();
	printk("OpenAMP demo ended\n");
	
}
