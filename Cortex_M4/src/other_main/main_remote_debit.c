/*
 * Copyright (c) 2020, STMICROLECTRONICS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <string.h>
#include <stdlib.h>
#include <string.h>

#include <drivers/ipm.h>
#include <openamp/open_amp.h>
#include <metal/device.h>
#include <resource_table.h>
#include <sys_clock.h>
#include <logging/log.h>
LOG_MODULE_REGISTER(openamp_rsc_table, LOG_LEVEL_DBG);

#define RPMSG_CHAN_NAME	"rpmsg-sdb-channel"
#define SHM_DEVICE_NAME	"shm"
//#define MSG "Hello!"
#if !DT_HAS_CHOSEN(zephyr_ipc_shm)
#error "Sample requires definition of shared memory for rpmsg"
#endif

/* Constants derived from device tree */
#define SHM_NODE		DT_CHOSEN(zephyr_ipc_shm)
#define SHM_START_ADDR	DT_REG_ADDR(SHM_NODE)
#define SHM_SIZE		DT_REG_SIZE(SHM_NODE)


#define RX_NO_MSG        0
#define RX_NEW_MSG       1
#define RX_BUF_FREE      2
int msg_received_ch1 = RX_NO_MSG;
int msg_received_ch2 = RX_NO_MSG;


#define APP_TASK_STACK_SIZE (512)
K_THREAD_STACK_DEFINE(thread_stack, APP_TASK_STACK_SIZE);
static struct k_thread thread_data;

static const struct device *ipm_handle;
//char testStr[14] = {"B0L0010000000"};
#define MSG "B0L00100000"
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
static char  buffer_msg[496]; 
static unsigned int rcv_len;
static struct rpmsg_endpoint rcv_ept;

static K_SEM_DEFINE(data_sem, 0, 1);
static K_SEM_DEFINE(data_rx_sem, 0, 1);

uint64_t start_time;
uint64_t time_spent;

uint8_t SDB0ChannelBuffRx[100];
char mUartBuffTx[512];
volatile uint8_t mArrayDdrBuffCount = 0;

static void platform_ipm_callback(const struct device *device, void *context,
				  uint32_t id, volatile void *data)
{
	printk("%s: msg received from mb %d\n", __func__, id);
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
	LOG_ERR("%s: unexpected ns service receive for name %s\n",
		__func__, name);
}

int MAILBOX_Poll(struct virtio_device *vdev)
{
  /* If we got an interrupt, ask for the corresponding virtqueue processing */
printk("we are in Mailbox Poll\r\n");
  if (msg_received_ch1 == RX_BUF_FREE) {
   printk("Running virt0 (ch_1 buf free)\r\n");
    rproc_virtio_notified(vdev, VRING0_ID);
    msg_received_ch1 = RX_NO_MSG;
    return 0;
  }

  if (msg_received_ch2 == RX_NEW_MSG) {
   printk("Running virt1 (ch_2 new msg)\r\n");
    rproc_virtio_notified(vdev, VRING1_ID);
    msg_received_ch2 = RX_NO_MSG;

    /* The OpenAMP framework does not notify for free buf: do it here */
      rproc_virtio_notified(NULL, VRING1_ID);
    return 0;
  }

  return -1;
}
void OPENAMP_check_for_message(void)
{
  MAILBOX_Poll(rvdev.vdev);
}

int mailbox_notify(void *priv, uint32_t id)
{
	ARG_UNUSED(priv);

	printk("%s: msg received\n", __func__);
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

void app_task(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);
	struct rpmsg_device *rpdev;
	unsigned char *msg;
	int *env;
	int q=10;
	env=&q;
	unsigned int len;
       	unsigned int msg_cnt = 0,i=0;
	int ret = 0; int rsend;
	
	
	printk("\r\nOpenAMP[remote]  linux responder demo started\r\n");

	/* Initialize platform */
	ret = platform_init();
	if (ret) {
		LOG_ERR("Failed to initialize platform\n");
		ret = -1;
		goto task_end;
	}

	rpdev = platform_create_rpmsg_vdev(0, VIRTIO_DEV_SLAVE, NULL,
					   new_service_cb);
	if (!rpdev) {
		LOG_ERR("Failed to create rpmsg virtio device\n");
		ret = -1;
		goto task_end;
	}

	ret = rpmsg_create_ept(&rcv_ept, rpdev, RPMSG_CHAN_NAME,
			       RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
			      rpmsg_recv_callback, NULL);
	if (ret != 0)
		LOG_ERR("error while creating endpoint(%d)\n", ret);
	printk("We are going to send information \n");
	
	start_time = k_uptime_get();
	while (msg_cnt < 10000) {
		
		receive_message(&msg,&len);
		buffer_msg[494]='0'+msg_cnt;
		rpmsg_send(&rcv_ept, buffer_msg, sizeof(buffer_msg));
		//printk("message number %u sended (len:%d )\n",msg_cnt,sizeof(buffer_msg));
		/*if(rsend){
			printk("ERROR Failed to send the message\n");
			ret=-1;
			goto task_end;
		}*/
		
		msg_cnt++;
	}

        time_spent = k_uptime_delta(&start_time);
	printk("Temps  écoulé (en ms): %" PRIu64 "\n",time_spent);
	rpmsg_destroy_ept(&rcv_ept);

task_end:
	cleanup_system();

	printk("OpenAMP demo ended\n");
}

void main(void)
{for(int i=0; i <496; i++){
	buffer_msg[i]='a';
}

buffer_msg[495]='\n';
	printk("Starting application thread!\n");
	/*k_thread_create(&thread_data, thread_stack, APP_TASK_STACK_SIZE,
			(k_thread_entry_t)app_task,
			NULL, NULL, NULL, K_PRIO_COOP(7), 0, K_NO_WAIT);*/
	app_task(NULL, NULL, NULL);
}
