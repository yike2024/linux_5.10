// Date Created   : 02-06-2023
// Description    : This is simple CAN IP driver 
// Generator      : IIP Compiler Version 1.1

#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/iopoll.h>
#include <linux/can/dev.h>
#include <linux/pinctrl/consumer.h>

#include "cvitek_can.h"
#include "sdvt_can_basic.h"
#include "sdvt_can_defines.h"


static struct sdvt_can_command cmd_o = {0};// Pointer to command object
static struct sdvt_can_config cfg_o = {0};// Pointer to configuration object


static const struct can_bittiming_const sdvt_can_bittiming_const = {
    .name = KBUILD_MODNAME,
    .tseg1_min = 1,     /* Time segment 1 = prop_seg + phase_seg1 */                                                             
    .tseg1_max = 16,
    .tseg2_min = 1,     /* Time segment 2 = phase_seg2 */
    .tseg2_max = 8,
    .sjw_max = 3,
    .brp_min = 1,
    .brp_max = 128,
    .brp_inc = 2,
};

static const struct can_bittiming_const sdvt_can_data_bittiming_const = {
    .name = KBUILD_MODNAME,
    .tseg1_min = 1,     /* Time segment 1 = prop_seg + phase_seg1 */                                                             
    .tseg1_max = 16,
    .tseg2_min = 1,     /* Time segment 2 = phase_seg2 */
    .tseg2_max = 8,
    .sjw_max = 3,
    .brp_min = 1,
    .brp_max = 128,
    .brp_inc = 2,
};


static inline void sdvt_can_enable_all_interrupts(struct sdvt_can_classdev *cdev)
{
	sdvt_unmask_irq(cdev,&cfg_o, SDVT_CAN_IRQ_ENABLE1, 1 << SDVT_CAN_IRQ_RX_DATA_FRAME);
	sdvt_unmask_irq(cdev,&cfg_o, SDVT_CAN_IRQ_ENABLE0, 1 << SDVT_CAN_IRQ_REMOTE_FRAME);
}

static inline void sdvt_can_disable_all_interrupts(struct sdvt_can_classdev *cdev)
{
	sdvt_mask_irq(cdev,&cfg_o, SDVT_CAN_IRQ_ENABLE1, 1 << SDVT_CAN_IRQ_RX_DATA_FRAME);
	sdvt_mask_irq(cdev,&cfg_o, SDVT_CAN_IRQ_ENABLE0, 1 << SDVT_CAN_IRQ_REMOTE_FRAME);
}

static void sdvt_can_clean(struct net_device *net)
{
	struct sdvt_can_classdev *cdev = netdev_priv(net);

	if (cdev->tx_skb) {
		int putidx = 0;

		net->stats.tx_errors++;
		
		can_free_echo_skb(cdev->net, putidx);
		cdev->tx_skb = NULL;
	}
}

static void sdvt_can_read_fifo(struct net_device *dev)
{
	struct net_device_stats *stats = &dev->stats;
	struct sdvt_can_classdev *cdev = netdev_priv(dev);
	struct canfd_frame *cf;
	struct sk_buff *skb;
	int m_i_i   = 0;

	skb = alloc_can_skb(dev, (struct can_frame **)&cf);
	// cdev->ops->write_reg(cdev,SDVT_CAN_FIFO_FLUSH,0x2);
	
	if (!skb) {
		stats->rx_dropped++;
		return;
	}

	if(cmd_o.irq_status0_8b & (1 << SDVT_CAN_IRQ_REMOTE_FRAME)){
		// Read the length from RX LENGTH FIFO 
		receive_remote_frame(cdev,&cmd_o,&cfg_o);
		cf->can_id = cmd_o.ident_32b | CAN_RTR_FLAG;
		netdev_dbg(dev, "remote frame\n");
	}else{
		receive_frame(cdev,&cmd_o,&cfg_o);
		cf->len =  cmd_o.rx_len_2d_8b[1];
		cf->can_id =  cmd_o.ident_32b;
		for (m_i_i = 0; m_i_i < cf->len; m_i_i += 1) {
			((u_int8_t *)cf->data)[m_i_i] = cmd_o.rx_data_2d_8b[m_i_i];
		}
	}

	cmd_o.irq_status1_8b = 0x00;
	cmd_o.irq_status0_8b = 0x00;
	dev_info(cdev->dev,"\n");

	stats->rx_packets++;
	stats->rx_bytes += cf->len;
	netif_receive_skb(skb);
}

static int sdvt_can_do_rx_poll(struct net_device *dev, int quota)
{
	u32 pkts = 0;
	while (((cmd_o.irq_status1_8b & (1 << SDVT_CAN_IRQ_RX_DATA_FRAME))
		|| (cmd_o.irq_status0_8b & (1 << SDVT_CAN_IRQ_REMOTE_FRAME))) && quota > 0) {

		sdvt_can_read_fifo(dev);
		quota--;
		pkts++;
	}

	return pkts;
}


static int sdvt_can_clk_start(struct sdvt_can_classdev *cdev)
{
	int err;

	if (cdev->pm_clock_support == 0)
		return 0;

	err = pm_runtime_get_sync(cdev->dev);
	if (err < 0) {
		pm_runtime_put_noidle(cdev->dev);
		return err;
	}

	return 0;
}

static void sdvt_can_clk_stop(struct sdvt_can_classdev *cdev)
{
	if (cdev->pm_clock_support)
		pm_runtime_put_sync(cdev->dev);
}


static int sdvt_can_rx_handler(struct net_device *dev, int quota)
{
	int work_done = 0;

	if(!((cmd_o.irq_status1_8b & (1 << SDVT_CAN_IRQ_RX_DATA_FRAME)) 
		|| (cmd_o.irq_status0_8b & (1 << SDVT_CAN_IRQ_REMOTE_FRAME)))){
		pr_err("status isn't true \n");
		goto end;
	}

	work_done += sdvt_can_do_rx_poll(dev, (quota - work_done));

end:
	return work_done;
}


static int sdvt_can_poll(struct napi_struct *napi, int quota)
{
	struct net_device *dev = napi->dev;
	struct sdvt_can_classdev *cdev = netdev_priv(dev);
	int work_done;

	work_done = sdvt_can_rx_handler(dev, quota);

	if (work_done < quota) {
		napi_complete_done(napi, work_done);
		sdvt_can_enable_all_interrupts(cdev);
	}

	return work_done;
}

void sdvt_set_can_bittiming(struct net_device *dev)
{
	struct sdvt_can_classdev *cdev = netdev_priv(dev);
	const struct can_bittiming *bt = &cdev->can.bittiming;
	const struct can_bittiming *dbt = &cdev->can.data_bittiming;
	
	cfg_o.cfg_nor_clk_divider_8b = 0;
	cfg_o.cfg_nor_time_triple_sample_b = 0;

	cfg_o.cfg_nor_baud_prescaler_6b = bt->brp/2 - 1;
	cfg_o.cfg_nor_sync_jump_width_2b = bt->sjw - 1;
	cfg_o.cfg_nor_time_segment1_3b = bt->prop_seg +bt->phase_seg1 - 1;
	cfg_o.cfg_nor_time_segment2_4b = bt->phase_seg2 - 1;

	if(cdev->can.ctrlmode & CAN_CTRLMODE_FD){
		cfg_o.cfg_fd_clk_divider_8b = 0;
		cfg_o.cfg_fd_baud_prescaler_6b = dbt->brp/2 - 1;
		cfg_o.cfg_fd_sync_jump_width_2b = dbt->sjw - 1;
		cfg_o.cfg_fd_time_segment1_4b = dbt->prop_seg +bt->phase_seg1 - 1;
		cfg_o.cfg_fd_time_segment2_3b = dbt->phase_seg2 - 1;
		cfg_o.cfg_fd_time_triple_sample_b = 0;
		cfg_o.cfg_fd_brs_b = 1;
	}
	
	sdvt_init_chip(cdev,&cfg_o);
}

static irqreturn_t sdvt_can_irq_handler(int irq, void *dev_id)
{
	struct net_device *dev = (struct net_device *)dev_id;
	struct sdvt_can_classdev *cdev = netdev_priv(dev);
	// struct net_device_stats *stats = &dev->stats;	
	sdvt_can_disable_all_interrupts(cdev);
	if (pm_runtime_suspended(cdev->dev))
		return IRQ_NONE;

	detect_irq_status(cdev,&cmd_o);
	//1-------------------------------------------------------------------------------------------------
	// Check IRQ status 
	//1-------------------------------------------------------------------------------------------------
	if ((cmd_o.irq_status1_8b & (1 << SDVT_CAN_IRQ_RX_DATA_FRAME)) ||
		(cmd_o.irq_status0_8b & (1 << SDVT_CAN_IRQ_REMOTE_FRAME))) {
		/* RX data */
		// receive_frame(&cmd_o);
		napi_schedule(&cdev->napi);
	} else if (cmd_o.irq_status1_8b & (1 << SDVT_CAN_IRQ_TX_DONE)) {
		pr_info("send done.\n");
		/* TX done */
	}

	return IRQ_HANDLED;
}

/* Configure SDVT_CAN chip:
 * - set rx buffer/fifo element size
 * - configure rx fifo
 * - accept non-matching frame into fifo 0
 * - configure tx buffer
 *		- >= v3.1.x: TX FIFO is used
 * - configure mode
 * - setup bittiming
 */
static void sdvt_can_chip_config(struct net_device *dev)
{
	struct sdvt_can_classdev *cdev = netdev_priv(dev);
	// const struct can_bittiming *bt = &cdev->can.bittiming;
	// const struct can_bittiming *dbt = &cdev->can.data_bittiming;

	cfg_o.irq_enable1_8b       = 0xFF;
	cfg_o.std_arb_id0_8b       = 0xB6;
	cfg_o.std_arb_id1_8b       = 0x43;
	cfg_o.std_arb_id2_8b       = 0x55;
	cfg_o.std_arb_id3_8b       = 0x57;
	cfg_o.std_arb_id4_8b       = 0x50;

	cfg_o.cfg_acceptance_code0_8b        = 0xEA;
	cfg_o.cfg_acceptance_code1_8b        = 0xEA;
	cfg_o.cfg_acceptance_code2_8b        = 0xEA;
	cfg_o.cfg_acceptance_code3_8b        = 0xEA;
	cfg_o.cfg_acceptance_mask0_8b        = 0xFF;
	cfg_o.cfg_acceptance_mask1_8b        = 0xFF;
	cfg_o.cfg_acceptance_mask2_8b        = 0xFF;
	cfg_o.cfg_acceptance_mask3_8b        = 0xFF;

	if(cdev->can.ctrlmode & CAN_CTRLMODE_FD){
		cfg_o.cfg_remote_resp_en_b           = 0;
		cfg_o.cfg_ext_ctrl_remote_en_b       = 0;
		cfg_o.cfg_ext_frame_mode_b           = SDVT_CAN_STANDARD_FRAME;
		cfg_o.cfg_can_mode_2b                = SDVT_CAN_FD_MODE;

	}else{
		cfg_o.cfg_remote_resp_en_b           = 0;
		cfg_o.cfg_ext_ctrl_remote_en_b       = 1;
		cfg_o.cfg_ext_frame_mode_b           = SDVT_CAN_EXTENDED_FRAME;
		cfg_o.cfg_can_mode_2b                = SDVT_CAN_CLASSIC_MODE;
	}

	if (cfg_o.cfg_ext_frame_mode_b == SDVT_CAN_STANDARD_FRAME) {
		;
	} else {
		cfg_o.cfg_ext_ctrl_listen_b = 0;
		cfg_o.cfg_ext_ctrl_self_test_b = 0;
		cfg_o.cfg_ext_ctrl_acc_filter_b = 0;
		cfg_o.cfg_ext_ctrl_rec_incr_b = 0;
		cfg_o.cfg_ext_ctrl_id_rx_b = 1;
	}
	
}

static void sdvt_can_start(struct net_device *dev)
{
	struct sdvt_can_classdev *cdev = netdev_priv(dev);

	/* basic sdvt_can configuration */
	sdvt_can_chip_config(dev);
	sdvt_set_can_bittiming(dev);
	cdev->can.state = CAN_STATE_ERROR_ACTIVE;

	sdvt_can_enable_all_interrupts(cdev);
}

int sdvt_can_set_mode(struct net_device *dev, enum can_mode mode)
{
	switch (mode) {
	case CAN_MODE_START:
		sdvt_can_clean(dev);
		sdvt_can_start(dev);
		netif_wake_queue(dev);
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static int sdvt_can_dev_setup(struct sdvt_can_classdev *sdvt_can_dev)
{
	struct net_device *dev = sdvt_can_dev->net;

	if (!sdvt_can_dev->is_peripheral)
		netif_napi_add(dev, &sdvt_can_dev->napi,
			       sdvt_can_poll, 64);

	sdvt_can_dev->can.do_set_mode = sdvt_can_set_mode;
	sdvt_can_dev->can.ctrlmode_supported = CAN_CTRLMODE_LOOPBACK |
                    CAN_CTRLMODE_LISTENONLY |
                    CAN_CTRLMODE_BERR_REPORTING |
                    CAN_CTRLMODE_FD |
                    CAN_CTRLMODE_ONE_SHOT;

	sdvt_can_dev->can.bittiming_const = &sdvt_can_bittiming_const;

	sdvt_can_dev->can.data_bittiming_const = &sdvt_can_data_bittiming_const;


	return 0;
}

static void sdvt_can_stop(struct net_device *dev)
{
	struct sdvt_can_classdev *cdev = netdev_priv(dev);

	/* disable all interrupts */
	sdvt_can_disable_all_interrupts(cdev);

	/* set the state as STOPPED */
	cdev->can.state = CAN_STATE_STOPPED;
}

static int sdvt_can_close(struct net_device *dev)
{
	struct sdvt_can_classdev *cdev = netdev_priv(dev);

	netif_stop_queue(dev);
	napi_disable(&cdev->napi);

	sdvt_can_stop(dev);
	sdvt_can_clk_stop(cdev);
	free_irq(dev->irq, dev);

	if (cdev->is_peripheral) {
		cdev->tx_skb = NULL;
		destroy_workqueue(cdev->tx_wq);
		cdev->tx_wq = NULL;
	}

	close_candev(dev);
	can_led_event(dev, CAN_LED_EVENT_STOP);

	return 0;
}

static netdev_tx_t stvd_can_tx_handler(struct sdvt_can_classdev *cdev)
{
	struct canfd_frame *cf = (struct canfd_frame *)cdev->tx_skb->data;
	// struct net_device *dev = cdev->net;
	// struct sk_buff *skb = cdev->tx_skb;

	int m_i_i   = 0;

	// cfg_o.cfg_ext_frame_mode_b = 0;
  	// cmd_o.remote_resp_en_b  = 0;
	cmd_o.data_len_code_4b  = cf->len;
	cmd_o.dlc_4b = cmd_o.data_len_code_4b;

	pr_info("send data:");
	for (m_i_i = 0; m_i_i < cmd_o.data_len_code_4b; m_i_i += 1) {
		cmd_o.data_2d_8b[m_i_i] = ((u_int8_t *)cf->data)[m_i_i];
		pr_info(" get xmit data %#x", cmd_o.data_2d_8b[m_i_i]);
	}
	pr_info("\n");

	send_command(cdev,&cmd_o,&cfg_o);

	return NETDEV_TX_OK;
}

static netdev_tx_t sdvt_can_start_xmit(struct sk_buff *skb,
				    struct net_device *dev)
{
	struct can_frame *frame = (struct can_frame *)skb->data;
	struct sdvt_can_classdev *cdev = netdev_priv(dev);

	if (can_dropped_invalid_skb(dev, skb))
		return NETDEV_TX_OK;
	if (frame->can_id & CAN_EFF_FLAG) {
		pr_err("extern frame:%x \n",frame->can_id);
		cmd_o.ident_32b  = frame->can_id & CAN_EFF_MASK;
		if (frame->can_id & CAN_RTR_FLAG){
			cmd_o.command_8b = EXTENDED_REMOTE_FRAME;
		}else{
			cmd_o.command_8b = EXTENDED_DATA_FRAME;
		}

	} else {
		pr_err("stand frame:%x \n",frame->can_id);
		cmd_o.ident_32b  = frame->can_id & CAN_SFF_MASK;
		if (frame->can_id & CAN_RTR_FLAG){
			cmd_o.command_8b = STANDARD_REMOTE_FRAME;
		}else{
			cmd_o.command_8b = STANDARD_DATA_FRAME;
		}
	}

	cdev->tx_skb = skb;
	return stvd_can_tx_handler(cdev);
}

static int sdvt_can_open(struct net_device *dev)
{
	struct sdvt_can_classdev *cdev = netdev_priv(dev);
	int err;

	err = sdvt_can_clk_start(cdev);
	if (err)
		return err;

	/* open the can device */
	err = open_candev(dev);
	if (err) {
		netdev_err(dev, "failed to open can device\n");
		goto exit_disable_clks;
	}

	err = request_irq(dev->irq, sdvt_can_irq_handler, IRQF_SHARED, dev->name,
			  dev);

	if (err < 0) {
		netdev_err(dev, "failed to request interrupt\n");
		goto exit_irq_fail;
	}

	/* start the sdvt_can controller */
	sdvt_can_start(dev);

	if (!cdev->is_peripheral)
		napi_enable(&cdev->napi);

	netif_start_queue(dev);

	return 0;

exit_irq_fail:
	if (cdev->is_peripheral)
		destroy_workqueue(cdev->tx_wq);
exit_disable_clks:
	sdvt_can_clk_stop(cdev);
	return err;
}

static const struct net_device_ops sdvt_can_netdev_ops = {
	.ndo_open = sdvt_can_open,
	.ndo_stop = sdvt_can_close,
	.ndo_start_xmit = sdvt_can_start_xmit,
	.ndo_change_mtu = can_change_mtu,
};

static int register_sdvt_can_dev(struct net_device *dev)
{
	dev->flags |= IFF_ECHO;	/* we support local echo */
	dev->netdev_ops = &sdvt_can_netdev_ops;

	return register_candev(dev);
}


int sdvt_can_class_get_clocks(struct sdvt_can_classdev *sdvt_can_dev)
{
	int ret = 0;

	sdvt_can_dev->cclk = devm_clk_get(sdvt_can_dev->dev, "clk_can");

	return ret;
}
EXPORT_SYMBOL_GPL(sdvt_can_class_get_clocks);

struct sdvt_can_classdev *sdvt_can_class_allocate_dev(struct device *dev)
{
	struct sdvt_can_classdev *class_dev = NULL;
	struct net_device *net_dev;

	/* allocate the sdvt_can device */
	net_dev = alloc_candev(sizeof(*class_dev), 16);
	if (!net_dev) {
		dev_err(dev, "Failed to allocate CAN device");
		goto out;
	}

	class_dev = netdev_priv(net_dev);
	if (!class_dev) {
		dev_err(dev, "Failed to init netdev cdevate");
		goto out;
	}

	class_dev->net = net_dev;
	class_dev->dev = dev;
	SET_NETDEV_DEV(net_dev, dev);

out:
	return class_dev;
}
EXPORT_SYMBOL_GPL(sdvt_can_class_allocate_dev);

void sdvt_can_class_free_dev(struct net_device *net)
{
	free_candev(net);
}
EXPORT_SYMBOL_GPL(sdvt_can_class_free_dev);

int sdvt_can_class_register(struct sdvt_can_classdev *sdvt_can_dev)
{
	int ret;

	if (sdvt_can_dev->pm_clock_support) {
		pm_runtime_enable(sdvt_can_dev->dev);
		ret = sdvt_can_clk_start(sdvt_can_dev);
		if (ret)
			goto pm_runtime_fail;
	}

	ret = sdvt_can_dev_setup(sdvt_can_dev);
	if (ret)
		goto clk_disable;

	ret = register_sdvt_can_dev(sdvt_can_dev->net);
	if (ret) {
		dev_err(sdvt_can_dev->dev, "registering %s failed (err=%d)\n",
			sdvt_can_dev->net->name, ret);
		goto clk_disable;
	}

	devm_can_led_init(sdvt_can_dev->net);
	of_can_transceiver(sdvt_can_dev->net);

	dev_info(sdvt_can_dev->dev, "%s device registered (irq=%d, version=%d)\n",
		 KBUILD_MODNAME, sdvt_can_dev->net->irq, sdvt_can_dev->version);

	/* Probe finished
	 * Stop clocks. They will be reactivated once the SDVT_CAN device is opened
	 */
clk_disable:
	sdvt_can_clk_stop(sdvt_can_dev);
pm_runtime_fail:
	if (ret) {
		if (sdvt_can_dev->pm_clock_support)
			pm_runtime_disable(sdvt_can_dev->dev);
	}

	return ret;
}
EXPORT_SYMBOL_GPL(sdvt_can_class_register);

int sdvt_can_class_suspend(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct sdvt_can_classdev *cdev = netdev_priv(ndev);

	if (netif_running(ndev)) {
		netif_stop_queue(ndev);
		netif_device_detach(ndev);
		sdvt_can_stop(ndev);
		sdvt_can_clk_stop(cdev);
	}

	pinctrl_pm_select_sleep_state(dev);
	cdev->can.state = CAN_STATE_SLEEPING;

	return 0;
}
EXPORT_SYMBOL_GPL(sdvt_can_class_suspend);

int sdvt_can_class_resume(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct sdvt_can_classdev *cdev = netdev_priv(ndev);

	pinctrl_pm_select_default_state(dev);
	cdev->can.state = CAN_STATE_ERROR_ACTIVE;

	if (netif_running(ndev)) {
		int ret;

		ret = sdvt_can_clk_start(cdev);
		if (ret)
			return ret;

		sdvt_can_start(ndev);
		netif_device_attach(ndev);
		netif_start_queue(ndev);
	}

	return 0;
}
// EXPORT_SYMBOL_GPL(sdvt_can_class_resume);

void sdvt_can_class_unregister(struct sdvt_can_classdev *sdvt_can_dev)
{
	unregister_candev(sdvt_can_dev->net);

	sdvt_can_clk_stop(sdvt_can_dev);
}

// EXPORT_SYMBOL_GPL(sdvt_can_class_unregister);

MODULE_AUTHOR("martin.xuan <martin.xuan@sophgo.com>");
MODULE_AUTHOR("martin.xuan <https://www.sophgo.com/>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("CAN bus driver for SDVT_CAN controller");
