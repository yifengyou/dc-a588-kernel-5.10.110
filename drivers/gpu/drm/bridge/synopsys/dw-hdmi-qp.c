// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) Rockchip Electronics Co.Ltd
 * Author:
 *      Algea Cao <algea.cao@rock-chips.com>
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/extcon-provider.h>
#include <linux/extcon.h>
#include <linux/hdmi.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regmap.h>
#include <linux/spinlock.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_dsc.h>
#include <drm/drm_edid.h>
#include <drm/drm_encoder_slave.h>
#include <drm/drm_of.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_scdc_helper.h>
#include <drm/bridge/dw_hdmi.h>

#include <uapi/linux/media-bus-format.h>
#include <uapi/linux/videodev2.h>

#include "dw-hdmi-qp-audio.h"
#include "dw-hdmi-qp.h"
#include "dw-hdmi-qp-cec.h"

#include <media/cec-notifier.h>

#define DDC_CI_ADDR		0x37
#define DDC_SEGMENT_ADDR	0x30

#define HDMI_EDID_LEN		512

/* DW-HDMI Controller >= 0x200a are at least compliant with SCDC version 1 */
#define SCDC_MIN_SOURCE_VERSION	0x1

#define HDMI14_MAX_TMDSCLK	340000000
#define HDMI20_MAX_TMDSCLK_KHZ	600000

static const unsigned int dw_hdmi_cable[] = {
	EXTCON_DISP_HDMI,
	EXTCON_NONE,
};

/*
 * Unless otherwise noted, entries in this table are 100% optimization.
 * Values can be obtained from hdmi_compute_n() but that function is
 * slow so we pre-compute values we expect to see.
 *
 * All 32k and 48k values are expected to be the same (due to the way
 * the math works) for any rate that's an exact kHz.
 */
static const struct dw_hdmi_audio_tmds_n common_tmds_n_table[] = {
	{ .tmds = 25175000, .n_32k = 4096, .n_44k1 = 12854, .n_48k = 6144, },
	{ .tmds = 25200000, .n_32k = 4096, .n_44k1 = 5656, .n_48k = 6144, },
	{ .tmds = 27000000, .n_32k = 4096, .n_44k1 = 5488, .n_48k = 6144, },
	{ .tmds = 28320000, .n_32k = 4096, .n_44k1 = 5586, .n_48k = 6144, },
	{ .tmds = 30240000, .n_32k = 4096, .n_44k1 = 5642, .n_48k = 6144, },
	{ .tmds = 31500000, .n_32k = 4096, .n_44k1 = 5600, .n_48k = 6144, },
	{ .tmds = 32000000, .n_32k = 4096, .n_44k1 = 5733, .n_48k = 6144, },
	{ .tmds = 33750000, .n_32k = 4096, .n_44k1 = 6272, .n_48k = 6144, },
	{ .tmds = 36000000, .n_32k = 4096, .n_44k1 = 5684, .n_48k = 6144, },
	{ .tmds = 40000000, .n_32k = 4096, .n_44k1 = 5733, .n_48k = 6144, },
	{ .tmds = 49500000, .n_32k = 4096, .n_44k1 = 5488, .n_48k = 6144, },
	{ .tmds = 50000000, .n_32k = 4096, .n_44k1 = 5292, .n_48k = 6144, },
	{ .tmds = 54000000, .n_32k = 4096, .n_44k1 = 5684, .n_48k = 6144, },
	{ .tmds = 65000000, .n_32k = 4096, .n_44k1 = 7056, .n_48k = 6144, },
	{ .tmds = 68250000, .n_32k = 4096, .n_44k1 = 5376, .n_48k = 6144, },
	{ .tmds = 71000000, .n_32k = 4096, .n_44k1 = 7056, .n_48k = 6144, },
	{ .tmds = 72000000, .n_32k = 4096, .n_44k1 = 5635, .n_48k = 6144, },
	{ .tmds = 73250000, .n_32k = 4096, .n_44k1 = 14112, .n_48k = 6144, },
	{ .tmds = 74250000, .n_32k = 4096, .n_44k1 = 6272, .n_48k = 6144, },
	{ .tmds = 75000000, .n_32k = 4096, .n_44k1 = 5880, .n_48k = 6144, },
	{ .tmds = 78750000, .n_32k = 4096, .n_44k1 = 5600, .n_48k = 6144, },
	{ .tmds = 78800000, .n_32k = 4096, .n_44k1 = 5292, .n_48k = 6144, },
	{ .tmds = 79500000, .n_32k = 4096, .n_44k1 = 4704, .n_48k = 6144, },
	{ .tmds = 83500000, .n_32k = 4096, .n_44k1 = 7056, .n_48k = 6144, },
	{ .tmds = 85500000, .n_32k = 4096, .n_44k1 = 5488, .n_48k = 6144, },
	{ .tmds = 88750000, .n_32k = 4096, .n_44k1 = 14112, .n_48k = 6144, },
	{ .tmds = 97750000, .n_32k = 4096, .n_44k1 = 14112, .n_48k = 6144, },
	{ .tmds = 101000000, .n_32k = 4096, .n_44k1 = 7056, .n_48k = 6144, },
	{ .tmds = 106500000, .n_32k = 4096, .n_44k1 = 4704, .n_48k = 6144, },
	{ .tmds = 108000000, .n_32k = 4096, .n_44k1 = 5684, .n_48k = 6144, },
	{ .tmds = 115500000, .n_32k = 4096, .n_44k1 = 5712, .n_48k = 6144, },
	{ .tmds = 119000000, .n_32k = 4096, .n_44k1 = 5544, .n_48k = 6144, },
	{ .tmds = 135000000, .n_32k = 4096, .n_44k1 = 5488, .n_48k = 6144, },
	{ .tmds = 146250000, .n_32k = 4096, .n_44k1 = 6272, .n_48k = 6144, },
	{ .tmds = 148500000, .n_32k = 4096, .n_44k1 = 5488, .n_48k = 6144, },
	{ .tmds = 154000000, .n_32k = 4096, .n_44k1 = 5544, .n_48k = 6144, },
	{ .tmds = 162000000, .n_32k = 4096, .n_44k1 = 5684, .n_48k = 6144, },

	/* For 297 MHz+ HDMI spec have some other rule for setting N */
	{ .tmds = 297000000, .n_32k = 3073, .n_44k1 = 4704, .n_48k = 5120, },
	{ .tmds = 594000000, .n_32k = 3073, .n_44k1 = 9408, .n_48k = 10240, },

	/* End of table */
	{ .tmds = 0,         .n_32k = 0,    .n_44k1 = 0,    .n_48k = 0, },
};

static const struct drm_display_mode dw_hdmi_default_modes[] = {
	/* 110 - 3440x1440@60Hz 16:9 */
/*
	{ DRM_MODE("3440x1440", DRM_MODE_TYPE_DRIVER, 297000, 3440, 3488,
		   3520, 3600, 0, 1440, 1443, 1453, 1481, 0,
		   DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_NVSYNC),
	  .picture_aspect_ratio = HDMI_PICTURE_ASPECT_16_9, },
*/
	/* 16 - 1920x1080@60Hz 16:9 */
	{ DRM_MODE("1920x1080", DRM_MODE_TYPE_DRIVER, 148500, 1920, 2008,
		   2052, 2200, 0, 1080, 1084, 1089, 1125, 0,
		   DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
	  .picture_aspect_ratio = HDMI_PICTURE_ASPECT_16_9, },
	/* 2 - 720x480@60Hz 4:3 */
	{ DRM_MODE("720x480", DRM_MODE_TYPE_DRIVER, 27000, 720, 736,
		   798, 858, 0, 480, 489, 495, 525, 0,
		   DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC),
	  .picture_aspect_ratio = HDMI_PICTURE_ASPECT_4_3, },
	/* 4 - 1280x720@60Hz 16:9 */
	{ DRM_MODE("1280x720", DRM_MODE_TYPE_DRIVER, 74250, 1280, 1390,
		   1430, 1650, 0, 720, 725, 730, 750, 0,
		   DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
	  .picture_aspect_ratio = HDMI_PICTURE_ASPECT_16_9, },
	/* 31 - 1920x1080@50Hz 16:9 */
	{ DRM_MODE("1920x1080", DRM_MODE_TYPE_DRIVER, 148500, 1920, 2448,
		   2492, 2640, 0, 1080, 1084, 1089, 1125, 0,
		   DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
	  .picture_aspect_ratio = HDMI_PICTURE_ASPECT_16_9, },
	/* 19 - 1280x720@50Hz 16:9 */
	{ DRM_MODE("1280x720", DRM_MODE_TYPE_DRIVER, 74250, 1280, 1720,
		   1760, 1980, 0, 720, 725, 730, 750, 0,
		   DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
	  .picture_aspect_ratio = HDMI_PICTURE_ASPECT_16_9, },
	/* 17 - 720x576@50Hz 4:3 */
	{ DRM_MODE("720x576", DRM_MODE_TYPE_DRIVER, 27000, 720, 732,
		   796, 864, 0, 576, 581, 586, 625, 0,
		   DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC),
	  .picture_aspect_ratio = HDMI_PICTURE_ASPECT_4_3, },
	/* 2 - 720x480@60Hz 4:3 */
	{ DRM_MODE("720x480", DRM_MODE_TYPE_DRIVER, 27000, 720, 736,
		   798, 858, 0, 480, 489, 495, 525, 0,
		   DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC),
	  .picture_aspect_ratio = HDMI_PICTURE_ASPECT_4_3, },
	/* 128 - 3840x400@30Hz 16:9 */
	{ DRM_MODE("3840x400", DRM_MODE_TYPE_DRIVER, 74250, 3840, 3888,
		   3920, 4400, 0, 400, 403, 409, 490, 0,
		   DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
	  .picture_aspect_ratio = HDMI_PICTURE_ASPECT_16_9, },
};

enum frl_mask {
	FRL_3GBPS_3LANE = 1,
	FRL_6GBPS_3LANE,
	FRL_6GBPS_4LANE,
	FRL_8GBPS_4LANE,
	FRL_10GBPS_4LANE,
	FRL_12GBPS_4LANE,
};

struct hdmi_vmode_qp {
	bool mdataenablepolarity;

	unsigned int previous_pixelclock;
	unsigned long mpixelclock;
	unsigned int mpixelrepetitioninput;
	unsigned int mpixelrepetitionoutput;
	unsigned long previous_tmdsclock;
	unsigned int mtmdsclock;
};

struct hdmi_qp_data_info {
	unsigned int enc_in_bus_format;
	unsigned int enc_out_bus_format;
	unsigned int enc_in_encoding;
	unsigned int enc_out_encoding;
	unsigned int quant_range;
	unsigned int pix_repet_factor;
	struct hdmi_vmode_qp video_mode;
	bool update;
};

struct dw_hdmi_qp_i2c {
	struct i2c_adapter	adap;

	struct mutex		lock;	/* used to serialize data transfers */
	struct completion	cmp;
	u32			stat;

	u8			slave_reg;
	bool			is_regaddr;
	bool			is_segment;

	unsigned int		scl_high_ns;
	unsigned int		scl_low_ns;
};

struct dw_hdmi_phy_data {
	enum dw_hdmi_phy_type type;
	const char *name;
	unsigned int gen;
	bool has_svsret;
	int (*configure)(struct dw_hdmi_qp *hdmi,
			 const struct dw_hdmi_plat_data *pdata,
			 unsigned long mpixelclock);
};

struct dw_hdmi_qp {
	struct drm_connector connector;
	struct drm_bridge bridge;
	struct platform_device *hdcp_dev;
	struct platform_device *audio;
	struct platform_device *cec;
	struct device *dev;
	struct dw_hdmi_qp_i2c *i2c;

	struct hdmi_qp_data_info hdmi_data;
	const struct dw_hdmi_plat_data *plat_data;

	int vic;
	int main_irq;
	int avp_irq;
	int earc_irq;

	u8 edid[HDMI_EDID_LEN];

	struct {
		const struct dw_hdmi_qp_phy_ops *ops;
		const char *name;
		void *data;
		bool enabled;
	} phy;

	struct drm_display_mode previous_mode;

	struct i2c_adapter *ddc;
	void __iomem *regs;
	bool sink_is_hdmi;
	bool sink_has_audio;
	bool dclk_en;
	bool frl_switch;
	bool cec_enable;

	struct mutex mutex;		/* for state below and previous_mode */
	struct drm_connector *curr_conn;/* current connector (only valid when !disabled) */
	enum drm_connector_force force;	/* mutex-protected force state */
	bool disabled;			/* DRM has disabled our bridge */
	bool bridge_is_on;		/* indicates the bridge is on */
	bool rxsense;			/* rxsense state */
	u8 phy_mask;			/* desired phy int mask settings */
	u8 mc_clkdis;			/* clock disable register */

	u32 scdc_intr;
	u32 flt_intr;
	u32 earc_intr;

	struct mutex audio_mutex;
	unsigned int sample_rate;
	unsigned int audio_cts;
	unsigned int audio_n;
	bool audio_enable;
	void (*enable_audio)(struct dw_hdmi_qp *hdmi);
	void (*disable_audio)(struct dw_hdmi_qp *hdmi);

	struct dentry *debugfs_dir;
	bool scramble_low_rates;

	struct extcon_dev *extcon;

	struct regmap *regm;

	bool initialized;		/* hdmi is enabled before bind */
	struct completion flt_cmp;
	struct completion earc_cmp;

	struct cec_notifier *cec_notifier;
	struct cec_adapter *cec_adap;
	struct mutex cec_notifier_mutex;

	hdmi_codec_plugged_cb plugged_cb;
	struct device *codec_dev;
	enum drm_connector_status last_connector_result;
};

static inline void hdmi_writel(struct dw_hdmi_qp *hdmi, u32 val, int offset)
{
	regmap_write(hdmi->regm, offset, val);
}

static inline u32 hdmi_readl(struct dw_hdmi_qp *hdmi, int offset)
{
	unsigned int val = 0;

	regmap_read(hdmi->regm, offset, &val);

	return val;
}

static void handle_plugged_change(struct dw_hdmi_qp *hdmi, bool plugged)
{
	if (hdmi->plugged_cb && hdmi->codec_dev)
		hdmi->plugged_cb(hdmi->codec_dev, plugged);
}

int dw_hdmi_qp_set_plugged_cb(struct dw_hdmi_qp *hdmi, hdmi_codec_plugged_cb fn,
			      struct device *codec_dev)
{
	bool plugged;

	mutex_lock(&hdmi->mutex);
	hdmi->plugged_cb = fn;
	hdmi->codec_dev = codec_dev;
	plugged = hdmi->last_connector_result == connector_status_connected;
	handle_plugged_change(hdmi, plugged);
	mutex_unlock(&hdmi->mutex);

	return 0;
}
EXPORT_SYMBOL_GPL(dw_hdmi_qp_set_plugged_cb);

static void hdmi_modb(struct dw_hdmi_qp *hdmi, u32 data, u32 mask, u32 reg)
{
	regmap_update_bits(hdmi->regm, reg, mask, data);
}

static void hdmi_set_cts_n(struct dw_hdmi_qp *hdmi, unsigned int cts,
			   unsigned int n)
{
	/* Set N */
	hdmi_modb(hdmi, n, AUDPKT_ACR_N_VALUE, AUDPKT_ACR_CONTROL0);

	/* Set CTS */
	if (cts)
		hdmi_modb(hdmi, AUDPKT_ACR_CTS_OVR_EN, AUDPKT_ACR_CTS_OVR_EN_MSK,
			  AUDPKT_ACR_CONTROL1);
	else
		hdmi_modb(hdmi, 0, AUDPKT_ACR_CTS_OVR_EN_MSK,
			  AUDPKT_ACR_CONTROL1);

	hdmi_modb(hdmi, AUDPKT_ACR_CTS_OVR_VAL(cts), AUDPKT_ACR_CTS_OVR_VAL_MSK,
		  AUDPKT_ACR_CONTROL1);
}

static int hdmi_match_tmds_n_table(struct dw_hdmi_qp *hdmi,
				   unsigned long pixel_clk,
				   unsigned long freq)
{
	const struct dw_hdmi_plat_data *plat_data = hdmi->plat_data;
	const struct dw_hdmi_audio_tmds_n *tmds_n = NULL;
	int i;

	if (plat_data->tmds_n_table) {
		for (i = 0; plat_data->tmds_n_table[i].tmds != 0; i++) {
			if (pixel_clk == plat_data->tmds_n_table[i].tmds) {
				tmds_n = &plat_data->tmds_n_table[i];
				break;
			}
		}
	}

	if (tmds_n == NULL) {
		for (i = 0; common_tmds_n_table[i].tmds != 0; i++) {
			if (pixel_clk == common_tmds_n_table[i].tmds) {
				tmds_n = &common_tmds_n_table[i];
				break;
			}
		}
	}

	if (tmds_n == NULL)
		return -ENOENT;

	switch (freq) {
	case 32000:
		return tmds_n->n_32k;
	case 44100:
	case 88200:
	case 176400:
		return (freq / 44100) * tmds_n->n_44k1;
	case 48000:
	case 96000:
	case 192000:
		return (freq / 48000) * tmds_n->n_48k;
	default:
		return -ENOENT;
	}
}

static u64 hdmi_audio_math_diff(unsigned int freq, unsigned int n,
				unsigned int pixel_clk)
{
	u64 final, diff;
	u64 cts;

	final = (u64)pixel_clk * n;

	cts = final;
	do_div(cts, 128 * freq);

	diff = final - (u64)cts * (128 * freq);

	return diff;
}

static unsigned int hdmi_compute_n(struct dw_hdmi_qp *hdmi,
				   unsigned long pixel_clk,
				   unsigned long freq)
{
	unsigned int min_n = DIV_ROUND_UP((128 * freq), 1500);
	unsigned int max_n = (128 * freq) / 300;
	unsigned int ideal_n = (128 * freq) / 1000;
	unsigned int best_n_distance = ideal_n;
	unsigned int best_n = 0;
	u64 best_diff = U64_MAX;
	int n;

	/* If the ideal N could satisfy the audio math, then just take it */
	if (hdmi_audio_math_diff(freq, ideal_n, pixel_clk) == 0)
		return ideal_n;

	for (n = min_n; n <= max_n; n++) {
		u64 diff = hdmi_audio_math_diff(freq, n, pixel_clk);

		if (diff < best_diff || (diff == best_diff &&
		    abs(n - ideal_n) < best_n_distance)) {
			best_n = n;
			best_diff = diff;
			best_n_distance = abs(best_n - ideal_n);
		}

		/*
		 * The best N already satisfy the audio math, and also be
		 * the closest value to ideal N, so just cut the loop.
		 */
		if ((best_diff == 0) && (abs(n - ideal_n) > best_n_distance))
			break;
	}

	return best_n;
}

static unsigned int hdmi_find_n(struct dw_hdmi_qp *hdmi, unsigned long pixel_clk,
				unsigned long sample_rate)
{
	int n;

	n = hdmi_match_tmds_n_table(hdmi, pixel_clk, sample_rate);
	if (n > 0)
		return n;

	dev_warn(hdmi->dev, "Rate %lu missing; compute N dynamically\n",
		 pixel_clk);

	return hdmi_compute_n(hdmi, pixel_clk, sample_rate);
}

/*
 * When transmitting IEC60958 linear PCM audio, these registers allow to
 * configure the channel status information of all the channel status
 * bits in the IEC60958 frame. For the moment this configuration is only
 * used when the I2S audio interface, General Purpose Audio (GPA),
 * or AHB audio DMA (AHBAUDDMA) interface is active
 * (for S/PDIF interface this information comes from the stream).
 */
void dw_hdmi_qp_set_channel_status(struct dw_hdmi_qp *hdmi,
				   u8 *channel_status, bool ref2stream)
{
	mutex_lock(&hdmi->audio_mutex);
	if (!hdmi->dclk_en) {
		mutex_unlock(&hdmi->audio_mutex);
		return;
	}

	/* Set channel status */
	hdmi_writel(hdmi, channel_status[3] | (channel_status[4] << 8),
		    AUDPKT_CHSTATUS_OVR1);

	if (ref2stream)
		hdmi_modb(hdmi, 0,
			  AUDPKT_PBIT_FORCE_EN_MASK | AUDPKT_CHSTATUS_OVR_EN_MASK,
			  AUDPKT_CONTROL0);
	else
		hdmi_modb(hdmi, AUDPKT_PBIT_FORCE_EN | AUDPKT_CHSTATUS_OVR_EN,
			  AUDPKT_PBIT_FORCE_EN_MASK | AUDPKT_CHSTATUS_OVR_EN_MASK,
			  AUDPKT_CONTROL0);

	mutex_unlock(&hdmi->audio_mutex);
}
EXPORT_SYMBOL_GPL(dw_hdmi_qp_set_channel_status);

static void hdmi_set_clk_regenerator(struct dw_hdmi_qp *hdmi,
	unsigned long pixel_clk, unsigned int sample_rate)
{
	unsigned int n = 0, cts = 0;

	n = hdmi_find_n(hdmi, pixel_clk, sample_rate);

	hdmi->audio_n = n;
	hdmi->audio_cts = cts;
	hdmi_set_cts_n(hdmi, cts, hdmi->audio_enable ? n : 0);
}

static void hdmi_init_clk_regenerator(struct dw_hdmi_qp *hdmi)
{
	mutex_lock(&hdmi->audio_mutex);
	if (hdmi->dclk_en)
		hdmi_set_clk_regenerator(hdmi, 74250000, hdmi->sample_rate);
	mutex_unlock(&hdmi->audio_mutex);
}

static void hdmi_clk_regenerator_update_pixel_clock(struct dw_hdmi_qp *hdmi)
{
	mutex_lock(&hdmi->audio_mutex);
	if (hdmi->dclk_en)
		hdmi_set_clk_regenerator(hdmi, hdmi->hdmi_data.video_mode.mtmdsclock,
					 hdmi->sample_rate);
	mutex_unlock(&hdmi->audio_mutex);
}

void dw_hdmi_qp_set_sample_rate(struct dw_hdmi_qp *hdmi, unsigned int rate)
{
	mutex_lock(&hdmi->audio_mutex);
	if (hdmi->dclk_en) {
		hdmi->sample_rate = rate;
		hdmi_set_clk_regenerator(hdmi, hdmi->hdmi_data.video_mode.mtmdsclock,
					 hdmi->sample_rate);
	}
	mutex_unlock(&hdmi->audio_mutex);
}
EXPORT_SYMBOL_GPL(dw_hdmi_qp_set_sample_rate);

void dw_hdmi_qp_set_channel_count(struct dw_hdmi_qp *hdmi, unsigned int cnt)
{
}
EXPORT_SYMBOL_GPL(dw_hdmi_qp_set_channel_count);

void dw_hdmi_qp_set_channel_allocation(struct dw_hdmi_qp *hdmi, unsigned int ca)
{
}
EXPORT_SYMBOL_GPL(dw_hdmi_qp_set_channel_allocation);

void dw_hdmi_qp_set_audio_infoframe(struct dw_hdmi_qp *hdmi,
				    struct hdmi_codec_params *hparms)
{
	u8 infoframe_buf[HDMI_INFOFRAME_SIZE(AUDIO)];
	int ret = 0;

	ret = hdmi_audio_infoframe_pack(&hparms->cea, infoframe_buf,
					sizeof(infoframe_buf));
	if (!ret) {
		dev_err(hdmi->dev, "%s: Failed to pack audio infoframe: %d\n",
			__func__, ret);
		return;
	}

	mutex_lock(&hdmi->audio_mutex);
	if (!hdmi->dclk_en) {
		mutex_unlock(&hdmi->audio_mutex);
		return;
	}

	/*
	 * AUDI_CONTENTS0: { RSV, HB2, HB1, RSV }
	 * AUDI_CONTENTS1: { PB3, PB2, PB1, PB0 }
	 * AUDI_CONTENTS2: { PB7, PB6, PB5, PB4 }
	 *
	 * PB0: CheckSum
	 * PB1: | CT3    | CT2  | CT1  | CT0  | F13  | CC2 | CC1 | CC0 |
	 * PB2: | F27    | F26  | F25  | SF2  | SF1  | SF0 | SS1 | SS0 |
	 * PB3: | F37    | F36  | F35  | F34  | F33  | F32 | F31 | F30 |
	 * PB4: | CA7    | CA6  | CA5  | CA4  | CA3  | CA2 | CA1 | CA0 |
	 * PB5: | DM_INH | LSV3 | LSV2 | LSV1 | LSV0 | F52 | F51 | F50 |
	 * PB6~PB10: Reserved
	 *
	 * AUDI_CONTENTS0 default value defined by HDMI specification,
	 * and shall only be changed for debug purposes.
	 * So, we only configure payload byte from PB0~PB7(2 word total).
	 */
	regmap_bulk_write(hdmi->regm, PKT_AUDI_CONTENTS1, &infoframe_buf[3], 2);

	/* Enable ACR, AUDI */
	hdmi_modb(hdmi, PKTSCHED_ACR_TX_EN | PKTSCHED_AUDI_TX_EN,
		  PKTSCHED_ACR_TX_EN | PKTSCHED_AUDI_TX_EN,
		  PKTSCHED_PKT_EN);

	/* Enable AUDS */
	hdmi_modb(hdmi, PKTSCHED_AUDS_TX_EN, PKTSCHED_AUDS_TX_EN, PKTSCHED_PKT_EN);
	mutex_unlock(&hdmi->audio_mutex);
}
EXPORT_SYMBOL_GPL(dw_hdmi_qp_set_audio_infoframe);

static void hdmi_enable_audio_clk(struct dw_hdmi_qp *hdmi, bool enable)
{
	if (enable)
		hdmi_modb(hdmi, 0,
			  AVP_DATAPATH_PACKET_AUDIO_SWDISABLE, GLOBAL_SWDISABLE);
	else
		hdmi_modb(hdmi, AVP_DATAPATH_PACKET_AUDIO_SWDISABLE,
			  AVP_DATAPATH_PACKET_AUDIO_SWDISABLE, GLOBAL_SWDISABLE);
}

static void dw_hdmi_i2s_audio_enable(struct dw_hdmi_qp *hdmi)
{
	hdmi_set_cts_n(hdmi, hdmi->audio_cts, hdmi->audio_n);
	hdmi_enable_audio_clk(hdmi, true);
}

static void dw_hdmi_i2s_audio_disable(struct dw_hdmi_qp *hdmi)
{
	/* Disable AUDS, ACR, AUDI, AMD */
	hdmi_modb(hdmi, 0,
		  PKTSCHED_ACR_TX_EN | PKTSCHED_AUDS_TX_EN |
		  PKTSCHED_AUDI_TX_EN | PKTSCHED_AMD_TX_EN,
		  PKTSCHED_PKT_EN);

	hdmi_enable_audio_clk(hdmi, false);
}

void dw_hdmi_qp_audio_enable(struct dw_hdmi_qp *hdmi)
{
	mutex_lock(&hdmi->audio_mutex);
	if (hdmi->dclk_en) {
		hdmi->audio_enable = true;
		if (hdmi->enable_audio)
			hdmi->enable_audio(hdmi);
	}
	mutex_unlock(&hdmi->audio_mutex);
}
EXPORT_SYMBOL_GPL(dw_hdmi_qp_audio_enable);

void dw_hdmi_qp_audio_disable(struct dw_hdmi_qp *hdmi)
{
	mutex_lock(&hdmi->audio_mutex);
	if (hdmi->dclk_en) {
		hdmi->audio_enable = false;
		if (hdmi->disable_audio)
			hdmi->disable_audio(hdmi);
	}
	mutex_unlock(&hdmi->audio_mutex);
}
EXPORT_SYMBOL_GPL(dw_hdmi_qp_audio_disable);

static bool hdmi_bus_fmt_is_rgb(unsigned int bus_format)
{
	switch (bus_format) {
	case MEDIA_BUS_FMT_RGB888_1X24:
	case MEDIA_BUS_FMT_RGB101010_1X30:
	case MEDIA_BUS_FMT_RGB121212_1X36:
	case MEDIA_BUS_FMT_RGB161616_1X48:
		return true;

	default:
		return false;
	}
}

static bool hdmi_bus_fmt_is_yuv444(unsigned int bus_format)
{
	switch (bus_format) {
	case MEDIA_BUS_FMT_YUV8_1X24:
	case MEDIA_BUS_FMT_YUV10_1X30:
	case MEDIA_BUS_FMT_YUV12_1X36:
	case MEDIA_BUS_FMT_YUV16_1X48:
		return true;

	default:
		return false;
	}
}

static bool hdmi_bus_fmt_is_yuv422(unsigned int bus_format)
{
	switch (bus_format) {
	case MEDIA_BUS_FMT_UYVY8_1X16:
	case MEDIA_BUS_FMT_UYVY10_1X20:
	case MEDIA_BUS_FMT_UYVY12_1X24:
		return true;

	default:
		return false;
	}
}

static bool hdmi_bus_fmt_is_yuv420(unsigned int bus_format)
{
	switch (bus_format) {
	case MEDIA_BUS_FMT_UYYVYY8_0_5X24:
	case MEDIA_BUS_FMT_UYYVYY10_0_5X30:
	case MEDIA_BUS_FMT_UYYVYY12_0_5X36:
	case MEDIA_BUS_FMT_UYYVYY16_0_5X48:
		return true;

	default:
		return false;
	}
}

static int hdmi_bus_fmt_color_depth(unsigned int bus_format)
{
	switch (bus_format) {
	case MEDIA_BUS_FMT_RGB888_1X24:
	case MEDIA_BUS_FMT_YUV8_1X24:
	case MEDIA_BUS_FMT_UYVY8_1X16:
	case MEDIA_BUS_FMT_UYYVYY8_0_5X24:
		return 8;

	case MEDIA_BUS_FMT_RGB101010_1X30:
	case MEDIA_BUS_FMT_YUV10_1X30:
	case MEDIA_BUS_FMT_UYVY10_1X20:
	case MEDIA_BUS_FMT_UYYVYY10_0_5X30:
		return 10;

	case MEDIA_BUS_FMT_RGB121212_1X36:
	case MEDIA_BUS_FMT_YUV12_1X36:
	case MEDIA_BUS_FMT_UYVY12_1X24:
	case MEDIA_BUS_FMT_UYYVYY12_0_5X36:
		return 12;

	case MEDIA_BUS_FMT_RGB161616_1X48:
	case MEDIA_BUS_FMT_YUV16_1X48:
	case MEDIA_BUS_FMT_UYYVYY16_0_5X48:
		return 16;

	default:
		return 0;
	}
}

static void dw_hdmi_i2c_init(struct dw_hdmi_qp *hdmi)
{
	/* Software reset */
	hdmi_writel(hdmi, 0x01, I2CM_CONTROL0);

	hdmi_writel(hdmi, 0x085c085c, I2CM_FM_SCL_CONFIG0);

	hdmi_modb(hdmi, 0, I2CM_FM_EN, I2CM_INTERFACE_CONTROL0);

	/* Clear DONE and ERROR interrupts */
	hdmi_writel(hdmi, I2CM_OP_DONE_CLEAR | I2CM_NACK_RCVD_CLEAR,
		    MAINUNIT_1_INT_CLEAR);
}

static int dw_hdmi_i2c_read(struct dw_hdmi_qp *hdmi,
			    unsigned char *buf, unsigned int length)
{
	struct dw_hdmi_qp_i2c *i2c = hdmi->i2c;
	int stat;

	if (!i2c->is_regaddr) {
		dev_dbg(hdmi->dev, "set read register address to 0\n");
		i2c->slave_reg = 0x00;
		i2c->is_regaddr = true;
	}

	while (length--) {
		reinit_completion(&i2c->cmp);

		hdmi_modb(hdmi, i2c->slave_reg++ << 12, I2CM_ADDR,
			  I2CM_INTERFACE_CONTROL0);

		hdmi_modb(hdmi, I2CM_FM_READ, I2CM_WR_MASK,
			  I2CM_INTERFACE_CONTROL0);

		stat = wait_for_completion_timeout(&i2c->cmp, HZ / 10);
		if (!stat) {
			dev_err(hdmi->dev, "i2c read time out!\n");
			hdmi_writel(hdmi, 0x01, I2CM_CONTROL0);
			return -EAGAIN;
		}

		/* Check for error condition on the bus */
		if (i2c->stat & I2CM_NACK_RCVD_IRQ) {
			dev_err(hdmi->dev, "i2c read err!\n");
			hdmi_writel(hdmi, 0x01, I2CM_CONTROL0);
			return -EIO;
		}

		*buf++ = hdmi_readl(hdmi, I2CM_INTERFACE_RDDATA_0_3) & 0xff;
		dev_dbg(hdmi->dev, "i2c read done! i2c->stat:%02x 0x%02x\n",
			i2c->stat, hdmi_readl(hdmi, I2CM_INTERFACE_RDDATA_0_3));
		hdmi_modb(hdmi, 0, I2CM_WR_MASK, I2CM_INTERFACE_CONTROL0);
	}
	i2c->is_segment = false;

	return 0;
}

static int dw_hdmi_i2c_write(struct dw_hdmi_qp *hdmi,
			     unsigned char *buf, unsigned int length)
{
	struct dw_hdmi_qp_i2c *i2c = hdmi->i2c;
	int stat;

	if (!i2c->is_regaddr) {
		/* Use the first write byte as register address */
		i2c->slave_reg = buf[0];
		length--;
		buf++;
		i2c->is_regaddr = true;
	}

	while (length--) {
		reinit_completion(&i2c->cmp);

		hdmi_writel(hdmi, *buf++, I2CM_INTERFACE_WRDATA_0_3);
		hdmi_modb(hdmi, i2c->slave_reg++ << 12, I2CM_ADDR,
			  I2CM_INTERFACE_CONTROL0);
		hdmi_modb(hdmi, I2CM_FM_WRITE, I2CM_WR_MASK,
			  I2CM_INTERFACE_CONTROL0);

		stat = wait_for_completion_timeout(&i2c->cmp, HZ / 10);
		if (!stat) {
			dev_err(hdmi->dev, "i2c write time out!\n");
			hdmi_writel(hdmi, 0x01, I2CM_CONTROL0);
			return -EAGAIN;
		}

		/* Check for error condition on the bus */
		if (i2c->stat & I2CM_NACK_RCVD_IRQ) {
			dev_err(hdmi->dev, "i2c write nack!\n");
			hdmi_writel(hdmi, 0x01, I2CM_CONTROL0);
			return -EIO;
		}
		hdmi_modb(hdmi, 0, I2CM_WR_MASK, I2CM_INTERFACE_CONTROL0);
	}
	dev_dbg(hdmi->dev, "i2c write done!\n");
	return 0;
}

static int dw_hdmi_i2c_xfer(struct i2c_adapter *adap,
			    struct i2c_msg *msgs, int num)
{
	struct dw_hdmi_qp *hdmi = i2c_get_adapdata(adap);
	struct dw_hdmi_qp_i2c *i2c = hdmi->i2c;
	u8 addr = msgs[0].addr;
	int i, ret = 0;

	if (addr == DDC_CI_ADDR)
		/*
		 * The internal I2C controller does not support the multi-byte
		 * read and write operations needed for DDC/CI.
		 * TOFIX: Blacklist the DDC/CI address until we filter out
		 * unsupported I2C operations.
		 */
		return -EOPNOTSUPP;

	dev_dbg(hdmi->dev, "i2c xfer: num: %d, addr: %#x\n", num, addr);

	for (i = 0; i < num; i++) {
		if (msgs[i].len == 0) {
			dev_err(hdmi->dev,
				"unsupported transfer %d/%d, no data\n",
				i + 1, num);
			return -EOPNOTSUPP;
		}
	}

	mutex_lock(&i2c->lock);

	/* Unmute DONE and ERROR interrupts */
	hdmi_modb(hdmi, I2CM_NACK_RCVD_MASK_N | I2CM_OP_DONE_MASK_N,
		  I2CM_NACK_RCVD_MASK_N | I2CM_OP_DONE_MASK_N,
		  MAINUNIT_1_INT_MASK_N);

	/* Set slave device address taken from the first I2C message */
	if (addr == DDC_SEGMENT_ADDR && msgs[0].len == 1)
		addr = DDC_ADDR;

	hdmi_modb(hdmi, addr << 5, I2CM_SLVADDR, I2CM_INTERFACE_CONTROL0);

	/* Set slave device register address on transfer */
	i2c->is_regaddr = false;

	/* Set segment pointer for I2C extended read mode operation */
	i2c->is_segment = false;

	for (i = 0; i < num; i++) {
		dev_dbg(hdmi->dev, "xfer: num: %d/%d, len: %d, flags: %#x\n",
			i + 1, num, msgs[i].len, msgs[i].flags);

		if (msgs[i].addr == DDC_SEGMENT_ADDR && msgs[i].len == 1) {
			i2c->is_segment = true;
			hdmi_modb(hdmi, DDC_SEGMENT_ADDR, I2CM_SEG_ADDR,
				  I2CM_INTERFACE_CONTROL1);
			hdmi_modb(hdmi, *msgs[i].buf, I2CM_SEG_PTR,
				  I2CM_INTERFACE_CONTROL1);
		} else {
			if (msgs[i].flags & I2C_M_RD)
				ret = dw_hdmi_i2c_read(hdmi, msgs[i].buf,
						       msgs[i].len);
			else
				ret = dw_hdmi_i2c_write(hdmi, msgs[i].buf,
							msgs[i].len);
		}
		if (ret < 0)
			break;
	}

	if (!ret)
		ret = num;

	/* Mute DONE and ERROR interrupts */
	hdmi_modb(hdmi, 0, I2CM_OP_DONE_MASK_N | I2CM_NACK_RCVD_MASK_N,
		  MAINUNIT_1_INT_MASK_N);

	mutex_unlock(&i2c->lock);

	return ret;
}

static u32 dw_hdmi_i2c_func(struct i2c_adapter *adapter)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm dw_hdmi_algorithm = {
	.master_xfer	= dw_hdmi_i2c_xfer,
	.functionality	= dw_hdmi_i2c_func,
};

static struct i2c_adapter *dw_hdmi_i2c_adapter(struct dw_hdmi_qp *hdmi)
{
	struct i2c_adapter *adap;
	struct dw_hdmi_qp_i2c *i2c;
	int ret;

	i2c = devm_kzalloc(hdmi->dev, sizeof(*i2c), GFP_KERNEL);
	if (!i2c)
		return ERR_PTR(-ENOMEM);

	mutex_init(&i2c->lock);
	init_completion(&i2c->cmp);

	adap = &i2c->adap;
	adap->class = I2C_CLASS_DDC;
	adap->owner = THIS_MODULE;
	adap->dev.parent = hdmi->dev;
	adap->algo = &dw_hdmi_algorithm;
	strscpy(adap->name, "ddc", sizeof(adap->name));
	i2c_set_adapdata(adap, hdmi);

	ret = i2c_add_adapter(adap);
	if (ret) {
		dev_warn(hdmi->dev, "cannot add %s I2C adapter\n", adap->name);
		devm_kfree(hdmi->dev, i2c);
		return ERR_PTR(ret);
	}

	hdmi->i2c = i2c;

	dev_info(hdmi->dev, "registered %s I2C bus driver\n", adap->name);

	return adap;
}

#define HDMI_PHY_EARC_MASK	BIT(29)

int dw_hdmi_qp_set_earc(struct dw_hdmi_qp *hdmi)
{
	u32 stat, ret;

	/* set hdmi phy earc mode */
	hdmi->phy.ops->set_mode(hdmi, hdmi->phy.data, HDMI_PHY_EARC_MASK,
				true);

	ret = hdmi->phy.ops->init(hdmi, hdmi->phy.data,
				  &hdmi->previous_mode);
	if (ret)
		return ret;

	reinit_completion(&hdmi->earc_cmp);

	hdmi_modb(hdmi, EARCRX_CMDC_DISCOVERY_TIMEOUT_IRQ |
		  EARCRX_CMDC_DISCOVERY_DONE_IRQ,
		  EARCRX_CMDC_DISCOVERY_TIMEOUT_IRQ |
		  EARCRX_CMDC_DISCOVERY_DONE_IRQ, EARCRX_0_INT_MASK_N);

	/* start discovery */
	hdmi_modb(hdmi, EARCRX_CMDC_DISCOVERY_EN, EARCRX_CMDC_DISCOVERY_EN,
		  EARCRX_CMDC_CONTROL);

	/*
	 * The eARC TX device drives a logic-high-voltage-level
	 * pulse on the physical HPD connector pin, after
	 * at least 100 ms of low voltage level to start the
	 * eARC Discovery process.
	 */
	hdmi_modb(hdmi, EARCRX_CONNECTOR_HPD, EARCRX_CONNECTOR_HPD,
		  EARCRX_CMDC_CONTROL);

	stat = wait_for_completion_timeout(&hdmi->earc_cmp, HZ / 10);
	if (!stat)
		return -EAGAIN;

	if (hdmi->earc_intr & EARCRX_CMDC_DISCOVERY_TIMEOUT_IRQ) {
		dev_err(hdmi->dev, "discovery timeout\n");
		return -ETIMEDOUT;
	} else if (hdmi->earc_intr & EARCRX_CMDC_DISCOVERY_DONE_IRQ) {
		dev_info(hdmi->dev, "discovery done\n");
	} else {
		dev_err(hdmi->dev, "discovery failed\n");
		return -EINVAL;
	}

	hdmi_writel(hdmi, 1, EARCRX_DMAC_PHY_CONTROL);
	hdmi_modb(hdmi, EARCRX_CMDC_SWINIT_P, EARCRX_CMDC_SWINIT_P,
		  EARCRX_CMDC_CONFIG0);

	hdmi_writel(hdmi, 0xf3, EARCRX_DMAC_CONFIG);
	hdmi_writel(hdmi, 0x63, EARCRX_DMAC_CONTROL0);
	hdmi_writel(hdmi, 0xff, EARCRX_DMAC_CONTROL1);

	hdmi_modb(hdmi, EARCRX_XACTREAD_STOP_CFG | EARCRX_XACTREAD_RETRY_CFG |
		  EARCRX_CMDC_DSCVR_EARCVALID0_TO_DISC1 | EARCRX_CMDC_XACT_RESTART_EN,
		  EARCRX_XACTREAD_STOP_CFG | EARCRX_XACTREAD_RETRY_CFG |
		  EARCRX_CMDC_DSCVR_EARCVALID0_TO_DISC1 | EARCRX_CMDC_XACT_RESTART_EN,
		  EARCRX_CMDC_CONFIG0);

	hdmi_writel(hdmi, 0, EARCRX_DMAC_CHSTATUS_STREAMER0);
	hdmi_writel(hdmi, 0x1b0e, EARCRX_DMAC_CHSTATUS_STREAMER1);
	hdmi_writel(hdmi, 0, EARCRX_DMAC_CHSTATUS_STREAMER2);
	hdmi_writel(hdmi, 0, EARCRX_DMAC_CHSTATUS_STREAMER3);
	hdmi_writel(hdmi, 0xf2000000, EARCRX_DMAC_CHSTATUS_STREAMER4);
	hdmi_writel(hdmi, 0, EARCRX_DMAC_CHSTATUS_STREAMER5);
	hdmi_writel(hdmi, 0, EARCRX_DMAC_CHSTATUS_STREAMER6);
	hdmi_writel(hdmi, 0, EARCRX_DMAC_CHSTATUS_STREAMER7);
	hdmi_writel(hdmi, 0, EARCRX_DMAC_CHSTATUS_STREAMER8);

	return 0;
}
EXPORT_SYMBOL_GPL(dw_hdmi_qp_set_earc);

/* -----------------------------------------------------------------------------
 * HDMI TX Setup
 */

static void hdmi_infoframe_set_checksum(u8 *ptr, int size)
{
	u8 csum = 0;
	int i;

	ptr[3] = 0;
	/* compute checksum */
	for (i = 0; i < size; i++)
		csum += ptr[i];

	ptr[3] = 256 - csum;
}

static bool is_hdmi2_sink(const struct drm_connector *connector)
{
	if (!connector)
		return true;

	return connector->display_info.hdmi.scdc.supported ||
		connector->display_info.color_formats & DRM_COLOR_FORMAT_YCRCB420;
}

static void hdmi_config_AVI(struct dw_hdmi_qp *hdmi,
			    const struct drm_connector *connector,
			    const struct drm_display_mode *mode)
{
	struct hdmi_avi_infoframe frame;
	u32 val, i, j;
	u8 buff[17];
	enum hdmi_quantization_range rgb_quant_range =
		hdmi->hdmi_data.quant_range;

	/* Initialise info frame from DRM mode */
	drm_hdmi_avi_infoframe_from_display_mode(&frame, connector, mode);

	/*
	 * Ignore monitor selectable quantization, use quantization set
	 * by the user
	 */
	drm_hdmi_avi_infoframe_quant_range(&frame, connector, mode, rgb_quant_range);
	if (hdmi_bus_fmt_is_yuv444(hdmi->hdmi_data.enc_out_bus_format))
		frame.colorspace = HDMI_COLORSPACE_YUV444;
	else if (hdmi_bus_fmt_is_yuv422(hdmi->hdmi_data.enc_out_bus_format))
		frame.colorspace = HDMI_COLORSPACE_YUV422;
	else if (hdmi_bus_fmt_is_yuv420(hdmi->hdmi_data.enc_out_bus_format))
		frame.colorspace = HDMI_COLORSPACE_YUV420;
	else
		frame.colorspace = HDMI_COLORSPACE_RGB;

	/* Set up colorimetry and quant range */
	if (!hdmi_bus_fmt_is_rgb(hdmi->hdmi_data.enc_out_bus_format)) {
		switch (hdmi->hdmi_data.enc_out_encoding) {
		case V4L2_YCBCR_ENC_601:
			if (hdmi->hdmi_data.enc_in_encoding == V4L2_YCBCR_ENC_XV601)
				frame.colorimetry = HDMI_COLORIMETRY_EXTENDED;
			else
				frame.colorimetry = HDMI_COLORIMETRY_ITU_601;
			frame.extended_colorimetry =
					HDMI_EXTENDED_COLORIMETRY_XV_YCC_601;
			break;
		case V4L2_YCBCR_ENC_709:
			if (hdmi->hdmi_data.enc_in_encoding == V4L2_YCBCR_ENC_XV709)
				frame.colorimetry = HDMI_COLORIMETRY_EXTENDED;
			else
				frame.colorimetry = HDMI_COLORIMETRY_ITU_709;
			frame.extended_colorimetry =
					HDMI_EXTENDED_COLORIMETRY_XV_YCC_709;
			break;
		case V4L2_YCBCR_ENC_BT2020:
			if (hdmi->hdmi_data.enc_in_encoding == V4L2_YCBCR_ENC_BT2020)
				frame.colorimetry = HDMI_COLORIMETRY_EXTENDED;
			else
				frame.colorimetry = HDMI_COLORIMETRY_ITU_709;
			frame.extended_colorimetry =
					HDMI_EXTENDED_COLORIMETRY_BT2020;
			break;
		default: /* Carries no data */
			frame.colorimetry = HDMI_COLORIMETRY_ITU_601;
			frame.extended_colorimetry =
					HDMI_EXTENDED_COLORIMETRY_XV_YCC_601;
			break;
		}

		frame.ycc_quantization_range = HDMI_YCC_QUANTIZATION_RANGE_LIMITED;
	} else {
		if (hdmi->hdmi_data.enc_out_encoding == V4L2_YCBCR_ENC_BT2020) {
			frame.colorimetry = HDMI_COLORIMETRY_EXTENDED;
			frame.extended_colorimetry =
				HDMI_EXTENDED_COLORIMETRY_BT2020;
		} else {
			frame.colorimetry = HDMI_COLORIMETRY_NONE;
			frame.extended_colorimetry =
				HDMI_EXTENDED_COLORIMETRY_XV_YCC_601;
		}

		if (is_hdmi2_sink(connector) &&
		    frame.quantization_range == HDMI_QUANTIZATION_RANGE_FULL)
			frame.ycc_quantization_range = HDMI_YCC_QUANTIZATION_RANGE_FULL;
		else
			frame.ycc_quantization_range = HDMI_YCC_QUANTIZATION_RANGE_LIMITED;
	}

	frame.scan_mode = HDMI_SCAN_MODE_NONE;

	hdmi_avi_infoframe_pack_only(&frame, buff, 17);

	/* mode which vic >= 128 must use avi version 3 */
	if (hdmi->vic >= 128) {
		frame.version = 3;
		buff[1] = frame.version;
		buff[4] &= 0x1f;
		buff[4] |= ((frame.colorspace & 0x7) << 5);
		buff[7] = hdmi->vic;
		hdmi_infoframe_set_checksum(buff, 17);
	}

	/*
	 * The Designware IP uses a different byte format from standard
	 * AVI info frames, though generally the bits are in the correct
	 * bytes.
	 */

	val = (frame.version << 8) | (frame.length << 16);
	hdmi_writel(hdmi, val, PKT_AVI_CONTENTS0);

	for (i = 0; i < 4; i++) {
		for (j = 0; j < 4; j++) {
			if (i * 4 + j >= 14)
				break;
			if (!j)
				val = buff[i * 4 + j + 3];
			val |= buff[i * 4 + j + 3] << (8 * j);
		}

		hdmi_writel(hdmi, val, PKT_AVI_CONTENTS1 + i * 4);
	}

	hdmi_modb(hdmi, 0, PKTSCHED_AVI_FIELDRATE, PKTSCHED_PKT_CONFIG1);

	hdmi_modb(hdmi, PKTSCHED_AVI_TX_EN, PKTSCHED_AVI_TX_EN, PKTSCHED_PKT_EN);
}

static void hdmi_config_vendor_specific_infoframe(struct dw_hdmi_qp *hdmi,
						  const struct drm_connector *connector,
						  const struct drm_display_mode *mode)
{
	struct hdmi_vendor_infoframe frame;
	u8 buffer[10];
	u32 val;
	ssize_t err;
	int i, reg;

	hdmi_modb(hdmi, 0, PKTSCHED_VSI_TX_EN, PKTSCHED_PKT_EN);
	err = drm_hdmi_vendor_infoframe_from_display_mode(&frame, connector,
							  mode);
	if (err < 0)
		/*
		 * Going into that statement does not means vendor infoframe
		 * fails. It just informed us that vendor infoframe is not
		 * needed for the selected mode. Only 4k or stereoscopic 3D
		 * mode requires vendor infoframe. So just simply return.
		 */
		return;

	err = hdmi_vendor_infoframe_pack(&frame, buffer, sizeof(buffer));
	if (err < 0) {
		dev_err(hdmi->dev, "Failed to pack vendor infoframe: %zd\n",
			err);
		return;
	}

	/* vsi header */
	val = (buffer[2] << 16) | (buffer[1] << 8) | buffer[0];
	hdmi_writel(hdmi, val, PKT_VSI_CONTENTS0);

	reg = PKT_VSI_CONTENTS1;
	for (i = 3; i < err; i++) {
		if (i % 4 == 3)
			val = buffer[i];
		if (i % 4 == 0)
			val |= buffer[i] << 8;
		if (i % 4 == 1)
			val |= buffer[i] << 16;
		if (i % 4 == 2)
			val |= buffer[i] << 24;

		if ((i % 4 == 2) || (i == (err - 1))) {
			hdmi_writel(hdmi, val, reg);
			reg += 4;
		}
	}

	hdmi_writel(hdmi, 0, PKT_VSI_CONTENTS7);

	hdmi_modb(hdmi, 0, PKTSCHED_VSI_FIELDRATE, PKTSCHED_PKT_CONFIG1);
	hdmi_modb(hdmi, PKTSCHED_VSI_TX_EN, PKTSCHED_VSI_TX_EN,
		  PKTSCHED_PKT_EN);
}

static void hdmi_config_CVTEM(struct dw_hdmi_qp *hdmi)
{
	u8 ds_type = 0;
	u8 sync = 1;
	u8 vfr = 1;
	u8 afr = 0;
	u8 new = 1;
	u8 end = 0;
	u8 data_set_length = 136;
	u8 hb1[6] = { 0x80, 0, 0, 0, 0, 0x40 };
	u8 *pps_body;
	u32 val, i, reg;
	struct drm_display_mode *mode = &hdmi->previous_mode;
	int hsync, hfront, hback;
	struct dw_hdmi_link_config *link_cfg;
	void *data = hdmi->plat_data->phy_data;

	hdmi_modb(hdmi, 0, PKTSCHED_EMP_CVTEM_TX_EN, PKTSCHED_PKT_EN);

	if (hdmi->plat_data->get_link_cfg) {
		link_cfg = hdmi->plat_data->get_link_cfg(data);
	} else {
		dev_err(hdmi->dev, "can't get frl link cfg\n");
		return;
	}

	if (!link_cfg->dsc_mode) {
		dev_info(hdmi->dev, "don't use dsc mode\n");
		return;
	}

	pps_body = link_cfg->pps_payload;

	hsync = mode->hsync_end - mode->hsync_start;
	hback = mode->htotal - mode->hsync_end;
	hfront = mode->hsync_start - mode->hdisplay;

	for (i = 0; i < 6; i++) {
		val = i << 16 | hb1[i] << 8;
		hdmi_writel(hdmi, val, PKT0_EMP_CVTEM_CONTENTS0 + i * 0x20);
	}

	val = new << 7 | end << 6 | ds_type << 4 | afr << 3 |
	      vfr << 2 | sync << 1;
	hdmi_writel(hdmi, val, PKT0_EMP_CVTEM_CONTENTS1);

	val = data_set_length << 16 | pps_body[0] << 24;
	hdmi_writel(hdmi, val, PKT0_EMP_CVTEM_CONTENTS2);

	reg = PKT0_EMP_CVTEM_CONTENTS3;
	for (i = 1; i < 125; i++) {
		if (reg == PKT1_EMP_CVTEM_CONTENTS0 ||
		    reg == PKT2_EMP_CVTEM_CONTENTS0 ||
		    reg == PKT3_EMP_CVTEM_CONTENTS0 ||
		    reg == PKT4_EMP_CVTEM_CONTENTS0 ||
		    reg == PKT5_EMP_CVTEM_CONTENTS0) {
			reg += 4;
			i--;
			continue;
		}
		if (i % 4 == 1)
			val = pps_body[i];
		if (i % 4 == 2)
			val |= pps_body[i] << 8;
		if (i % 4 == 3)
			val |= pps_body[i] << 16;
		if (!(i % 4)) {
			val |= pps_body[i] << 24;
			hdmi_writel(hdmi, val, reg);
			reg += 4;
		}
	}

	val = (hfront & 0xff) << 24 | pps_body[127] << 16 |
	      pps_body[126] << 8 | pps_body[125];
	hdmi_writel(hdmi, val, PKT4_EMP_CVTEM_CONTENTS6);

	val = (hback & 0xff) << 24 | ((hsync >> 8) & 0xff) << 16 |
	      (hsync & 0xff) << 8 | ((hfront >> 8) & 0xff);
	hdmi_writel(hdmi, val, PKT4_EMP_CVTEM_CONTENTS7);

	val = link_cfg->hcactive << 8 | ((hback >> 8) & 0xff);
	hdmi_writel(hdmi, val, PKT5_EMP_CVTEM_CONTENTS1);

	for (i = PKT5_EMP_CVTEM_CONTENTS2; i <= PKT5_EMP_CVTEM_CONTENTS7; i += 4)
		hdmi_writel(hdmi, 0, i);

	hdmi_modb(hdmi, PKTSCHED_EMP_CVTEM_TX_EN, PKTSCHED_EMP_CVTEM_TX_EN,
		  PKTSCHED_PKT_EN);
}

static void hdmi_config_drm_infoframe(struct dw_hdmi_qp *hdmi,
				      const struct drm_connector *connector)
{
	const struct drm_connector_state *conn_state = connector->state;
	struct hdr_output_metadata *hdr_metadata;
	struct hdmi_drm_infoframe frame;
	u8 buffer[30];
	ssize_t err;
	int i;
	u32 val;

	if (!hdmi->plat_data->use_drm_infoframe)
		return;

	hdmi_modb(hdmi, 0, PKTSCHED_DRMI_TX_EN, PKTSCHED_PKT_EN);

	if (!hdmi->connector.hdr_sink_metadata.hdmi_type1.eotf) {
		DRM_DEBUG("No need to set HDR metadata in infoframe\n");
		return;
	}

	if (!conn_state->hdr_output_metadata) {
		DRM_DEBUG("source metadata not set yet\n");
		return;
	}

	hdr_metadata = (struct hdr_output_metadata *)
		conn_state->hdr_output_metadata->data;

	if (!(hdmi->connector.hdr_sink_metadata.hdmi_type1.eotf &
	      BIT(hdr_metadata->hdmi_metadata_type1.eotf))) {
		DRM_ERROR("Not support EOTF %d\n",
			  hdr_metadata->hdmi_metadata_type1.eotf);
		return;
	}

	err = drm_hdmi_infoframe_set_hdr_metadata(&frame, conn_state);
	if (err < 0)
		return;

	err = hdmi_drm_infoframe_pack(&frame, buffer, sizeof(buffer));
	if (err < 0) {
		dev_err(hdmi->dev, "Failed to pack drm infoframe: %zd\n", err);
		return;
	}

	val = (frame.version << 8) | (frame.length << 16);
	hdmi_writel(hdmi, val, PKT_DRMI_CONTENTS0);

	for (i = 0; i <= frame.length; i++) {
		if (i % 4 == 0)
			val = buffer[3 + i];
		val |= buffer[3 + i] << ((i % 4) * 8);

		if (i % 4 == 3 || (i == (frame.length)))
			hdmi_writel(hdmi, val, PKT_DRMI_CONTENTS1 + ((i / 4) * 4));
	}

	hdmi_modb(hdmi, 0, PKTSCHED_DRMI_FIELDRATE, PKTSCHED_PKT_CONFIG1);

	hdmi_modb(hdmi, PKTSCHED_DRMI_TX_EN, PKTSCHED_DRMI_TX_EN, PKTSCHED_PKT_EN);

	DRM_DEBUG("%s eotf %d end\n", __func__,
		  hdr_metadata->hdmi_metadata_type1.eotf);
}

/* Filter out invalid setups to avoid configuring SCDC and scrambling */
static bool dw_hdmi_support_scdc(struct dw_hdmi_qp *hdmi,
				 const struct drm_display_info *display)
{
	/* Disable if no DDC bus */
	if (!hdmi->ddc)
		return false;

	/* Disable if SCDC is not supported, or if an HF-VSDB block is absent */
	if (!display->hdmi.scdc.supported ||
	    !display->hdmi.scdc.scrambling.supported)
		return false;

	/*
	 * Disable if display only support low TMDS rates and scrambling
	 * for low rates is not supported either
	 */
	if (!display->hdmi.scdc.scrambling.low_rates &&
	    display->max_tmds_clock <= 340000)
		return false;

	return true;
}

static int hdmi_set_frl_mask(int frl_rate)
{
	switch (frl_rate) {
	case 48:
		return FRL_12GBPS_4LANE;
	case 40:
		return FRL_10GBPS_4LANE;
	case 32:
		return FRL_8GBPS_4LANE;
	case 24:
		return FRL_6GBPS_4LANE;
	case 18:
		return FRL_6GBPS_3LANE;
	case 9:
		return FRL_3GBPS_3LANE;
	}

	return 0;
}

static int hdmi_start_flt(struct dw_hdmi_qp *hdmi, u8 rate)
{
	u8 val;
	u32 value;
	u8 ffe_lv = 0;
	int i = 0;
	bool ltsp = false;

	hdmi_modb(hdmi, AVP_DATAPATH_VIDEO_SWDISABLE,
		  AVP_DATAPATH_VIDEO_SWDISABLE, GLOBAL_SWDISABLE);

	/* FLT_READY & FFE_LEVELS read */
	for (i = 0; i < 20; i++) {
		drm_scdc_readb(hdmi->ddc, SCDC_STATUS_FLAGS_0, &val);
		if (val & BIT(6))
			break;
		msleep(20);
	}

	if (i == 20) {
		dev_err(hdmi->dev, "sink flt isn't ready\n");
		return -EINVAL;
	}

	/* clear flt flags */
	drm_scdc_readb(hdmi->ddc, 0x10, &val);
	if (val & BIT(5))
		drm_scdc_writeb(hdmi->ddc, 0x10, BIT(5));

	/* max ffe level 3 */
	val = 0 << 4 | hdmi_set_frl_mask(rate);
	drm_scdc_writeb(hdmi->ddc, 0x31, val);

	/* select FRL_RATE & FFE_LEVELS */
	hdmi_writel(hdmi, ffe_lv, FLT_CONFIG0);

	/* we set max 2s timeout */
	i = 4000;
	while (i--) {
		/* source should poll update flag every 2ms or less */
		usleep_range(400, 500);
		drm_scdc_readb(hdmi->ddc, 0x10, &val);

		if (!(val & 0x30))
			continue;

		if (val & BIT(5)) {
			u8 reg_val, ln0, ln1, ln2, ln3;

			drm_scdc_readb(hdmi->ddc, 0x41, &reg_val);
			ln0 = reg_val & 0xf;
			ln1 = (reg_val >> 4) & 0xf;

			drm_scdc_readb(hdmi->ddc, 0x42, &reg_val);
			ln2 = reg_val & 0xf;
			ln3 = (reg_val >> 4) & 0xf;

			if (!ln0 && !ln1 && !ln2 && !ln3) {
				dev_info(hdmi->dev, "goto ltsp\n");
				ltsp = true;
				hdmi_writel(hdmi, 0, FLT_CONFIG1);
			} else if ((ln0 == 0xf) | (ln1 == 0xf) | (ln2 == 0xf) | (ln3 == 0xf)) {
				dev_err(hdmi->dev, "goto lts4\n");
				break;
			} else if ((ln0 == 0xe) | (ln1 == 0xe) | (ln2 == 0xe) | (ln3 == 0xe)) {
				dev_info(hdmi->dev, "goto ffe\n");
				break;
			} else {
				value = (ln3 << 16) | (ln2 << 12) | (ln1 << 8) | (ln0 << 4) | 0xf;
				hdmi_writel(hdmi, value, FLT_CONFIG1);
			}
		}

		/* only clear frl_start and flt_update */
		drm_scdc_writeb(hdmi->ddc, 0x10, val & 0x30);

		if ((val & BIT(4)) && ltsp) {
			hdmi_modb(hdmi, 0, AVP_DATAPATH_VIDEO_SWDISABLE, GLOBAL_SWDISABLE);
			dev_info(hdmi->dev, "flt success\n");
			break;
		}
	}

	if (i < 0) {
		dev_err(hdmi->dev, "flt time out\n");
		return -ETIMEDOUT;
	}

	return 0;
}

#define HDMI_MODE_FRL_MASK     BIT(30)

static int hdmi_set_op_mode(struct dw_hdmi_qp *hdmi,
			    struct dw_hdmi_link_config *link_cfg,
			    const struct drm_connector *connector)
{
	int frl_rate;
	int i, ret;

	if (hdmi->frl_switch)
		return 0;

	if (!link_cfg->frl_mode) {
		dev_info(hdmi->dev, "dw hdmi qp use tmds mode\n");
		hdmi_modb(hdmi, 0, OPMODE_FRL, LINK_CONFIG0);
		hdmi_modb(hdmi, 0, OPMODE_FRL_4LANES, LINK_CONFIG0);
		return hdmi->phy.ops->init(hdmi, hdmi->phy.data, &hdmi->previous_mode);
	}

	if (link_cfg->frl_lanes == 4)
		hdmi_modb(hdmi, OPMODE_FRL_4LANES, OPMODE_FRL_4LANES,
			  LINK_CONFIG0);
	else
		hdmi_modb(hdmi, 0, OPMODE_FRL_4LANES, LINK_CONFIG0);

	hdmi_modb(hdmi, 1, OPMODE_FRL, LINK_CONFIG0);

	frl_rate = link_cfg->frl_lanes * link_cfg->rate_per_lane;

	ret = hdmi->phy.ops->init(hdmi, hdmi->phy.data, &hdmi->previous_mode);
	if (ret)
		return ret;
	msleep(50);

	ret = hdmi_start_flt(hdmi, frl_rate);
	if (ret) {
		hdmi_writel(hdmi, 0, FLT_CONFIG0);
		drm_scdc_writeb(hdmi->ddc, 0x31, 0);
		hdmi_modb(hdmi, 0, AVP_DATAPATH_VIDEO_SWDISABLE, GLOBAL_SWDISABLE);
		return ret;
	}

	for (i = 0; i < 200; i++) {
		hdmi_modb(hdmi, PKTSCHED_NULL_TX_EN, PKTSCHED_NULL_TX_EN, PKTSCHED_PKT_EN);
		usleep_range(50, 60);
		hdmi_modb(hdmi, 0, PKTSCHED_NULL_TX_EN, PKTSCHED_PKT_EN);
		usleep_range(50, 60);
	}

	return 0;
}

static unsigned long
hdmi_get_tmdsclock(struct dw_hdmi_qp *hdmi, unsigned long mpixelclock)
{
	unsigned long tmdsclock = mpixelclock;
	unsigned int depth =
		hdmi_bus_fmt_color_depth(hdmi->hdmi_data.enc_out_bus_format);

	if (!hdmi_bus_fmt_is_yuv422(hdmi->hdmi_data.enc_out_bus_format)) {
		switch (depth) {
		case 16:
			tmdsclock = mpixelclock * 2;
			break;
		case 12:
			tmdsclock = mpixelclock * 3 / 2;
			break;
		case 10:
			tmdsclock = mpixelclock * 5 / 4;
			break;
		default:
			break;
		}
	}

	return tmdsclock;
}

static int dw_hdmi_qp_setup(struct dw_hdmi_qp *hdmi,
			    const struct drm_connector *connector,
			    struct drm_display_mode *mode)
{
	void *data = hdmi->plat_data->phy_data;
	struct hdmi_vmode_qp *vmode = &hdmi->hdmi_data.video_mode;
	struct dw_hdmi_link_config *link_cfg;
	u8 bytes = 0;

	hdmi->vic = drm_match_cea_mode(mode);
	if (!hdmi->vic)
		dev_dbg(hdmi->dev, "Non-CEA mode used in HDMI\n");
	else
		dev_dbg(hdmi->dev, "CEA mode used vic=%d\n", hdmi->vic);

	if (hdmi->plat_data->get_enc_out_encoding)
		hdmi->hdmi_data.enc_out_encoding =
			hdmi->plat_data->get_enc_out_encoding(data);
	else if ((hdmi->vic == 6) || (hdmi->vic == 7) ||
		 (hdmi->vic == 21) || (hdmi->vic == 22) ||
		 (hdmi->vic == 2) || (hdmi->vic == 3) ||
		 (hdmi->vic == 17) || (hdmi->vic == 18))
		hdmi->hdmi_data.enc_out_encoding = V4L2_YCBCR_ENC_601;
	else
		hdmi->hdmi_data.enc_out_encoding = V4L2_YCBCR_ENC_709;

	if (mode->flags & DRM_MODE_FLAG_DBLCLK) {
		hdmi->hdmi_data.video_mode.mpixelrepetitionoutput = 1;
		hdmi->hdmi_data.video_mode.mpixelrepetitioninput = 1;
	} else {
		hdmi->hdmi_data.video_mode.mpixelrepetitionoutput = 0;
		hdmi->hdmi_data.video_mode.mpixelrepetitioninput = 0;
	}
	/*  Get input format from plat data or fallback to RGB888 */
	if (hdmi->plat_data->get_input_bus_format)
		hdmi->hdmi_data.enc_in_bus_format =
			hdmi->plat_data->get_input_bus_format(data);
	else if (hdmi->plat_data->input_bus_format)
		hdmi->hdmi_data.enc_in_bus_format =
			hdmi->plat_data->input_bus_format;
	else
		hdmi->hdmi_data.enc_in_bus_format = MEDIA_BUS_FMT_RGB888_1X24;

	/* Default to RGB888 output format */
	if (hdmi->plat_data->get_output_bus_format)
		hdmi->hdmi_data.enc_out_bus_format =
			hdmi->plat_data->get_output_bus_format(data);
	else
		hdmi->hdmi_data.enc_out_bus_format = MEDIA_BUS_FMT_RGB888_1X24;

	/* Get input encoding from plat data or fallback to none */
	if (hdmi->plat_data->get_enc_in_encoding)
		hdmi->hdmi_data.enc_in_encoding =
			hdmi->plat_data->get_enc_in_encoding(data);
	else if (hdmi->plat_data->input_bus_encoding)
		hdmi->hdmi_data.enc_in_encoding =
			hdmi->plat_data->input_bus_encoding;
	else
		hdmi->hdmi_data.enc_in_encoding = V4L2_YCBCR_ENC_DEFAULT;

	if (hdmi->plat_data->get_quant_range)
		hdmi->hdmi_data.quant_range =
			hdmi->plat_data->get_quant_range(data);
	else
		hdmi->hdmi_data.quant_range = HDMI_QUANTIZATION_RANGE_DEFAULT;

	if (hdmi->plat_data->get_link_cfg)
		link_cfg = hdmi->plat_data->get_link_cfg(data);
	else
		return -EINVAL;

	hdmi->phy.ops->set_mode(hdmi, hdmi->phy.data, HDMI_MODE_FRL_MASK,
				link_cfg->frl_mode);

	if (hdmi->plat_data->link_clk_set && !hdmi->frl_switch)
		hdmi->plat_data->link_clk_set(data, true);

	/*
	 * According to the dw-hdmi specification 6.4.2
	 * vp_pr_cd[3:0]:
	 * 0000b: No pixel repetition (pixel sent only once)
	 * 0001b: Pixel sent two times (pixel repeated once)
	 */
	hdmi->hdmi_data.pix_repet_factor =
		(mode->flags & DRM_MODE_FLAG_DBLCLK) ? 1 : 0;
	hdmi->hdmi_data.video_mode.mdataenablepolarity = true;

	vmode->previous_pixelclock = vmode->mpixelclock;
	if (hdmi->plat_data->split_mode)
		mode->crtc_clock /= 2;
	vmode->mpixelclock = mode->crtc_clock * 1000;
	if ((mode->flags & DRM_MODE_FLAG_3D_MASK) == DRM_MODE_FLAG_3D_FRAME_PACKING)
		vmode->mpixelclock *= 2;
	dev_dbg(hdmi->dev, "final pixclk = %ld\n", vmode->mpixelclock);
	vmode->previous_tmdsclock = vmode->mtmdsclock;
	vmode->mtmdsclock = hdmi_get_tmdsclock(hdmi, vmode->mpixelclock);
	if (hdmi_bus_fmt_is_yuv420(hdmi->hdmi_data.enc_out_bus_format))
		vmode->mtmdsclock /= 2;
	dev_info(hdmi->dev, "final tmdsclk = %d\n", vmode->mtmdsclock);

	if (hdmi->plat_data->set_grf_cfg)
		hdmi->plat_data->set_grf_cfg(data);

	if (hdmi->sink_has_audio) {
		dev_dbg(hdmi->dev, "sink has audio support\n");

		/* HDMI Initialization Step E - Configure audio */
		hdmi_clk_regenerator_update_pixel_clock(hdmi);
		hdmi_enable_audio_clk(hdmi, hdmi->audio_enable);
	}

	/* not for DVI mode */
	if (hdmi->sink_is_hdmi) {
		int ret;

		dev_dbg(hdmi->dev, "%s HDMI mode\n", __func__);
		hdmi_modb(hdmi, 0, OPMODE_DVI, LINK_CONFIG0);
		hdmi_modb(hdmi, HDCP2_BYPASS, HDCP2_BYPASS, HDCP2LOGIC_CONFIG0);
		hdmi_modb(hdmi, KEEPOUT_REKEY_ALWAYS, KEEPOUT_REKEY_CFG, FRAME_COMPOSER_CONFIG9);

		if (!link_cfg->frl_mode && dw_hdmi_support_scdc(hdmi, &connector->display_info)) {
			if (vmode->mtmdsclock > HDMI14_MAX_TMDSCLK) {
				drm_scdc_readb(hdmi->ddc, SCDC_SINK_VERSION, &bytes);
				drm_scdc_writeb(hdmi->ddc, SCDC_SOURCE_VERSION,
						min_t(u8, bytes, SCDC_MIN_SOURCE_VERSION));
				drm_scdc_set_high_tmds_clock_ratio(hdmi->ddc, 1);
				drm_scdc_set_scrambling(hdmi->ddc, 1);
				hdmi_writel(hdmi, 1, SCRAMB_CONFIG0);
				/* Wait for resuming transmission of TMDS clock and data */
				msleep(100);
			} else {
				drm_scdc_set_high_tmds_clock_ratio(hdmi->ddc, 0);
				drm_scdc_set_scrambling(hdmi->ddc, 0);
				hdmi_writel(hdmi, 0, SCRAMB_CONFIG0);
			}
		}
		/* HDMI Initialization Step F - Configure AVI InfoFrame */
		hdmi_config_AVI(hdmi, connector, mode);
		hdmi_config_vendor_specific_infoframe(hdmi, connector, mode);
		hdmi_config_CVTEM(hdmi);
		hdmi_config_drm_infoframe(hdmi, connector);
		ret = hdmi_set_op_mode(hdmi, link_cfg, connector);
		msleep(50);
		/* clear avmute */
		hdmi_writel(hdmi, 2, PKTSCHED_PKT_CONTROL0);
		hdmi_modb(hdmi, PKTSCHED_GCP_TX_EN, PKTSCHED_GCP_TX_EN, PKTSCHED_PKT_EN);
		if (ret) {
			dev_err(hdmi->dev, "%s hdmi set operation mode failed\n", __func__);
			hdmi->frl_switch = false;
			return ret;
		}
	} else {
		hdmi_modb(hdmi, HDCP2_BYPASS, HDCP2_BYPASS, HDCP2LOGIC_CONFIG0);
		hdmi_modb(hdmi, OPMODE_DVI, OPMODE_DVI, LINK_CONFIG0);
		dev_info(hdmi->dev, "%s DVI mode\n", __func__);
	}

	hdmi->frl_switch = false;
	return 0;
}

static enum drm_connector_status
dw_hdmi_connector_detect(struct drm_connector *connector, bool force)
{
	struct dw_hdmi_qp *hdmi =
		container_of(connector, struct dw_hdmi_qp, connector);
	struct dw_hdmi_qp *secondary = NULL;
	enum drm_connector_status result, result_secondary;

	mutex_lock(&hdmi->mutex);
	hdmi->force = DRM_FORCE_UNSPECIFIED;
	mutex_unlock(&hdmi->mutex);

	if (hdmi->plat_data->left)
		secondary = hdmi->plat_data->left;
	else if (hdmi->plat_data->right)
		secondary = hdmi->plat_data->right;

	result = hdmi->phy.ops->read_hpd(hdmi, hdmi->phy.data);

	if (secondary) {
		result_secondary = secondary->phy.ops->read_hpd(secondary, secondary->phy.data);
		if (result == connector_status_connected &&
		    result_secondary == connector_status_connected)
			result = connector_status_connected;
		else
			result = connector_status_disconnected;
	}

	mutex_lock(&hdmi->mutex);
	if (result != hdmi->last_connector_result) {
		dev_dbg(hdmi->dev, "read_hpd result: %d", result);
		handle_plugged_change(hdmi,
				      result == connector_status_connected);
		hdmi->last_connector_result = result;
	}
	mutex_unlock(&hdmi->mutex);

	return result;
}

static int
dw_hdmi_update_hdr_property(struct drm_connector *connector)
{
	struct drm_device *dev = connector->dev;
	struct dw_hdmi_qp *hdmi = container_of(connector, struct dw_hdmi_qp,
					       connector);
	void *data = hdmi->plat_data->phy_data;
	const struct hdr_static_metadata *metadata =
		&connector->hdr_sink_metadata.hdmi_type1;
	size_t size = sizeof(*metadata);
	struct drm_property *property;
	struct drm_property_blob *blob;
	int ret;

	if (hdmi->plat_data->get_hdr_property)
		property = hdmi->plat_data->get_hdr_property(data);
	else
		return -EINVAL;

	if (hdmi->plat_data->get_hdr_blob)
		blob = hdmi->plat_data->get_hdr_blob(data);
	else
		return -EINVAL;

	ret = drm_property_replace_global_blob(dev, &blob, size, metadata,
					       &connector->base, property);
	return ret;
}

static int dw_hdmi_connector_get_modes(struct drm_connector *connector)
{
	struct dw_hdmi_qp *hdmi =
		container_of(connector, struct dw_hdmi_qp, connector);
	struct hdr_static_metadata *metedata =
		&connector->hdr_sink_metadata.hdmi_type1;
	struct edid *edid;
	struct drm_display_mode *mode;
	struct drm_display_info *info = &connector->display_info;
	void *data = hdmi->plat_data->phy_data;
	int i, ret = 0;

	if (!hdmi->ddc)
		return 0;

	memset(metedata, 0, sizeof(*metedata));
	edid = drm_get_edid(connector, hdmi->ddc);

	#if 0 //add by ztl,if need to 
	edid = NULL;
	#endif

	if (edid) {
		dev_dbg(hdmi->dev, "got edid: width[%d] x height[%d]\n",
			edid->width_cm, edid->height_cm);

		hdmi->sink_is_hdmi = drm_detect_hdmi_monitor(edid);
		hdmi->sink_has_audio = drm_detect_monitor_audio(edid);
		drm_connector_update_edid_property(connector, edid);
		if (hdmi->cec_notifier)
			cec_notifier_set_phys_addr_from_edid(hdmi->cec_notifier, edid);
		if (hdmi->plat_data->get_edid_dsc_info)
			hdmi->plat_data->get_edid_dsc_info(data, edid);
		ret = drm_add_edid_modes(connector, edid);
		dw_hdmi_update_hdr_property(connector);
		if (ret > 0 && hdmi->plat_data->split_mode) {
			struct dw_hdmi_qp *secondary = NULL;
			void *secondary_data;

			if (hdmi->plat_data->left)
				secondary = hdmi->plat_data->left;
			else if (hdmi->plat_data->right)
				secondary = hdmi->plat_data->right;

			if (!secondary)
				return -ENOMEM;
			secondary_data = secondary->plat_data->phy_data;

			list_for_each_entry(mode, &connector->probed_modes, head)
				hdmi->plat_data->convert_to_split_mode(mode);

			secondary->sink_is_hdmi = drm_detect_hdmi_monitor(edid);
			secondary->sink_has_audio = drm_detect_monitor_audio(edid);
			if (secondary->cec_notifier)
				cec_notifier_set_phys_addr_from_edid(secondary->cec_notifier,
								     edid);
			if (secondary->plat_data->get_edid_dsc_info)
				secondary->plat_data->get_edid_dsc_info(secondary_data, edid);
		}
		kfree(edid);
	} else {
		hdmi->sink_is_hdmi = true;
		hdmi->sink_has_audio = true;

		if (hdmi->plat_data->split_mode) {
			if (hdmi->plat_data->left) {
				hdmi->plat_data->left->sink_is_hdmi = true;
				hdmi->plat_data->left->sink_has_audio = true;
			} else if (hdmi->plat_data->right) {
				hdmi->plat_data->right->sink_is_hdmi = true;
				hdmi->plat_data->right->sink_has_audio = true;
			}
		}

		for (i = 0; i < ARRAY_SIZE(dw_hdmi_default_modes); i++) {
			const struct drm_display_mode *ptr =
				&dw_hdmi_default_modes[i];

			mode = drm_mode_duplicate(connector->dev, ptr);
			if (mode) {
				if (!i)
					mode->type = DRM_MODE_TYPE_PREFERRED;
				drm_mode_probed_add(connector, mode);
				ret++;
			}
		}
		if (ret > 0 && hdmi->plat_data->split_mode) {
			struct drm_display_mode *mode;
			printk("ztl test 111 mode = %d\n", mode->hdisplay);
			list_for_each_entry(mode, &connector->probed_modes, head)
				hdmi->plat_data->convert_to_split_mode(mode);
		}
		info->edid_hdmi_dc_modes = 0;
		info->hdmi.y420_dc_modes = 0;
		info->color_formats = 0;

		dev_info(hdmi->dev, "failed to get edid\n");
	}

	return ret;
}

static int
dw_hdmi_atomic_connector_set_property(struct drm_connector *connector,
				      struct drm_connector_state *state,
				      struct drm_property *property,
				      uint64_t val)
{
	struct dw_hdmi_qp *hdmi =
		container_of(connector, struct dw_hdmi_qp, connector);
	const struct dw_hdmi_property_ops *ops = hdmi->plat_data->property_ops;

	if (ops && ops->set_property)
		return ops->set_property(connector, state, property,
					 val, hdmi->plat_data->phy_data);
	else
		return -EINVAL;
}

static int
dw_hdmi_atomic_connector_get_property(struct drm_connector *connector,
				      const struct drm_connector_state *state,
				      struct drm_property *property,
				      uint64_t *val)
{
	struct dw_hdmi_qp *hdmi =
		container_of(connector, struct dw_hdmi_qp, connector);
	const struct dw_hdmi_property_ops *ops = hdmi->plat_data->property_ops;

	if (ops && ops->get_property)
		return ops->get_property(connector, state, property,
					 val, hdmi->plat_data->phy_data);
	else
		return -EINVAL;
}

static int
dw_hdmi_connector_set_property(struct drm_connector *connector,
			       struct drm_property *property, uint64_t val)
{
	return dw_hdmi_atomic_connector_set_property(connector, NULL,
						     property, val);
}

static void dw_hdmi_attach_properties(struct dw_hdmi_qp *hdmi)
{
	u64 color = MEDIA_BUS_FMT_YUV8_1X24;
	const struct dw_hdmi_property_ops *ops =
				hdmi->plat_data->property_ops;
	void *data = hdmi->plat_data->phy_data;
	enum drm_connector_status connect_status =
		hdmi->phy.ops->read_hpd(hdmi, hdmi->phy.data);

	if (hdmi->plat_data->get_grf_color_fmt &&
	    (connect_status == connector_status_connected) &&
	    hdmi->initialized)
		color = hdmi->plat_data->get_grf_color_fmt(data);

	if (ops && ops->attach_properties)
		return ops->attach_properties(&hdmi->connector, color, 0,
					      hdmi->plat_data->phy_data);
}

static void dw_hdmi_destroy_properties(struct dw_hdmi_qp *hdmi)
{
	const struct dw_hdmi_property_ops *ops =
				hdmi->plat_data->property_ops;

	if (ops && ops->destroy_properties)
		return ops->destroy_properties(&hdmi->connector,
					       hdmi->plat_data->phy_data);
}

static struct drm_encoder *
dw_hdmi_connector_best_encoder(struct drm_connector *connector)
{
	struct dw_hdmi_qp *hdmi =
		container_of(connector, struct dw_hdmi_qp, connector);

	return hdmi->bridge.encoder;
}

static bool dw_hdmi_color_changed(struct drm_connector *connector,
				  struct drm_atomic_state *state)
{
	struct dw_hdmi_qp *hdmi =
		container_of(connector, struct dw_hdmi_qp, connector);
	void *data = hdmi->plat_data->phy_data;
	struct drm_connector_state *old_state =
		drm_atomic_get_old_connector_state(state, connector);
	struct drm_connector_state *new_state =
		drm_atomic_get_new_connector_state(state, connector);
	bool ret = false;

	if (hdmi->plat_data->get_color_changed)
		ret = hdmi->plat_data->get_color_changed(data);

	if (new_state->colorspace != old_state->colorspace)
		ret = true;

	return ret;
}

static bool hdr_metadata_equal(const struct drm_connector_state *old_state,
			       const struct drm_connector_state *new_state)
{
	struct drm_property_blob *old_blob = old_state->hdr_output_metadata;
	struct drm_property_blob *new_blob = new_state->hdr_output_metadata;

	if (!old_blob || !new_blob)
		return old_blob == new_blob;

	if (old_blob->length != new_blob->length)
		return false;

	return !memcmp(old_blob->data, new_blob->data, old_blob->length);
}

static int dw_hdmi_connector_atomic_check(struct drm_connector *connector,
					  struct drm_atomic_state *state)
{
	struct drm_connector_state *old_state =
		drm_atomic_get_old_connector_state(state, connector);
	struct drm_connector_state *new_state =
		drm_atomic_get_new_connector_state(state, connector);
	struct drm_crtc *crtc = new_state->crtc;
	struct drm_crtc_state *crtc_state;
	struct dw_hdmi_qp *hdmi =
		container_of(connector, struct dw_hdmi_qp, connector);
	struct drm_display_mode *mode = NULL;
	void *data = hdmi->plat_data->phy_data;
	struct hdmi_vmode_qp *vmode = &hdmi->hdmi_data.video_mode;

	if (!crtc)
		return 0;

	crtc_state = drm_atomic_get_crtc_state(state, crtc);
	if (IS_ERR(crtc_state))
		return PTR_ERR(crtc_state);

	mode = &crtc_state->mode;
	/*
	 * If HDMI is enabled in uboot, it's need to record
	 * drm_display_mode and set phy status to enabled.
	 */
	if (!vmode->mpixelclock) {
		if (hdmi->plat_data->get_enc_in_encoding)
			hdmi->hdmi_data.enc_in_encoding =
				hdmi->plat_data->get_enc_in_encoding(data);
		if (hdmi->plat_data->get_enc_out_encoding)
			hdmi->hdmi_data.enc_out_encoding =
				hdmi->plat_data->get_enc_out_encoding(data);
		if (hdmi->plat_data->get_input_bus_format)
			hdmi->hdmi_data.enc_in_bus_format =
				hdmi->plat_data->get_input_bus_format(data);
		if (hdmi->plat_data->get_output_bus_format)
			hdmi->hdmi_data.enc_out_bus_format =
				hdmi->plat_data->get_output_bus_format(data);

		if (hdmi->plat_data->split_mode) {
			hdmi->plat_data->convert_to_origin_mode(mode);
			mode->crtc_clock /= 2;
		}
		memcpy(&hdmi->previous_mode, mode, sizeof(hdmi->previous_mode));
		vmode->mpixelclock = mode->crtc_clock * 1000;
		vmode->previous_pixelclock = mode->clock;
		vmode->previous_tmdsclock = mode->clock;
		vmode->mtmdsclock = hdmi_get_tmdsclock(hdmi,
						       vmode->mpixelclock);
		if (hdmi_bus_fmt_is_yuv420(hdmi->hdmi_data.enc_out_bus_format))
			vmode->mtmdsclock /= 2;

		/*
		 * If uboot logo enabled, atomic_enable won't be called,
		 * but atomic_disable will be called when hdmi plug out.
		 * That will cause dclk enable count is incorrect. So
		 * we should check ipi/link/video clk to determine whether
		 * uboot logo is enabled.
		 */
		if (hdmi->initialized && !hdmi->dclk_en) {
			mutex_lock(&hdmi->audio_mutex);
			if (hdmi->plat_data->dclk_set)
				hdmi->plat_data->dclk_set(data, true);
			hdmi->dclk_en = true;
			mutex_unlock(&hdmi->audio_mutex);
			hdmi->curr_conn = connector;
		}
	}

	if (!hdr_metadata_equal(old_state, new_state) ||
	    dw_hdmi_color_changed(connector, state)) {
		crtc_state = drm_atomic_get_crtc_state(state, crtc);
		if (IS_ERR(crtc_state))
			return PTR_ERR(crtc_state);

		crtc_state->mode_changed = true;
		if (mode->clock > 600000)
			hdmi->frl_switch = true;
	}

	return 0;
}

static void dw_hdmi_connector_force(struct drm_connector *connector)
{
	struct dw_hdmi_qp *hdmi =
		container_of(connector, struct dw_hdmi_qp, connector);

	mutex_lock(&hdmi->mutex);

	if (hdmi->force != connector->force) {
		if (!hdmi->disabled && connector->force == DRM_FORCE_OFF)
			extcon_set_state_sync(hdmi->extcon, EXTCON_DISP_HDMI,
					      false);
		else if (hdmi->disabled && connector->force == DRM_FORCE_ON)
			extcon_set_state_sync(hdmi->extcon, EXTCON_DISP_HDMI,
					      true);
	}

	hdmi->force = connector->force;
	mutex_unlock(&hdmi->mutex);
}

static int dw_hdmi_qp_fill_modes(struct drm_connector *connector, u32 max_x,
								 u32 max_y)
{
	return drm_helper_probe_single_connector_modes(connector, 9000, 9000);
}

static const struct drm_connector_funcs dw_hdmi_connector_funcs = {
	.fill_modes = dw_hdmi_qp_fill_modes,
	.detect = dw_hdmi_connector_detect,
	.destroy = drm_connector_cleanup,
	.force = dw_hdmi_connector_force,
	.reset = drm_atomic_helper_connector_reset,
	.set_property = dw_hdmi_connector_set_property,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
	.atomic_set_property = dw_hdmi_atomic_connector_set_property,
	.atomic_get_property = dw_hdmi_atomic_connector_get_property,
};

static const struct drm_connector_helper_funcs dw_hdmi_connector_helper_funcs = {
	.get_modes = dw_hdmi_connector_get_modes,
	.best_encoder = dw_hdmi_connector_best_encoder,
	.atomic_check = dw_hdmi_connector_atomic_check,
};

static int dw_hdmi_qp_bridge_attach(struct drm_bridge *bridge,
				    enum drm_bridge_attach_flags flags)
{
	struct dw_hdmi_qp *hdmi = bridge->driver_private;
	struct drm_encoder *encoder = bridge->encoder;
	struct drm_connector *connector = &hdmi->connector;
	struct cec_connector_info conn_info;
	struct cec_notifier *notifier;

	if (flags & DRM_BRIDGE_ATTACH_NO_CONNECTOR)
		return 0;

	connector->interlace_allowed = 1;
	connector->polled = DRM_CONNECTOR_POLL_HPD;

	drm_connector_helper_add(connector, &dw_hdmi_connector_helper_funcs);

	drm_connector_init(bridge->dev, connector, &dw_hdmi_connector_funcs,
			   DRM_MODE_CONNECTOR_HDMIA);

	drm_connector_attach_encoder(connector, encoder);
	dw_hdmi_attach_properties(hdmi);

	if (hdmi->cec_enable) {
		cec_fill_conn_info_from_drm(&conn_info, connector);
		notifier = cec_notifier_conn_register(hdmi->dev, NULL, &conn_info);
		if (!notifier)
			return -ENOMEM;

		mutex_lock(&hdmi->cec_notifier_mutex);
		hdmi->cec_notifier = notifier;
		mutex_unlock(&hdmi->cec_notifier_mutex);
	}

	return 0;
}

static void dw_hdmi_qp_bridge_detach(struct drm_bridge *bridge)
{
	struct dw_hdmi_qp *hdmi = bridge->driver_private;

	if (hdmi->cec_notifier) {
		mutex_lock(&hdmi->cec_notifier_mutex);
		cec_notifier_conn_unregister(hdmi->cec_notifier);
		hdmi->cec_notifier = NULL;
		mutex_unlock(&hdmi->cec_notifier_mutex);
	}
}

static enum drm_mode_status
dw_hdmi_qp_bridge_mode_valid(struct drm_bridge *bridge,
			     const struct drm_display_info *info,
			     const struct drm_display_mode *mode)
{
	return MODE_OK;
}

static void dw_hdmi_qp_bridge_mode_set(struct drm_bridge *bridge,
				       const struct drm_display_mode *orig_mode,
				       const struct drm_display_mode *mode)
{
	struct dw_hdmi_qp *hdmi = bridge->driver_private;

	mutex_lock(&hdmi->mutex);

	if (!drm_mode_equal(orig_mode, mode))
		hdmi->frl_switch = false;
	/* Store the display mode for plugin/DKMS poweron events */
	memcpy(&hdmi->previous_mode, mode, sizeof(hdmi->previous_mode));
	if (hdmi->plat_data->split_mode)
		hdmi->plat_data->convert_to_origin_mode(&hdmi->previous_mode);

	mutex_unlock(&hdmi->mutex);
}

static void dw_hdmi_qp_bridge_atomic_disable(struct drm_bridge *bridge,
					     struct drm_bridge_state *old_state)
{
	struct dw_hdmi_qp *hdmi = bridge->driver_private;
	void *data = hdmi->plat_data->phy_data;

	/* set avmute */
	hdmi_writel(hdmi, 1, PKTSCHED_PKT_CONTROL0);
	mdelay(50);

	extcon_set_state_sync(hdmi->extcon, EXTCON_DISP_HDMI, false);
	handle_plugged_change(hdmi, false);
	mutex_lock(&hdmi->mutex);

	if (hdmi->dclk_en) {
		mutex_lock(&hdmi->audio_mutex);
		if (hdmi->plat_data->dclk_set)
			hdmi->plat_data->dclk_set(data, false);
		hdmi->dclk_en = false;
		mutex_unlock(&hdmi->audio_mutex);
	};

	if (hdmi->phy.ops->disable && !hdmi->frl_switch) {
		hdmi_writel(hdmi, 0, FLT_CONFIG0);
		/* set sink frl mode disable */
		if (dw_hdmi_support_scdc(hdmi, &hdmi->curr_conn->display_info))
			drm_scdc_writeb(hdmi->ddc, 0x31, 0);

		hdmi->phy.ops->disable(hdmi, hdmi->phy.data);
		if (hdmi->plat_data->link_clk_set)
			hdmi->plat_data->link_clk_set(data, false);
	}

	hdmi->curr_conn = NULL;
	hdmi->disabled = true;
	mutex_unlock(&hdmi->mutex);
}

static void dw_hdmi_qp_bridge_atomic_enable(struct drm_bridge *bridge,
					    struct drm_bridge_state *old_state)
{
	struct dw_hdmi_qp *hdmi = bridge->driver_private;
	struct drm_atomic_state *state = old_state->base.state;
	struct drm_connector *connector;
	void *data = hdmi->plat_data->phy_data;

	connector = drm_atomic_get_new_connector_for_encoder(state,
							     bridge->encoder);

	mutex_lock(&hdmi->mutex);
	hdmi->curr_conn = connector;

	dw_hdmi_qp_setup(hdmi, hdmi->curr_conn, &hdmi->previous_mode);
	hdmi->disabled = false;
	mutex_unlock(&hdmi->mutex);

	if (!hdmi->dclk_en) {
		mutex_lock(&hdmi->audio_mutex);
		if (hdmi->plat_data->dclk_set)
			hdmi->plat_data->dclk_set(data, true);
		hdmi->dclk_en = true;
		mutex_unlock(&hdmi->audio_mutex);
	}

	extcon_set_state_sync(hdmi->extcon, EXTCON_DISP_HDMI, true);
	handle_plugged_change(hdmi, true);
}

static const struct drm_bridge_funcs dw_hdmi_bridge_funcs = {
	.atomic_duplicate_state = drm_atomic_helper_bridge_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_bridge_destroy_state,
	.atomic_reset = drm_atomic_helper_bridge_reset,
	.attach = dw_hdmi_qp_bridge_attach,
	.detach = dw_hdmi_qp_bridge_detach,
	.mode_set = dw_hdmi_qp_bridge_mode_set,
	.mode_valid = dw_hdmi_qp_bridge_mode_valid,
	.atomic_enable = dw_hdmi_qp_bridge_atomic_enable,
	.atomic_disable = dw_hdmi_qp_bridge_atomic_disable,
};

void dw_hdmi_qp_set_cec_adap(struct dw_hdmi_qp *hdmi, struct cec_adapter *adap)
{
	hdmi->cec_adap = adap;
}
EXPORT_SYMBOL_GPL(dw_hdmi_qp_set_cec_adap);

static irqreturn_t dw_hdmi_qp_main_hardirq(int irq, void *dev_id)
{
	struct dw_hdmi_qp *hdmi = dev_id;
	struct dw_hdmi_qp_i2c *i2c = hdmi->i2c;
	u32 stat;

	stat = hdmi_readl(hdmi, MAINUNIT_1_INT_STATUS);

	i2c->stat = stat & (I2CM_OP_DONE_IRQ | I2CM_READ_REQUEST_IRQ |
			    I2CM_NACK_RCVD_IRQ);
	hdmi->scdc_intr = stat & (SCDC_UPD_FLAGS_RD_IRQ |
				  SCDC_UPD_FLAGS_CHG_IRQ |
				  SCDC_UPD_FLAGS_CLR_IRQ |
				  SCDC_RR_REPLY_STOP_IRQ |
				  SCDC_NACK_RCVD_IRQ);
	hdmi->flt_intr = stat & (FLT_EXIT_TO_LTSP_IRQ |
				 FLT_EXIT_TO_LTS4_IRQ |
				 FLT_EXIT_TO_LTSL_IRQ);

	dev_dbg(hdmi->dev, "i2c main unit irq:%#x\n", stat);
	if (i2c->stat) {
		hdmi_writel(hdmi, i2c->stat, MAINUNIT_1_INT_CLEAR);
		complete(&i2c->cmp);
	}

	if (hdmi->flt_intr) {
		dev_dbg(hdmi->dev, "i2c flt irq:%#x\n", hdmi->flt_intr);
		hdmi_writel(hdmi, hdmi->flt_intr, MAINUNIT_1_INT_CLEAR);
		complete(&hdmi->flt_cmp);
	}

	if (hdmi->scdc_intr) {
		u8 val;

		dev_dbg(hdmi->dev, "i2c scdc irq:%#x\n", hdmi->scdc_intr);
		hdmi_writel(hdmi, hdmi->scdc_intr, MAINUNIT_1_INT_CLEAR);
		val = hdmi_readl(hdmi, SCDC_STATUS0);

		/* frl start */
		if (val & BIT(4)) {
			hdmi_modb(hdmi, 0, SCDC_UPD_FLAGS_POLL_EN |
				  SCDC_UPD_FLAGS_AUTO_CLR, SCDC_CONFIG0);
			hdmi_modb(hdmi, 0, SCDC_UPD_FLAGS_RD_IRQ,
				  MAINUNIT_1_INT_MASK_N);
			dev_info(hdmi->dev, "frl start\n");
		}

	}

	if (stat)
		return IRQ_HANDLED;

	return IRQ_NONE;
}

static irqreturn_t dw_hdmi_qp_avp_hardirq(int irq, void *dev_id)
{
	struct dw_hdmi_qp *hdmi = dev_id;
	u32 stat;

	stat = hdmi_readl(hdmi, AVP_1_INT_STATUS);
	if (stat) {
		dev_dbg(hdmi->dev, "HDCP irq %#x\n", stat);
		stat &= ~stat;
		hdmi_writel(hdmi, stat, AVP_1_INT_MASK_N);
		return IRQ_WAKE_THREAD;
	}

	return IRQ_NONE;
}

static irqreturn_t dw_hdmi_qp_earc_hardirq(int irq, void *dev_id)
{
	struct dw_hdmi_qp *hdmi = dev_id;
	u32 stat;

	stat = hdmi_readl(hdmi, EARCRX_0_INT_STATUS);
	if (stat) {
		dev_dbg(hdmi->dev, "earc irq %#x\n", stat);
		stat &= ~stat;
		hdmi_writel(hdmi, stat, EARCRX_0_INT_MASK_N);
		return IRQ_WAKE_THREAD;
	}

	return IRQ_NONE;
}

static irqreturn_t dw_hdmi_qp_avp_irq(int irq, void *dev_id)
{
	struct dw_hdmi_qp *hdmi = dev_id;
	u32 stat;

	stat = hdmi_readl(hdmi, AVP_1_INT_STATUS);

	if (!stat)
		return IRQ_NONE;

	hdmi_writel(hdmi, stat, AVP_1_INT_CLEAR);

	return IRQ_HANDLED;
}

static irqreturn_t dw_hdmi_qp_earc_irq(int irq, void *dev_id)
{
	struct dw_hdmi_qp *hdmi = dev_id;
	u32 stat;

	stat = hdmi_readl(hdmi, EARCRX_0_INT_STATUS);

	if (!stat)
		return IRQ_NONE;

	hdmi_writel(hdmi, stat, EARCRX_0_INT_CLEAR);

	hdmi->earc_intr = stat;
	complete(&hdmi->earc_cmp);

	return IRQ_HANDLED;
}

static int dw_hdmi_detect_phy(struct dw_hdmi_qp *hdmi)
{
	u8 phy_type;

	phy_type = hdmi->plat_data->phy_force_vendor ?
				DW_HDMI_PHY_VENDOR_PHY : 0;

	if (phy_type == DW_HDMI_PHY_VENDOR_PHY) {
		/* Vendor PHYs require support from the glue layer. */
		if (!hdmi->plat_data->qp_phy_ops || !hdmi->plat_data->phy_name) {
			dev_err(hdmi->dev,
				"Vendor HDMI PHY not supported by glue layer\n");
			return -ENODEV;
		}

		hdmi->phy.ops = hdmi->plat_data->qp_phy_ops;
		hdmi->phy.data = hdmi->plat_data->phy_data;
		hdmi->phy.name = hdmi->plat_data->phy_name;
	}

	return 0;
}

void dw_hdmi_qp_cec_set_hpd(struct dw_hdmi_qp *hdmi, bool plug_in, bool change)
{
	enum drm_connector_status status = plug_in ?
		connector_status_connected : connector_status_disconnected;

	if (!hdmi->cec_notifier)
		return;

	if (!plug_in)
		cec_notifier_set_phys_addr(hdmi->cec_notifier,
					   CEC_PHYS_ADDR_INVALID);

	if (hdmi->bridge.dev) {
		if (change && hdmi->cec_adap && hdmi->cec_adap->devnode.registered)
			cec_queue_pin_hpd_event(hdmi->cec_adap, plug_in, ktime_get());
		drm_bridge_hpd_notify(&hdmi->bridge, status);
	}
}
EXPORT_SYMBOL_GPL(dw_hdmi_qp_cec_set_hpd);

static void dw_hdmi_qp_cec_enable(struct dw_hdmi_qp *hdmi)
{
	mutex_lock(&hdmi->mutex);
	hdmi_modb(hdmi, 0, CEC_SWDISABLE, GLOBAL_SWDISABLE);
	mutex_unlock(&hdmi->mutex);
}

static void dw_hdmi_qp_cec_disable(struct dw_hdmi_qp *hdmi)
{
	mutex_lock(&hdmi->mutex);
	hdmi_modb(hdmi, CEC_SWDISABLE, CEC_SWDISABLE, GLOBAL_SWDISABLE);
	mutex_unlock(&hdmi->mutex);
}

static const struct dw_hdmi_qp_cec_ops dw_hdmi_qp_cec_ops = {
	.enable = dw_hdmi_qp_cec_enable,
	.disable = dw_hdmi_qp_cec_disable,
	.write = hdmi_writel,
	.read = hdmi_readl,
};

static const struct regmap_config hdmi_regmap_config = {
	.reg_bits	= 32,
	.val_bits	= 32,
	.reg_stride	= 4,
	.max_register	= EARCRX_1_INT_FORCE,
};

struct dw_hdmi_qp_reg_table {
	int reg_base;
	int reg_end;
};

static const struct dw_hdmi_qp_reg_table hdmi_reg_table[] = {
	{0x0, 0xc},
	{0x14, 0x1c},
	{0x44, 0x48},
	{0x50, 0x58},
	{0x80, 0x84},
	{0xa0, 0xc4},
	{0xe0, 0xe8},
	{0xf0, 0x118},
	{0x140, 0x140},
	{0x150, 0x150},
	{0x160, 0x168},
	{0x180, 0x180},
	{0x800, 0x800},
	{0x808, 0x808},
	{0x814, 0x814},
	{0x81c, 0x824},
	{0x834, 0x834},
	{0x840, 0x864},
	{0x86c, 0x86c},
	{0x880, 0x89c},
	{0x8e0, 0x8e8},
	{0x900, 0x900},
	{0x908, 0x90c},
	{0x920, 0x938},
	{0x920, 0x938},
	{0x960, 0x960},
	{0x968, 0x968},
	{0xa20, 0xa20},
	{0xa30, 0xa30},
	{0xa40, 0xa40},
	{0xa54, 0xa54},
	{0xa80, 0xaac},
	{0xab4, 0xab8},
	{0xb00, 0xcbc},
	{0xce0, 0xce0},
	{0xd00, 0xddc},
	{0xe20, 0xe24},
	{0xe40, 0xe44},
	{0xe4c, 0xe4c},
	{0xe60, 0xe80},
	{0xea0, 0xf24},
	{0x1004, 0x100c},
	{0x1020, 0x1030},
	{0x1040, 0x1050},
	{0x1060, 0x1068},
	{0x1800, 0x1820},
	{0x182c, 0x182c},
	{0x1840, 0x1940},
	{0x1960, 0x1a60},
	{0x1b00, 0x1b00},
	{0x1c00, 0x1c00},
	{0x3000, 0x3000},
	{0x3010, 0x3014},
	{0x3020, 0x3024},
	{0x3800, 0x3800},
	{0x3810, 0x3814},
	{0x3820, 0x3824},
	{0x3830, 0x3834},
	{0x3840, 0x3844},
	{0x3850, 0x3854},
	{0x3860, 0x3864},
	{0x3870, 0x3874},
	{0x4000, 0x4004},
	{0x4800, 0x4800},
	{0x4810, 0x4814},
};

static int dw_hdmi_ctrl_show(struct seq_file *s, void *v)
{
	struct dw_hdmi_qp *hdmi = s->private;
	u32 i = 0, j = 0, val = 0;

	seq_puts(s, "\n---------------------------------------------------");

	for (i = 0; i < ARRAY_SIZE(hdmi_reg_table); i++) {
		for (j = hdmi_reg_table[i].reg_base;
		     j <= hdmi_reg_table[i].reg_end; j += 4) {
			val = hdmi_readl(hdmi, j);

			if ((j - hdmi_reg_table[i].reg_base) % 16 == 0)
				seq_printf(s, "\n>>>hdmi_ctl %04x:", j);
			seq_printf(s, " %08x", val);
		}
	}
	seq_puts(s, "\n---------------------------------------------------\n");

	return 0;
}

static int dw_hdmi_ctrl_open(struct inode *inode, struct file *file)
{
	return single_open(file, dw_hdmi_ctrl_show, inode->i_private);
}

static ssize_t
dw_hdmi_ctrl_write(struct file *file, const char __user *buf,
		   size_t count, loff_t *ppos)
{
	struct dw_hdmi_qp *hdmi =
		((struct seq_file *)file->private_data)->private;
	u32 reg, val;
	char kbuf[25];

	if (count > 24) {
		dev_err(hdmi->dev, "out of buf range\n");
		return count;
	}

	if (copy_from_user(kbuf, buf, count))
		return -EFAULT;
	kbuf[count - 1] = '\0';

	if (sscanf(kbuf, "%x %x", &reg, &val) == -1)
		return -EFAULT;
	if (reg > EARCRX_1_INT_FORCE) {
		dev_err(hdmi->dev, "it is no a hdmi register\n");
		return count;
	}
	dev_info(hdmi->dev, "/**********hdmi register config******/");
	dev_info(hdmi->dev, "\n reg=%x val=%x\n", reg, val);
	hdmi_writel(hdmi, val, reg);
	return count;
}

static const struct file_operations dw_hdmi_ctrl_fops = {
	.owner = THIS_MODULE,
	.open = dw_hdmi_ctrl_open,
	.read = seq_read,
	.write = dw_hdmi_ctrl_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static int dw_hdmi_status_show(struct seq_file *s, void *v)
{
	struct dw_hdmi_qp *hdmi = s->private;
	u32 val;

	seq_puts(s, "PHY: ");
	if (hdmi->disabled) {
		seq_puts(s, "disabled\n");
		return 0;
	}
	seq_puts(s, "enabled\t\t\tMode: ");
	if (hdmi->sink_is_hdmi)
		seq_puts(s, "HDMI\n");
	else
		seq_puts(s, "DVI\n");

	if (hdmi->hdmi_data.video_mode.mpixelclock > 600000000) {
		seq_printf(s, "FRL Mode Pixel Clk: %luHz\n",
			   hdmi->hdmi_data.video_mode.mpixelclock);
	} else {
		if (hdmi->hdmi_data.video_mode.mtmdsclock > 340000000)
			val = hdmi->hdmi_data.video_mode.mtmdsclock / 4;
		else
			val = hdmi->hdmi_data.video_mode.mtmdsclock;
		seq_printf(s, "TMDS Mode Pixel Clk: %luHz\t\tTMDS Clk: %uHz\n",
			   hdmi->hdmi_data.video_mode.mpixelclock, val);
	}

	seq_puts(s, "Color Format: ");
	if (hdmi_bus_fmt_is_rgb(hdmi->hdmi_data.enc_out_bus_format))
		seq_puts(s, "RGB");
	else if (hdmi_bus_fmt_is_yuv444(hdmi->hdmi_data.enc_out_bus_format))
		seq_puts(s, "YUV444");
	else if (hdmi_bus_fmt_is_yuv422(hdmi->hdmi_data.enc_out_bus_format))
		seq_puts(s, "YUV422");
	else if (hdmi_bus_fmt_is_yuv420(hdmi->hdmi_data.enc_out_bus_format))
		seq_puts(s, "YUV420");
	else
		seq_puts(s, "UNKNOWN");
	val =  hdmi_bus_fmt_color_depth(hdmi->hdmi_data.enc_out_bus_format);
	seq_printf(s, "\t\tColor Depth: %d bit\n", val);
	seq_puts(s, "Colorimetry: ");
	switch (hdmi->hdmi_data.enc_out_encoding) {
	case V4L2_YCBCR_ENC_601:
		seq_puts(s, "ITU.BT601");
		break;
	case V4L2_YCBCR_ENC_709:
		seq_puts(s, "ITU.BT709");
		break;
	case V4L2_YCBCR_ENC_BT2020:
		seq_puts(s, "ITU.BT2020");
		break;
	default: /* Carries no data */
		seq_puts(s, "ITU.BT601");
		break;
	}

	seq_puts(s, "\t\tEOTF: ");

	val = hdmi_readl(hdmi, PKTSCHED_PKT_EN);
	if (!(val & PKTSCHED_DRMI_TX_EN)) {
		seq_puts(s, "Off\n");
		return 0;
	}

	val = hdmi_readl(hdmi, PKT_DRMI_CONTENTS1);
	val = (val >> 8) & 0x7;
	switch (val) {
	case HDMI_EOTF_TRADITIONAL_GAMMA_SDR:
		seq_puts(s, "SDR");
		break;
	case HDMI_EOTF_TRADITIONAL_GAMMA_HDR:
		seq_puts(s, "HDR");
		break;
	case HDMI_EOTF_SMPTE_ST2084:
		seq_puts(s, "ST2084");
		break;
	case HDMI_EOTF_BT_2100_HLG:
		seq_puts(s, "HLG");
		break;
	default:
		seq_puts(s, "Not Defined\n");
		return 0;
	}

	val = hdmi_readl(hdmi, PKT_DRMI_CONTENTS1);
	val = (val >> 16) & 0xffff;
	seq_printf(s, "\nx0: %d", val);
	val = hdmi_readl(hdmi, PKT_DRMI_CONTENTS2);
	val = val & 0xffff;
	seq_printf(s, "\t\t\t\ty0: %d\n", val);
	val = hdmi_readl(hdmi, PKT_DRMI_CONTENTS2);
	val = (val >> 16) & 0xffff;
	seq_printf(s, "x1: %d", val);
	val = hdmi_readl(hdmi, PKT_DRMI_CONTENTS3);
	val = val & 0xffff;
	seq_printf(s, "\t\t\t\ty1: %d\n", val);
	val = hdmi_readl(hdmi, PKT_DRMI_CONTENTS3);
	val = (val >> 16) & 0xffff;
	seq_printf(s, "x2: %d", val);
	val = hdmi_readl(hdmi, PKT_DRMI_CONTENTS4);
	val = val & 0xffff;
	seq_printf(s, "\t\t\t\ty2: %d\n", val);
	val = hdmi_readl(hdmi, PKT_DRMI_CONTENTS4);
	val = (val >> 16) & 0xffff;
	seq_printf(s, "white x: %d", val);
	val = hdmi_readl(hdmi, PKT_DRMI_CONTENTS5);
	val = val & 0xffff;
	seq_printf(s, "\t\t\twhite y: %d\n", val);
	val = hdmi_readl(hdmi, PKT_DRMI_CONTENTS5);
	val = (val >> 16) & 0xffff;
	seq_printf(s, "max lum: %d", val);
	val = hdmi_readl(hdmi, PKT_DRMI_CONTENTS6);
	val = val & 0xffff;
	seq_printf(s, "\t\t\tmin lum: %d\n", val);
	val = hdmi_readl(hdmi, PKT_DRMI_CONTENTS6);
	val = (val >> 16) & 0xffff;
	seq_printf(s, "max cll: %d", val);
	val = hdmi_readl(hdmi, PKT_DRMI_CONTENTS7);
	val = val & 0xffff;
	seq_printf(s, "\t\t\tmax fall: %d\n", val);
	return 0;
}

static int dw_hdmi_status_open(struct inode *inode, struct file *file)
{
	return single_open(file, dw_hdmi_status_show, inode->i_private);
}

static const struct file_operations dw_hdmi_status_fops = {
	.owner = THIS_MODULE,
	.open = dw_hdmi_status_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static void dw_hdmi_register_debugfs(struct device *dev, struct dw_hdmi_qp *hdmi)
{
	u8 buf[11];

	snprintf(buf, sizeof(buf), "dw-hdmi%d", hdmi->plat_data->id);
	hdmi->debugfs_dir = debugfs_create_dir(buf, NULL);
	if (IS_ERR(hdmi->debugfs_dir)) {
		dev_err(dev, "failed to create debugfs dir!\n");
		return;
	}

	debugfs_create_file("status", 0400, hdmi->debugfs_dir,
			    hdmi, &dw_hdmi_status_fops);
	debugfs_create_file("ctrl", 0600, hdmi->debugfs_dir,
			    hdmi, &dw_hdmi_ctrl_fops);
}

static struct dw_hdmi_qp *
__dw_hdmi_probe(struct platform_device *pdev,
		const struct dw_hdmi_plat_data *plat_data)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct device_node *ddc_node;
	struct dw_hdmi_qp *hdmi;
	struct dw_hdmi_qp_i2s_audio_data audio;
	struct platform_device_info pdevinfo;
	struct dw_hdmi_qp_cec_data cec;
	struct resource *iores = NULL;
	int irq;
	int ret;

	hdmi = devm_kzalloc(dev, sizeof(*hdmi), GFP_KERNEL);
	if (!hdmi)
		return ERR_PTR(-ENOMEM);

	hdmi->connector.stereo_allowed = 1;
	hdmi->plat_data = plat_data;
	hdmi->dev = dev;
	hdmi->sample_rate = 48000;
	hdmi->disabled = true;

	mutex_init(&hdmi->mutex);
	mutex_init(&hdmi->audio_mutex);
	mutex_init(&hdmi->cec_notifier_mutex);

	ddc_node = of_parse_phandle(np, "ddc-i2c-bus", 0);
	if (ddc_node) {
		hdmi->ddc = of_get_i2c_adapter_by_node(ddc_node);
		of_node_put(ddc_node);
		if (!hdmi->ddc) {
			dev_dbg(hdmi->dev, "failed to read ddc node\n");
			return ERR_PTR(-EPROBE_DEFER);
		}

	} else {
		dev_dbg(hdmi->dev, "no ddc property found\n");
	}

	if (!plat_data->regm) {
		const struct regmap_config *reg_config;

		reg_config = &hdmi_regmap_config;

		iores = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		hdmi->regs = devm_ioremap_resource(dev, iores);
		if (IS_ERR(hdmi->regs)) {
			ret = PTR_ERR(hdmi->regs);
			goto err_res;
		}

		hdmi->regm = devm_regmap_init_mmio(dev, hdmi->regs, reg_config);
		if (IS_ERR(hdmi->regm)) {
			dev_err(dev, "Failed to configure regmap\n");
			ret = PTR_ERR(hdmi->regm);
			goto err_res;
		}
	} else {
		hdmi->regm = plat_data->regm;
	}

	ret = dw_hdmi_detect_phy(hdmi);
	if (ret < 0)
		goto err_res;

	hdmi_writel(hdmi, 0, MAINUNIT_0_INT_MASK_N);
	hdmi_writel(hdmi, 0, MAINUNIT_1_INT_MASK_N);
	hdmi_writel(hdmi, 428571429, TIMER_BASE_CONFIG0);
	if ((hdmi_readl(hdmi, CMU_STATUS) & DISPLAY_CLK_MONITOR) == DISPLAY_CLK_LOCKED) {
		hdmi->initialized = true;
		hdmi->disabled = false;
	}

	hdmi->sink_is_hdmi = true;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		ret = irq;
		goto err_res;
	}

	hdmi->avp_irq = irq;
	ret = devm_request_threaded_irq(dev, hdmi->avp_irq,
					dw_hdmi_qp_avp_hardirq,
					dw_hdmi_qp_avp_irq, IRQF_SHARED,
					dev_name(dev), hdmi);
	if (ret)
		goto err_res;

	irq = platform_get_irq(pdev, 1);
	if (irq < 0) {
		ret = irq;
		goto err_res;
	}

	cec.irq = irq;

	irq = platform_get_irq(pdev, 2);
	if (irq < 0) {
		ret = irq;
		goto err_res;
	}

	hdmi->earc_irq = irq;
	ret = devm_request_threaded_irq(dev, hdmi->earc_irq,
					dw_hdmi_qp_earc_hardirq,
					dw_hdmi_qp_earc_irq, IRQF_SHARED,
					dev_name(dev), hdmi);
	if (ret)
		goto err_res;

	irq = platform_get_irq(pdev, 3);
	if (irq < 0) {
		ret = irq;
		goto err_res;
	}

	hdmi->main_irq = irq;
	ret = devm_request_threaded_irq(dev, hdmi->main_irq,
					dw_hdmi_qp_main_hardirq, NULL,
					IRQF_SHARED, dev_name(dev), hdmi);
	if (ret)
		goto err_res;

	hdmi_init_clk_regenerator(hdmi);

	/* If DDC bus is not specified, try to register HDMI I2C bus */
	if (!hdmi->ddc) {
		hdmi->ddc = dw_hdmi_i2c_adapter(hdmi);
		if (IS_ERR(hdmi->ddc))
			hdmi->ddc = NULL;
		/*
		 * Read high and low time from device tree. If not available use
		 * the default timing scl clock rate is about 99.6KHz.
		 */
		if (of_property_read_u32(np, "ddc-i2c-scl-high-time-ns",
					 &hdmi->i2c->scl_high_ns))
			hdmi->i2c->scl_high_ns = 4708;
		if (of_property_read_u32(np, "ddc-i2c-scl-low-time-ns",
					 &hdmi->i2c->scl_low_ns))
			hdmi->i2c->scl_low_ns = 4916;
	}

	hdmi->bridge.driver_private = hdmi;
	hdmi->bridge.funcs = &dw_hdmi_bridge_funcs;
#ifdef CONFIG_OF
	hdmi->bridge.of_node = pdev->dev.of_node;
#endif

	if (hdmi->phy.ops->setup_hpd)
		hdmi->phy.ops->setup_hpd(hdmi, hdmi->phy.data);

	hdmi->connector.ycbcr_420_allowed = hdmi->plat_data->ycbcr_420_allowed;

	audio.hdmi	= hdmi;
	audio.eld	= hdmi->connector.eld;
	audio.write	= hdmi_writel;
	audio.read	= hdmi_readl;
	audio.mod	= hdmi_modb;
	hdmi->enable_audio = dw_hdmi_i2s_audio_enable;
	hdmi->disable_audio = dw_hdmi_i2s_audio_disable;

	memset(&pdevinfo, 0, sizeof(pdevinfo));
	pdevinfo.parent = dev;
	pdevinfo.id = PLATFORM_DEVID_AUTO;
	pdevinfo.name = "dw-hdmi-qp-i2s-audio";
	pdevinfo.data = &audio;
	pdevinfo.size_data = sizeof(audio);
	pdevinfo.dma_mask = DMA_BIT_MASK(32);
	hdmi->audio = platform_device_register_full(&pdevinfo);

	hdmi->extcon = devm_extcon_dev_allocate(hdmi->dev, dw_hdmi_cable);
	if (IS_ERR(hdmi->extcon)) {
		dev_err(hdmi->dev, "allocate extcon failed\n");
		ret = PTR_ERR(hdmi->extcon);
		goto err_res;
	}

	ret = devm_extcon_dev_register(hdmi->dev, hdmi->extcon);
	if (ret) {
		dev_err(hdmi->dev, "failed to register extcon: %d\n", ret);
		goto err_res;
	}

	ret = extcon_set_property_capability(hdmi->extcon, EXTCON_DISP_HDMI,
					     EXTCON_PROP_DISP_HPD);
	if (ret) {
		dev_err(hdmi->dev,
			"failed to set USB property capability: %d\n", ret);
		goto err_res;
	}

	if (of_property_read_bool(np, "cec-enable")) {
		hdmi->cec_enable = true;
		cec.hdmi = hdmi;
		cec.ops = &dw_hdmi_qp_cec_ops;
		pdevinfo.name = "dw-hdmi-qp-cec";
		pdevinfo.data = &cec;
		pdevinfo.size_data = sizeof(cec);
		pdevinfo.dma_mask = 0;
		hdmi->cec = platform_device_register_full(&pdevinfo);
	}

	/* Reset HDMI DDC I2C master controller and mute I2CM interrupts */
	if (hdmi->i2c)
		dw_hdmi_i2c_init(hdmi);

	init_completion(&hdmi->flt_cmp);
	init_completion(&hdmi->earc_cmp);

	if (of_property_read_bool(np, "scramble-low-rates"))
		hdmi->scramble_low_rates = true;

	dw_hdmi_register_debugfs(dev, hdmi);

	return hdmi;

err_res:
	if (hdmi->i2c)
		i2c_del_adapter(&hdmi->i2c->adap);
	else
		i2c_put_adapter(hdmi->ddc);

	return ERR_PTR(ret);
}

static void __dw_hdmi_remove(struct dw_hdmi_qp *hdmi)
{
	if (hdmi->avp_irq)
		disable_irq(hdmi->avp_irq);

	if (hdmi->main_irq)
		disable_irq(hdmi->main_irq);

	if (hdmi->earc_irq)
		disable_irq(hdmi->earc_irq);

	debugfs_remove_recursive(hdmi->debugfs_dir);

	if (!hdmi->plat_data->first_screen) {
		dw_hdmi_destroy_properties(hdmi);
		hdmi->connector.funcs->destroy(&hdmi->connector);
	}

	if (hdmi->audio && !IS_ERR(hdmi->audio))
		platform_device_unregister(hdmi->audio);

	if (hdmi->bridge.encoder && !hdmi->plat_data->first_screen)
		hdmi->bridge.encoder->funcs->destroy(hdmi->bridge.encoder);
	if (!IS_ERR(hdmi->cec))
		platform_device_unregister(hdmi->cec);
	if (hdmi->i2c)
		i2c_del_adapter(&hdmi->i2c->adap);
	else
		i2c_put_adapter(hdmi->ddc);
}

/* -----------------------------------------------------------------------------
 * Bind/unbind API, used from platforms based on the component framework.
 */
struct dw_hdmi_qp *dw_hdmi_qp_bind(struct platform_device *pdev,
				   struct drm_encoder *encoder,
				   struct dw_hdmi_plat_data *plat_data)
{
	struct dw_hdmi_qp *hdmi;
	int ret;

	hdmi = __dw_hdmi_probe(pdev, plat_data);
	if (IS_ERR(hdmi))
		return hdmi;

	if (!plat_data->first_screen) {
		ret = drm_bridge_attach(encoder, &hdmi->bridge, NULL, 0);
		if (ret) {
			__dw_hdmi_remove(hdmi);
			dev_err(hdmi->dev, "Failed to initialize bridge with drm\n");
			return ERR_PTR(ret);
		}

		plat_data->connector = &hdmi->connector;
	}

	if (plat_data->split_mode && !hdmi->plat_data->first_screen) {
		struct dw_hdmi_qp *secondary = NULL;

		if (hdmi->plat_data->left)
			secondary = hdmi->plat_data->left;
		else if (hdmi->plat_data->right)
			secondary = hdmi->plat_data->right;

		if (!secondary)
			return ERR_PTR(-ENOMEM);
		ret = drm_bridge_attach(encoder, &secondary->bridge, &hdmi->bridge,
					DRM_BRIDGE_ATTACH_NO_CONNECTOR);
		if (ret)
			return ERR_PTR(ret);
	}

	return hdmi;
}
EXPORT_SYMBOL_GPL(dw_hdmi_qp_bind);

void dw_hdmi_qp_unbind(struct dw_hdmi_qp *hdmi)
{
	__dw_hdmi_remove(hdmi);
}
EXPORT_SYMBOL_GPL(dw_hdmi_qp_unbind);

void dw_hdmi_qp_suspend(struct device *dev, struct dw_hdmi_qp *hdmi)
{
	if (!hdmi) {
		dev_warn(dev, "Hdmi has not been initialized\n");
		return;
	}

	mutex_lock(&hdmi->mutex);

	/*
	 * When system shutdown, hdmi should be disabled.
	 * When system suspend, dw_hdmi_qp_bridge_disable will disable hdmi first.
	 * To prevent duplicate operation, we should determine whether hdmi
	 * has been disabled.
	 */
	if (!hdmi->disabled)
		hdmi->disabled = true;
	mutex_unlock(&hdmi->mutex);

	if (hdmi->avp_irq)
		disable_irq(hdmi->avp_irq);

	if (hdmi->main_irq)
		disable_irq(hdmi->main_irq);

	if (hdmi->earc_irq)
		disable_irq(hdmi->earc_irq);

	pinctrl_pm_select_sleep_state(dev);
}
EXPORT_SYMBOL_GPL(dw_hdmi_qp_suspend);

void dw_hdmi_qp_resume(struct device *dev, struct dw_hdmi_qp *hdmi)
{
	if (!hdmi) {
		dev_warn(dev, "Hdmi has not been initialized\n");
		return;
	}

	hdmi_writel(hdmi, 0, MAINUNIT_0_INT_MASK_N);
	hdmi_writel(hdmi, 0, MAINUNIT_1_INT_MASK_N);
	hdmi_writel(hdmi, 428571429, TIMER_BASE_CONFIG0);

	pinctrl_pm_select_default_state(dev);

	if (hdmi->cec_adap)
		hdmi->cec_adap->ops->adap_enable(hdmi->cec_adap, true);

	mutex_lock(&hdmi->mutex);
	if (hdmi->i2c)
		dw_hdmi_i2c_init(hdmi);
	if (hdmi->avp_irq)
		enable_irq(hdmi->avp_irq);

	if (hdmi->main_irq)
		enable_irq(hdmi->main_irq);

	if (hdmi->earc_irq)
		enable_irq(hdmi->earc_irq);

	mutex_unlock(&hdmi->mutex);
}
EXPORT_SYMBOL_GPL(dw_hdmi_qp_resume);

MODULE_AUTHOR("Algea Cao <algea.cao@rock-chips.com>");
MODULE_DESCRIPTION("DW HDMI QP transmitter driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:dw-hdmi-qp");
