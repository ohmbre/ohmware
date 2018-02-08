/*
 * DRM driver for Touchscreen TFTs with Ft8xx EVE chipsets
 *
 * Copyright 2017 Matt Gattis
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include <linux/kernel.h>
#include <linux/ktime.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/spi/spi.h>
#include <linux/of_device.h>
#include <linux/input/touchscreen.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/dma-mapping.h>
#include <linux/debugfs.h>
#include <linux/hrtimer.h>
//include <linux/zlib.h>

#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_modes.h>
#include <drm/drm_vblank.h>
#include <drm/tinydrm/tinydrm.h>
#include <drm/tinydrm/tinydrm-helpers.h>

u8 ohm_logo[] = {
#include "ohm_logo.h"
};


#define SETUP_SPI_SPEED 10000000 /* Hz */
//define MAX_SPI_SPEED 75000000 /* Hz */
#define MAX_SPI_SPEED 110000000 /* Hz */

#define W 320UL
#define H 240UL
#define WSTR "320"
#define HSTR "240"
#define WMM 70
#define HMM 52
#define NBUFFERS 1
#define CLKSYNC_SAMPLES 50

#define PREFER_FORMAT DRM_FORMAT_RGB888
#define PREFER_DEPTH 24

#define HW_PCLK 12

#define HW_HCYCLE 618UL // recommended 408 plus extra we need for xrbg888 gpu drawing
#define HW_HOFFSET 70UL
#define HW_HSYNC0 0
#define HW_HSYNC1 10UL

#define HW_VCYCLE 263UL
#define HW_VOFFSET 13UL
#define HW_VSYNC0 0
#define HW_VSYNC1 2UL

//define HW_VBLANK_NS 16253400UL
#define ROTATION 0UL
#define BRIGHTNESS 55

#define KILOS(b) (1024U*b)
#define MEGAS(b) (1024U*KILOS(b))
#define MAX_BPP 4UL
#define SPI_MAX_XFER_SIZE (KILOS(48))

#define INT_DIV_UP(numer,denom) (((numer) + (denom) - 1) / (denom))
#define SPI_MAX_XFERS (INT_DIV_UP(MAX_BPP*W*H, SPI_MAX_XFER_SIZE)+1)

#define CHIP_CONST 0x0C0000UL
#define CHIP_ID 0x0C0001UL
#define REG_ID_VAL 0x7C

#define INT_CONVCOMPLETE 0x80UL
#define INT_SWAP         0x01UL

#define CMD_CALIBRATE		0xFFFFFF15UL
#define CMD_DLSTART           0xFFFFFF00UL
#define CMD_SWAP              0xFFFFFF01UL
#define CMD_MEMZERO           0xFFFFFF1CUL
#define CMD_INFLATE           0xFFFFFF22UL
#define CMD_SETROTATE		0xFFFFFF36UL

#define BORDER 0
#define NEAREST 0
#define BILINEAR 1
#define BITMAPS 1
#define DLSWAP_FRAME 2
#define ARGB1555 0
#define RGB565 7
#define L8 3
#define ONE 1
#define ZERO 0
#define DST_ALPHA 3
#define ONE_MINUS_DST_ALPHA 5

#define RESET 0x68
#define ACTIVE 0
#define CLKINT 0x48
#define CLKEXT 0x44
#define CLK48M 0x62
#define CLKSEL 0x62
#define PWRDOWN 0x43
#define SLEEP 0x42

#define CHIP_ID_813 0x13

#define RAM_G                   0x000000UL
#define RAM_DL                  0x300000UL
#define RAM_REG                 0x302000UL
#define RAM_CMD                 0x308000UL

#define RAM(x) (RAM_ ## x)
#define XSTR(x) STR(x)
#define STR(x) #x

// name              offset #bits r? w? default(por)
#define REGS(_) \
_(ID                  ,0x00  ,8  ,1 ,0 ,0x7CUL) \
_(FRAMES              ,0x04  ,32 ,1 ,0 ,0) \
_(CLOCK               ,0x08  ,32 ,1 ,0 ,0) \
_(FREQUENCY           ,0x0C  ,28 ,1 ,1 ,0x3938700UL)	\
_(HCYCLE              ,0x2C  ,12 ,1 ,1 ,0x224UL) \
_(HOFFSET             ,0x30  ,12 ,1 ,1 ,0x2BUL) \
_(HSIZE               ,0x34  ,12 ,1 ,1 ,0x1E0UL)    \
_(HSYNC0              ,0x38  ,12 ,1 ,1 ,0) \
_(HSYNC1              ,0x3C  ,12 ,1 ,1 ,0x29UL) \
_(VCYCLE              ,0x40  ,12 ,1 ,1 ,0x124UL) \
_(VOFFSET             ,0x44  ,12 ,1 ,1 ,0xCUL) \
_(VSIZE               ,0x48  ,12 ,1 ,1 ,0x110UL) \
_(VSYNC0              ,0x4C  ,10 ,1 ,1 ,0) \
_(VSYNC1              ,0x50  ,10 ,1 ,1 ,0xAUL) \
_(DLSWAP              ,0x54  ,2  ,1 ,1 ,0)  \
_(ROTATE              ,0x58  ,3  ,1 ,1 ,0)  \
_(DITHER              ,0x60  ,1  ,1 ,1 ,1)  \
_(SWIZZLE             ,0x64  ,4  ,1 ,1 ,0)  \
_(CSPREAD             ,0x68  ,1  ,1 ,1 ,1)  \
_(PCLK_POL            ,0x6C  ,1  ,1 ,1 ,0)  \
_(PCLK                ,0x70  ,8  ,1 ,1 ,0)  \
_(GPIO_DIR            ,0x90  ,8  ,1 ,1 ,0x80UL)  \
_(GPIO                ,0x94  ,8  ,1 ,1 ,0)  \
_(INT_FLAGS           ,0xA8  ,8  ,1 ,0 ,0)  \
_(INT_EN              ,0xAC  ,1  ,1 ,1 ,0)  \
_(INT_MASK            ,0xB0  ,8  ,1 ,1 ,0xFFUL)  \
_(PWM_HZ              ,0xD0  ,14 ,1 ,1 ,250UL) \
_(PWM_DUTY            ,0xD4  ,8  ,1 ,1 ,128UL)  \
_(CMD_READ            ,0xF8  ,12 ,1 ,1 ,0) \
_(CMD_WRITE           ,0xFC  ,12 ,1 ,0 ,0) \
_(CTOUCH_MODE         ,0x104 ,2  ,1 ,1 ,3)  \
_(CTOUCH_EXTENDED     ,0x108 ,1  ,1 ,1 ,1)  \
_(CTOUCH_TOUCH1_XY    ,0x11C ,32 ,1 ,0 ,0) \
_(CTOUCH_TOUCH0_XY    ,0x124 ,32 ,1 ,0 ,0) \
_(CTOUCH_TRANSFORM_A  ,0x150 ,32 ,1 ,1 ,0x10000UL) \
_(CTOUCH_TRANSFORM_B  ,0x154 ,32 ,1 ,1 ,0) \
_(CTOUCH_TRANSFORM_C  ,0x158 ,32 ,1 ,1 ,0) \
_(CTOUCH_TRANSFORM_D  ,0x15C ,32 ,1 ,1 ,0) \
_(CTOUCH_TRANSFORM_E  ,0x160 ,32 ,1 ,1 ,0x10000UL) \
_(CTOUCH_TRANSFORM_F  ,0x164 ,32 ,1 ,1 ,0) \
_(TRIM                ,0x180 ,8  ,1 ,1 ,0)  \
_(CMDB_SPACE          ,0x574 ,12 ,1 ,1 ,0xFFCUL) \
_(CMDB_WRITE          ,0x578 ,32 ,0 ,1 ,0) \

#define REG_IDS(id, offset, nbits, readable, writable, default) id,
enum {REGS(REG_IDS) NREGS};
struct ft8reg {
	char name[30];
	u16 offset;
	u8 nbits;
	u8 readable;
	u8 writable;
	u32 defval;
};
#define REG_DATA(id, offset, nbits, readable, writable, defval) {XSTR(id), offset, nbits, readable, writable, defval},
static const struct ft8reg ft8regs[NREGS] = {REGS(REG_DATA)};
#define WREG(id,val) ft8reg_write(ft8, id, val)
#define RREG(id) ft8reg_read(ft8, id)

#define REGMASK(val,nbits) (((u32)(val))&((2UL<<((nbits)-1))-1))
#define ADDR_READ(addr) ((((addr)&0xFFUL)<<16)|((addr)&0xFF00UL)|(((addr)&0x3F0000UL)>>16))
#define ADDR_WRITE(addr) ((((addr)&0xFFUL)<<16)|((addr)&0xFF00UL)|(((addr)&0x3F0000UL)>>16)|0x80UL)

#define DLMASK(i,hibit,lobit) (((u32)(i))&((2UL<<(hibit-lobit))-1))
#define DLSHIFT(i,lobit) (((u32)(i))<<lobit)
#define DLC(cmd,args) (DLSHIFT(cmd,24)|args)
#define DLA(arg,hibit,lobit) DLSHIFT(DLMASK(arg,hibit,lobit),lobit)

#define BEGIN(prim) DLC(0x1F,DLA(prim,3,0))
#define BITMAP_SIZE(filter,xr,yr,w,h) DLC(0x08,DLA(filter,20,20)|DLA(xr,19,19)|DLA(yr,18,18)|DLA(w,17,9)|DLA(h,8,0))
#define BITMAP_LAYOUT(format,linestride,height) DLC(0x07,DLA(format,23,19)|DLA(linestride,18,9)|DLA(height,8,0))
#define BITMAP_LAYOUT_H(linestride,height) DLC(0x28,DLA(linestride,3,2)|DLA(height,1,0))
#define BITMAP_HANDLE(handle) DLC(0x05,DLA(handle,4,0))
#define BITMAP_SOURCE(addr) DLC(0x01,DLA(addr,21,0))
#define BITMAP_TRANSFORM_A(a) DLC(0x15,DLA(a,16,0))
#define BITMAP_TRANSFORM_B(b) DLC(0x16,DLA(b,16,0))
#define BITMAP_TRANSFORM_C(c) DLC(0x17,DLA(c,23,0))
#define BITMAP_TRANSFORM_D(d) DLC(0x18,DLA(d,16,0))
#define BITMAP_TRANSFORM_E(e) DLC(0x19,DLA(e,16,0))
#define BITMAP_TRANSFORM_F(f) DLC(0x1A,DLA(f,23,0))
#define BLEND_FUNC(src,dst) DLC(0x0B,DLA(src,5,3)|DLA(dst,2,0))
#define COLOR_A(alpha) DLC(0x10,DLA(alpha,7,0))
#define COLOR_MASK(r,g,b,a) DLC(0x20,DLA(r,3,3)|DLA(g,2,2)|DLA(b,1,1)|DLA(a,0,0))
#define COLOR_RGB(r,g,b) DLC(0x04,DLA(r,23,16)|DLA(b,15,8)|DLA(g,7,0))
#define CLEAR(c,s,t)  DLC(0x26,DLA(c,2,2)|DLA(s,1,1)|DLA(t,0,0))
#define CLEAR_COLOR_A(alpha) DLC(0x0F,DLA(alpha,7,0))
#define CLEAR_COLOR_RGB(r,g,b) DLC(0x02,DLA(r,23,16)|DLA(b,15,8)|DLA(g,7,0))
#define DISPLAY() DLC(0x0,0)
#define END() DLC(0x21,0)
#define VERTEX2II(x,y,handle,cell) (DLA(0x02,31,30)|DLA(x,29,21)|DLA(y,20,12)|DLA(handle,11,7)|DLA(cell,6,0))
#define VERTEX_TRANSLATE_X(x) DLC(0x2B,DLA(x,16,0))

#define DL(cmd) ft8->dl[o++] = cmd;

#define STATS(_) \
	_(UPDATES)	\
	_(DIRTIES)	\
	_(UPDIRTIES)    \
	_(SWAPS)	\
	_(WAITS)	\
	_(DLWRITES)	\
	_(SPURINT)	\
	_(IGNINT)	\
	_(NFRAMES)	\
	_(NBLANKS)	\
	_(ENABLED)	\
	_(DISABLED)

#define STAT_IDS(id) id,
enum {STATS(STAT_IDS) NSTATS};
#define STAT_STRINGS(id) XSTR(id),
static const char stat_strings[30][NSTATS] = {STATS(STAT_STRINGS)};

struct ft8device {
	struct tinydrm_device tinydrm;
	struct spi_device *spi;
	struct input_dev *input_emu;
	struct gpio_desc *interrupt_gpio;
	struct mutex dirty_lock;
	int irq;
	bool enabled;
	u8 chip_id;
	struct spi_transfer xfers[SPI_MAX_XFERS];
	uint32_t format;

	//u32 drawbuf;
	//u32 frontbuf;

	struct hrtimer vblank_timer;
	ktime_t vblank_interval;

	u8 bpp;
	u32 dl[8196];
	s64 stats[NSTATS];
	//u8 *zx_buf;
	//z_stream zstream
};


u32 ft8mem_read(struct ft8device *ft8, u32 base, u32 offset) {
	u32 result = 0;
	u32 fmt_addr = ADDR_READ(base+offset);
	spi_write_then_read(ft8->spi, &fmt_addr, 4, &result, 4);
	return result;
}

int ft8mem_write(struct ft8device *ft8, u32 base, u32 offset, u32 val) {
	u8 buf[7];
	*(u32*)buf = ADDR_WRITE(base+offset);
	*(u32*)(buf+3) = val;
	return spi_write(ft8->spi, buf, 7);
}

int ft8mem_write_bulk(struct ft8device *ft8, u32 base, u32 offset, u32 *src, u32 size, u8 fast) {
	struct spi_transfer xfers[2] = {(struct spi_transfer){.len = 3}, (struct spi_transfer){.len = size, .tx_buf = src}};
	u32 addr = ADDR_WRITE(base + offset);
	xfers[0].tx_buf = &addr;
	if (fast) {
		xfers[0].speed_hz = MAX_SPI_SPEED;
		xfers[1].speed_hz = MAX_SPI_SPEED;
	}
	return spi_sync_transfer(ft8->spi, xfers, 2);
}

int ft8reg_write(struct ft8device *ft8, u32 reg, u32 val) {
	return ft8mem_write(ft8, RAM(REG), ft8regs[reg].offset, REGMASK(val, ft8regs[reg].nbits));
}


u32 ft8reg_read(struct ft8device *ft8, u32 reg) {
	return REGMASK(ft8mem_read(ft8, RAM(REG), ft8regs[reg].offset), ft8regs[reg].nbits);
}

int ft8host_cmd(struct ft8device *ft8, u8 command) {
	u8 buf[3];
	buf[0] = command;
	buf[1] = 0;
	buf[2] = 0;
	return spi_write(ft8->spi, buf, 3);
}

int ft8host_cmd_long(struct ft8device *ft8, u8 command, u8 arg) {
	u8 buf[3];
	buf[0] = command;
	buf[1] = arg;
	buf[2] = 0;
	return spi_write(ft8->spi, buf, 3);
}


int ft8copro_cmds(struct ft8device *ft8, u32 *cmds, u32 ncmds)  {
	u32 bytesleft = 0, tries = 0, offset = ft8regs[CMDB_WRITE].offset, size = ncmds*4;
	while (bytesleft < size && tries < 1000) {
		usleep_range(1000, 2000);
		bytesleft = RREG(CMDB_SPACE);
		tries++;
	}
	if (bytesleft < size) {
		dev_err(&ft8->spi->dev, "waited too long for coprocessor command buffer availability");
		return -EIO;
	}
	return ft8mem_write_bulk(ft8, RAM(REG), offset, cmds, size, false);
}


int ft8copro_wait(struct ft8device *ft8, u32 timeout) {
	u32 cmdr = RREG(CMD_READ);
	u32 cmdw = RREG(CMD_WRITE);
	int tries = 0;
	while (cmdr != cmdw && (!timeout || tries < timeout)) {
		usleep_range(1000, 2000);
		cmdr = RREG(CMD_READ);
		cmdw = RREG(CMD_WRITE);
		tries++;
	}
	if (cmdr != cmdw) {
		dev_err(&ft8->spi->dev, "waited too long for coprocessor command to finish");
		return -EIO;
	}
	return 0;
}

int ft8buf_transfer(struct ft8device *ft8, void *src, u32 base, u32 offset, u32 len) {
	int ret = 0, i;
	u32 nxfers = INT_DIV_UP(len,SPI_MAX_XFER_SIZE)+1;
	u32 addr = ADDR_WRITE(base+offset), chunk;
	ft8->xfers[0].tx_buf = &addr;
	ft8->xfers[0].len = 3;
	ft8->xfers[0].speed_hz = MAX_SPI_SPEED;
	for (i = 1; i < nxfers; i++) {
		chunk = min(len, SPI_MAX_XFER_SIZE);
		ft8->xfers[i].len = chunk;
		ft8->xfers[i].tx_buf = src;
		ft8->xfers[i].speed_hz = MAX_SPI_SPEED;
		src += chunk;
		len -= chunk;
	}
	ret = spi_sync_transfer(ft8->spi, ft8->xfers, nxfers);
	if (ret)
		dev_err(&ft8->spi->dev, "spi xfer error to addr %u, total length %u\n", addr, len);
	return ret;
}

int ft8dl_write(struct ft8device *ft8) {
	u32 o = 0, x;
	u8 pseq[3] = {0, 1, 2};

	DL(CLEAR_COLOR_RGB(0,0,0));
	DL(CLEAR_COLOR_A(255));
	DL(CLEAR(1,1,1));
	DL(COLOR_RGB(255,255,255));
	DL(COLOR_A(255));
	DL(BEGIN(BITMAPS));

	switch (ft8->format) {
	case DRM_FORMAT_RGB565:
		DL(BITMAP_SOURCE(RAM(G)/*ft8->frontbuf*/));
		DL(BITMAP_LAYOUT(RGB565, W*ft8->bpp, H));
		DL(BITMAP_LAYOUT_H((W*ft8->bpp) >> 10, H >> 10));
		DL(BITMAP_SIZE(BILINEAR, BORDER, BORDER, W, H));
		DL(VERTEX2II(0,0,0,0));
		break;
	case DRM_FORMAT_XBGR8888:
	case DRM_FORMAT_ABGR8888:
		pseq[0] = 2;
		pseq[2] = 0;
	case DRM_FORMAT_XRGB8888:
	case DRM_FORMAT_ARGB8888:
	case DRM_FORMAT_RGB888:
		DL(BITMAP_LAYOUT(L8, W*ft8->bpp, H));
		DL(BITMAP_LAYOUT_H((W*ft8->bpp) >> 10, H >> 10));
		DL(BITMAP_SIZE(NEAREST, BORDER, BORDER, 1, H));
		DL(COLOR_MASK(0,0,1,0));
		for (x = 0; x < W; x++)	{
			DL(BITMAP_SOURCE(RAM(G)/*ft8->frontbuf*/ + x*ft8->bpp+pseq[0]));
			DL(VERTEX2II(x,0,0,0));
		}
		DL(COLOR_MASK(0,1,0,0));
		for (x = 0; x < W; x++)	{
			DL(BITMAP_SOURCE(RAM(G)/*ft8->frontbuf*/ + x*ft8->bpp+pseq[1]));
			DL(VERTEX2II(x,0,0,0));
		}
		DL(COLOR_MASK(1,0,0,0));
		for (x = 0; x < W; x++)	{
			DL(BITMAP_SOURCE(RAM(G)/*ft8->frontbuf*/ + x*ft8->bpp+pseq[2]));
			DL(VERTEX2II(x,0,0,0));
		}
		break;
	case DRM_FORMAT_ARGB1555:
	case DRM_FORMAT_XRGB1555:
		DL(BITMAP_SOURCE(RAM(G)/*ft8->frontbuf*/));
		DL(BITMAP_LAYOUT(ARGB1555, W * 2, H));
		DL(BITMAP_SIZE(NEAREST, BORDER, BORDER, W, H));
		DL(VERTEX2II(0,0,0,0));
		break;
	default:
		dev_err(&ft8->spi->dev, "unknown format 0x%X", ft8->format);
		return -EINVAL;
	}

	DL(END());
	DL(DISPLAY());
	ft8mem_write_bulk(ft8, RAM(DL), 0, ft8->dl, o*4, true);
	WREG(DLSWAP, DLSWAP_FRAME);
	ft8->stats[DLWRITES]++;

	return 0;
}


void ft8clear_buffers(struct ft8device *ft8) {
	//ft8->frontbuf = RAM(G);
	//ft8->drawbuf = RAM(G) + ft8->bpp * W * H;

	u32 cmds[3] = {CMD_MEMZERO, 0, ft8->bpp * W * H * NBUFFERS};
	ft8copro_cmds(ft8, cmds, 3);
	ft8copro_wait(ft8, 50);
	ft8dl_write(ft8);

}

void ft8set_format(struct ft8device *ft8, u32 format) {
	struct drm_format_name_buf fmt_name;
	const struct drm_format_info *fmt_info = drm_format_info(format);
	dev_info(&ft8->spi->dev, "changing format to %s, bpp=%d, depth=%d, nplanes=%d",
		drm_get_format_name(format, &fmt_name), fmt_info->cpp[0], fmt_info->depth, fmt_info->num_planes);
	ft8->format = format;
	ft8->bpp = fmt_info->cpp[0];
	ft8clear_buffers(ft8);
}

static void ft8pipe_enable(struct drm_simple_display_pipe *pipe,
			struct drm_crtc_state *crtc_state)
{

	struct tinydrm_device *tdev = pipe_to_tinydrm(pipe);
	struct ft8device *ft8 = container_of(tdev, struct ft8device, tinydrm);
	int i;

	ft8clear_buffers(ft8);
	WREG(PWM_DUTY, BRIGHTNESS);
	for (i = 0; i < NSTATS; i++)
		ft8->stats[i] = 0;

	drm_crtc_vblank_on(&ft8->tinydrm.pipe.crtc);

	ft8->enabled = true;
	ft8->stats[ENABLED] = ktime_get();

}

static void ft8pipe_disable(struct drm_simple_display_pipe *pipe)
{
	struct tinydrm_device *tdev = pipe_to_tinydrm(pipe);
	struct ft8device *ft8 = container_of(tdev, struct ft8device, tinydrm);

	ft8->stats[DISABLED] = ktime_get();
	ft8->enabled = false;

	drm_crtc_vblank_off(&ft8->tinydrm.pipe.crtc);
	WREG(PWM_DUTY, 0);


}

void ft8display_pipe_update(struct drm_simple_display_pipe *pipe,
				 struct drm_plane_state *old_state)
{
	struct tinydrm_device *tdev = pipe_to_tinydrm(pipe);
	struct drm_framebuffer *fb = pipe->plane.state->fb;
	struct drm_crtc *crtc = &tdev->pipe.crtc;
	struct ft8device *ft8 = container_of(tdev, struct ft8device, tinydrm);
	//ktime_t start;

	ft8->stats[UPDATES]++;

	/*if (fb && (fb != old_state->fb)) {
		pipe->plane.fb = fb;
		if (fb->funcs->dirty) {
			fb->funcs->dirty(fb, NULL, 0, 0, NULL, 0);
			ft8->stats[UPDIRTIES]++;
		}
		}*/

	if (crtc->state->event) {
		spin_lock_irq(&crtc->dev->event_lock);
		if (crtc->state->active && drm_crtc_vblank_get(crtc) == 0)
                        drm_crtc_arm_vblank_event(crtc, crtc->state->event);
                else
                        drm_crtc_send_vblank_event(crtc, crtc->state->event);
		//tmpbuf = ft8->frontbuf;
		//ft8->frontbuf = ft8->drawbuf;
		//ft8->drawbuf = tmpbuf;
		ft8->stats[SWAPS]++;
		spin_unlock_irq(&crtc->dev->event_lock);

		if (fb) {
			pipe->plane.fb = fb;
			fb->funcs->dirty(fb, NULL, 0, 0, NULL, 0);
			ft8->stats[UPDIRTIES]++;
		}
		//while (RREG(DLSWAP) != 0)
		//	ft8->stats[WAITS]++;
		//ft8dl_write(ft8);
		ft8->stats[NFRAMES] = RREG(FRAMES);

		crtc->state->event = NULL;
	}
}

static const struct drm_simple_display_pipe_funcs ft8pipe_funcs = {
	.enable = ft8pipe_enable,
	.disable = ft8pipe_disable,
	.update = ft8display_pipe_update,
	.prepare_fb = tinydrm_display_pipe_prepare_fb,
};

static int ft8fb_dirty(struct drm_framebuffer *fb,
		struct drm_file *file_priv,
		unsigned int flags, unsigned int color,
		struct drm_clip_rect *clips,
		unsigned int num_clips) {

	struct tinydrm_device *tdev = fb->dev->dev_private;
	struct ft8device *ft8 = container_of(tdev, struct ft8device, tinydrm);
	struct drm_clip_rect clip;
	struct drm_gem_cma_object *cma_obj = drm_fb_cma_get_gem_obj(fb, 0);
	u8 cpp = fb->format->cpp[0];
	int startpix,npix;

	ft8->stats[DIRTIES]++;

	mutex_lock(&ft8->dirty_lock);

	if (!ft8->enabled)
		goto out_unlock;

	if (tdev->pipe.plane.fb != fb)
		goto out_unlock;

	if (fb->format->format != ft8->format)
		ft8set_format(ft8, fb->format->format);

	tinydrm_merge_clips(&clip, clips, num_clips, flags, W, H);
	clip.x1 = 0;
	clip.x2 = W;
	startpix = W * clip.y1;
	npix = W * (clip.y2 - clip.y1);
	if (npix == 0) goto out_unlock;

	ft8buf_transfer(ft8, cma_obj->vaddr + startpix * cpp, RAM(G) /*ft8->drawbuf*/, startpix * cpp, npix * cpp);

	// TODO: make the swap atomic


out_unlock:
	mutex_unlock(&ft8->dirty_lock);

	return 0;
}

static const struct drm_framebuffer_funcs ft8fb_funcs = {
        .destroy        = drm_gem_fb_destroy,
        .create_handle  = drm_gem_fb_create_handle,
        .dirty = ft8fb_dirty,
};


static const uint32_t ft8formats[] = {
//	DRM_FORMAT_RGB565,
	DRM_FORMAT_RGB888,
//	DRM_FORMAT_XRGB8888,
//	DRM_FORMAT_ARGB8888,
//	DRM_FORMAT_ABGR8888,
//	DRM_FORMAT_XBGR8888,
//	DRM_FORMAT_ARGB1555,
//	DRM_FORMAT_XRGB1555
};

static enum hrtimer_restart ft8vblank_isr(struct hrtimer *timer) {
	struct ft8device *ft8 = container_of(timer, struct ft8device, vblank_timer);
	hrtimer_forward_now(&ft8->vblank_timer, ft8->vblank_interval);
	hrtimer_restart(&ft8->vblank_timer);
	ft8->stats[NBLANKS]++;
	drm_crtc_handle_vblank(&ft8->tinydrm.pipe.crtc);
	return HRTIMER_NORESTART;
}

static int ft8vblank_enable(struct drm_device *drm, unsigned int crtc) {
	struct tinydrm_device *tdev = drm->dev_private;
	struct ft8device *ft8 = container_of(tdev, struct ft8device, tinydrm);
	//hrtimer_start(&ft8->vblank_timer, ktime_sub_us(ft8->vblank_interval, 100), HRTIMER_MODE_REL);
	hrtimer_start(&ft8->vblank_timer, ft8->vblank_interval, HRTIMER_MODE_REL);
	return 0;
}

static void ft8vblank_disable(struct drm_device *drm, unsigned int crtc) {
	struct tinydrm_device *tdev = drm->dev_private;
	struct ft8device *ft8 = container_of(tdev, struct ft8device, tinydrm);
	hrtimer_cancel(&ft8->vblank_timer);
}

int ft8debug_stats_show(struct seq_file *m, void *arg) {
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	struct tinydrm_device *tdev = dev->dev_private;
	struct ft8device *ft8 = container_of(tdev, struct ft8device, tinydrm);
	s64 now,delta;
	int i;
	for (i = 0; i < NSTATS; i++)
		seq_printf(m,"%s: %llu\n", stat_strings[i], ft8->stats[i]);
	now = ktime_get();
	delta = ktime_ms_delta(now, ft8->stats[ENABLED]);
	seq_printf(m,"delta: %lldms\n", delta);
	//for (i = 0; i < ENABLED; i++)
	//	seq_printf(m,"%s/sec: %lld", stat_strings[i], (ft8->stats[i] * 1000) / delta);

	return 0;
}

static const struct drm_info_list ft8debug_list[] = {
	{"fb",   drm_fb_cma_debugfs_show, 0},
	{"stats", ft8debug_stats_show, 0},
};

static int ft8debug_regshow(struct seq_file *m, void *d) {
        struct ft8device *ft8 = m->private;
	int i;
	dev_warn(&ft8->spi->dev, "path: %s\n", m->file->f_path.dentry->d_iname);
	for (i = 0; i < NREGS; i++)
		if (strcmp(m->file->f_path.dentry->d_iname, ft8regs[i].name) == 0) {
			if (!ft8regs[i].readable) return -EINVAL;
			seq_printf(m, "0x%X\n", RREG(i));
			return 0;
		}
        return -EINVAL;
}

static int ft8debug_regopen(struct inode *inode, struct file *file) {
	struct ft8device *ft8 = inode->i_private;
	return single_open(file, ft8debug_regshow, ft8);
}

static ssize_t ft8debug_regwrite(struct file *file, const char __user *user_buf, size_t count, loff_t *ppos)
{
        struct seq_file *m = file->private_data;
        struct ft8device *ft8 = m->private;
	int i,ret;
	unsigned long val;
	char *buf;

	buf = memdup_user_nul(user_buf, count);
	if (IS_ERR(buf))
		return PTR_ERR(buf);
	ret = kstrtoul(buf, 0, &val);
	if (ret < 0) {
		kfree(buf);
		return ret;
	}
	kfree(buf);
	for (i = 0; i < NREGS; i++)
	        if (strcmp(file->f_path.dentry->d_iname, ft8regs[i].name) == 0) {
			if (!ft8regs[i].writable) return -EINVAL;
                        ret = WREG(i,val);
			return ret < 0 ? ret : count;
		}
	return -EINVAL;
}


static const struct file_operations ft8reg_debugfs_ops = {
	.owner = THIS_MODULE,
	.open = ft8debug_regopen,
	.read = seq_read,
	.write = ft8debug_regwrite,
	.llseek = seq_lseek,
	.release = single_release,
};

int ft8debug_init(struct drm_minor *minor) {
	struct tinydrm_device *tdev = minor->dev->dev_private;
	struct ft8device *ft8 = container_of(tdev, struct ft8device, tinydrm);
	int i;
	for (i = 0; i < NREGS; i++)
		debugfs_create_file(ft8regs[i].name, S_IFREG | S_IWUSR | S_IRUGO, minor->debugfs_root, ft8, &ft8reg_debugfs_ops);

	return drm_debugfs_create_files(ft8debug_list, ARRAY_SIZE(ft8debug_list), minor->debugfs_root, minor);
}

DEFINE_DRM_GEM_CMA_FOPS(ft8drm_fops);
static struct drm_driver ft8drm_driver = {
	.driver_features	   = DRIVER_GEM | DRIVER_MODESET | DRIVER_PRIME | DRIVER_ATOMIC,
	.lastclose		   = tinydrm_lastclose,
	.fops                      = &ft8drm_fops,
	TINYDRM_GEM_DRIVER_OPS,
	.enable_vblank             = &ft8vblank_enable,
	.disable_vblank            = &ft8vblank_disable,
	//.get_vblank_counter        = &drm_crtc_accurate_vblank_count,
	.debugfs_init              = ft8debug_init,
	.name			   = "ft8xx",
	.desc			   = "FTDI FT8XX",
	.date			   = "20170829",
	.major			   = 1,
	.minor			   = 0,
	.patchlevel                = 0,
};

static const struct drm_display_mode ft8mode = {
	.name = WSTR "x" HSTR,
	TINYDRM_MODE(W,H,WMM,HMM)
};

static int ft8drm_init(struct ft8device *ft8) {

	struct spi_device *spi = ft8->spi;
	struct device *dev = &spi->dev;
	struct tinydrm_device *tdev = &ft8->tinydrm;
	int ret = 0;

	mutex_init(&ft8->dirty_lock);

	ret = devm_tinydrm_init(dev, tdev, &ft8fb_funcs, &ft8drm_driver);
        if (ret)
                return ret;

	ret = tinydrm_display_pipe_init(tdev, &ft8pipe_funcs, DRM_MODE_CONNECTOR_VIRTUAL, ft8formats,
					ARRAY_SIZE(ft8formats), &ft8mode, 0);
	if (ret) {
		dev_err(dev, "Failed to init display: %d\n", ret);
		return ret;
	}

	tdev->drm->mode_config.preferred_depth = PREFER_DEPTH;
	drm_mode_config_reset(tdev->drm);

	return 0;
}

static inline void report_fingered(struct input_dev *input, int slot, u32 xy) {
	bool down = xy != 0x80008000;
	input_mt_slot(input, slot);
	input_mt_report_slot_state(input, MT_TOOL_FINGER, down);
	if (down) {
		input_report_abs(input, ABS_MT_POSITION_X, 0xFFFF & (xy >> 16));
		input_report_abs(input, ABS_MT_POSITION_Y, 0xFFFF & xy);
	}
}

static irqreturn_t ft8isr(int irq, void *dev_id)
{
	struct ft8device *ft8 = dev_id;
	u8 flags;
	u32 xy1,xy2;

	flags = RREG(INT_FLAGS);
	if (!(flags & INT_CONVCOMPLETE)) {
		ft8->stats[SPURINT]++;
                if (flags != 0)
			dev_warn(&ft8->spi->dev, "spurious interrupt %d\n", flags);
		//else
		//	dev_warn(&ft8->spi->dev, "flagst %d\n",flags);
		return IRQ_HANDLED;
	}

	if (flags & INT_CONVCOMPLETE) {
		xy1 = RREG(CTOUCH_TOUCH0_XY);
		xy2 = RREG(CTOUCH_TOUCH1_XY);
		//dev_warn(&ft8->spi->dev, "xy1: %u, xy2: %u\n", xy1,xy2);
		report_fingered(ft8->input_emu, 0, xy1);
		report_fingered(ft8->input_emu, 1, xy2);
		input_mt_report_pointer_emulation(ft8->input_emu, true);
		input_sync(ft8->input_emu);
	}

	return IRQ_HANDLED;
}

int ft8touch_init(struct spi_device *spi, struct ft8device *ft8) {

	struct input_dev *input_emu;
	int ret;

	input_emu = devm_input_allocate_device(&spi->dev);
	if (!input_emu) {
		dev_err(&spi->dev, "failed to allocate input device.\n");
		return -ENOMEM;
	}

	ft8->input_emu = input_emu;

	dev_dbg(&spi->dev,"ft8 touchscreen initialized\n");

	input_emu->name = "FT8XX Touchscreen";
	input_emu->phys = "input/ts";
	input_emu->id.bustype = BUS_SPI;
	input_emu->dev.parent = &spi->dev;
	input_set_abs_params(input_emu, ABS_MT_POSITION_X, 0, W-1, 0, 0);
	input_set_abs_params(input_emu, ABS_MT_POSITION_Y, 0, H-1, 0, 0);
	input_mt_init_slots(input_emu, 2, INPUT_MT_DIRECT); // INPUT_MT_POINTER);

	ret = input_register_device(input_emu);
	if (ret) {
	        dev_err(&spi->dev, "Unable to register touchpad input\n");
		return ret;
	}

	return 0;
}

static int ft8chip_powerup(struct ft8device *ft8) {

	int ret,tries,cmds[4],i,frame;
	ktime_t last,now;
	u64 sum = 0;
	struct device *dev = &ft8->spi->dev;

	dev_info(dev, "initializing ft8");

	ft8->spi->bits_per_word = 8;
	ft8->spi->mode = SPI_MODE_0;
	ft8->spi->max_speed_hz = SETUP_SPI_SPEED;
	ret = spi_setup(ft8->spi);
	if (ret) {
		dev_err(dev, "bad spi_setup ret %d", ret);
		return ret;
	}

	ft8host_cmd(ft8, RESET);
	ft8host_cmd(ft8, PWRDOWN);
	ft8host_cmd(ft8, ACTIVE);
	ft8host_cmd(ft8, CLKEXT);
	WREG(PWM_DUTY, 0);
	msleep(50);

	tries = 0;
	while (RREG(ID) != REG_ID_VAL && tries < 20) { // wait
	  msleep(50);
	  tries++;
	}
	if (tries >= 10) {
	  dev_err(&ft8->spi->dev, "couldnt find display\n");
	  return -ENODEV;
	}

	/*ftstart = RREG(CLOCK);
	kstart = ktime_get();
	msleep(1000);
	ftend = RREG(CLOCK);
	kend = ktime_get();
	kdelta = ktime_divns(ktime_sub(kend,kstart),100);
	ftdelta = (ftend - ftstart)/6;
	trim_result = kdelta - ftdelta;
	if (trim_result < 0) trim_result *= -1;
	dev_info(&ft8->spi->dev, "diff: %d\n", trim_result);*/


	WREG(PWM_DUTY, BRIGHTNESS);

	WREG(HSIZE, W);
	WREG(HCYCLE, HW_HCYCLE);
	WREG(HOFFSET, HW_HOFFSET);
	WREG(HSYNC0, HW_HSYNC0);
	WREG(HSYNC1, HW_HSYNC1);

	WREG(VSIZE, H);
	WREG(VCYCLE, HW_VCYCLE);
	WREG(VOFFSET, HW_VOFFSET);
	WREG(VSYNC0, HW_VSYNC0);
	WREG(VSYNC1, HW_VSYNC1);

	WREG(SWIZZLE, 2);
	WREG(PCLK_POL, 1);
	WREG(CSPREAD, 1);
	WREG(DITHER, 0);
	WREG(GPIO_DIR, 0xff);
	WREG(GPIO, 0xff);

	ft8set_format(ft8, PREFER_FORMAT);
	ft8mem_write_bulk(ft8, RAM(G), 0, (u32*)ohm_logo, W*H*ft8->bpp, true);
	WREG(PWM_HZ, 128);

	cmds[0] = CMD_SETROTATE;
	cmds[1] = ROTATION;
	ft8copro_cmds(ft8, cmds, 3);
	WREG(PCLK, HW_PCLK);

	frame = RREG(FRAMES);
	while (RREG(FRAMES) == frame);
	frame++;
	last = ktime_get();
	for (i = 0; i < CLKSYNC_SAMPLES; i++) {
		while (RREG(FRAMES) == frame);
		frame++;
		now = ktime_get();
		sum += ktime_to_ns(ktime_sub(now,last));
		last = now;
	}
	ft8->vblank_interval = ktime_divns(ns_to_ktime(sum),CLKSYNC_SAMPLES);
	hrtimer_init(&ft8->vblank_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ft8->vblank_timer.function = &ft8vblank_isr;
	dev_info(&ft8->spi->dev, "blanking interval: %lld, hrtimer resolution: %u", ktime_to_ns(ft8->vblank_interval), hrtimer_resolution);

	/*WREG(PWM_DUTY, 128);
	cmds[0] = CMD_CALIBRATE;
	cmds[1] = 0x69696969UL;
	ft8copro_cmds(ft8, cmds, 2);
	ft8copro_wait(ft8, 0);
	dev_info(&ft8->spi->dev, "calibration results: 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X",
		RREG(CTOUCH_TRANSFORM_A),
		RREG(CTOUCH_TRANSFORM_B),
		RREG(CTOUCH_TRANSFORM_C),
		RREG(CTOUCH_TRANSFORM_D),
		RREG(CTOUCH_TRANSFORM_E),
		RREG(CTOUCH_TRANSFORM_F));
	WREG(CTOUCH_TRANSFORM_A, 0x00005d2b);
	WREG(CTOUCH_TRANSFORM_B, 0xfffffe28);
	WREG(CTOUCH_TRANSFORM_C, 0xfff388da);
	WREG(CTOUCH_TRANSFORM_D, 0x0000007e);
	WREG(CTOUCH_TRANSFORM_E, 0x0000686b);
        WREG(CTOUCH_TRANSFORM_F, 0xfff23bed);*/

	WREG(INT_MASK, INT_CONVCOMPLETE);
	WREG(INT_EN, 1);
	WREG(CTOUCH_EXTENDED, 0);
	RREG(INT_FLAGS); // dummy read to clear the flag
	
	return 0;
}

static int ft8probe(struct spi_device *spi) {

	struct device *dev = &spi->dev;
	struct ft8device *ft8;
	int ret;

	ft8 = devm_kzalloc(dev, sizeof(*ft8), GFP_KERNEL);
	if (!ft8)
		return -ENOMEM;

	ft8->spi = spi;
	ft8->chip_id = 0x13;
	ret = ft8chip_powerup(ft8);
	if (ret)
		return ret;

	// theoretical max zlib out buffer, plus 8 bytes ft8 command overhead
	//ft8->zx_buf = devm_kmalloc(dev, H * W + H * W / 1000 + 13 + 8, GFP_KERNEL);
	//wsize = zlib_deflate_workspacesize(MAX_WBITS, MAX_MEM_LEVEL);
	//ft8->zstream.workspace = devm_kzalloc(dev, wsize, GFP_KERNEL);

	//if (!ft8->tx_header_buf || !ft8->tx_buf) // || !ft8->zx_buf || !ft8->zstream.workspace)
	//	return -ENOMEM;

	ft8->interrupt_gpio = devm_gpiod_get(dev, "irq", GPIOD_IN);
	if (IS_ERR(ft8->interrupt_gpio)) {
		dev_err(dev, "Failed to get gpio 'irq' %ld", PTR_ERR(ft8->interrupt_gpio));
		return PTR_ERR(ft8->interrupt_gpio);
	}

	//dev->dma_ops = &dma_virt_ops;
	//set_dma_ops(dev, &dma_virt_ops);
	arch_setup_dma_ops(dev, 0, DMA_BIT_MASK(32), NULL, true);
	if (!dev->coherent_dma_mask) {
		ret = dma_coerce_mask_and_coherent(dev, DMA_BIT_MASK(32));
		if (ret) {
			dev_warn(dev, "Failed to set dma mask %d", ret);
			//return ret;
		}
	}

	ret = ft8drm_init(ft8);
	if (ret) {
		dev_err(dev, "failed to initialize drm: %d", ret);
		return ret;
	}

	ret = drm_vblank_init(ft8->tinydrm.drm, 1);
	if (ret != 0) {
		dev_err(dev, "Failed to init vblank: %d\n", ret);
		return ret;
	}

	ret = devm_tinydrm_register(&ft8->tinydrm);
	if (ret)
		return ret;

	spi_set_drvdata(spi, &ft8->tinydrm);

	ret = ft8touch_init(spi, ft8);
	if (ret) {
		dev_err(dev, "failed to initialize touch panel: %d", ret);
		return ret;
	}

	ft8->irq = gpiod_to_irq(ft8->interrupt_gpio);
	dev_info(dev, "using irq %d from irq-gpio\n", ft8->irq);
	ret = request_threaded_irq(ft8->irq,NULL,ft8isr,IRQF_TRIGGER_FALLING | IRQF_ONESHOT,"ft8xx", ft8);
	if (ret) {
		dev_err(&spi->dev, "Unable to request IRQ: %d\n", ret);
		return ret;
	}

	RREG(INT_FLAGS); // dummy read to clear the flag

	return 0;
}

static void ft8shutdown(struct spi_device *spi) {
	struct tinydrm_device *tdev = spi_get_drvdata(spi);
  	struct ft8device *ft8 = container_of(tdev, struct ft8device, tinydrm);

	ft8->enabled = false;
        dev_info(&spi->dev, "shutting down ft8xx\n");
	WREG(PWM_DUTY, 0);
	input_unregister_device(ft8->input_emu);
	free_irq(ft8->irq, ft8);
	input_free_device(ft8->input_emu);
	mutex_destroy(&ft8->dirty_lock);
	tinydrm_shutdown(tdev);

}


static int ft8remove(struct spi_device *spi) {
	ft8shutdown(spi);
	return 0;
}


static const struct spi_device_id ft8id[] = {
	{ "ft8xx", (unsigned long)&ft8pipe_funcs },
	{ },
};
MODULE_DEVICE_TABLE(spi, ft8id);

static const struct of_device_id ft8of_match[] = {
	{ .compatible = "ftdi,ft8xx" },
	{},
};
MODULE_DEVICE_TABLE(of, ft8of_match);

static int __maybe_unused ft8suspend(struct device *dev)
{
	struct tinydrm_device *tdev = dev_get_drvdata(dev);
	struct ft8device *ft8 = container_of(tdev, struct ft8device, tinydrm);
	int ret;

	ret = tinydrm_suspend(tdev);
        if (ret)
                return ret;

	WREG(PWM_DUTY, 0);

        return 0;
}

static int __maybe_unused ft8resume(struct device *dev)
{
	struct tinydrm_device *tdev = dev_get_drvdata(dev);
	struct ft8device *ft8 = container_of(tdev, struct ft8device, tinydrm);

	ft8chip_powerup(ft8);

        return tinydrm_resume(tdev);
}


static const struct dev_pm_ops ft8pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(ft8suspend, ft8resume)
};

static struct spi_driver ft8spi_driver = {
	.driver = {
		.name = "ft8xx",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(ft8of_match),
		.pm = &ft8pm_ops,
	},
	.id_table = ft8id,
	.probe = ft8probe,
	.remove = ft8remove,
	.shutdown = ft8shutdown,
};
module_spi_driver(ft8spi_driver);

/*static int __init ft8xx_init(void)
{
	return spi_register_driver(&ft8spi_driver);
}
subsys_initcall(ft8xx_init);*/


MODULE_DESCRIPTION("FTDI FT8xx DRM driver");
MODULE_AUTHOR("Matt Gattis");
MODULE_LICENSE("GPL");


/*int ft8buf_ztransfer(struct ft8device *ft8, void *src, u32 addr, size_t len) {
	int ret;
	struct spi_message msg;
	struct spi_transfer header = {
		.tx_buf = ft8->tx_header_buf,
		.rx_buf = NULL,
		.len = 3,
		.bits_per_word = 8,
	};
	struct spi_transfer body = {
		.tx_buf = ft8->zx_buf,
		.rx_buf = NULL,
		.len = 0,
		.bits_per_word = 8,
	};

	ret = zlib_deflateInit(&ft8->zstream, 3);
	if (ret != Z_OK) {
		dev_err(&ft8->spi->dev, "deflateInit failed: %d", ret);
		return ret;
	}
	ft8->zstream.total_in = 0;
	ft8->zstream.total_out = 0;
	ft8->zstream.next_in = src;
	ft8->zstream.avail_in = len;
	ft8->zstream.next_out = ft8->zx_buf + 8;
	ft8->zstream.avail_out = H * W + H * W / 1000 + 13;
	ret = zlib_deflate(&ft8->zstream, Z_FINISH);
	if (ret != Z_STREAM_END) {
		dev_err(&ft8->spi->dev, "zlib did not finish stream");
		zlib_deflateEnd(&ft8->zstream);
		return -EIO;
	}

	body.len = 8 + ft8->zstream.total_out;
	zlib_deflateEnd(&ft8->zstream);
	if (body.len % 4 != 0)
		body.len += 4 - (body.len % 4);

	spi_message_init(&msg);
	spi_message_add_tail(&header, &msg);
	spi_message_add_tail(&body, &msg);
	ft8set_mem_addr(ft8->tx_header_buf, REG(CMDB_WRITE), false);
	((uint32_t*)ft8->zx_buf)[0] = CMD_INFLATE;
	((uint32_t*)ft8->zx_buf)[1] = addr;

	if(spi_sync(ft8->spi, &msg)) {
		dev_err(&ft8->spi->dev, "zbuf spi err");
		zlib_deflateEnd(&ft8->zstream);
		return -EIO;
	}

        while (RREG(CMD_READ)) != RREG(CMD_WRITE)))
                usleep_range(500, 1500);

	return 0;
	}*/
