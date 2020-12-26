// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2019 IMBEL http://www.imbel.gov.br 
 *	Rodrigo Alencar <alencar.fmce@imbel.gov.br>
 */

#include <linux/fb.h>
#include <linux/bitrev.h>
#include <linux/gpio/consumer.h>
#include <linux/spi/spi.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/pwm.h>

#define SMEMLCD_DATA_UPDATE		0x80
#define SMEMLCD_FRAME_INVERSION		0x40
#define SMEMLCD_ALL_CLEAR		0x20
#define SMEMLCD_DUMMY_DATA		0x00

struct smemlcd_info {
	u32 width;
	u32 height;
};

struct smemlcd_par {
	struct spi_device *spi;
	struct fb_info *info;
	struct pwm_device *extcomin_pwm;
	struct gpio_desc *disp;
	struct delayed_work d_work;
	struct mutex update_lock;

	u8 *spi_buf;

	bool extmode;
	u32 spi_width;
	u32 vmem_width;

	u8 vcom;
	u32 start;
	u32 height;
};

static struct fb_fix_screeninfo smemlcd_fix = {
	.id = "Sharp Memory LCD",
	.type = FB_TYPE_PACKED_PIXELS,
	.visual = FB_VISUAL_MONO10,
	.xpanstep = 0,
	.ypanstep = 0,
	.ywrapstep = 0,
	.accel = FB_ACCEL_NONE,
};

static struct fb_var_screeninfo smemlcd_var = {
	.bits_per_pixel = 1,
	.red = {0, 1, 0},
	.green = {0, 1, 0},
	.blue = {0, 1, 0},
	.transp = {0, 0, 0},
	.left_margin = 0,
	.right_margin = 0,
	.upper_margin = 0,
	.lower_margin = 0,
	.vmode = FB_VMODE_NONINTERLACED,
};

static void smemlcd_update(struct smemlcd_par *par)
{
	struct spi_device *spi = par->spi;
	u8 *vmem = par->info->screen_base;
	u8 *buf_ptr = par->spi_buf;
	int ret;
	u32 i,j;

	if (par->start + par->height > par->info->var.yres) {
		par->start = 0;
		par->height = 0;
	}
	/* go to start line */
	vmem += par->start * par->vmem_width;
	/* update vcom */
	par->vcom ^= SMEMLCD_FRAME_INVERSION;
	/* mode selection */
	*(buf_ptr++) = (par->height)? (SMEMLCD_DATA_UPDATE | par->vcom) : par->vcom;

	/* not all SPI masters have LSB-first mode, bitrev8 is used */
	for (i = par->start + 1; i < par->start + par->height + 1; i++) {
		/* gate line address */
		*(buf_ptr++) = bitrev8(i);
		/* data writing */
		for (j = 0; j < par->spi_width; j++)
			*(buf_ptr++) = bitrev8(*(vmem++));
		/* dummy data */
		*(buf_ptr++) = SMEMLCD_DUMMY_DATA;
		/* video memory alignment */
		for (; j < par->vmem_width; j++)
			vmem++;
	}
	/* dummy data */
	*(buf_ptr++) = SMEMLCD_DUMMY_DATA;

	ret = spi_write(spi, &(par->spi_buf[0]), par->height * (par->spi_width + 2) + 2);
	if (ret < 0)
		dev_err(&spi->dev, "Couldn't send SPI command.\n");

	par->start = U32_MAX;
	par->height = 0;
}

static void smemlcd_frame(struct smemlcd_par *par, u32 req_start, u32 req_height)
{
	u32 end = par->start + par->height;
	u32 req_end = req_start + req_height;
	if (req_end > par->info->var.yres)
		req_end = par->info->var.yres;
	if (par->start > req_start)
		par->start = req_start;
	if (end < req_end || end > par->info->var.yres)
		end = req_end;
	par->height = end - par->start;
}

static void smemlcd_fillrect(struct fb_info *info, const struct fb_fillrect *rect)
{
	struct smemlcd_par *par = info->par;
	sys_fillrect(info, rect);

	mutex_lock(&par->update_lock);
	smemlcd_frame(par, rect->dy, rect->height);
	if(par->extmode)
		smemlcd_update(par);
	mutex_unlock(&par->update_lock);
}

static void smemlcd_imageblit(struct fb_info *info, const struct fb_image *image)
{
	struct smemlcd_par *par = info->par;
	sys_imageblit(info, image);

	mutex_lock(&par->update_lock);
	smemlcd_frame(par, image->dy, image->height);
	if(par->extmode)
		smemlcd_update(par);
	mutex_unlock(&par->update_lock);
}

static void smemlcd_copyarea(struct fb_info *info, const struct fb_copyarea *area)
{
	struct smemlcd_par *par = info->par;
	sys_copyarea(info, area);

	mutex_lock(&par->update_lock);
	smemlcd_frame(par, area->dy, area->height);
	if(par->extmode)
		smemlcd_update(par);
	mutex_unlock(&par->update_lock);
}

static ssize_t smemlcd_write(struct fb_info *info, const char __user * buf, size_t count, loff_t * ppos)
{
	ssize_t ret;
	struct smemlcd_par *par = info->par;
	u32 req_start, req_height;
	u32 offset = (u32) * ppos;

	ret = fb_sys_write(info, buf, count, ppos);
	if (ret > 0) {
		mutex_lock(&par->update_lock);
		req_start = max((int)(offset / par->vmem_width), 0);
		req_height = ret / par->vmem_width + 1;
		smemlcd_frame(par, req_start, req_height);
		if(par->extmode)
			smemlcd_update(par);
		mutex_unlock(&par->update_lock);
	}

	return ret;
}

static int smemlcd_blank(int blank_mode, struct fb_info *info)
{
	struct smemlcd_par *par = info->par;

	if (par->disp) {
		if (blank_mode != FB_BLANK_UNBLANK)
			gpiod_set_value_cansleep(par->disp, 0);
		else
			gpiod_set_value_cansleep(par->disp, 1);
	}

	return 0;
}

static void smemlcd_deferred_io(struct fb_info *info, struct list_head *pagelist)
{
	struct smemlcd_par *par = info->par;

	mutex_lock(&par->update_lock);

	if (!list_empty(pagelist)) {
		struct page *cur;
		u32 req_start;
		u32 req_height = (PAGE_SIZE / par->vmem_width) + 1;

		list_for_each_entry(cur, pagelist, lru) {
			req_start = (cur->index << PAGE_SHIFT) / par->vmem_width;
			smemlcd_frame(par, req_start, req_height);
		}
	}

	if(par->extmode)
		smemlcd_update(par);
	mutex_unlock(&par->update_lock);
}

static void smemlcd_update_work(struct work_struct *work)
{
	struct smemlcd_par *par = container_of(work, struct smemlcd_par, d_work.work);
	struct fb_info *info = par->info;

	mutex_lock(&par->update_lock);
	smemlcd_update(par);
	mutex_unlock(&par->update_lock);

	if (!par->extmode)
		schedule_delayed_work(&par->d_work, info->fbdefio->delay);
}

static struct fb_ops smemlcd_ops = {
	.owner = THIS_MODULE,
	.fb_read = fb_sys_read,
	.fb_write = smemlcd_write,
	.fb_fillrect = smemlcd_fillrect,
	.fb_copyarea = smemlcd_copyarea,
	.fb_imageblit = smemlcd_imageblit,
	.fb_blank = smemlcd_blank,
};

static struct smemlcd_info ls027b7dh01_info = {
	.width = 400,
	.height = 240,
};

static struct smemlcd_info ls044q7dh01_info = {
	.width = 320,
	.height = 240,
};

static struct smemlcd_info ls013b7dh05_info = {
	.width = 144,
	.height = 168,
};

static struct smemlcd_info ls013b7dh03_info = {
	.width = 128,
	.height = 128,
};

static struct smemlcd_info ls032b7dd02_info = {
        .width = 536,
        .height = 336,
};

static const struct of_device_id smemlcd_of_match[] = {
	{
	 .compatible = "sharp,ls027b7dh01",
	 .data = (void *)&ls027b7dh01_info,
	 },
	{
	 .compatible = "sharp,ls044q7dh01",
	 .data = (void *)&ls044q7dh01_info,
	 },
	{
	 .compatible = "sharp,ls013b7dh05",
	 .data = (void *)&ls013b7dh05_info,
	 },
	{
	 .compatible = "sharp,ls013b7dh03",
	 .data = (void *)&ls013b7dh03_info,
	 },
	{
	 .compatible = "sharp,ls032b7dd02",
	 .data = (void *)&sharp,ls032b7dd02_info,
	 },
	{},
};
MODULE_DEVICE_TABLE(of, smemlcd_of_match);

static int smemlcd_probe(struct spi_device *spi)
{
	struct fb_info *info;
	struct smemlcd_par *par;
	const struct smemlcd_info *devinfo;
	struct fb_deferred_io *smemlcd_defio;
	struct device_node *node = spi->dev.of_node;
	struct pwm_state state;
	u32 vmem_size, fps;
	u8 *vmem;
	u8 *buf_ptr;
	int ret;

	info = framebuffer_alloc(sizeof(struct smemlcd_par), &spi->dev);
	if (!info) {
		dev_err(&spi->dev, "Failed to allocate framebuffer.\n");
		return -ENOMEM;
	}

	par = info->par;
	par->info = info;
	par->spi = spi;

	devinfo = of_device_get_match_data(&spi->dev);

	par->disp = devm_gpiod_get_optional(&spi->dev, "disp", GPIOD_OUT_LOW);
	if (IS_ERR(par->disp)) {
		ret = PTR_ERR(par->disp);
		dev_err(&spi->dev, "Failed to get DISP gpio: %d\n", ret);
		goto free_fb;
	}

	mutex_init(&par->update_lock);
	INIT_DELAYED_WORK(&par->d_work, smemlcd_update_work);
	par->spi_width = devinfo->width / 8;
	par->vmem_width = ((devinfo->width + 31) & ~31) >> 3;
	par->vcom = 0;
	par->start = 0;
	par->height = 0;

	par->spi_buf = kzalloc(devinfo->height * (par->spi_width + 2) + 2, GFP_KERNEL);
	if (!par->spi_buf) {
		ret = -ENOMEM;
		dev_err(&spi->dev, "Failed to allocate data for spi transfers.\n");
		goto free_fb;
	}

	vmem_size = par->vmem_width * devinfo->height;

	vmem = (void *)__get_free_pages(GFP_KERNEL | __GFP_ZERO, get_order(vmem_size));
	if (!vmem) {
		ret = -ENOMEM;
		dev_err(&spi->dev, "Failed to allocate video memory.\n");
		goto free_spi_buf;
	}

	par->extmode = of_property_read_bool(node, "sharp,extmode-high");

	if (of_property_read_u32(node, "sharp,frames-per-sec", &fps))
		fps = 10;

	smemlcd_defio = devm_kzalloc(&spi->dev, sizeof(struct fb_deferred_io), GFP_KERNEL);
	if (!smemlcd_defio) {
		dev_err(&spi->dev, "Couldn't allocate deferred io.\n");
		ret = -ENOMEM;
		goto free_vmem;
	}

	smemlcd_defio->delay = HZ / fps;
	smemlcd_defio->deferred_io = &smemlcd_deferred_io;

	info->var = smemlcd_var;
	info->var.xres = devinfo->width;
	info->var.xres_virtual = devinfo->width;
	info->var.yres = devinfo->height;
	info->var.yres_virtual = devinfo->height;

	info->screen_base = (u8 __force __iomem *) vmem;
	info->screen_size = vmem_size;

	info->fbops = &smemlcd_ops;
	info->fix = smemlcd_fix;
	info->fix.line_length = par->vmem_width;
	info->fix.smem_start = __pa(vmem);
	info->fix.smem_len = vmem_size;

	info->fbdefio = smemlcd_defio;

	fb_deferred_io_init(info);

	spi_set_drvdata(spi, par);

	if (par->extmode) {
		par->extcomin_pwm = pwm_get(&spi->dev, NULL);
		if (IS_ERR(par->extcomin_pwm)) {
			ret = PTR_ERR(par->extcomin_pwm);
			dev_warn(&spi->dev, "Failed to get EXTCOMIN pwm: %d\n", ret);
			par->extcomin_pwm = NULL;
		} else {

			pwm_init_state(par->extcomin_pwm, &state);

			if (!state.period)
				state.period = NSEC_PER_SEC/fps;

			/* The duty cycle is not really important */
			state.enabled = true;
			state.duty_cycle = state.period/2;

			ret = pwm_apply_state(par->extcomin_pwm, &state);
			if (ret)
				dev_warn(&spi->dev, "failed to apply EXTCOMIN pwm state: %d\n", ret);
		}
	} else {
		par->extcomin_pwm = NULL;
	}

	if (par->disp)
		gpiod_set_value_cansleep(par->disp, 1);

	ret = register_framebuffer(info);
	if (ret < 0) {
		dev_err(&spi->dev, "Failed to register framebuffer.\n");
		goto disable_hw;
	}

	/* spi test by clearing the display */
	par->spi_buf[0] = SMEMLCD_ALL_CLEAR;
	par->spi_buf[1] = SMEMLCD_DUMMY_DATA;
	ret = spi_write(spi, &(par->spi_buf[0]), 2);
	if (ret < 0){
		dev_err(&spi->dev, "Couldn't send SPI command.\n");
		goto disable_hw;
	}

	dev_info(&spi->dev, "fb%d: %s framebuffer device registered, using %d bytes of video memory\n", info->node, info->fix.id, vmem_size);

	if (!par->extmode)
		schedule_delayed_work(&par->d_work, smemlcd_defio->delay);

	return 0;

disable_hw:
	if (par->disp)
		gpiod_set_value_cansleep(par->disp, 0);
	if (par->extcomin_pwm) {
		state.enabled = false;
		state.duty_cycle = 0;
		pwm_apply_state(par->extcomin_pwm, &state);
		pwm_put(par->extcomin_pwm);
	}
	kfree(smemlcd_defio);
free_vmem:
	kfree((void *)vmem);
free_spi_buf:
	kfree(par->spi_buf);
free_fb:
	cancel_delayed_work_sync(&par->d_work);
	mutex_destroy(&par->update_lock);
	framebuffer_release(info);

	return ret;
}

static int smemlcd_remove(struct spi_device *spi)
{
	struct smemlcd_par *par = dev_get_drvdata(&spi->dev);
	struct fb_info *info = par->info;
	struct pwm_state state;

	par->extmode = true;
	cancel_delayed_work_sync(&par->d_work);

	fb_deferred_io_cleanup(info);

	unregister_framebuffer(info);

	if (par->disp)
		gpiod_set_value_cansleep(par->disp, 0);

	if (par->extcomin_pwm) {
		pwm_get_state(par->extcomin_pwm, &state);
		state.enabled = false;
		state.duty_cycle = 0;
		pwm_apply_state(par->extcomin_pwm, &state);
		pwm_put(par->extcomin_pwm);
	}

	fb_deferred_io_cleanup(info);
	__free_pages(__va(info->fix.smem_start), get_order(info->fix.smem_len));

	kfree(par->spi_buf);

	mutex_destroy(&par->update_lock);

	framebuffer_release(info);

	return 0;
}

static const struct spi_device_id smemlcd_spi_id[] = {
	{"ls027b7dh01", 0},
	{"ls044q7dh01", 0},
	{"ls013b7dh05", 0},
	{"ls013b7dh03", 0},
	{"ls032b7dd02", 0},
	{}
};
MODULE_DEVICE_TABLE(spi, smemlcd_spi_id);

static struct spi_driver smemlcd_driver = {
	.probe = smemlcd_probe,
	.remove = smemlcd_remove,
	.id_table = smemlcd_spi_id,
	.driver = {
		   .name = "smemlcdfb",
		   .of_match_table = smemlcd_of_match,
		   },
};
module_spi_driver(smemlcd_driver);

MODULE_AUTHOR("Rodrigo Alencar <455.rodrigo.alencar@gmail.com>");
MODULE_DESCRIPTION("Sharp Memory LCD Linux Framebuffer Driver");
MODULE_LICENSE("GPL");
