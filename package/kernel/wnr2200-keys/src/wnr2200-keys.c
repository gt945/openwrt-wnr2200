/*
 * wnr2200-keys.c
 *
 *  Created on: 2014年1月20日
 *      Author: tao
 */
#include <linux/module.h>
#include <linux/version.h>
#include <linux/kmod.h>

#include <linux/workqueue.h>
#include <linux/skbuff.h>
#include <linux/netlink.h>
#include <linux/kobject.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <linux/gpio_keys.h>
#include <asm/mach-ath79/ar71xx_regs.h>


#define DRV_NAME	"wnr2200-keys"
#undef BH_DEBUG
#define BH_SKB_SIZE		2048

#ifdef BH_DEBUG
#define BH_DBG(fmt, args...) printk(KERN_DEBUG "%s: " fmt, DRV_NAME, ##args )
#else
#define BH_DBG(fmt, args...) do {} while (0)
#endif

#define BH_ERR(fmt, args...) printk(KERN_ERR "%s: " fmt, DRV_NAME, ##args )

#define WNR2200_GPIO_BUTTON_WIFI			3
#define WNR2200_GPIO_BUTTON_WPS				5
#define WNR2200_GPIO_BUTTON_RESET			6

#define WNR2200_PCI_GPIO_BASE 			0x20
#define WNR2200_PCI_GPIO_COUNT			10
#define WNR2200_KEYS_POLL_INTERVAL	20 /* msecs */
#define WNR2200_KEYS_DEBOUNCE_INTERVAL	(3 * WNR2200_KEYS_POLL_INTERVAL)

#define AR9287_BASE 																	AR71XX_PCI_MEM_BASE
#define AR9287_GPIO_IN_OUT                           0x4048 // GPIO input / output register
#define AR9287_GPIO_IN_VAL                           0x001FF800
#define AR9287_GPIO_IN_VAL_S                         11

#define AR9287_GPIO_OE_OUT                           0x404c // GPIO output enable register
#define AR9287_GPIO_OE_OUT_DRV                       0x3    // 2 bit field mask, shifted by 2*bitpos
#define AR9287_GPIO_OE_OUT_DRV_NO                    0x0    // tristate
#define AR9287_GPIO_OE_OUT_DRV_LOW                   0x1    // drive if low
#define AR9287_GPIO_OE_OUT_DRV_HI                    0x2    // drive if high
#define AR9287_GPIO_OE_OUT_DRV_ALL                   0x3    // drive always

#define AR9287_GPIO_OUTPUT_MUX_AS_OUTPUT             0
#define AR9287_GPIO_OUTPUT_MUX_AS_PCIE_ATTENTION_LED 1
#define AR9287_GPIO_OUTPUT_MUX_AS_PCIE_POWER_LED     2
#define AR9287_GPIO_OUTPUT_MUX_AS_TX_FRAME           3
#define AR9287_GPIO_OUTPUT_MUX_AS_RX_CLEAR_EXTERNAL  4
#define AR9287_GPIO_OUTPUT_MUX_AS_MAC_NETWORK_LED    5
#define AR9287_GPIO_OUTPUT_MUX_AS_MAC_POWER_LED      6

#define AR9287_GPIO_OUTPUT_MUX1                      0x4060
#define AR9287_GPIO_OUTPUT_MUX2                      0x4064
#define AR9287_GPIO_IE_VALUE                         0x4054 // GPIO input enable and value register

#define AR9287_GPIO_BIT(_gpio) (1 << (_gpio))

#define MS(_v, _f)  (((_v) & _f) >> _f##_S)

struct bh_priv {
	unsigned long		seen;
};

struct bh_event {
	const char		*name;
	unsigned int		type;
	char			*action;
	unsigned long		seen;

	struct sk_buff		*skb;
	struct work_struct	work;
};

struct bh_map {
	unsigned int	code;
	const char	*name;
};

struct gpio_keys_button_data {
	struct delayed_work work;
	struct bh_priv bh;
	int last_state;
	int count;
	int threshold;
	int can_sleep;
	struct gpio_keys_button *b;
};

struct gpio_keys_button_dev {
	int polled;
	struct delayed_work work;

	struct device *dev;
	struct gpio_keys_platform_data *pdata;
	struct gpio_keys_button_data data[0];
};
void __iomem *ar9287_gpio_base;

extern u64 uevent_next_seqnum(void);

#define BH_MAP(_code, _name)		\
	{				\
		.code = (_code),	\
		.name = (_name),	\
	}

static struct bh_map button_map[] = {
	BH_MAP(KEY_RESTART,	"reset"),
	BH_MAP(KEY_RFKILL,	"rfkill"),
	BH_MAP(KEY_WPS_BUTTON,	"wps"),
};

static struct platform_device *wnr2200_key_devs;

static int ar9287_gpio_get_value(struct gpio_chip *chip, unsigned offset)
{
	return (MS(__raw_readl(ar9287_gpio_base + AR9287_GPIO_IN_OUT), AR9287_GPIO_IN_VAL) & AR9287_GPIO_BIT(offset)) != 0;
}
static void ar9287_gpio_set_value(struct gpio_chip *chip,
				  unsigned offset, int value)
{
	unsigned long reg, mask;
	reg = __raw_readl(ar9287_gpio_base + AR9287_GPIO_IN_OUT);
	mask = ~(1 << offset);
	reg &= mask | ((value&1) << offset);
	__raw_writel(reg, ar9287_gpio_base + AR9287_GPIO_IN_OUT);
}

static int ar9287_gpio_direction_input(struct gpio_chip *chip,
				       unsigned offset)
{
	/*FIXME*/
	return 0;
}

static int ar9287_gpio_direction_output(struct gpio_chip *chip,
					unsigned offset, int value)
{
	/*FIXME*/
	return 0;
}

static struct gpio_chip ar9287_gpio_chip = {
	.label		= "ar9287-gpio",
	.get			= ar9287_gpio_get_value,
	.set			= ar9287_gpio_set_value,
	.direction_input	= ar9287_gpio_direction_input,
	.direction_output	= ar9287_gpio_direction_output,
	.ngpio		= WNR2200_PCI_GPIO_COUNT,
	.base			=	WNR2200_PCI_GPIO_BASE
};

static struct gpio_keys_button wnr2200_gpio_keys[] = {
	{
		.desc		= "rfkill",
		.type		= EV_KEY,
		.code		= KEY_RFKILL,
		.debounce_interval = WNR2200_KEYS_DEBOUNCE_INTERVAL,
		.gpio		= 0x20 + WNR2200_GPIO_BUTTON_WIFI,
		.active_low	= 1,
	},
	{
		.desc		= "wps",
		.type		= EV_KEY,
		.code		= KEY_WPS_BUTTON,
		.debounce_interval = WNR2200_KEYS_DEBOUNCE_INTERVAL,
		.gpio		= 0x20 + WNR2200_GPIO_BUTTON_WPS,
		.active_low	= 1,
	},
	{
		.desc		= "reset",
		.type		= EV_KEY,
		.code		= KEY_RESTART,
		.debounce_interval = WNR2200_KEYS_DEBOUNCE_INTERVAL,
		.gpio		= 0x20 + WNR2200_GPIO_BUTTON_RESET,
		.active_low	= 1,
	}
};

static int bh_event_add_var(struct bh_event *event, int argv,
		const char *format, ...)
{
	static char buf[128];
	char *s;
	va_list args;
	int len;

	if (argv)
		return 0;

	va_start(args, format);
	len = vsnprintf(buf, sizeof(buf), format, args);
	va_end(args);

	if (len >= sizeof(buf)) {
		BH_ERR("buffer size too small\n");
		WARN_ON(1);
		return -ENOMEM;
	}

	s = skb_put(event->skb, len + 1);
	strcpy(s, buf);

	BH_DBG("added variable '%s'\n", s);

	return 0;
}

static int button_hotplug_fill_event(struct bh_event *event)
{
	int ret;

	ret = bh_event_add_var(event, 0, "HOME=%s", "/");
	if (ret)
		return ret;

	ret = bh_event_add_var(event, 0, "PATH=%s",
					"/sbin:/bin:/usr/sbin:/usr/bin");
	if (ret)
		return ret;

	ret = bh_event_add_var(event, 0, "SUBSYSTEM=%s", "button");
	if (ret)
		return ret;

	ret = bh_event_add_var(event, 0, "ACTION=%s", event->action);
	if (ret)
		return ret;

	ret = bh_event_add_var(event, 0, "BUTTON=%s", event->name);
	if (ret)
		return ret;

	if (event->type == EV_SW) {
		ret = bh_event_add_var(event, 0, "TYPE=%s", "switch");
		if (ret)
			return ret;
	}

	ret = bh_event_add_var(event, 0, "SEEN=%ld", event->seen);
	if (ret)
		return ret;

	ret = bh_event_add_var(event, 0, "SEQNUM=%llu", uevent_next_seqnum());

	return ret;
}

static void button_hotplug_work(struct work_struct *work)
{
	struct bh_event *event = container_of(work, struct bh_event, work);
	int ret = 0;

	event->skb = alloc_skb(BH_SKB_SIZE, GFP_KERNEL);
	if (!event->skb)
		goto out_free_event;

	ret = bh_event_add_var(event, 0, "%s@", event->action);
	if (ret)
		goto out_free_skb;

	ret = button_hotplug_fill_event(event);
	if (ret)
		goto out_free_skb;

	NETLINK_CB(event->skb).dst_group = 1;
	broadcast_uevent(event->skb, 0, 1, GFP_KERNEL);

 out_free_skb:
	if (ret) {
		BH_ERR("work error %d\n", ret);
		kfree_skb(event->skb);
	}
 out_free_event:
	kfree(event);
}

static int button_hotplug_create_event(const char *name, unsigned int type,
		unsigned long seen, int pressed)
{
	struct bh_event *event;

	BH_DBG("create event, name=%s, seen=%lu, pressed=%d\n",
		name, seen, pressed);

	event = kzalloc(sizeof(*event), GFP_KERNEL);
	if (!event)
		return -ENOMEM;

	event->name = name;
	event->type = type;
	event->seen = seen;
	event->action = pressed ? "pressed" : "released";

	INIT_WORK(&event->work, (void *)(void *)button_hotplug_work);
	schedule_work(&event->work);

	return 0;
}
#ifdef	CONFIG_HOTPLUG
static int button_get_index(unsigned int code)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(button_map); i++)
		if (button_map[i].code == code)
			return i;

	return -1;
}

static void button_hotplug_event(struct gpio_keys_button_data *data,
			   unsigned int type, int value)
{
	struct bh_priv *priv = &data->bh;
	unsigned long seen = jiffies;
	int btn;

	BH_DBG("event type=%u, code=%u, value=%d\n", type, data->b->code, value);

	if ((type != EV_KEY) && (type != EV_SW))
		return;

	btn = button_get_index(data->b->code);
	if (btn < 0)
		return;

	button_hotplug_create_event(button_map[btn].name, type,
			(seen - priv->seen) / HZ, value);
	priv->seen = seen;
}
#else
static void button_hotplug_event(struct gpio_keys_button_data *data,
			   unsigned int type, int value)
{
}
#endif	/* CONFIG_HOTPLUG */

static int gpio_button_get_value(struct gpio_keys_button_data *bdata)
{
	int val;

	if (bdata->can_sleep)
		val = !!gpio_get_value_cansleep(bdata->b->gpio);
	else
		val = !!gpio_get_value(bdata->b->gpio);

	return val ^ bdata->b->active_low;
}

static void gpio_keys_polled_check_state(struct gpio_keys_button_data *bdata)
{
	int state = gpio_button_get_value(bdata);

	if (state != bdata->last_state) {
		unsigned int type = bdata->b->type ?: EV_KEY;

		if (bdata->count < bdata->threshold) {
			bdata->count++;
			return;
		}

		if ((bdata->last_state != -1) || (type == EV_SW))
			button_hotplug_event(bdata, type, state);

		bdata->last_state = state;
	}

	bdata->count = 0;
}

static void gpio_keys_polled_queue_work(struct gpio_keys_button_dev *bdev)
{
	struct gpio_keys_platform_data *pdata = bdev->pdata;
	unsigned long delay = msecs_to_jiffies(pdata->poll_interval);

	if (delay >= HZ)
		delay = round_jiffies_relative(delay);
	schedule_delayed_work(&bdev->work, delay);
}


static void gpio_keys_polled_poll(struct work_struct *work)
{
	struct gpio_keys_button_dev *bdev =
		container_of(work, struct gpio_keys_button_dev, work.work);
	int i;

	for (i = 0; i < bdev->pdata->nbuttons; i++) {
		struct gpio_keys_button_data *bdata = &bdev->data[i];
		gpio_keys_polled_check_state(bdata);
	}
	gpio_keys_polled_queue_work(bdev);
}

static void gpio_keys_polled_close(struct gpio_keys_button_dev *bdev)
{
	struct gpio_keys_platform_data *pdata = bdev->pdata;

	cancel_delayed_work_sync(&bdev->work);

	if (pdata->disable)
		pdata->disable(bdev->dev);
}

static int gpio_keys_button_probe(struct platform_device *pdev,
		struct gpio_keys_button_dev **_bdev, int polled)
{
	struct gpio_keys_platform_data *pdata = pdev->dev.platform_data;
	struct device *dev = &pdev->dev;
	struct gpio_keys_button_dev *bdev;
	struct gpio_keys_button *buttons;
	int error;
	int i;

	if (polled && !pdata->poll_interval) {
		dev_err(dev, "missing poll_interval value\n");
		return -EINVAL;
	}

	buttons = devm_kzalloc(dev, pdata->nbuttons * sizeof(struct gpio_keys_button),
		       GFP_KERNEL);
	if (!buttons) {
		dev_err(dev, "no memory for button data\n");
		return -ENOMEM;
	}
	memcpy(buttons, pdata->buttons, pdata->nbuttons * sizeof(struct gpio_keys_button));

	bdev = devm_kzalloc(dev, sizeof(struct gpio_keys_button_dev) +
		       pdata->nbuttons * sizeof(struct gpio_keys_button_data),
		       GFP_KERNEL);
	if (!bdev) {
		dev_err(dev, "no memory for private data\n");
		return -ENOMEM;
	}

	bdev->polled = polled;

	for (i = 0; i < pdata->nbuttons; i++) {
		struct gpio_keys_button *button = &buttons[i];
		struct gpio_keys_button_data *bdata = &bdev->data[i];
		unsigned int gpio = button->gpio;

		if (button->wakeup) {
			dev_err(dev, DRV_NAME "does not support wakeup\n");
			return -EINVAL;
		}

		error = devm_gpio_request(dev, gpio,
				     button->desc ? button->desc : DRV_NAME);
		if (error) {
			dev_err(dev, "unable to claim gpio %u, err=%d\n",
				gpio, error);
			return error;
		}

		error = gpio_direction_input(gpio);
		if (error) {
			dev_err(dev,
				"unable to set direction on gpio %u, err=%d\n",
				gpio, error);
			return error;
		}

		bdata->can_sleep = gpio_cansleep(gpio);
		bdata->last_state = -1;

		if (bdev->polled)
			bdata->threshold = DIV_ROUND_UP(button->debounce_interval,
						pdata->poll_interval);
		else
			bdata->threshold = 1;

		bdata->b = &pdata->buttons[i];
	}

	bdev->dev = &pdev->dev;
	bdev->pdata = pdata;
	platform_set_drvdata(pdev, bdev);

	*_bdev = bdev;

	return 0;
}
static int wnr2200_key_probe(struct platform_device *pdev)
{
	struct gpio_keys_platform_data *pdata;
	struct gpio_keys_button_dev *bdev;
	struct device *dev;
	int ret = -1;
	int i;

	dev = &pdev->dev;

	ret = gpio_keys_button_probe(pdev, &bdev, 1);

	if (ret) {
		dev_err(dev, "gpio_keys_button_probe failed\n");
		goto exit;
	}

	INIT_DELAYED_WORK(&bdev->work, gpio_keys_polled_poll);

	pdata = bdev->pdata;

	if (pdata->enable)
		pdata->enable(bdev->dev);

	for (i = 0; i < pdata->nbuttons; i++)
		gpio_keys_polled_check_state(&bdev->data[i]);

	gpio_keys_polled_queue_work(bdev);

exit:
	return ret;
}

static int wnr2200_key_remove(struct platform_device *pdev)
{
	struct gpio_keys_button_dev *bdev = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);

	if (bdev->polled)
	gpio_keys_polled_close(bdev);

	return 0;
}



static struct platform_driver wnr2200_key_driver = {
	.probe		= wnr2200_key_probe,
	.remove		= wnr2200_key_remove,
	.driver		= {
		.name	= DRV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(wnr2200_key_of_match),
	},
};

static int __init wnr2200_key_init(void)
{
	int ret = -1;
	struct gpio_keys_platform_data pdata;

	ar9287_gpio_base = ioremap_nocache(AR71XX_PCI_MEM_BASE, AR71XX_PCI_MEM_SIZE);
	if (ar9287_gpio_base == NULL) {
		printk("ar9287_gpio_base ioremap failed\n");
		goto exit1;
	}
	ret = gpiochip_add(&ar9287_gpio_chip);
	if (ret) {
		printk("gpiochip_add failed\n");
		goto exit2;
	}

	wnr2200_key_devs = platform_device_alloc(DRV_NAME, -1);
	if (!wnr2200_key_devs) {
		printk("platform_device_alloc failed\n");
		ret = -ENOMEM;
		goto exit3;
	}
	memset(&pdata, 0, sizeof(pdata));
	pdata.poll_interval = WNR2200_KEYS_POLL_INTERVAL;
	pdata.nbuttons = ARRAY_SIZE(wnr2200_gpio_keys);
	pdata.buttons = wnr2200_gpio_keys;

	ret = platform_device_add_data(wnr2200_key_devs, &pdata, sizeof(pdata));
	if (ret) {
		printk("platform_device_add_data failed\n");
		goto exit4;
	}

	ret = platform_device_add(wnr2200_key_devs);
	if (ret) {
		printk("platform_device_add failed\n");
		goto exit4;
	}
	ret = platform_driver_register(&wnr2200_key_driver);
	if (ret == 0) {
		return 0;
	}
	printk("platform_driver_register failed\n");
	platform_device_del(wnr2200_key_devs);
exit4:
	platform_device_put(wnr2200_key_devs);
exit3:
	gpiochip_remove(&ar9287_gpio_chip);
exit2:
	iounmap(ar9287_gpio_base);
exit1:
	wnr2200_key_devs = NULL;
	ar9287_gpio_base = NULL;
	return ret;
}

static void __exit wnr2200_key_exit(void)
{
	struct platform_device *dev = wnr2200_key_devs;


	platform_driver_unregister(&wnr2200_key_driver);
	platform_device_unregister(dev);

	gpiochip_remove(&ar9287_gpio_chip);
	iounmap(ar9287_gpio_base);
	ar9287_gpio_base = NULL;
	wnr2200_key_devs = NULL;

}
module_init(wnr2200_key_init);
module_exit(wnr2200_key_exit);


MODULE_AUTHOR("Tao Guo <guotao945@gmail.com>");
MODULE_AUTHOR("Gabor Juhos <juhosg@openwrt.org>");
MODULE_AUTHOR("Felix Fietkau <nbd@openwrt.org>");
MODULE_DESCRIPTION("WNR2200 keys driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform: " DRV_NAME);
