#include <linux/platform_device.h>
#include <linux/input-polldev.h>
#include <linux/regulator/consumer.h>
#include <linux/miscdevice.h>

enum blts_reg {
	ID 			= 0x00,
	OUTX_L		= 0x01,
	OUTX_H		= 0x02,
	OUTX		= 0x02,
	OUTY_L		= 0x03,
	OUTY_H		= 0x04,
	OUTY		= 0x04,
	OUTZ_L		= 0x05,
	OUTZ_H		= 0x06,
	OUTZ		= 0x06,
};

union axis_conversion {
	struct {
		int x, y, z;
	};
	int as_array[3];

};

struct bonelogic_ts {
	void			*bus_priv; /* used by the bus layer only */
	struct device		*pm_dev; /* for pm_runtime purposes */
	int (*init) (struct bonelogic_ts *dev);
	int (*write) (struct bonelogic_ts *dev, int reg, u8 val);
	int (*read) (struct bonelogic_ts *dev, int reg, u8 *ret);
	int (*blkread) (struct bonelogic_ts *dev, int reg, int len, u8 *ret);
	int (*reg_ctrl) (struct bonelogic_ts *dev, bool state);

	u8			whoami;    /* indicates measurement precision */
	s16 (*read_data) (struct bonelogic_ts *dev, int reg);
	int			mdps_max_val;
	int			pwron_delay;

	struct input_polled_dev	*idev;     /* input device */
	struct platform_device	*pdev;     /* platform device */
	struct regulator_bulk_data regulators[2];
	atomic_t		count;     /* interrupt count after last read */
	union axis_conversion	ac;        /* hw -> logical axis */
	int			mapped_btns[3];

	u32			irq;       /* IRQ number */
	struct fasync_struct	*async_queue; /* queue for the misc device */
	wait_queue_head_t	misc_wait; /* Wait queue for the misc device */
	unsigned long		misc_opened; /* bit0: whether the device is open */
	struct miscdevice	miscdev;

	int                     data_ready_count[2];
	atomic_t		wake_thread;
	unsigned char           irq_cfg;
	unsigned int		shift_adj;

	struct bonelogic_ts_platform_data *pdata;	/* for passing board config */
	struct mutex		mutex;     /* Serialize poll and selftest */

#ifdef CONFIG_OF
	struct device_node	*of_node;
#endif
};

int bonelogic_ts_init_device(struct bonelogic_ts *dev);
int bonelogic_ts_joystick_enable(struct bonelogic_ts *dev);
void bonelogic_ts_joystick_disable(struct bonelogic_ts *dev);
void bonelogic_ts_poweroff(struct bonelogic_ts *dev);
int bonelogic_ts_poweron(struct bonelogic_ts *dev);
int bonelogic_ts_remove_fs(struct bonelogic_ts *dev);
int bonelogic_ts_init_dt(struct bonelogic_ts *dev);

extern struct bonelogic_ts blts_dev;
