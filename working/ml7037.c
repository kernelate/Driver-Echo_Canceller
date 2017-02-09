#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/delay.h>

#include <linux/fcntl.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <asm/uaccess.h>
#include <mach/board.h>
#include <linux/spi/spi.h>

#include <mach/gpio.h>

#include "ml7037.h"

#if ML7037_DEBUG
#define DPRINTK(fmt, args...)   printk(fmt,## args)
#define TAG	"RAN_EDIT"
#endif

#define DEV_NAME "ml7037-003"
#define VERSION     "1.0.1" //added i2c interface for PDN & RST PIN

struct ml7037_device {
	struct device 		*dev;
	struct spi_device	*spi;
	struct workqueue_struct	*rx_wq;
	struct workqueue_struct *tx_wq;
	struct work_struct	rx_work;
	struct work_struct	tx_work;

	u8 spi_wbuffer[2];
	u8 spi_rbuffer[2];
	/* Image of the SPI registers in ML7037-003. */
	
	int master;	

	int master_rts;
	int master_rdy;

	char *rx_buf;
	int rx_len;

	char *tx_buf;
	int tx_len;
};

struct ml7037_i2c {
    struct input_dev *dev;
    struct timer_list   rd_timer;           // keyscan timer
    struct i2c_client *client;
    unsigned int                wakeup_delay;           // key wakeup delay
};

static struct   i2c_client *this_client = NULL;	
struct ml7037_device *ml7037;
struct delayed_work spi_read_work;

GPIO_PINS gpio_pins;

static u8 cr[21];
static u8 reg_old;

int delay;


static void ml7037_default(void)
{
	cr[2]	= CR2_DEFAULT;
	cr[3]	= CR3_DEFAULT;
	cr[4] 	= CR4_DEFAULT;
	cr[5] 	= CR5_DEFAULT;
	cr[11] 	= CR11_DEFAULT;
	cr[12] 	= CR12_DEFAULT;
	cr[13]	= CR13_DEFAULT;
	cr[14]	= CR14_DEFAULT;
	cr[16]	= CR16_DEFAULT;
	cr[17]	= CR17_DEFAULT;
	cr[18]  = CR18_DEFAULT;
	cr[20]	= CR20_DEFAULT;
}


static int ml7037_spi_write(unsigned int reg, unsigned int val)
{
	struct spi_message msg;
	struct spi_transfer xfer;
	int ret;

#if ML7037_DEBUG
	printk("%s: %s\n", TAG, __func__);
#endif
	spi_message_init(&msg);

	ml7037->spi_wbuffer[0] = reg;
	ml7037->spi_wbuffer[1] = val;

	xfer.tx_buf = ml7037->spi_wbuffer;
	xfer.rx_buf = ml7037->spi_rbuffer;
	spi_message_add_tail(&xfer, &msg);

	ret = spi_sync(ml7037->spi, &msg);

	return ret;
}

static void update_pins(int gpio, int value)
{
	if(gpio==1)
		gpio_direction_output(DOUT_GPIO, value);
	if(gpio==2)
		gpio_direction_output(DIN_GPIO, value);
	if(gpio==3)
		gpio_direction_output(EXCK_GPIO, value);
	if(gpio==4)
		gpio_direction_output(DEN_GPIO, value);
//	schedule_delayed_work(&updatepins, 20);
}

/****************************************************************************/
/*  ML7037-003 regsiter read                                                */
/*  Function : ml7037_read                                                     */
/*      Parameters                                                          */
/*          Input   :   unsigned char craddr                                */
/*          Output  :   unsigned char                                       */
/****************************************************************************/
unsigned char ml7037_read(unsigned char craddr)
{
	unsigned char ret_data = 0, i;
	int ret;
	u8 buf_rx[2];

#if ML7037_DEBUG
	printk("%s: %s\n", TAG, __func__);
#endif
	gpio_set_value(DEN_GPIO,HIGH);
	gpio_set_value(EXCK_GPIO,HIGH);
	mdelay(5);
	gpio_set_value(DEN_GPIO,LOW);
	gpio_set_value(EXCK_GPIO,LOW);

	//write address
	craddr = craddr | 0x80;
	for(i=0; i<8; i++)
	{
		gpio_set_value(EXCK_GPIO,LOW);
		if(craddr & 0x80)
		{
			gpio_set_value(DIN_GPIO,HIGH);
		}
		else
		{
			gpio_set_value(DIN_GPIO,LOW);
		}	
		gpio_set_value(EXCK_GPIO,HIGH);
		craddr <<= 1;
	}

	//read data
	for(i=0; i<8; i++)
	{
		ret_data <<= 1;
		gpio_set_value(EXCK_GPIO,LOW);
		ret = gpio_get_value(DOUT_GPIO);
		if(ret)
		{
			ret_data|=0x01;
		}
		else
		{
			ret_data&=0xfe;
		}
		gpio_set_value(EXCK_GPIO,HIGH);
	}
	spi_write(ml7037->spi, craddr, 0);
	ret = spi_read(ml7037->spi, buf_rx, 2);
	if (ret < 0)
	{
#if ML7037_DEBUG
		printk("%s: ret:%d\n", TAG, ret);
#endif
		return ret;
	}
	gpio_set_value(DEN_GPIO,HIGH);
	gpio_set_value(EXCK_GPIO,LOW);
	gpio_set_value(EXCK_GPIO,HIGH);
	return (buf_rx[0] << 8) |  buf_rx[1];
}

/****************************************************************************/
/*  ML7037-003 register write                                               */
/*  Function : ml7037_write                                                 */
/*      Parameters                                                          */
/*          Input   :   unsigned char craddr, unsigned char crdata          */
/*          Output  :   void                                        	    */
/****************************************************************************/
void ml7037_write(unsigned char craddr, unsigned char crdata)
{
	unsigned char i;
	int ret;
	u8 data[2];

#if ML7037_DEBUG
	printk("%s: %s\n", TAG, __func__);
#endif
	gpio_set_value(DEN_GPIO,HIGH);
	gpio_set_value(EXCK_GPIO,HIGH);
	mdelay(5);
	gpio_set_value(DEN_GPIO,LOW);
	gpio_set_value(EXCK_GPIO,LOW);
	//write address
	craddr = craddr & 0x7f;	
	for(i=0; i<8; i++)
	{
		gpio_set_value(EXCK_GPIO,LOW);
		if(craddr & 0x80)
		{
			gpio_set_value(DIN_GPIO,HIGH);
		}
		else
		{
			gpio_set_value(DIN_GPIO,LOW);
		}
		gpio_set_value(EXCK_GPIO,HIGH);
		craddr <<= 1;
	}

	//write data
	for(i=0; i<8; i++)
	{
		gpio_set_value(EXCK_GPIO,LOW);
		if(crdata & 0x80)
		{
			gpio_set_value(DIN_GPIO,HIGH);
		}
		else
		{
			gpio_set_value(DIN_GPIO,LOW);
		}
		gpio_set_value(EXCK_GPIO,HIGH);
		crdata <<= 1;
	}
	data[0] = craddr;
	data[1] = crdata;
	ret = spi_write(ml7037->spi, data, 2);
//	ret = ml7037_spi_write(craddr,crdata);
	if (ret) {
#if ML7037_DEBUG
		printk("%s: ml7037 reg read error\n", TAG);
#endif
	}
	gpio_set_value(DEN_GPIO,HIGH);
	gpio_set_value(EXCK_GPIO,LOW);
	gpio_set_value(EXCK_GPIO,HIGH);
		
}

/****************************************************************************/
/*  ML7037-003 internal data memory write                                   */
/*  Function : memory_write                                                 */
/*      Parameters                                                          */
/*          Input   :   unsigned int adrs, unsigned int data              */
/*          Output  :   void                                              */
/****************************************************************************/
void memory_write(unsigned char adr_h, unsigned char adr_l, unsigned char data_h, unsigned char data_l)
{
	/* write high address (CR6) */
    	ml7037_write(CR6, adr_h);
    	/* write low address (CR7) */
    	ml7037_write(CR7, adr_l);
    	/* write high data (CR8) */
	ml7037_write(CR8, data_h);
    	/* write low data (CR9) */
    	ml7037_write(CR9, data_l);
    	/* Internal memory write operation  */
    	ml7037_write(CR1, 0x80);
    	/* Write operation check */
    	read_addr = CR1;
	delay = msecs_to_jiffies(100);
	schedule_delayed_work(&spi_read_work,10);
}

static void ml7037_write_specific(unsigned char craddr, unsigned char reg_val, int cr_no)
{
	unsigned char reg_new;

	reg_old = cr[cr_no];
	reg_old = reg_old | 0xf0;
	reg_new = reg_val | reg_old; 
	cr[cr_no] = reg_new;
	ml7037_write(craddr, reg_new);
}

static long ml7037_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	ML7037_data m_data;
	int len;

	len = 0;

#if ML7037_DEBUG
	printk("%s: %s\n", TAG, __func__);
#endif
	len = copy_from_user(&m_data, (struct __user ML7037_data *) arg, sizeof(m_data));
	if (len > 0)
		return -EFAULT;

	switch(cmd)
	{
		case ML7037_RVOL:
			ml7037_write_specific(CR2, (unsigned char) m_data.value, 2);		
			break;
		case ML7037_TVOL:		
			ml7037_write(CR3, m_data.value);
			break;
		case ML7037_APGA:		
			ml7037_write(CR4, m_data.value);
			break;
		case ML7037_LPGA:		
			ml7037_write(CR5, m_data.value);
			break;
		case ML7037_LTHR_MODE: //Line Echo Canceller Through Mode Selection Register		
			ml7037_write_specific(CR11, (unsigned char) m_data.value, 11);	
			break;
		case ML7037_LECEN: //Line Echo Canceller Enable Register		
			ml7037_write_specific(CR11, (unsigned char) m_data.value, 11);	
			break;
		case ML7037_LHLD: //Line Echo Canceller Filter Coefficients Update Suspension Register		
			ml7037_write_specific(CR11, (unsigned char) m_data.value, 11);
			break;
		case ML7037_LCLP: //Line Echo Canceller Center Clip On/Off Register	
			ml7037_write_specific(CR11, (unsigned char) m_data.value, 11);
			break;
		case ML7037_LSLC: //Line Echo Canceller Automatic SinL Level Control Register	
			ml7037_write_specific(CR11, (unsigned char) m_data.value, 11);	
			break;
		case ML7037_LATT: //Line Echo Canceller ATT On/Off Register	
			ml7037_write_specific(CR11, (unsigned char) m_data.value, 11);
			break;
		case ML7037_ATHR_MODE: //Acoustic Echo Canceller Through Mode Selection Register	
			ml7037_write_specific(CR12, (unsigned char) m_data.value, 12);
			break;
		case ML7037_AECEN: //Acoustic Echo Canceller Enable Register	
			ml7037_write_specific(CR12, (unsigned char) m_data.value, 12);	
			break;
		case ML7037_AHLD: //Acoustic Echo Canceller Filter Coefficients Update Suspension Register		
			ml7037_write_specific(CR12, (unsigned char) m_data.value, 12);
			break;
		case ML7037_ACLP: //Acoustic Echo Canceller Center Clip On/Off Register	
			ml7037_write_specific(CR12, (unsigned char) m_data.value, 12);
			break;
		case ML7037_ASLC: //Acoustic Echo Canceller Automatic SinA Level Control Register		
			ml7037_write_specific(CR12, (unsigned char) m_data.value, 12);	
			break;
		case ML7037_AATT: //Acoustic Echo Canceller ATT On/Off Register		
			ml7037_write_specific(CR12, (unsigned char) m_data.value, 12);
			break;
		case ML7037_ASOPAD: //Acoustic Echo Canceller SoutA Gain Level Control Registers		
			ml7037_write_specific(CR13, (unsigned char) m_data.value, 13);
			break;
		case ML7037_ASIPAD: //Acoustic Echo Canceller SinA Loss Level Control Registers		
			ml7037_write_specific(CR13, (unsigned char) m_data.value, 13);
			break;
		case ML7037_LSOPAD: //Line Echo Canceller SoutL Gain Level Control Registers		
			ml7037_write_specific(CR13, (unsigned char) m_data.value, 13);	
			break;
		case ML7037_LSIPAD: //Line Echo Canceller SinL Loss Level Control Registers		
			ml7037_write_specific(CR13, (unsigned char) m_data.value, 13);
			break;
		case ML7037_SLPTHR_MODE: //Slope Filter Through Mode Selection Register		
			ml7037_write_specific(CR14, (unsigned char) m_data.value, 14);	
			break;
		case ML7037_NCTHR_MODE: //Noise Canceller Through Mode Selection Register	
			ml7037_write_specific(CR14, (unsigned char) m_data.value, 14);
			break;
		case ML7037_AVREFEN: //Power-Down State AVREF/Analog Output Amps Control Register		
			ml7037_write_specific(CR16, (unsigned char) m_data.value, 16);
			break;
		case ML7037_AVFROSEL: //Acoustic Side Analog Output Selection Register	
			ml7037_write_specific(CR16, (unsigned char) m_data.value, 16);	
			break;
		case ML7037_LVFROSEL: //Line- Side Analog Output Selection Register	
			ml7037_write_specific(CR16, (unsigned char) m_data.value, 16);
			break;
		case ML7037_AATT_MODE: //ATTsA Operation Mode Selection Register	
			ml7037_write_specific(CR17, (unsigned char) m_data.value, 17);
			break;
		case ML7037_LATT_MODE: //ATTsA Operation Mode Selection Register	
			ml7037_write_specific(CR18, (unsigned char) m_data.value, 18);
			break;
		case ML7037_EQLEN: //Equalizer On/Off Register
			ml7037_write_specific(CR20, (unsigned char) m_data.value, 20);
			break;
		case ML7037_EQL_MODE: //Equalizer mode selection register	
			ml7037_write_specific(CR20, (unsigned char) m_data.value, 20);
			break;
		case ML7037_PDN: //Power Button	
			i2c_smbus_write_byte_data(this_client,PDN_EN_REG,m_data.value);
			break;

		case ML7037_RST: //Reset Button
			i2c_smbus_write_byte_data(this_client,RST_EN_REG,m_data.value);
			break;
		
		default:
			DPRINTK("RAN_EDIT ioctl cmd not found\n");
			return -1;
			break;
	}		

	return 0;
}

static int ml7037_open(struct inode *inode, struct file *file)
{
//	printk("RAN_EDIT: dt2_open\n");
	return 0;
}

static int ml7037_close(struct inode *inode, struct file *file)
{
//	printk("RAN_EDIT: dt2_close\n");
	return 0;
}

static long ml7037_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
static int ml7037_open(struct inode *inode, struct file *file);
static int ml7037_close(struct inode *inode, struct file *file);

static const struct file_operations ml7037_fops = 
{
        .owner = THIS_MODULE,
        .open = ml7037_open,
        .release = ml7037_close,
        .unlocked_ioctl = ml7037_ioctl,
};

static struct miscdevice ml7037_misc = 
{
	MISC_DYNAMIC_MINOR,
	DEV_NAME,
	&ml7037_fops,
};

static void work_spi_read(struct work_struct *pWork) {

	unsigned char value;

#if ML7037_DEBUG
	printk("%s: %s\n", TAG, __func__);
#endif

	value = ml7037_read(read_addr);
	if(read_addr == CR10)
	{
		if(value == STATUS_CHECK && status == CHECK)
		{
			status = READY;
#if ML7037_DEBUG
			printk("%s: status%d\n", TAG, status);
#endif
			goto stop;
		}
		if(value == STATUS_READY && status == READY)
		{
#if ML7037_DEBUG
			printk("%s: status%d\n", TAG, status);
#endif
			goto stop;
		}
	}

	if(read_addr == CR1)
	{
		if(value == STATUS_READY)
		{
			goto stop;
		}
	}	
	schedule_delayed_work(&spi_read_work,delay); //250 ms
stop:
#if ML7037_DEBUG
	printk("%s: STOP work_spi_read!\n", TAG);
#endif
	cancel_delayed_work(&spi_read_work);
}

void init_spi_read_work(void)
{
	INIT_DELAYED_WORK(&spi_read_work,work_spi_read);			
}

void deinit_spi_read_work(void){
	cancel_delayed_work_sync(&spi_read_work);
}

void ml7037_power_off(void)
{
	i2c_smbus_write_byte_data(this_client,PDN_EN_REG,0x00);
	gpio_set_value(DEN_GPIO,HIGH);
	gpio_set_value(EXCK_GPIO,LOW);
	gpio_set_value(DIN_GPIO,LOW);
}

void ml7037_power_on(void)
{
	i2c_smbus_write_byte_data(this_client,PDN_EN_REG,0x01);
	gpio_set_value(DEN_GPIO,HIGH);
	gpio_set_value(EXCK_GPIO,LOW);
	gpio_set_value(DIN_GPIO,LOW);	
}

/********************************************/
/* Internal Data Memory Access              */
/********************************************/
void init_ml7037_reg(void)
{
#if ML7037_DEBUG
	printk("%s: %s\n", TAG, __func__);
#endif
	ml7037_default();
	//ML7037 POWER ON/OFF
	ml7037_power_off();
	mdelay(20);
	ml7037_power_on();

	read_addr = CR10;
	delay = msecs_to_jiffies(300);
	schedule_delayed_work(&spi_read_work,10);

	// Initial Set (Do not edit following 17 lines.)
    	memory_write(0x0F, 0x00, 0x1F, 0xC6);
    	memory_write(0x0B, 0xB5, 0x00, 0x09);
    	memory_write(0x0B, 0xB6, 0x00, 0x12);
    	memory_write(0x0F, 0x8B, 0x70, 0x00);
    	memory_write(0x0F, 0x8C, 0x08, 0x00);
    	memory_write(0x0F, 0x96, 0x00, 0x01);
    	memory_write(0x07, 0x82, 0x7F, 0xFF);
    	memory_write(0x07, 0x83, 0x7F, 0xFF);
    	memory_write(0x07, 0x84, 0x55, 0x55);
    	memory_write(0x07, 0x85, 0x55, 0x55);
    	memory_write(0x07, 0x6B, 0x02, 0x07);
    	memory_write(0x07, 0x7E, 0x02, 0x07);
    	memory_write(0x07, 0x7F, 0x02, 0x07);
    	memory_write(0x07, 0x80, 0x02, 0x07);
    	memory_write(0x07, 0x81, 0x02, 0x07);
    	memory_write(0x07, 0x06, 0x00, 0x80);
    	memory_write(0x07, 0x0B, 0x75, 0xC4);

    	// [ALC Maximum Gain = 9dB]
    	// (0x0F11) = 0x0284
    	memory_write(0x0F, 0x11, 0x02, 0x84);

    	// [ATTsA On to Off Transition Speed Mode = 2]
    	// (0x0777) = 0x0000
    	// (0x077D) = 0x0050
    	memory_write(0x07, 0x77, 0x00, 0x00);
    	memory_write(0x07, 0x7D, 0x00, 0x50);

    	// [ATTrA Attenuation = 9dB]
    	// (0x0ABB) = 0x2D6A
    	memory_write(0x0A, 0xBB, 0x2D, 0x6A);

    	// [Acoustic Echo Canceller Cancellable Echo Delay Time = 50ms]
    	// (0x0ABD) = 0x0190
    	memory_write(0x0A, 0xBD, 0x01, 0x90);

    	/********************************************/
    	/* Control Registers set                    */
    	/********************************************/

    	// CR2  Bit4    RALC  OFF:0x10, ON:0x00
    	//      Bit3-0  RPAD  0dB: 0x00,  +3dB: 0x01,  +6dB: 0x02,  +9dB: 0x03
    	//                  +12dB: 0x04, +15dB: 0x05, +18dB: 0x06, +21dB: 0x07
    	//                   -3dB: 0x0F,  -6dB: 0x0E,  -9dB; 0x0D, -12dB: 0x0C
    	//                  -15dB: 0x0B, -18dB: 0x0A, -21dB; 0x09,  MUTE: 0x08
    	ml7037_write(CR2,  CR2_DEFAULT);

    	// CR3  Bit3-0  TPAD  0dB: 0x00,  +3dB: 0x01,  +6dB: 0x02,  +9dB: 0x03
    	//                  +12dB: 0x04, +15dB: 0x05, +18dB: 0x06, +21dB: 0x07
    	//                   -3dB: 0x0F,  -6dB: 0x0E,  -9dB; 0x0D, -12dB: 0x0C
    	//                  -15dB: 0x0B, -18dB: 0x0A, -21dB; 0x09,  MUTE: 0x08
    	ml7037_write(CR3,  CR3_DEFAULT);

    	// CR4  Bit4-0  APGA  0dB: 0x00,  +2dB: 0x01,  +4dB: 0x02,  +6dB: 0x03
    	//                   +8dB: 0x04, +10dB: 0x05, +12dB: 0x06, +14dB: 0x07
    	//                  +16dB: 0x08, +18dB: 0x09, +20dB: 0x0A
    	//                   -2dB: 0x1F,  -4dB: 0x1E,  -6dB; 0x1D,  -8dB: 0x1C
    	//                  -10dB: 0x1B, -12dB: 0x1A, -14dB; 0x19, -16dB: 0x18
    	//                  -18dB: 0x17, -20dB: 0x16
    	ml7037_write(CR4,  CR4_DEFAULT);

    	// CR5  Bit4-0  LPGA  0dB: 0x00,  +2dB: 0x01,  +4dB: 0x02,  +6dB: 0x03
    	//                   +8dB: 0x04, +10dB: 0x05, +12dB: 0x06, +14dB: 0x07
    	//                  +16dB: 0x08, +18dB: 0x09, +20dB: 0x0A
    	//                   -2dB: 0x1F,  -4dB: 0x1E,  -6dB; 0x1D,  -8dB: 0x1C
    	//                  -10dB: 0x1B, -12dB: 0x1A, -14dB; 0x19, -16dB: 0x18
    	//                  -18dB: 0x17, -20dB: 0x16
    	ml7037_write(CR5,  CR5_DEFAULT);

    	// CR11  Line Echo Canceller
    	//       Bit7  Path        0x00:Normal,  0x80:Through
    	//       Bit6  Operation   0x00:Disable, 0x40:Enable
    	//       Bit5  Coefficient 0x00:Update,  0x20:Hold
    	//       Bit3  Center Clip 0x00:Disable, 0x08:Enable
    	//       Bit2  ASLLC       0x00:Disable, 0x04:Enable   *Automatic SinL Level Control
    	//       Bit1  ATTsL       0x00:Enable,  0x02:Disable
    	ml7037_write(CR11, CR11_DEFAULT);

    	// CR12  Acoustic Echo Canceller
    	//       Bit7  Path        0x00:Normal,  0x80:Through
    	//       Bit6  Operation   0x00:Disable, 0x40:Enable
    	//       Bit5  Coefficient 0x00:Update,  0x20:Hold
    	//       Bit3  Center Clip 0x00:Disable, 0x08:Enable
    	//       Bit2  ASALC       0x00:Disable, 0x04:Enable   *Automatic SinA Level Control
    	//       Bit1  ATTsA       0x00:Enable,  0x02:Disable
    	ml7037_write(CR12, CR12_DEFAULT);

    	// CR13  GLPAD
    	//       Bit7-6  ASOPAD  0dB: 0x00, +6dB: 0x40, +12dB: 0x80, +18dB: 0xC0
    	//       Bit5-4  ASIPAD  0dB: 0x00, -6dB: 0x10, -12dB: 0x20, -18dB: 0x30
    	//       Bit3-2  LSOPAD  0dB: 0x00, +6dB: 0x04, +12dB: 0x08, +18dB: 0x0C
    	//       Bit1-0  LSIPAD  0dB: 0x00, -6dB: 0x01, -12dB: 0x02, -18dB: 0x03
    	ml7037_write(CR13, CR13_DEFAULT);

    	// CR14  Bit7  SLPTHR  0x00:Normal,  0x80:Through   *Slopfilter
    	//       Bit3  NCTHR   0x00:Normal,  0x08:Through   *Noise Canceller
    	ml7037_write(CR14, CR14_DEFAULT);

    	// CR16  Bit7  AVREFEN   0x00:Disable,  0x80:Enable
    	//       Bit1  AVFROSEL  0x00:Disable,  0x02:Enable
    	//       Bit0  LVFROSEL  0x00:Disable,  0x01:Enable
    	ml7037_write(CR16, CR16_DEFAULT);

    	// CR17  Bit1-0  AATTMODE  0x03:Type A, 0x02:Type B, 0x01:Type C, 0x00:Type D
    	ml7037_write(CR17, CR17_DEFAULT);

    	// CR18  Bit1-0  LATTMODE  0x03:Type A, 0x02:Type B, 0x01:Type C, 0x00:Type D
    	ml7037_write(CR18, CR18_DEFAULT);

    	// CR20  Bit4    RDC_NULL_EN  0x00:Disable, 0x10:Enable
    	//       Bit3    EQL_EN       0x00:Disable, 0x08:Enable
    	//       Bit2-0  EQL          0x00:Mode0, 0x01:Mode1, 0x02:Mode2, 0x03:Mode3
    	//                            0x04:Mode4, 0x05:Mode5, 0x06:Mode6, 0x07:Mode7
    	//               Lower Frequency Level
    	//               Mode5 > Mode4 > Mode3 > Mode2 > Mode1 > Mode0 > Mode6 > Mode7
    	ml7037_write(CR20, CR20_DEFAULT);

    	// CR0  Bit7  SPDN      0x00:Normal,  0x80:Power Donw
    	//      Bit6  RST       0x00:Normal,  0x40:Reset       *Internal Coefficient Reset
    	//      Bit5  /LINEEN   0x00:Enable,  0x20:Disable     *Line Analog Interface
    	//      Bit4  PCMSEL    0x00:16bit ,  0x10:G.711       *PCM I/F Format
    	//      Bit3  /CLKEN    0x00:Output,  0x08:Not Output  *BCLK, SYNC Output
    	//      Bit2  /PCMEN    0x00:Enabled, 0x04:Disabled    *PCM I/F
    	//      Bit1  ECSEL     0x00:Single,  0x02:Dual        *Echo Canceller Mode
    	//      Bit0  OPE_STAT  0x00:Initial, 0x01:Normal      *Initial Mode or Normal Mode
    	//
    	// ******** CR0 has to be set after setting of other registers. ********
    	ml7037_write(CR0, CR0_DEFAULT);
	/********************************************/
    	/* Start Operation                          */
    	/********************************************/
	read_addr = CR10;
	delay = msecs_to_jiffies(100);
	schedule_delayed_work(&spi_read_work,10);
}

int init_gpio(void)
{
	int ret;

#if ML7037_DEBUG
	printk("%s: %s\n", TAG, __func__);
#endif
	ret = gpio_request(DOUT_GPIO, "DOUT_GPIO");
	if(ret<0)
	{
#if ML7037_DEBUG
		printk("%s: ERR: DOUT_GPIO request failed!\n", TAG);
#endif
		return ret;
	}
	gpio_direction_input(DOUT_GPIO);

	ret = gpio_request(DIN_GPIO, "DIN_GPIO");
	if(ret<0)
	{
#if ML7037_DEBUG
		printk("%s: ERR: DIN_GPIO request failed!\n", TAG);
#endif
		return ret;
	}
	ret = gpio_request(EXCK_GPIO, "EXCK_GPIO");
	if(ret<0)
	{
#if ML7037_DEBUG
		printk("%s: ERR: EXCK_GPIO request failed!\n", TAG);
#endif
		return ret;
	}
	ret = gpio_request(DEN_GPIO, "DEN_GPIO");
	if(ret<0)
	{
#if ML7037_DEBUG
		printk("%s: ERR: DEN_GPIO request failed!\n", TAG);
#endif
		return ret;
	}

	i2c_smbus_write_byte_data(this_client,PDN_EN_REG,0x01);
	i2c_smbus_write_byte_data(this_client,RST_EN_REG,0x01);

	gpio_direction_output(DIN_GPIO, LOW);
	gpio_direction_output(EXCK_GPIO, LOW);
	gpio_direction_output(DEN_GPIO, HIGH);

	return 0;
}

static int __devinit ml7037_probe(struct spi_device *spi)
{
	int ret;

#if ML7037_DEBUG
	printk("%s: %s\n", TAG, __func__);
#endif	
	/* Allocate driver data */
	ml7037 = (struct ml7037_device *)kzalloc(sizeof(struct ml7037_device), GFP_KERNEL);
	if (!ml7037){
#if ML7037_DEBUG
		printk("%s: ERR: no memory for ml7037_data\n", TAG);
#endif		
		return -ENOMEM;
	}

	/* Initialize the driver data */
	ml7037->spi = spi;
	ml7037->dev = &spi->dev;
	dev_set_drvdata(ml7037->dev,ml7037);	

	spi->bits_per_word = 16;
	spi->mode = SPI_MODE_0;
	spi->max_speed_hz = 12288000;
	ret = spi_setup(spi);
	if(ret<0)
	{
#if ML7037_DEBUG
		printk("%s: ERR:fail to setup spi driver\n", TAG);
#endif
		return ret;
	}
#if ML7037_DEBUG
		printk("%s: ML7037-003 spi device initialized!\n", TAG);
#endif	
	init_gpio();
	init_spi_read_work();
	init_ml7037_reg();

	return 0;
}

static int ml7037_remove(struct spi_device *spi)
{
	struct ml7037_device *ml7037 = dev_get_drvdata(&spi->dev);
	status = CHECK;

	cancel_delayed_work(&spi_read_work);
	
	gpio_free(DOUT_GPIO);
	gpio_free(DIN_GPIO);
	gpio_free(EXCK_GPIO);
	gpio_free(DEN_GPIO);
	kfree(ml7037);
	ml7037 = NULL;
	return 0;
}

static int i2c_init_client(struct i2c_client *client)
{
	int status = 0;
	struct ml7037_i2c *data = i2c_get_clientdata(client);
	data->client = client;

//	printk("RAN_EDIT: %s\n", __func__);
	return status;
}

static int __devexit i2c_remove(struct i2c_client *client)
{
	i2c_smbus_write_byte_data(this_client,RST_EN_REG,0x00);
	ml7037_power_off();
	kfree(i2c_get_clientdata(client));
	return 0;
}

static int __devinit i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;
	struct ml7037_i2c *data;

#if ML7037_DEBUG
	printk("RAN_EDIT: I2C- PROBE START\n");
#endif
	data = kzalloc(sizeof(struct ml7037_i2c), GFP_KERNEL);
//	if(!data)
    	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		err = -ENOMEM;
#if ML7037_DEBUG     
		printk("RAN_EDIT: !i2c_check_functionality\n");
#endif        
		goto exit;
	}

#if ML7037_DEBUG
	printk("%s: I2C init\n", TAG);
#endif
	i2c_set_clientdata(client, data);
    	data->client=client;
    	this_client=client;	
	/* Initialize the chip */
	err = i2c_init_client(client);
	if(err)
		goto exit_free;

	goto exit;
	
	exit_free:
		kfree(data);
	exit:
		return err;
}

static const struct i2c_device_id ml7037_i2c_id[] =
{
	{ DEV_NAME, 0 },
	{  }
};

MODULE_DEVICE_TABLE(i2c, ml7037_i2c_id);

struct i2c_driver ml7037_i2c = 
{
	.driver =
	{
		.name = DEV_NAME,
		.owner = THIS_MODULE,
	},
	.id_table = ml7037_i2c_id,
	.probe    = i2c_probe,
	.remove   = i2c_remove
}; 

static const struct spi_device_id ml7037_id[] = 
{
	{DEV_NAME, 1}
};

static struct spi_driver ml7037_driver = {
	.driver		= {
		.name	= DEV_NAME,
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.id_table	= ml7037_id,
	.probe		= ml7037_probe,
	.remove		= __devexit_p(ml7037_remove),
};

static int __init ml7037_init(void)
{
	int ret;
#if ML7037_DEBUG
	printk("%s: %s\n", TAG, __func__);
#endif	
	ret = i2c_add_driver(&ml7037_i2c);
	if (ret)
	{
		printk("i2c add driver failed\n");
		return -1;
	}
	misc_register(&ml7037_misc);
	return spi_register_driver(&ml7037_driver);
}

static void __exit ml7037_exit(void)
{
#if ML7037_DEBUG
	printk("%s: %s\n", TAG, __func__);
#endif
	misc_deregister(&ml7037_misc);
	spi_unregister_driver(&ml7037_driver);
}

module_init(ml7037_init);
module_exit(ml7037_exit);

MODULE_AUTHOR("Randy M. Aguinaldo (ranaguinaldo@gmail.com)");
MODULE_DESCRIPTION("ML7037-003 Echo Canceller Driver");
MODULE_VERSION(VERSION);
MODULE_LICENSE("GPL");
