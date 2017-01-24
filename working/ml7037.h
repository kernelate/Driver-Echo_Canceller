#define ML7037_DEBUG	1

#define DOUT_GPIO	RK30_PIN0_PD4
#define DIN_GPIO	RK30_PIN0_PD5
#define EXCK_GPIO	RK30_PIN0_PD6
#define DEN_GPIO	RK30_PIN0_PD7

#define HIGH	1
#define LOW	0

#define STATUS_CHECK		0x80
#define STATUS_READY		0x00

#define CHECK			0
#define READY			1

#define WRITE_CHECK		0x80
#define WRITE_READY		0x00

//DEFAULT VALUES
#define CR0_DEFAULT		0x0D
#define CR2_DEFAULT		0x02 //+6db
#define CR3_DEFAULT		0x02 //+6db
#define CR4_DEFAULT		0x03 //+6db
#define CR5_DEFAULT		0x03 //+6db LINE SIDE
#define CR11_DEFAULT		0x4C //NORMAL|LECER = ENABLE|UPDATE|CLP=ON|ASLLC=ENABLE|ATTSL=ENABLE
#define CR12_DEFAULT		0x4C //NORMAL|LECER = ENABLE|UPDATE|CLP=ON|ASLLC=ENABLE|ATTSL=ENABLE
#define CR13_DEFAULT		0x44
#define CR14_DEFAULT		0x80
#define CR16_DEFAULT		0x83
#define CR17_DEFAULT		0x02
#define CR18_DEFAULT		0x02
#define CR20_DEFAULT		0x10


int status = CHECK;
unsigned char read_addr = 0xFF;

typedef struct
{
	int dout;
	int din;
	int exck;
	int den;
}GPIO_PINS;

#define ML7037_IOCTL  'm'

typedef struct {
    unsigned int value;
    unsigned int reg_num;
    unsigned int read;
} ML7037_data;


/* functions */
#if 0
void memory_write(unsigned int adr_h, unsigned int data_h);
void ml7037_write(unsigned int craddr, unsigned int crdata); /*ML7037 register write*/
#endif
void memory_write(unsigned char adr_h, unsigned char adr_l, unsigned char data_h, unsigned char data_l);
void ml7037_write(unsigned char craddr, unsigned char crdata); /*ML7037 register write*/
unsigned char ml7037_read(unsigned char craddr); /*ML7037 register read*/


#define ML7037_RVOL					_IOW(ML7037_IOCTL, 0x01, ML7037_data) //CR2
#define ML7037_TVOL					_IOW(ML7037_IOCTL, 0x02, ML7037_data) //CR3
#define ML7037_APGA					_IOW(ML7037_IOCTL, 0x03, ML7037_data) //CR4
#define ML7037_LPGA					_IOW(ML7037_IOCTL, 0x04, ML7037_data) //CR5
//CR11
#define ML7037_LTHR_MODE				_IOW(ML7037_IOCTL, 0x05, ML7037_data) 
#define ML7037_LECEN					_IOW(ML7037_IOCTL, 0x06, ML7037_data) 
#define ML7037_LHLD					_IOW(ML7037_IOCTL, 0x07, ML7037_data) 
#define ML7037_LCLP					_IOW(ML7037_IOCTL, 0x08, ML7037_data) 
#define ML7037_LSLC					_IOW(ML7037_IOCTL, 0x09, ML7037_data) 
#define ML7037_LATT					_IOW(ML7037_IOCTL, 0x0A, ML7037_data) 
//CR12
#define ML7037_ATHR_MODE				_IOW(ML7037_IOCTL, 0x0B, ML7037_data) 
#define ML7037_AECEN					_IOW(ML7037_IOCTL, 0x0C, ML7037_data) 
#define ML7037_AHLD					_IOW(ML7037_IOCTL, 0x0D, ML7037_data) 
#define ML7037_ACLP					_IOW(ML7037_IOCTL, 0x0E, ML7037_data) 
#define ML7037_ASLC					_IOW(ML7037_IOCTL, 0x0F, ML7037_data) 
#define ML7037_AATT					_IOW(ML7037_IOCTL, 0x10, ML7037_data)
//CR13
#define ML7037_ASOPAD					_IOW(ML7037_IOCTL, 0x11, ML7037_data) 
#define ML7037_ASIPAD					_IOW(ML7037_IOCTL, 0x12, ML7037_data) 
#define ML7037_LSOPAD					_IOW(ML7037_IOCTL, 0x13, ML7037_data) 
#define ML7037_LSIPAD					_IOW(ML7037_IOCTL, 0x14, ML7037_data)
//CR14 
#define ML7037_SLPTHR_MODE				_IOW(ML7037_IOCTL, 0x15, ML7037_data) 
#define ML7037_NCTHR_MODE				_IOW(ML7037_IOCTL, 0x16, ML7037_data)
//CR16
#define ML7037_AVREFEN					_IOW(ML7037_IOCTL, 0x17, ML7037_data) 
#define ML7037_AVFROSEL					_IOW(ML7037_IOCTL, 0x18, ML7037_data) 
#define ML7037_LVFROSEL					_IOW(ML7037_IOCTL, 0x19, ML7037_data) 
//CR17
#define ML7037_AATT_MODE				_IOW(ML7037_IOCTL, 0x1A, ML7037_data)
//CR18
#define ML7037_LATT_MODE				_IOW(ML7037_IOCTL, 0x1B, ML7037_data)
//CR20
#define ML7037_EQLEN					_IOW(ML7037_IOCTL, 0x1C, ML7037_data)
#define ML7037_EQL_MODE					_IOW(ML7037_IOCTL, 0x1D, ML7037_data)

/************************************************************************************/
/*                                                                                  */
/*      Copyright (C) 2011 LAPIS Semiconductor Co., LTD.                            */
/*                                                                                  */
/*      System Name     :  ML674051/ML674061 series                                 */
/*      Module Name     :  ML7037-003 header file                                   */
/*      File   Name     :  ML7037_003.h                                             */
/*      Revision        :  01.00                                                    */
/*      Date            :  2011/10/01                                               */
/*                                                                                  */
/************************************************************************************/

/*****************************************************/
/*    Control Register Address                       */
/*****************************************************/
#define CR0 (0x00)
#define CR1 (0x01)
#define CR2 (0x02)
#define CR3 (0x03)
#define CR4 (0x04)
#define CR5 (0x05)
#define CR6 (0x06)
#define CR7 (0x07)
#define CR8 (0x08)
#define CR9 (0x09)
#define CR10 (0x0A)
#define CR11 (0x0B)
#define CR12 (0x0C)
#define CR13 (0x0D)
#define CR14 (0x0E)
#define CR15 (0x0F)
#define CR16 (0x10)
#define CR17 (0x11)
#define CR18 (0x12)
#define CR19 (0x13)
#define CR20 (0x14)
#define CR21 (0x15)
#define CR22 (0x16)
#define CR23 (0x17)
#define CR24 (0x18)
#define CR25 (0x19)
#define CR26 (0x1A)
#define CR27 (0x1B)
#define CR28 (0x1C)
#define CR29 (0x1D)
#define CR30 (0x1E)
#define CR31 (0x1F)

/*****************************************************/
/*    General-Purpose Port Control Register Address  */
/*****************************************************/
#define GPCR0 (0x20)
#define GPCR1 (0x21)
#define GPCR2 (0x22)
#define GPCR3 (0x23)
#define GPCR4 (0x24)
#define GPCR5 (0x25)
#define GPCR6 (0x26)
#define GPCR7 (0x27)
#define GPCR8 (0x28)

/*****************************************************/
/*    ML7037-003 Control Register Map                */
/*****************************************************/
/* CR0 */
#define SPDN_NORMAL_OPE         0x00    /* Software Power-Down Control Register Normal operation */
#define SPDN_POWER_OFF          0x80    /* Software Power-Down Control Register Software power-down reset */
#define RST_NORMAL_OPE          0x00    /* Reset Control Register Normal operation */
#define RST_ON                  0x40    /* Reset Control Register Reset */
#define LINEEN_NORMAL_OPE       0x00    /* Line-Side Analog Interface Power-Down Control Register Normal operation */
#define LINEEN_POWER_DOWN       0x20    /* Line-Side Analog Interface Power-Down Control Register Power-down */
#define PCMSEL_LINEAR           0x00    /* Coding Format Selection Register for PCM Interface 16bit linear PCM */
#define PCMSEL_ULAW             0x10    /* Coding Format Selection Register for PCM Interface u-law PCM */
#define CLKEN_ON                0x00    /* BCLK, SYNC Clock Output On/Off Selection Register On (outputs clocks) */
#define CLKEN_OFF               0x08    /* BCLK, SYNC Clock Output On/Off Selection Register Off (outputs no clocks) */
#define PCMEN_ENA               0x00    /* PCM I/O Control Register PCM interface enabled */
#define PCMEN_DIS               0x04    /* PCM I/O Control Register PCM interface disabled */
#define ECSEL_SINGLE            0x00    /* Echo Canceller Mode Selection Register Single echo canceller mode */
#define ECSEL_DUAL              0x02    /* Echo Canceller Mode Selection Register Dual echo canceller mode */
#define OPE_STAT_INIT_MODE      0x00    /* Operation Mode Selection Register Initial mode */
#define OPE_STAT_NORMAL_OPE     0x01    /* Operation Mode Selection Register Normal operation */

/* CR1 */
#define DMWR_WRITE_INHIBITED    0x00    /* Internal Data Memory Write Execution Register Write inhibited */
#define DMWR_WRITE              0x80    /* Internal Data Memory Write Execution Register Write */

/* CR2 */
#define RALCTHR_NORMAL          0x00    /* Receive-Side ALC Through Mode Selection Register Normal mode (ALC enabled) */
#define RALCTHR_THR             0x10    /* Receive-Side ALC Through Mode Selection Register Through mode */
#define RPAD_PLUS_21DB          0x07    /* Receive-Side Volume Control Register +21dB */
#define RPAD_PLUS_18DB          0x06    /* Receive-Side Volume Control Register +18dB */
#define RPAD_PLUS_15DB          0x05    /* Receive-Side Volume Control Register +15dB */
#define RPAD_PLUS_12DB          0x04    /* Receive-Side Volume Control Register +12dB */
#define RPAD_PLUS_9DB           0x03    /* Receive-Side Volume Control Register  +9dB */
#define RPAD_PLUS_6DB           0x02    /* Receive-Side Volume Control Register  +6dB */
#define RPAD_PLUS_3DB           0x01    /* Receive-Side Volume Control Register  +3dB */
#define RPAD_0DB                0x00    /* Receive-Side Volume Control Register   0dB */
#define RPAD_MINUS_3DB          0x0f    /* Receive-Side Volume Control Register  -3dB */
#define RPAD_MINUS_6DB          0x0e    /* Receive-Side Volume Control Register  -6dB */
#define RPAD_MINUS_9DB          0x0d    /* Receive-Side Volume Control Register  -9dB */
#define RPAD_MINUS_12DB         0x0c    /* Receive-Side Volume Control Register -12dB */
#define RPAD_MINUS_15DB         0x0b    /* Receive-Side Volume Control Register -15dB */
#define RPAD_MINUS_18DB         0x0a    /* Receive-Side Volume Control Register -18dB */
#define RPAD_MINUS_21DB         0x09    /* Receive-Side Volume Control Register -21dB */
#define RPAD_MUTE               0x08    /* Receive-Side Volume Control Register MUTE */

/* CR3 */
#define TPAD_PLUS_21DB          0x07    /* Transmit-Side Volume Control Register +21dB */
#define TPAD_PLUS_18DB          0x06    /* Transmit-Side Volume Control Register +18dB */
#define TPAD_PLUS_15DB          0x05    /* Transmit-Side Volume Control Register +15dB */
#define TPAD_PLUS_12DB          0x04    /* Transmit-Side Volume Control Register +12dB */
#define TPAD_PLUS_9DB           0x03    /* Transmit-Side Volume Control Register  +9dB */
#define TPAD_PLUS_6DB           0x02    /* Transmit-Side Volume Control Register  +6dB */
#define TPAD_PLUS_3DB           0x01    /* Transmit-Side Volume Control Register  +3dB */
#define TPAD_0DB                0x00    /* Transmit-Side Volume Control Register   0dB */
#define TPAD_MINUS_3DB          0x0f    /* Transmit-Side Volume Control Register  -3dB */
#define TPAD_MINUS_6DB          0x0e    /* Transmit-Side Volume Control Register  -6dB */
#define TPAD_MINUS_9DB          0x0d    /* Transmit-Side Volume Control Register  -9dB */
#define TPAD_MINUS_12DB         0x0c    /* Transmit-Side Volume Control Register -12dB */
#define TPAD_MINUS_15DB         0x0b    /* Transmit-Side Volume Control Register -15dB */
#define TPAD_MINUS_18DB         0x0a    /* Transmit-Side Volume Control Register -18dB */
#define TPAD_MINUS_21DB         0x09    /* Transmit-Side Volume Control Register -21dB */
#define TPAD_MUTE               0x08    /* Transmit-Side Volume Control Register MUTE */

/* CR4 */
#define APGA_PLUS_20DB          0x0A    /* Acoustic Side PGA Gain Level Tuning Registers +20dB */
#define APGA_PLUS_18DB          0x09    /* Acoustic Side PGA Gain Level Tuning Registers +18dB */
#define APGA_PLUS_16DB          0x08    /* Acoustic Side PGA Gain Level Tuning Registers +16dB */
#define APGA_PLUS_14DB          0x07    /* Acoustic Side PGA Gain Level Tuning Registers +14dB */
#define APGA_PLUS_12DB          0x06    /* Acoustic Side PGA Gain Level Tuning Registers +12dB */
#define APGA_PLUS_10DB          0x05    /* Acoustic Side PGA Gain Level Tuning Registers +10dB */
#define APGA_PLUS_8DB           0x04    /* Acoustic Side PGA Gain Level Tuning Registers  +8dB */
#define APGA_PLUS_6DB           0x03    /* Acoustic Side PGA Gain Level Tuning Registers  +6dB */
#define APGA_PLUS_4DB           0x02    /* Acoustic Side PGA Gain Level Tuning Registers  +4dB */
#define APGA_PLUS_2DB           0x01    /* Acoustic Side PGA Gain Level Tuning Registers  +2dB */
#define APGA_0DB                0x00    /* Acoustic Side PGA Gain Level Tuning Registers   0dB */
#define APGA_MINUS_2DB          0x1f    /* Acoustic Side PGA Gain Level Tuning Registers  -2dB */
#define APGA_MINUS_4DB          0x1e    /* Acoustic Side PGA Gain Level Tuning Registers  -4dB */
#define APGA_MINUS_6DB          0x1d    /* Acoustic Side PGA Gain Level Tuning Registers  -6dB */
#define APGA_MINUS_8DB          0x1c    /* Acoustic Side PGA Gain Level Tuning Registers  -8dB */
#define APGA_MINUS_10DB         0x1b    /* Acoustic Side PGA Gain Level Tuning Registers -10dB */
#define APGA_MINUS_12DB         0x1a    /* Acoustic Side PGA Gain Level Tuning Registers -12dB */
#define APGA_MINUS_14DB         0x19    /* Acoustic Side PGA Gain Level Tuning Registers -14dB */
#define APGA_MINUS_16DB         0x18    /* Acoustic Side PGA Gain Level Tuning Registers -16dB */
#define APGA_MINUS_18DB         0x17    /* Acoustic Side PGA Gain Level Tuning Registers -18dB */
#define APGA_MINUS_20DB         0x16    /* Acoustic Side PGA Gain Level Tuning Registers -20dB */

/* CR5 */
#define LPGA_PLUS_20DB          0x0A    /* Line Side PGA Gain Level Tuning Registers +20dB */
#define LPGA_PLUS_18DB          0x09    /* Line Side PGA Gain Level Tuning Registers +18dB */
#define LPGA_PLUS_16DB          0x08    /* Line Side PGA Gain Level Tuning Registers +16dB */
#define LPGA_PLUS_14DB          0x07    /* Line Side PGA Gain Level Tuning Registers +14dB */
#define LPGA_PLUS_12DB          0x06    /* Line Side PGA Gain Level Tuning Registers +12dB */
#define LPGA_PLUS_10DB          0x05    /* Line Side PGA Gain Level Tuning Registers +10dB */
#define LPGA_PLUS_8DB           0x04    /* Line Side PGA Gain Level Tuning Registers  +8dB */
#define LPGA_PLUS_6DB           0x03    /* Line Side PGA Gain Level Tuning Registers  +6dB */
#define LPGA_PLUS_4DB           0x02    /* Line Side PGA Gain Level Tuning Registers  +4dB */
#define LPGA_PLUS_2DB           0x01    /* Line Side PGA Gain Level Tuning Registers  +2dB */
#define LPGA_0DB                0x00    /* Line Side PGA Gain Level Tuning Registers   0dB */
#define LPGA_MINUS_2DB          0x1f    /* Line Side PGA Gain Level Tuning Registers  -2dB */
#define LPGA_MINUS_4DB          0x1e    /* Line Side PGA Gain Level Tuning Registers  -4dB */
#define LPGA_MINUS_6DB          0x1d    /* Line Side PGA Gain Level Tuning Registers  -6dB */
#define LPGA_MINUS_8DB          0x1c    /* Line Side PGA Gain Level Tuning Registers  -8dB */
#define LPGA_MINUS_10DB         0x1b    /* Line Side PGA Gain Level Tuning Registers -10dB */
#define LPGA_MINUS_12DB         0x1a    /* Line Side PGA Gain Level Tuning Registers -12dB */
#define LPGA_MINUS_14DB         0x19    /* Line Side PGA Gain Level Tuning Registers -14dB */
#define LPGA_MINUS_16DB         0x18    /* Line Side PGA Gain Level Tuning Registers -16dB */
#define LPGA_MINUS_18DB         0x17    /* Line Side PGA Gain Level Tuning Registers -18dB */
#define LPGA_MINUS_20DB         0x16    /* Line Side PGA Gain Level Tuning Registers -20dB */

/* CR10 */
#define READY_NOT_INIT          0x00    /* Initial Mode Status Flag Register Not in the initial mode */
#define READY_INIT              0x80    /* Initial Mode Status Flag Register In the initial mode */

/* CR11 */
#define LTHR_NORMAL             0x00    /* Line Echo Canceller Through Mode Selection Register Normal mode */
#define LTHR_THR                0x80    /* Line Echo Canceller Through Mode Selection Register Through mode */
#define LECEN_DIS               0x00    /* Line Echo Canceller Enable Register Disabled */
#define LECEN_ENA               0x40    /* Line Echo Canceller Enable Register Enabled */
#define LHLD_UPDATE             0x00    /* Line Echo Canceller Filter Coefficients Update Suspension Register Allows updates */
#define LHLD_FIX                0x20    /* Line Echo Canceller Filter Coefficients Update Suspension Register Update suspended */
#define LCLP_OFF                0x00    /* Line Echo Canceller Center Clip On/Off Register Off */
#define LCLP_ON                 0x08    /* Line Echo Canceller Center Clip On/Off Register On */
#define LSLC_OFF                0x00    /* Line Echo Canceller Automatic SinL Level Control Register Off */
#define LSLC_ON                 0x04    /* Line Echo Canceller Automatic SinL Level Control Register On */
#define LATT_ON                 0x00    /* Line Echo Canceller ATT On/Off Register On */
#define LATT_OFF                0x02    /* Line Echo Canceller ATT On/Off Register Off */

/* CR12 */
#define ATHR_NORMAL             0x00    /* Acoustic Echo Canceller Through Mode Selection Register Normal mode */
#define ATHR_THR                0x80    /* Acoustic Echo Canceller Through Mode Selection Register Through mode */
#define AECEN_DIS               0x00    /* Acoustic Echo Canceller Enable Register Disabled */
#define AECEN_ENA               0x40    /* Acoustic Echo Canceller Enable Register Enabled */
#define AHLD_UPDATE             0x00    /* Acoustic Echo Canceller Filter Coefficients Update Suspension Register Allows updates */
#define AHLD_FIX                0x20    /* Acoustic Echo Canceller Filter Coefficients Update Suspension Register Update suspended */
#define ACLP_OFF                0x00    /* Acoustic Echo Canceller Center Clip On/Off Register Off */
#define ACLP_ON                 0x08    /* Acoustic Echo Canceller Center Clip On/Off Register On */
#define ASLC_OFF                0x00    /* Acoustic Echo Canceller Automatic SinA Level Control Register Off */
#define ASLC_ON                 0x04    /* Acoustic Echo Canceller Automatic SinA Level Control Register On */
#define AATT_ON                 0x00    /* Acoustic Echo Canceller ATT On/Off Register On */
#define AATT_OFF                0x02    /* Acoustic Echo Canceller ATT On/Off Register Off */

/* CR13 */
#define ASOPAD_0DB              0x00    /* Acoustic Echo Canceller SoutA Gain Level Control Registers  0dB */
#define ASOPAD_6DB              0x40    /* Acoustic Echo Canceller SoutA Gain Level Control Registers  6dB */
#define ASOPAD_12DB             0x80    /* Acoustic Echo Canceller SoutA Gain Level Control Registers 12dB */
#define ASOPAD_18DB             0xc0    /* Acoustic Echo Canceller SoutA Gain Level Control Registers 18dB */
#define ASIPAD_0DB              0x00    /* Acoustic Echo Canceller SinA Loss Level Control Registers   0dB */
#define ASIPAD_6DB              0x10    /* Acoustic Echo Canceller SinA Loss Level Control Registers  -6dB */
#define ASIPAD_12DB             0x20    /* Acoustic Echo Canceller SinA Loss Level Control Registers -12dB */
#define ASIPAD_18DB             0x30    /* Acoustic Echo Canceller SinA Loss Level Control Registers -18dB */
#define LSOPAD_0DB              0x00    /* Line Echo Canceller SoutL Gain Level Control Registers  0dB */
#define LSOPAD_6DB              0x04    /* Line Echo Canceller SoutL Gain Level Control Registers  6dB */
#define LSOPAD_12DB             0x08    /* Line Echo Canceller SoutL Gain Level Control Registers 12dB */
#define LSOPAD_18DB             0x0c    /* Line Echo Canceller SoutL Gain Level Control Registers 18dB */
#define LSIPAD_0DB              0x00    /* Line Echo Canceller SinL Loss Level Control Registers   0dB */
#define LSIPAD_6DB              0x01    /* Line Echo Canceller SinL Loss Level Control Registers  -6dB */
#define LSIPAD_12DB             0x02    /* Line Echo Canceller SinL Loss Level Control Registers -12dB */
#define LSIPAD_18DB             0x03    /* Line Echo Canceller SinL Loss Level Control Registers -18dB */

/* CR14 */
#define SLPTHR_NORMAL           0x00    /* Slope Filter Through Mode Selection Register Normal mode */
#define SLPTHR_THR              0x80    /* Slope Filter Through Mode Selection Register Through mode */
#define NCTHR_NORMAL            0x00    /* Noise Canceller Through Mode Selection Register Normal Mode (enabled) */
#define NCTHR_THR               0x08    /* Noise Canceller Through Mode Selection Register Through Mode */

/* CR16 */
#define AVREFEN_OFF             0x00    /* Power-Down State AVREF/Analog Output Amps Control Register Powered down during power-down mode */
#define AVREFEN_ON              0x80    /* Power-Down State AVREF/Analog Output Amps Control Register Powered up even during power-down mode */
#define AVFROSEL_AVREF          0x00    /* Acoustic Side Analog Output Selection Register AVREF */
#define AVFROSEL_SIGNAL         0x02    /* Acoustic Side Analog Output Selection Register Speech signals */
#define LVFROSEL_AVREF          0x00    /* Line- Side Analog Output Selection Register AVREF */
#define LVFROSEL_SIGNAL         0x01    /* Line- Side Analog Output Selection Register Speech signals */

/* CR17 */
#define AATTMODE_TYPE_A         0x03    /* ATTsA Operation Mode Selection Register Type A */
#define AATTMODE_TYPE_B         0x02    /* ATTsA Operation Mode Selection Register Type B */
#define AATTMODE_TYPE_C         0x01    /* ATTsA Operation Mode Selection Register Type C */
#define AATTMODE_TYPE_D         0x00    /* ATTsA Operation Mode Selection Register Type D */

/* CR18 */
#define LATTMODE_TYPE_A         0x03    /* ATTsL Operation Mode Selection Register Type A */
#define LATTMODE_TYPE_B         0x02    /* ATTsL Operation Mode Selection Register Type B */
#define LATTMODE_TYPE_C         0x01    /* ATTsL Operation Mode Selection Register Type C */
#define LATTMODE_TYPE_D         0x00    /* ATTsL Operation Mode Selection Register Type D */

/* CR20 */
#define EQL_OFF                 0x00    /* Equalizer Operation Disabled */
#define EQL_ON                  0x08    /* Equalizer Operation Enabled */
#define EQL_MODE_0              0x00    /* Equalizer Operation Mode Selection Register Mode 0 */
#define EQL_MODE_1              0x01    /* Equalizer Operation Mode Selection Register Mode 1 */
#define EQL_MODE_2              0x02    /* Equalizer Operation Mode Selection Register Mode 2 */
#define EQL_MODE_3              0x03    /* Equalizer Operation Mode Selection Register Mode 3 */
#define EQL_MODE_4              0x04    /* Equalizer Operation Mode Selection Register Mode 4 */
#define EQL_MODE_5              0x05    /* Equalizer Operation Mode Selection Register Mode 5 */
#define EQL_MODE_6              0x06    /* Equalizer Operation Mode Selection Register Mode 6 */
#define EQL_MODE_7              0x07    /* Equalizer Operation Mode Selection Register Mode 7 */

/*****************************************************/
/*    ML7037-003 Internal Data Memory Address        */
/*****************************************************/
/* Automatic Level Controller */
#define ALC_MAXIMUM_GAIN_adrs               0x0F11  /* ALC Maximun Gain Level */

/* Noise Canceller */
#define NC_DATA1_adrs                       0x01B5  /* Noise Attenuation Level 1 */
#define NC_DATA2_adrs                       0x01AC  /* Noise Attenuation Level 2 */

/* Line Echo Canceller */
#define ATTSL_ATTENUATION_adrs              0x0788  /* ATTsL Attenuation Level */
#define ATTRL_ATTENUATION1_adrs             0x0AA2  /* ATTrL Attenuation Level 1 */
#define ATTRL_ATTENUATION2_adrs             0x0AA0  /* ATTrL Attenuation Level 2 */
#define ATTRL_ATTENUATION3_adrs             0x0AA1  /* ATTrL Attenuation Level 3 */
#define LCLP_THRESHOLD_adrs                 0x07B2  /* LCLP Threshold Level */
#define LCLP_ATTENUATION_adrs               0x07B3  /* LCLP Attenuation Level */
#define LEC_CANCELLABLE_DELAY_TIME_adrs     0x0AA4  /* Line Echo Cancellable Delay Time */

/* Acoustic Echo Canceller */
#define ATTSA_ATTENUATION_adrs              0x076B  /* ATTsA Attenuation */
#define ATTSA_TRANSITION_SPEED1_adrs        0x077D  /* ATTsA On to Off Transition Speed Data1 */
#define ATTSA_TRANSITION_SPEED2_adrs        0x0777  /* ATTsA On to Off Transition Speed Data2 */
#define ATTRA_ATTENUATION_adrs              0x0ABB  /* ATTrA Attenuation */
#define ACLP_THRESHOLD_adrs                 0x07AB  /* ACLP Threshold Level */
#define ACLP_ATTENUATION_adrs               0x07AC  /* ACLP Attenuation Level */
#define SPEECH_SILENCE_THRESHOLD_adrs       0x0AC0  /* Speech/Silence Judging Threshold on Receive-Side */
#define AEC_CANCELLABLE_DELAY_TIME_adrs     0x0ABD  /* Acoustic Echo Cancellable Delay Time */

