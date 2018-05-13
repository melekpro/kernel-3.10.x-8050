#include <cust_vibrator.h>
#include <linux/types.h>

static struct vibrator_hw cust_vibrator_hw = {
	.vib_timer = 25,
  #ifdef CUST_VIBR_LIMIT
	.vib_limit = 9,
  #endif
  #ifdef CUST_VIBR_VOL
	.vib_vol = 0x5,//3.3V for vibr 	/*[BUGFIX]-Add-BEGIN by TCTSZ.pingao.yang, 9/14/2015,  change  vibrator  output voltage to 3.3V */
  #endif
};

struct vibrator_hw *get_cust_vibrator_hw(void)
{
    return &cust_vibrator_hw;
}

