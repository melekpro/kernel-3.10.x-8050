
enum PSENSOR_STATUS
{
    PSENSOR_STATUS_NEAR = 0,
    PSENSOR_STATUS_FAR = 1,
};

struct psensor_attrs
{
    struct device_attribute **attr;
    int num;
};

struct psensor_driver_t
{
    char *name;
    int (*init)(void);
    struct psensor_attrs *attrs;
};

#if 1

#define PSENSOR_EN_GPIO_MODE 1   //GPIO Control psensor power on or off
#define IQS128_EINT_PIN         (GPIO15 | 0x80000000)
#define IQS128_EINT_PIN_M_GPIO  GPIO_MODE_06
#define IQS128_EINT_PIN_M_EINT  IQS128_EINT_PIN_M_GPIO
#define IQS128_EINT_NUM   15
#define IQS128_EINT_DEBOUNCE_CN      0
#define IQS128_EINT_DEBOUNCE_EN      CUST_EINT_DEBOUNCE_DISABLE
#else
#define PSENSOR_EN_GPIO_MODE 0  //GPIO Control psensor power on or off
#define IQS128_EINT_PIN         (GPIO6 | 0x80000000)
#define IQS128_EINT_PIN_M_GPIO  GPIO_MODE_06
#define IQS128_EINT_PIN_M_EINT  IQS128_EINT_PIN_M_GPIO
#define IQS128_EINT_NUM   6
#define IQS128_EINT_DEBOUNCE_CN      0
#define IQS128_EINT_DEBOUNCE_EN      CUST_EINT_DEBOUNCE_DISABLE

#endif

//GPIO Control psensor power on or off
#if PSENSOR_EN_GPIO_MODE

#define IQS128_EN_PIN      (GPIO10 | 0x80000000)
#define IQS128_EN_PIN_M  GPIO_MODE_00
#define IQS128_EN_PIN_ENABLE GPIO_OUT_ONE
#define IQS128_EN_PIN_DISABLE GPIO_OUT_ZERO

#endif

void send_psensor_uevent(enum PSENSOR_STATUS val);
