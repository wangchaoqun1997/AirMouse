/* register address */

#define WD3153_ADDRESS 0x45

#define WD_REG_RESET			0x00
#define WD_REG_GLOBAL_CONTROL		0x01
#define WD_REG_LED_STATUS		0x02
#define WD_REG_LED_ENABLE		0x30
#define WD_REG_LED_CONFIG_BASE		0x31
#define WD_REG_LED_BRIGHTNESS_BASE	0x34
#define WD_REG_TIMESET0_BASE		0x37
#define WD_REG_TIMESET1_BASE		0x38
/* register bits */
#define WD3153_CHIPID			0x33
#define WD_LED_MOUDLE_ENABLE_MASK	0x01
#define WD_LED_FADE_OFF_MASK		0x40
#define WD_LED_FADE_ON_MASK		0x20
#define WD_LED_BREATHE_MODE_MASK	0x10
#define WD_LED_RESET_MASK		0x55

#define OUT_CURRENT 0x01

#define LED_RED 0x01
#define LED_GREEN 0x02
#define LED_BLUE 0x04

#define RISE_TIME 0x2
#define FALL_TIME 0x2
#define HOLD_TIME 0x1
#define OFF_TIME  0x2
enum sw3153_light_type{
OFF,ON,
GREEN,RED,BLUE,
BLUE_GREEN,BLUE_GREEN_RED,
SUSPEND,
};
enum sw3153_blink_level{
BLINK_LEVEL_NON,
BLINK_LEVEL_0,
BLINK_LEVEL_1,
BLINK_LEVEL_2,
BLINK_LEVEL_3,
BLINK_LEVEL_4,
};

void sw3153_config(void);
void sw3153_light_select(enum sw3153_light_type type,enum sw3153_blink_level level);
