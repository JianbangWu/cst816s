#ifndef DRV_CST816S_TOUCH_H__
#define DRV_CST816S_TOUCH_H__

/* PB8 SCL=24 PB9 SDA=25 */

#ifndef TOUCH_I2C_NAME
#define TOUCH_I2C_NAME "i2c1"
#endif

#define TOUCH_DEVICE_NAME "cst816s"

#ifndef TOUCH_IRQ_PIN
#define TOUCH_IRQ_PIN (64)
#endif

#ifndef TOUCH_RST_PIN
#define TOUCH_RST_PIN (24)
#endif

#define CST816S_DEV_ADDR 0x15

#define CST816S_GestureID 0x01

#define CST816S_FingerNum 0x02

#define CST816S_XposH 0x03 // X coordinate high 4 bits
#define CST816S_XposL 0x04 // X coordinate low 8 bits

#define CST816S_YposH 0x05 // Y coordinate high 4 bits
#define CST816S_YposL 0x06 // Y coordinate low 8 bits

#define CST816S_BPC0H 0xB0
#define CST816S_BPC0L 0xB1
#define CST816S_BPC1H 0xB2
#define CST816S_BPC1L 0xB3

#define CST816S_ChipID 0xA7
#define CST816S_ProjID 0xA8
#define CST816S_FwVersion 0xA9

#define CST816S_MotionMask 0xEC
#define CST816S_IrqPluseWidth 0xED
#define CST816S_NorScanPer 0xEE
#define CST816S_MotionSlAngle 0xEF

#define CST816S_LpScanRaw1H 0xF0
#define CST816S_LpScanRaw1L 0xF1
#define CST816S_LpScanRaw2H 0xF2
#define CST816S_LpScanRaw2L 0xF3

#define CST816S_LpAutoWakeTime 0xF4
#define CST816S_LpScanTH 0xF5
#define CST816S_LpScanWin 0xF6
#define CST816S_LpScanFreq 0xF7
#define CST816S_LpScanIdac 0xF8
#define CST816S_AutoSleepTimec 0xF9

#define CST816S_IrqCtl 0xFA
#define CST816S_AutoReset 0xFB
#define CST816S_LongPressTime 0xFC
#define CST816S_IOCtl 0xFD
#define CST816S_DisAutoSleep 0xFE

#endif // DRV_CST816S_TOUCH_H__
