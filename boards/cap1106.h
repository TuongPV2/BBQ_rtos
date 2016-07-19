#ifndef __CAP1106_H__
#define __CAP1106_H__

#include "fsl_common.h"
#include "fsl_i2c.h"

#define CAP1106_REG_MAIN_CONTROL        0x00
#define CAP1106_REG_MAIN_CONTROL_GAIN_SHIFT     (6)
#define CAP1106_REG_MAIN_CONTROL_GAIN_MASK      (0xc0)
#define CAP1106_REG_MAIN_CONTROL_DLSEEP         BIT(4)
#define CAP1106_REG_GENERAL_STATUS      0x02
#define CAP1106_REG_SENSOR_INPUT        0x03
#define CAP1106_REG_NOISE_FLAG_STATUS   0x0a
#define CAP1106_REG_SENOR_DELTA(X)      (0x10 + (X))
#define CAP1106_REG_SENSITIVITY_CONTROL 0x1f
#define CAP1106_REG_CONFIG              0x20
#define CAP1106_REG_SENSOR_ENABLE       0x21
#define CAP1106_REG_SENSOR_CONFIG       0x22
#define CAP1106_REG_SENSOR_CONFIG2      0x23
#define CAP1106_REG_SAMPLING_CONFIG     0x24
#define CAP1106_REG_CALIBRATION         0x26
#define CAP1106_REG_INT_ENABLE          0x27
#define CAP1106_REG_REPEAT_RATE         0x28
#define CAP1106_REG_MT_CONFIG           0x2a
#define CAP1106_REG_MT_PATTERN_CONFIG   0x2b
#define CAP1106_REG_MT_PATTERN          0x2d
#define CAP1106_REG_RECALIB_CONFIG      0x2f
#define CAP1106_REG_SENSOR_THRESH(X)    (0x30 + (X))
#define CAP1106_REG_SENSOR_NOISE_THRESH 0x38
#define CAP1106_REG_STANDBY_CHANNEL     0x40
#define CAP1106_REG_STANDBY_CONFIG      0x41
#define CAP1106_REG_STANDBY_SENSITIVITY 0x42
#define CAP1106_REG_STANDBY_THRESH      0x43
#define CAP1106_REG_CONFIG2             0x44
#define CAP1106_REG_SENSOR_BASE_CNT(X)  (0x50 + (X))
#define CAP1106_REG_SENSOR_CALIB        (0xb1 + (X))
#define CAP1106_REG_SENSOR_CALIB_LSB1   0xb9
#define CAP1106_REG_SENSOR_CALIB_LSB2   0xba
#define CAP1106_REG_PRODUCT_ID          0xfd
#define CAP1106_REG_MANUFACTURER_ID     0xfe
#define CAP1106_REG_REVISION            0xff

#define CAP1106_NUM_CHN 6
#define CAP1106_PRODUCT_ID      0x55
#define CAP1106_MANUFACTURER_ID 0x5d

static const uint8_t cap1106_reg_defaults[31][2] = {
        { CAP1106_REG_MAIN_CONTROL,             0x00 },
        { CAP1106_REG_GENERAL_STATUS,           0x00 },
        { CAP1106_REG_SENSOR_INPUT,             0x00 },
        { CAP1106_REG_NOISE_FLAG_STATUS,        0x00 },
        { CAP1106_REG_SENSITIVITY_CONTROL,      0x2f },
        { CAP1106_REG_CONFIG,                   0x20 },
        { CAP1106_REG_SENSOR_ENABLE,            0x3f },
        { CAP1106_REG_SENSOR_CONFIG,            0xa4 },
        { CAP1106_REG_SENSOR_CONFIG2,           0x07 },
        { CAP1106_REG_SAMPLING_CONFIG,          0x39 },
        { CAP1106_REG_CALIBRATION,              0x00 },
        { CAP1106_REG_INT_ENABLE,               0x3f },
        { CAP1106_REG_REPEAT_RATE,              0x3f },
        { CAP1106_REG_MT_CONFIG,                0x80 },
        { CAP1106_REG_MT_PATTERN_CONFIG,        0x00 },
        { CAP1106_REG_MT_PATTERN,               0x3f },
        { CAP1106_REG_RECALIB_CONFIG,           0x8a },
        { CAP1106_REG_SENSOR_THRESH(0),         0x40 },
        { CAP1106_REG_SENSOR_THRESH(1),         0x40 },
        { CAP1106_REG_SENSOR_THRESH(2),         0x40 },
        { CAP1106_REG_SENSOR_THRESH(3),         0x40 },
        { CAP1106_REG_SENSOR_THRESH(4),         0x40 },
        { CAP1106_REG_SENSOR_THRESH(5),         0x40 },
        { CAP1106_REG_SENSOR_NOISE_THRESH,      0x01 },
        { CAP1106_REG_STANDBY_CHANNEL,          0x00 },
        { CAP1106_REG_STANDBY_CONFIG,           0x39 },
        { CAP1106_REG_STANDBY_SENSITIVITY,      0x02 },
        { CAP1106_REG_STANDBY_THRESH,           0x40 },
        { CAP1106_REG_CONFIG2,                  0x40 },
        { CAP1106_REG_SENSOR_CALIB_LSB1,        0x00 },
        { CAP1106_REG_SENSOR_CALIB_LSB2,        0x00 },
};

typedef enum _working_mode
{
    Cap1106_Touch = 0U,
    Cap1106_Proxymity
} working_mode_t;

typedef enum _state
{
    Normal = 0U,
    Standby
} state_t;

typedef struct _cap1106_handle
{
    I2C_Type *base;                 /*!< I2C base. */
    i2c_master_handle_t *i2cHandle; /*!< I2C master transfer context */
    i2c_master_transfer_t xfer;     /*!< I2C master xfer */
    state_t state;
    working_mode_t workmode;
} cap1106_handle_t;

status_t CAP1106_Init(cap1106_handle_t *handle);
status_t CAP1106_ReadReg(cap1106_handle_t *handle, uint8_t reg, uint8_t rx_size, uint8_t *rx_buff);
status_t CAP1106_WriteReg(cap1106_handle_t *handle, uint8_t reg, uint8_t val);
void CAP1106_ResetStatus(void);
void CAP1106_SetWorkMode(working_mode_t mode);
void CAP1106_SetState(state_t state);
void CAP1106_EnableInterrupt(bool enable);

#endif
