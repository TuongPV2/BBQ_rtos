#include "cap1106.h"

static cap1106_handle_t *g_handle;

status_t CAP1106_Init(cap1106_handle_t *handle)
{
    uint8_t reg = 0;

    g_handle = handle;

    // set gain
    //reg = 1 << 6; // gain 4
    //CAP1106_WriteReg(handle, CAP1106_REG_MAIN_CONTROL, reg);

    CAP1106_ReadReg(handle, CAP1106_REG_SENSOR_ENABLE, 1, &reg);
    reg &= 0x0F;
    // enable CS1-4
    CAP1106_WriteReg(handle, CAP1106_REG_SENSOR_ENABLE, reg);

    CAP1106_ReadReg(handle, CAP1106_REG_STANDBY_CHANNEL, 1, &reg);
    reg &= 0x0F;
    // enable CS1-4 in standby mode
    CAP1106_WriteReg(handle, CAP1106_REG_STANDBY_CHANNEL, reg);

    // set sensitivity in standby mode
    reg = 0x00; // 0x01: 64x, 0x00: 128x, default 0x02: 32x
    CAP1106_WriteReg(handle, CAP1106_REG_STANDBY_SENSITIVITY, reg);

    // set thresh hold in standby mode, default 0x40: 64,
    reg = 0x08; // thresh hold 32
    CAP1106_WriteReg(handle, CAP1106_REG_STANDBY_THRESH, reg);

    CAP1106_ReadReg(handle, CAP1106_REG_CONFIG, 1, &reg);
    // enable digital noise thresh hold
    reg |= 1 << 5;
    // to do: check analog noise, max duration recalibration bit
    CAP1106_WriteReg(handle, CAP1106_REG_CONFIG, reg);

    // set repeat 525ms(1110), duration 7800ms(1100)
    reg = 0xCE;
    CAP1106_WriteReg(handle, CAP1106_REG_SENSOR_CONFIG, reg);
    // set m press = 560ms
    reg = 0x0F;
    CAP1106_WriteReg(handle, CAP1106_REG_SENSOR_CONFIG2, reg);

    // CAP1106_ReadReg(handle, CAP1106_REG_REPEAT_RATE, 1, &reg);
    // // disable repeat rate for CS1-4
    // reg &= 3 << 4;
    // CAP1106_WriteReg(handle, CAP1106_REG_REPEAT_RATE, reg);

    // CAP1106_ReadReg(handle, CAP1106_REG_CONFIG2, 1, &reg);
    // // disable release touch interrupt
    // reg |= 1 << 0;
    // CAP1106_WriteReg(handle, CAP1106_REG_CONFIG2, reg);


    return kStatus_Success;
}

status_t CAP1106_ReadReg(cap1106_handle_t *handle, uint8_t reg, uint8_t rx_size, uint8_t *rx_buff)
{
    status_t status = kStatus_Success;

    /* Configure I2C xfer */
    handle->xfer.subaddress = (uint32_t)reg;
    handle->xfer.subaddressSize = 1U;
    handle->xfer.data = rx_buff;
    handle->xfer.dataSize = rx_size;
    handle->xfer.direction = kI2C_Read;
    handle->xfer.flags = kI2C_TransferDefaultFlag;

    status = I2C_MasterTransferBlocking(handle->base, &handle->xfer);

    return status;
}

status_t CAP1106_WriteReg(cap1106_handle_t *handle, uint8_t reg, uint8_t val)
{
    status_t status = kStatus_Success;
    uint8_t buff[1];

    buff[0] = val;
    /* Set I2C xfer structure */
    handle->xfer.subaddress = (uint32_t)reg;
    handle->xfer.subaddressSize = 1U;
    handle->xfer.data = buff;
    handle->xfer.dataSize = 1U;
    handle->xfer.direction = kI2C_Write;
    handle->xfer.flags = kI2C_TransferDefaultFlag;

    status = I2C_MasterTransferBlocking(handle->base, &handle->xfer);

    return status;
}

void CAP1106_SetWorkMode(working_mode_t mode)
{

}

void CAP1106_SetState(state_t state)
{
    uint8_t reg = 0;

    CAP1106_ReadReg(g_handle, CAP1106_REG_MAIN_CONTROL, 1, &reg);

    if (state == Normal)
    {
        reg &=  ~(1 << 5); // clear standby bit
        g_handle->state = Normal;
    }
    else if(state == Standby)
    {
        reg |= 1<< 5; // set standby bit
        g_handle->state = Standby;
    }

    CAP1106_WriteReg(g_handle, CAP1106_REG_MAIN_CONTROL, reg);
}

void CAP1106_ResetStatus(void)
{
    uint8_t value = 0;

    CAP1106_ReadReg(g_handle, CAP1106_REG_MAIN_CONTROL, 1, &value);
    
    value &= 0xFE;  // Clear the INT bit (bit 0)
    CAP1106_WriteReg(g_handle, CAP1106_REG_MAIN_CONTROL, value);
}

void CAP1106_EnableInterrupt(bool enable)
{

}
