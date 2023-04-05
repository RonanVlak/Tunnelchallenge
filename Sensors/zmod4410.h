#include <Wire.h>
#include <stdint.h>
#include <stdio.h>

#define D_RISING_M1  4.9601944386079566e-05
#define D_FALLING_M1 0.3934693402873666
#define D_CLASS_M1   0.024690087971667385
#define STABILIZATION_SAMPLES 10

#define ZMOD4410_I2C_ADDRESS          (0x32)
#define ZMOD44XX_ADDR_PID             (0x00)
#define ZMOD44XX_ADDR_CONF            (0x20)
#define ZMOD44XX_ADDR_GENERAL_PURPOSE (0x26)
#define ZMOD44XX_ADDR_CMD             (0x93)
#define ZMOD44XX_ADDR_STATUS          (0x94)
#define ZMOD44XX_ADDR_TRACKING        (0x3A)

#define ZMOD44XX_LEN_PID             (2)
#define ZMOD44XX_LEN_CONF            (6)
#define ZMOD44XX_LEN_TRACKING        (6)
#define ZMOD44XX_LEN_GENERAL_PURPOSE (9)
//
#define STATUS_SEQUENCER_RUNNING_MASK   (0x80) /**< Sequencer is running */
#define STATUS_SLEEP_TIMER_ENABLED_MASK (0x40) /**< SleepTimer_enabled */
#define STATUS_ALARM_MASK               (0x20) /**< Alarm */
#define STATUS_LAST_SEQ_STEP_MASK       (0x1F) /**< Last executed sequencer step */
#define STATUS_POR_EVENT_MASK           (0x80) /**< POR_event */
#define STATUS_ACCESS_CONFLICT_MASK     (0x40) /**< AccessConflict */

#define ZMOD4410_PID              (0x2310)

#define ZMOD44XX_OK               (0)
#define ERROR_INIT_OUT_OF_RANGE   (1) /**< The initialize value is out of range. */
#define ERROR_GAS_TIMEOUT         (2) /**< The operation took too long. */
#define ERROR_I2C                 (3) /**< Failure in i2c communication. */
#define ERROR_SENSOR_UNSUPPORTED  (4) /**< Sensor is not supported with this firmware. */
#define ERROR_CONFIG_MISSING      (5) /**< There is no pointer to a valid configuration. */
#define ERROR_SENSOR              (6) /**< Sensor malfunction. */
#define ERROR_ACCESS_CONFLICT     (7) /**< AccessConflict. */
#define ERROR_POR_EVENT           (8) /**< POR_event. */

typedef int8_t (*zmod44xx_i2c_ptr_t)(uint8_t addr, uint8_t reg_addr, uint8_t *data, uint8_t len);
//typedef void (*zmod44xx_delay_ptr_p)(uint32_t ms);

typedef struct {
  uint8_t addr;
  uint8_t len;
  uint8_t *data;
} zmod44xx_conf_str;

typedef struct {
  uint8_t start;
  zmod44xx_conf_str h;
  zmod44xx_conf_str d;
  zmod44xx_conf_str m;
  zmod44xx_conf_str s;
  zmod44xx_conf_str r;
} zmod44xx_conf;

typedef struct {
  uint8_t i2c_addr; /**< i2c address of the sensor */
  uint8_t config[6]; /**< configuration parameter set */
  uint8_t general_purpose[9]; /**< general purpose data */
  uint16_t mox_er; /**< sensor specific parameter */
  uint16_t mox_lr; /**< sensor specific parameter */
  uint16_t pid; /**< product id of the sensor */
  zmod44xx_i2c_ptr_t read; /**< function pointer to i2c read */
  zmod44xx_i2c_ptr_t write; /**< function pointer to i2c write */
//  zmod44xx_delay_ptr_p delay_ms; /**< function pointer to delay function */
  zmod44xx_conf *init_conf; /**< pointer to the initialize configuration */
  zmod44xx_conf *meas_conf; /**< pointer to the measurement configuration */
  } zmod44xx_dev_t;

int8_t zmod44xx_read_sensor_info(zmod44xx_dev_t *dev);
int8_t zmod44xx_init_sensor(zmod44xx_dev_t *dev);
int8_t zmod44xx_init_measurement(zmod44xx_dev_t *dev);
int8_t zmod44xx_start_measurement(zmod44xx_dev_t *dev);
int8_t zmod44xx_read_status(zmod44xx_dev_t *dev, uint8_t *status);
int8_t zmod44xx_read_adc_results(zmod44xx_dev_t *dev, uint8_t *adc_result);
int8_t zmod44xx_calc_rmox(zmod44xx_dev_t *dev, uint8_t *adc_result, float *rmox);

int8_t arduino_i2c_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *buf, uint8_t len)
{
  Wire.beginTransmission(i2c_addr);
  Wire.write(reg_addr);
  if (Wire.endTransmission() != 0)
  {
 //   Serial.print("NACK");
    return 3; //Error: Sensor did not ack
  }

  Wire.requestFrom(static_cast<uint8_t>(i2c_addr), len);
  for (uint8_t i = 0; i < len; i++)
  {
    *(buf + i) = Wire.read();
  }
  return 0;
}


int8_t arduino_i2c_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *buf, uint8_t len)
{
  Wire.beginTransmission(i2c_addr);
  Wire.write(reg_addr);
  for (uint8_t i = 0; i < len; i++)
  {
    Wire.write(buf[i]);
  }

  if (Wire.endTransmission() != 0)
  {
  //  Serial.print("NACK");
    return 3; //Error: Sensor did not ack
  }
  return 0;
}

uint8_t data_set_4410[] = { 0x20, 0x04, 0x40, 0x09, 0x03,
                            0x00, 0x00, 0x80, 0x08
                          };

uint8_t data_set_4410i[] = { 0x00, 0x28, 0xC3, 0xE3,
                             0x00, 0x00, 0x80, 0x40
                           };

//uint8_t data_set_4410[] = { 0x00, 0x50, 0xFF, 0x38,
//                            0xFE, 0xD4, 0xFE, 0x70,
//                            0xFE, 0x0C, 0xFD, 0xA8,
//                            0xFD, 0x44, 0xFC, 0xE0,
//                            0x00, 0x52, 0x02, 0x67,
//                            0x00, 0xCD, 0x03, 0x34,
//                            0x23, 0x03, 0xA3, 0x43,
//                            0x00, 0x00, 0x06, 0x49,
//                            0x06, 0x4A, 0x06, 0x4B,
//                            0x06, 0x4C, 0x06, 0x4D,
//                            0x06, 0x4E, 0x06, 0x97,
//                            0x06, 0xD7, 0x06, 0x57,
//                            0x06, 0x4E, 0x06, 0x4D,
//                            0x06, 0x4C, 0x06, 0x4B,
//                            0x06, 0x4A, 0x86, 0x59};
//
//uint8_t data_set_4410i[] = { 0x00, 0x50,
//                             0x00, 0x28, 0xC3, 0xE3,
//                             0x00, 0x00, 0x80, 0x40};

zmod44xx_conf zmod4410 = {
  .start = 0x80,
  .h = { .addr = 0x40, .len = 2 },
  .d = { .addr = 0x50, .len = 4, .data = &data_set_4410[0] },
  .m = { .addr = 0x60, .len = 1, .data = &data_set_4410[4] },
  .s = { .addr = 0x68, .len = 4, .data = &data_set_4410[5] },
  .r = { .addr = 0x97, .len = 2 }
};

zmod44xx_conf zmod44xxi = {
  .start = 0x80,
  .h = { .addr = 0x40, .len = 2 },
  .d = { .addr = 0x50, .len = 2, .data = &data_set_4410i[0] },
  .m = { .addr = 0x60, .len = 2, .data = &data_set_4410i[2] },
  .s = { .addr = 0x68, .len = 4, .data = &data_set_4410i[4] },
  .r = { .addr = 0x97, .len = 4 }
};

//zmod44xx_conf zmod4410 = {
//  .start = 0x80,
//  .h = { .addr = 0x40, .len = 16, .data = &data_set_4410[0] },
//  .d = { .addr = 0x50, .len = 8,  .data = &data_set_4410[16] },
//  .m = { .addr = 0x60, .len = 4,  .data = &data_set_4410[24] },
//  .s = { .addr = 0x68, .len = 32, .data = &data_set_4410[28] },
//  .r = { .addr = 0x97, .len = 32 }
//};

//zmod44xx_conf zmod44xxi = {
//  .start = 0x80,
//  .h = { .addr = 0x40, .len = 2, .data = &data_set_4410i[0] },
//  .d = { .addr = 0x50, .len = 2, .data = &data_set_4410i[2] },
//  .m = { .addr = 0x60, .len = 2, .data = &data_set_4410i[4] },
//  .s = { .addr = 0x68, .len = 4, .data = &data_set_4410i[6] },
//  .r = { .addr = 0x97, .len = 4 }
//};

int8_t zmod44xx_read_sensor_info(zmod44xx_dev_t *dev)
{
  int8_t ret = 0;
  uint8_t data[ZMOD44XX_LEN_PID];
  uint8_t status = 0;
  uint8_t cmd = 0;
  uint16_t i = 0;

  /* wait for sensor ready */
  do {
    ret = dev->write(dev->i2c_addr, ZMOD44XX_ADDR_CMD, &cmd, 1);
    if (ret) {
      return ERROR_I2C;
    }

    delay(200);

    ret = zmod44xx_read_status(dev, &status);
    if (ret) {
      return ret;
    }
    i++;
  } while ((0x00 != (status & 0x80)) && (i < 100));

  if (100 <= i) {
    return ERROR_GAS_TIMEOUT;
  }

  ret = dev->read(dev->i2c_addr, ZMOD44XX_ADDR_PID, data, ZMOD44XX_LEN_PID);
  if (ret) {
    return ERROR_I2C;
  }
  dev->pid = data[0] << 8 | data[1];

  ret = dev->read(dev->i2c_addr, ZMOD44XX_ADDR_GENERAL_PURPOSE,
                  dev->general_purpose, ZMOD44XX_LEN_GENERAL_PURPOSE);
  if (ret) {
    return ERROR_I2C;
  }

  ret = dev->read(dev->i2c_addr, ZMOD44XX_ADDR_CONF, dev->config,
                  ZMOD44XX_LEN_CONF);
  if (ret) {
    return ERROR_I2C;
  }

  switch (dev->pid) {
    case ZMOD4410_PID: {
        dev->meas_conf = &zmod4410;
        dev->init_conf = &zmod44xxi;
      } break;

    default: {
        return ERROR_SENSOR_UNSUPPORTED;
      } break;
  }

  return ZMOD44XX_OK;
}

int8_t zmod44xx_calc_factor(zmod44xx_dev_t *dev, float factor, uint8_t *data)
{
  float hspf;

  hspf = (-((float)dev->config[2] * 256.0 + dev->config[3]) *
          ((dev->config[4] + 640.0) * (dev->config[5] + factor) - 512000.0)) /
         12288000.0;
  if ((0.0 > hspf) || (4096.0 < hspf)) {
    return ERROR_INIT_OUT_OF_RANGE;
  }
  *data = (uint8_t)((uint16_t)hspf >> 8);
  *(data + 1) = (uint8_t)((uint16_t)hspf & 0x00FF);

  return ZMOD44XX_OK;
}

int8_t zmod44xx_init_sensor(zmod44xx_dev_t *dev)
{
  int8_t ret = 0;
  uint8_t data[4] = { 0 };
  uint8_t zmod44xx_status;

  if (!dev->init_conf) {
    return ERROR_CONFIG_MISSING;
  }

  ret = dev->read(dev->i2c_addr, 0xB7, data, 1);
  if (ret) {
    return ERROR_I2C;
  }

  ret = zmod44xx_calc_factor(dev, 80, data);
  if (ret) {
    return ret;
  }

  ret = dev->write(dev->i2c_addr, dev->init_conf->h.addr, data,
                   dev->init_conf->h.len);
  if (ret) {
    return ERROR_I2C;
  }

  ret = dev->write(dev->i2c_addr, dev->init_conf->d.addr,
                   dev->init_conf->d.data, dev->init_conf->d.len);
  if (ret) {
    return ERROR_I2C;
  }

  ret = dev->write(dev->i2c_addr, dev->init_conf->m.addr,
                   dev->init_conf->m.data, dev->init_conf->m.len);
  if (ret) {
    return ERROR_I2C;
  }

  ret = dev->write(dev->i2c_addr, dev->init_conf->s.addr,
                   dev->init_conf->s.data, dev->init_conf->s.len);
  if (ret) {
    return ERROR_I2C;
  }

  ret =
    dev->write(dev->i2c_addr, ZMOD44XX_ADDR_CMD, &dev->init_conf->start, 1);
  if (ret) {
    return ERROR_I2C;
  }

  do {
    ret = zmod44xx_read_status(dev, &zmod44xx_status);
    if (ret) {
      //SerialUSB.print("Error %d, exiting program!\n", ret);
      return ret;
    }
    delay(50);
  } while (zmod44xx_status & STATUS_SEQUENCER_RUNNING_MASK);

  ret = dev->read(dev->i2c_addr, dev->init_conf->r.addr, data,
                  dev->init_conf->r.len);
  if (ret) {
    return ERROR_I2C;
  }

  dev->mox_lr = (uint16_t)(data[0] << 8) | data[1];
  dev->mox_er = (uint16_t)(data[2] << 8) | data[3];

  ret = dev->read(dev->i2c_addr, 0xB7, data, 1);
  if (ret) {
    return ERROR_I2C;
  }
  if (0 != data[0]) {
    if (STATUS_ACCESS_CONFLICT_MASK & data[0]) {
      return ERROR_ACCESS_CONFLICT;
    } else if (STATUS_POR_EVENT_MASK & data[0]) {
      return ERROR_POR_EVENT;
    }
  }

  return ZMOD44XX_OK;
}

int8_t zmod44xx_init_measurement(zmod44xx_dev_t *dev)
{
  int8_t ret = 0;
  uint8_t data[10] = { 0 };

  if (!dev->meas_conf) {
    return ERROR_CONFIG_MISSING;
  }

  ret = dev->read(dev->i2c_addr, 0xB7, data, 1);
  if (ret) {
    return ERROR_I2C;
  }

  ret = zmod44xx_calc_factor(dev, -440, data);
  if (ret) {
    return ret;
  }

  ret = zmod44xx_calc_factor(dev, -490, data + 2);
  if (ret) {
    return ret;
  }

  ret = zmod44xx_calc_factor(dev, -540, data + 4);
  if (ret) {
    return ret;
  }

  ret = zmod44xx_calc_factor(dev, -590, data + 6);
  if (ret) {
    return ret;
  }

  ret = zmod44xx_calc_factor(dev, -640, data + 8);
  if (ret) {
    return ret;
  }

  ret = dev->write(dev->i2c_addr, dev->meas_conf->h.addr, data,
                   dev->meas_conf->h.len);
  if (ret) {
    return ERROR_I2C;
  }

  ret = dev->write(dev->i2c_addr, dev->meas_conf->d.addr,
                   dev->meas_conf->d.data, dev->meas_conf->d.len);
  if (ret) {
    return ERROR_I2C;
  }

  ret = dev->write(dev->i2c_addr, dev->meas_conf->m.addr,
                   dev->meas_conf->m.data, dev->meas_conf->m.len);
  if (ret) {
    return ERROR_I2C;
  }

  ret = dev->write(dev->i2c_addr, dev->meas_conf->s.addr,
                   dev->meas_conf->s.data, dev->meas_conf->s.len);
  if (ret) {
    return ERROR_I2C;
  }

  return ZMOD44XX_OK;
}

int8_t zmod44xx_start_measurement(zmod44xx_dev_t *dev)
{
  int8_t ret = 0;

  ret =
    dev->write(dev->i2c_addr, ZMOD44XX_ADDR_CMD, &dev->meas_conf->start, 1);
  if (ret) {
    return ERROR_I2C;
  }
  return ZMOD44XX_OK;
}

int8_t zmod44xx_read_status(zmod44xx_dev_t *dev, uint8_t *status)
{
  int8_t ret;
  uint8_t st;

  ret = dev->read(dev->i2c_addr, ZMOD44XX_ADDR_STATUS, &st, 1);
  if (0 != ret) {
    return ret;
  }
  *status = st;
  return ZMOD44XX_OK;
}

int8_t zmod44xx_read_adc_results(zmod44xx_dev_t *dev, uint8_t *adc_result)
{
  int8_t ret = 0;
  uint8_t data[2] = { 0 };

  ret = dev->read(dev->i2c_addr, dev->meas_conf->r.addr, adc_result,
                  dev->meas_conf->r.len);
  if (ret) {
    return ERROR_I2C;
  }

  ret = dev->read(dev->i2c_addr, 0xB7, data, 1);
  if (ret) {
    return ERROR_I2C;
  }

  if (0 != data[0]) {
    if (STATUS_ACCESS_CONFLICT_MASK & data[0]) {
      return ERROR_ACCESS_CONFLICT;
    } else if (STATUS_POR_EVENT_MASK & data[0]) {
      return ERROR_POR_EVENT;
    }
  }

  return ZMOD44XX_OK;
}

int8_t zmod44xx_calc_rmox(zmod44xx_dev_t *dev, uint8_t *adc_result, float *rmox)
{
  uint8_t i;
  uint16_t adc_value = 0;
  float *p = rmox;
  float rmox_local = 0;

  for (i = 0; i < dev->meas_conf->r.len; i += 2) {
    adc_value = (uint16_t)(*(adc_result + i) << 8) | *(adc_result + i + 1);
    if (0.0 >= (adc_value - dev->mox_lr)) {
      rmox_local = 1e-3;
    } else if (0.0 >= (dev->mox_er - adc_value)) {
      rmox_local = 1e9;
    } else {
      rmox_local = dev->config[0] * 0.01 * (adc_value - dev->mox_lr) /
                   (dev->mox_er - adc_value);
    }

    if (1e12 < rmox_local) {
      rmox_local = 1e12;
    }

    *p = rmox_local;
    p++;
  }

  return ZMOD44XX_OK;
}
