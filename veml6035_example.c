/***************************************************************************/ /**
 * @file veml6035_example.c
 * @brief VEML6035 Automatic Street Light
 *******************************************************************************
 */

#include "rsi_debug.h"
#include "sl_si91x_veml6035.h"
#include "veml6035_example.h"
#include "sl_sleeptimer.h"
#include "sl_si91x_i2c.h"
#include "sl_si91x_driver_gpio.h"

/*******************************************************************************
 ***************************  Defines / Macros  ********************************
 ******************************************************************************/

#define TX_THRESHOLD       0
#define RX_THRESHOLD       0
#define I2C                VEML6035_I2C_INSTANCE
#define DELAY_PERIODIC_MS1 2000
#define MODE_0             0
#define OUTPUT_VALUE       1
#define SYNC_TIME          10

/* BRD2605A onboard GREEN LED
 * GPIO 51 = Port D, Pin 3
 * LED is ACTIVE LOW
 */
#define GREEN_LED_PORT     SL_GPIO_PORT_D
#define GREEN_LED_PIN      3

/*******************************************************************************
 ******************************  Data Types  ***********************************
 ******************************************************************************/

typedef sl_i2c_config_t sl_i2c_configuration_t;

/*******************************************************************************
 *************************** LOCAL VARIABLES ***********************************
 ******************************************************************************/

sl_sleeptimer_timer_handle_t timer1;
volatile boolean_t delay_timeout = false;

static sl_gpio_t green_led = {
  GREEN_LED_PORT,
  GREEN_LED_PIN
};

/*******************************************************************************
 **********************  Local Function prototypes  ***************************
 ******************************************************************************/

static void i2c_leader_callback(sl_i2c_instance_t i2c_instance,
                                 uint32_t status);

static sl_status_t veml6035_measure_als_lux(void);
static sl_status_t veml6035_measure_white_ch_lux(void);

static void on_timeout_timer1(sl_sleeptimer_timer_handle_t *handle,
                              void *data);

static void delay(uint32_t idelay);

/*******************************************************************************
 **************************   GLOBAL FUNCTIONS   *******************************
 ******************************************************************************/

void veml6035_example_init(void)
{
  sl_status_t status;
  sl_i2c_configuration_t i2c_config;

  i2c_config.mode           = SL_I2C_LEADER_MODE;
  i2c_config.transfer_type  = SL_I2C_USING_NON_DMA;
  i2c_config.operating_mode = SL_I2C_STANDARD_MODE;
  i2c_config.i2c_callback   = i2c_leader_callback;

  do {

#if defined(SENSOR_ENABLE_GPIO_MAPPED_TO_UULP)

    if (sl_si91x_gpio_driver_get_uulp_npss_pin(SENSOR_ENABLE_GPIO_PIN) != 1) {

      status = sl_si91x_gpio_driver_enable_clock(
          (sl_si91x_gpio_select_clock_t)ULPCLK_GPIO);

      if (status != SL_STATUS_OK) {
        DEBUGOUT("GPIO clock enable failed\r\n");
        break;
      }

      status = sl_si91x_gpio_driver_set_uulp_npss_pin_mux(
          SENSOR_ENABLE_GPIO_PIN,
          NPSS_GPIO_PIN_MUX_MODE0);

      if (status != SL_STATUS_OK) {
        DEBUGOUT("UULP GPIO mux failed\r\n");
        break;
      }

      status = sl_si91x_gpio_driver_set_uulp_npss_direction(
          SENSOR_ENABLE_GPIO_PIN,
          (sl_si91x_gpio_direction_t)GPIO_OUTPUT);

      if (status != SL_STATUS_OK) {
        DEBUGOUT("UULP GPIO direction failed\r\n");
        break;
      }

      status = sl_si91x_gpio_driver_set_uulp_npss_pin_value(
          SENSOR_ENABLE_GPIO_PIN,
          SET);

      if (status != SL_STATUS_OK) {
        DEBUGOUT("UULP GPIO value failed\r\n");
        break;
      }
    }

#else

    sl_gpio_t sensor_enable_port_pin = {
      SENSOR_ENABLE_GPIO_PORT,
      SENSOR_ENABLE_GPIO_PIN
    };

    uint8_t pin_value;

    status = sl_gpio_driver_get_pin(
        &sensor_enable_port_pin,
        &pin_value);

    if (status != SL_STATUS_OK) {
      DEBUGOUT("Sensor GPIO read failed\r\n");
      break;
    }

    if (pin_value != 1) {

#ifdef SENSOR_ENABLE_GPIO_MAPPED_TO_ULP

      status = sl_si91x_gpio_driver_enable_clock(
          (sl_si91x_gpio_select_clock_t)ULPCLK_GPIO);

#else

      status = sl_si91x_gpio_driver_enable_clock(
          (sl_si91x_gpio_select_clock_t)M4CLK_GPIO);

#endif

      if (status != SL_STATUS_OK) {
        DEBUGOUT("Sensor GPIO clock failed\r\n");
        break;
      }

      status = sl_gpio_driver_set_pin_mode(
          &sensor_enable_port_pin,
          MODE_0,
          OUTPUT_VALUE);

      if (status != SL_STATUS_OK) {
        DEBUGOUT("Sensor GPIO mode failed\r\n");
        break;
      }

      status = sl_si91x_gpio_driver_set_pin_direction(
          SENSOR_ENABLE_GPIO_PORT,
          SENSOR_ENABLE_GPIO_PIN,
          (sl_si91x_gpio_direction_t)GPIO_OUTPUT);

      if (status != SL_STATUS_OK) {
        DEBUGOUT("Sensor GPIO direction failed\r\n");
        break;
      }

      status = sl_gpio_driver_set_pin(
          &sensor_enable_port_pin);

      if (status != SL_STATUS_OK) {
        DEBUGOUT("Sensor GPIO output failed\r\n");
        break;
      }
    }

#endif

    /***********************************************************************
     * GREEN LED INITIALIZATION
     *
     * GPIO 51 -> GREEN LED
     * Active LOW
     ***********************************************************************/

    status = sl_si91x_gpio_driver_enable_clock(
        (sl_si91x_gpio_select_clock_t)M4CLK_GPIO);

    if (status != SL_STATUS_OK) {
      DEBUGOUT("GREEN LED GPIO clock failed\r\n");
      break;
    }

    /*
     * GPIO 51 corresponds to PAD 15.
     * Select MCU control of the pad.
     */
    status = sl_si91x_gpio_driver_enable_pad_selection(15);

    if (status != SL_STATUS_OK) {
      DEBUGOUT("GREEN LED pad selection failed\r\n");
      break;
    }

    /*
     * IMPORTANT:
     * Do NOT call enable_pad_receiver() here.
     * Receiver is not required for LED output.
     */

    status = sl_gpio_driver_set_pin_mode(
        &green_led,
        MODE_0,
        OUTPUT_VALUE);

    if (status != SL_STATUS_OK) {
      DEBUGOUT("GREEN LED pin mode failed\r\n");
      break;
    }

    status = sl_si91x_gpio_driver_set_pin_direction(
        GREEN_LED_PORT,
        GREEN_LED_PIN,
        (sl_si91x_gpio_direction_t)GPIO_OUTPUT);

    if (status != SL_STATUS_OK) {
      DEBUGOUT("GREEN LED direction failed\r\n");
      break;
    }

    /*
     * Active LOW:
     * HIGH = LED OFF
     */
    status = sl_gpio_driver_set_pin(&green_led);

    if (status != SL_STATUS_OK) {
      DEBUGOUT("GREEN LED OFF failed\r\n");
      break;
    }

    DEBUGOUT("GREEN LED initialized successfully\r\n");

    /***********************************************************************
     * START PERIODIC TIMER
     ***********************************************************************/

    status = sl_sleeptimer_start_periodic_timer_ms(
        &timer1,
        DELAY_PERIODIC_MS1,
        on_timeout_timer1,
        NULL,
        0,
        SL_SLEEPTIMER_NO_HIGH_PRECISION_HF_CLOCKS_REQUIRED_FLAG);

    if (status != SL_STATUS_OK) {
      DEBUGOUT("Timer start failed\r\n");
      break;
    }

    /***********************************************************************
     * I2C INITIALIZATION
     ***********************************************************************/

    status = sl_i2c_driver_init(I2C, &i2c_config);

    if (status != SL_I2C_SUCCESS) {
      DEBUGOUT(
          "sl_i2c_driver_init failed: 0x%ld\r\n",
          status);
      break;
    }

    DEBUGOUT("Successfully initialized I2C\r\n");

    status = sl_i2c_driver_configure_fifo_threshold(
        I2C,
        TX_THRESHOLD,
        RX_THRESHOLD);

    if (status != SL_I2C_SUCCESS) {
      DEBUGOUT(
          "I2C FIFO configuration failed: 0x%ld\r\n",
          status);
      break;
    }

    DEBUGOUT("Successfully configured I2C FIFO\r\n");

    /***********************************************************************
     * VEML6035 INITIALIZATION
     ***********************************************************************/

    status = sl_si91x_veml6035_init(
        I2C,
        VEML6035_ADDR,
        true);

    if (status != SL_STATUS_OK) {
      DEBUGOUT(
          "Sensor initialization failed: 0x%ld\r\n",
          status);
      break;
    }

    DEBUGOUT("VEML6035 initialized successfully\r\n");

  } while (false);
}

/*******************************************************************************
 * PROCESS ACTION
 ******************************************************************************/

void veml6035_example_process_action(void)
{
  sl_status_t status;

  if (delay_timeout == true) {

    delay_timeout = false;

    status = veml6035_measure_als_lux();

    if (status != SL_STATUS_OK) {
      DEBUGOUT(
          "lux measurement failed: 0x%ld\r\n",
          status);
    }

    delay(SYNC_TIME);

    status = veml6035_measure_white_ch_lux();

    if (status != SL_STATUS_OK) {
      DEBUGOUT(
          "white channel measurement failed: 0x%ld\r\n",
          status);
    }
  }
}

/*******************************************************************************
 * ALS LUX MEASUREMENT
 ******************************************************************************/

static sl_status_t veml6035_measure_als_lux(void)
{
  sl_status_t status = SL_STATUS_OK;
  float lux = 0.0f;

  status = sl_si91x_veml6035_get_als_lux(
      I2C,
      VEML6035_ADDR,
      &lux);

  if (status != SL_STATUS_OK) {

    DEBUGOUT(
        "ALS lux measurement failed: 0x%ld\r\n",
        status);

    return status;
  }

  DEBUGOUT(
      "measured ALS lux value: %.2f\r\n",
      (double)lux);

  /***********************************************************************
   * AUTOMATIC STREET LIGHT
   *
   * < 50 lux  = DARK   = GREEN LED ON
   * >= 50 lux = BRIGHT = GREEN LED OFF
   *
   * LED is ACTIVE LOW
   ***********************************************************************/

  if (lux < 50.0f) {

    /* LOW = ON */
    sl_gpio_driver_clear_pin(&green_led);

    DEBUGOUT("DARK -> GREEN LIGHT ON\r\n");

  } else {

    /* HIGH = OFF */
    sl_gpio_driver_set_pin(&green_led);

    DEBUGOUT("BRIGHT -> GREEN LIGHT OFF\r\n");
  }

  return status;
}

/*******************************************************************************
 * WHITE CHANNEL LUX MEASUREMENT
 ******************************************************************************/

static sl_status_t veml6035_measure_white_ch_lux(void)
{
  sl_status_t status = SL_STATUS_OK;
  float lux = 0.0f;

  status = sl_si91x_veml6035_get_white_lux(
      I2C,
      VEML6035_ADDR,
      &lux);

  if (status != SL_STATUS_OK) {

    DEBUGOUT(
        "white ch lux measurement failed: 0x%ld\r\n",
        status);

    return status;
  }

  DEBUGOUT(
      "measured white ch lux value: %.2f\r\n",
      (double)lux);

  return status;
}

/*******************************************************************************
 * I2C CALLBACK
 ******************************************************************************/

void i2c_leader_callback(
    sl_i2c_instance_t i2c_instance,
    uint32_t status)
{
  (void)i2c_instance;

  switch (status) {

    case SL_I2C_DATA_TRANSFER_COMPLETE:
      break;

    default:
      break;
  }
}

/*******************************************************************************
 * SLEEPTIMER CALLBACK
 ******************************************************************************/

static void on_timeout_timer1(
    sl_sleeptimer_timer_handle_t *handle,
    void *data)
{
  (void)handle;
  (void)data;

  delay_timeout = true;
}

/*******************************************************************************
 * DELAY
 ******************************************************************************/

static void delay(uint32_t idelay)
{
  for (uint32_t x = 0; x < 4600 * idelay; x++) {
    __NOP();
  }
}