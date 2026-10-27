/*
 * Power Controller - car battery monitor + MPU6050 IMU
 *
 * Reads a car battery (nominal 12V, ~11V..15V range) through a resistive divider
 * on an ADC pin of an ESP32-C3 SuperMini and logs the reconstructed voltage.
 * Classifies the reading as CAR OFF (engine off, battery resting ~12.0-12.7V) or
 * CAR ON (engine running, alternator charging ~13.7-14.7V) and lights the
 * onboard LED while the car is on. Also reads an MPU6050 accel/gyro over I2C and
 * logs the gyro rates.
 *
 * Divider (external hardware):
 *
 *   12V rail ---[ R1 ]---+---[ R2 ]--- GND
 *                        |
 *                        +--> ADC pin (GPIO4)
 *
 *   Vadc = Vin * R2 / (R1 + R2)
 *
 *   Recommended: R1 = 100k, R2 = 20k  ->  ratio = 1/6
 *     Vin =  9.0V -> Vadc = 1.50V
 *     Vin = 14.0V -> Vadc = 2.33V
 *
 *   A ~100nF cap from the ADC pin to GND is recommended to filter noise.
 *
 * ADC config: ADC1, 12 dB attenuation (~0..3.1V full-scale), 12-bit width.
 *
 * MPU6050 wiring (external hardware):
 *
 *   MPU6050 SCL --- GPIO7   (needs a pull-up to 3V3, ~4.7k; many breakouts
 *   MPU6050 SDA --- GPIO6    have these on-board)
 *   MPU6050 VCC --- 3V3
 *   MPU6050 GND --- GND
 *   MPU6050 AD0 --- GND  ->  I2C address 0x68
 *
 * Raspberry Pi shutdown handshake (external hardware):
 *
 *   ESP GPIO10 --> Pi  : shutdown request (HIGH = please shut down)
 *   ESP GPIO5  <-- Pi  : Pi alive (Pi holds HIGH while running; LOW once
 *                        it has shut down / lost power)
 *
 * On CAR ON -> CAR OFF the ESP raises GPIO10 and waits for GPIO5 to stay LOW
 * for PI_ALIVE_LOW_MS (or PI_SHUTDOWN_TIMEOUT_MS to expire), then cuts the
 * Pi's power via the load switch and drops GPIO10 again.
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "esp_rom_sys.h"

static const char *TAG = "power_ctrl";

/* ---- User-configurable hardware constants ---- */
#define ADC_UNIT            ADC_UNIT_1
#define ADC_CHANNEL         ADC_CHANNEL_4      /* GPIO4 on the ESP32-C3 SuperMini */
#define ADC_ATTEN          ADC_ATTEN_DB_12    /* full-scale ~3.1V at the pin      */
#define ADC_BITWIDTH        ADC_BITWIDTH_DEFAULT

/* Onboard addressable RGB LED (WS2812) on the ESP32-C3 SuperMini: GPIO8.
 * Driven over the RMT peripheral using WS2812 single-wire timing.
 * Set LED_ENABLED to 1 to have the LED track the CAR ON state. */
#define LED_ENABLED        1
#define LED_GPIO           8

/* Color shown while the car is ON (GRB order, as WS2812 expects). Off is
 * always all-zero. */
#define LED_ON_R           0
#define LED_ON_G           255
#define LED_ON_B           0

/* WS2812 bit timing (ns), per datasheet. */
#define WS2812_T0H_NS      300
#define WS2812_T0L_NS      900
#define WS2812_T1H_NS      900
#define WS2812_T1L_NS      300
#define WS2812_RESET_US    280

/* MOSFET load switch: PWM input on GPIO1, driven as a plain on/off signal.
 * This module's PWM input is active-low: driving it LOW closes the switch
 * (current flows to the load). Follows the CAR ON state.
 *
 * Because it is active-low, use a 10k pull-UP on GPIO1 (to the module's logic
 * rail, or the ESP 3V3) so the load defaults OFF while the ESP boots -- not a
 * pulldown. The module may already have an internal pull-up on PWM. */
#define LOAD_SWITCH_GPIO   1
#define LOAD_SWITCH_ACTIVE_LOW 0

/* ---- Raspberry Pi shutdown handshake ---- */
#define PI_SHUTDOWN_REQ_GPIO    10   /* output: HIGH asks the Pi to shut down   */
#define PI_ALIVE_GPIO           5    /* input:  HIGH while the Pi is running     */
#define PI_ALIVE_POLL_MS        20
#define PI_ALIVE_LOW_MS         200  /* GPIO5 must stay LOW this long = Pi down  */
#define PI_SHUTDOWN_TIMEOUT_MS  60000

/* ---- MPU6050 I2C ---- */
#define IMU_I2C_PORT        I2C_NUM_0
#define IMU_SCL_GPIO        7
#define IMU_SDA_GPIO        6
#define IMU_I2C_FREQ_HZ     400000
#define IMU_ADDR            0x68              /* AD0 tied low */

#define MPU6050_REG_SMPLRT_DIV    0x19
#define MPU6050_REG_CONFIG        0x1A
#define MPU6050_REG_GYRO_CONFIG   0x1B
#define MPU6050_REG_ACCEL_CONFIG  0x1C
#define MPU6050_REG_PWR_MGMT_1    0x6B
#define MPU6050_REG_WHO_AM_I      0x75
#define MPU6050_REG_ACCEL_XOUT_H  0x3B        /* start of the 14-byte burst    */

/* Gyro full-scale = +/-250 deg/s  ->  131 LSB per deg/s (FS_SEL = 0) */
#define MPU6050_GYRO_SENS_LSB_PER_DPS  131.0f

/* Divider scaling: Vin = Vadc * (R1 + R2) / R2  */
#define DIVIDER_R1_OHMS     103290.0f
#define DIVIDER_R2_OHMS     17594.0f
#define DIVIDER_SCALE       ((DIVIDER_R1_OHMS + DIVIDER_R2_OHMS) / DIVIDER_R2_OHMS)

/* Residual bench calibration: after DIVIDER_SCALE above, Vin still read low
 * by a roughly constant amount (measured on a bench supply, R1/R2 already
 * set from multimeter values). Re-measure and update this constant if the
 * board, divider, or ADC changes.
 *   Bench: 12.00 V -> read 11.67 V  =>  offset = +0.33 V */
#define VIN_CAL_OFFSET_V    0.13f

/* Sampling */
#define SAMPLES_PER_READING 64
#define READING_PERIOD_MS   1000

/*
 * Car on/off detection.
 *
 * Engine off, battery resting:      ~12.0 - 12.7 V
 * Engine running, alternator up:    ~13.7 - 14.7 V
 *
 * A threshold of 13.2 V separates the two with margin. Hysteresis of +/-0.3 V
 * keeps the state from flickering when the voltage sits near the boundary.
 */
#define CAR_ON_THRESHOLD_V     13.2f
#define CAR_STATE_HYST_V        0.3f

/*
 * Engine-stop detection by voltage drop.
 *
 * After shutoff, surface charge holds the battery near ~13.7 V for minutes,
 * so the < (thr - hyst) rule above fires late. Instead, while CAR ON, track a
 * slow moving average (baseline) of Vbat. A fall of more than DROP_TRIGGER_V
 * below it freezes the baseline and starts a timer; if Vbat has not recovered
 * to within DROP_RECOVER_V of the baseline after DROP_CONFIRM_MS, the engine
 * is declared stopped. Shorter dips (fan, AC kicking in) are ignored. The
 * < (thr - hyst) rule is kept as a fallback.
 */
#define DROP_BASELINE_TAU_MS    8000     /* baseline moving-average time constant */
#define DROP_TRIGGER_V          0.4f     /* drop below baseline that starts timer */
#define DROP_RECOVER_V          0.2f     /* back within this of baseline = cancel */
#define DROP_CONFIRM_MS         3000     /* drop must persist this long = STOPPED */
#define DROP_BASELINE_ALPHA     ((float)READING_PERIOD_MS / DROP_BASELINE_TAU_MS)

/*
 * After CAR OFF, the surface-charge voltage (~13.7 V) can still be above the
 * normal (thr + hyst) start level. To restart, Vbat must also rise at least
 * RESTART_RISE_V above the lowest voltage seen since CAR OFF.
 */
#define RESTART_RISE_V          0.3f

/* Sanity window for the raw log flag (outside this = wiring/measurement issue). */
#define VIN_MIN_V              10.0f
#define VIN_MAX_V              15.5f

static adc_oneshot_unit_handle_t s_adc_handle;
static adc_cali_handle_t         s_cali_handle;
static bool                      s_cali_enabled;

static i2c_master_bus_handle_t   s_i2c_bus;
static i2c_master_dev_handle_t   s_imu_dev;
static bool                      s_imu_ok;

static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel,
                                 adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cfg = {
        .unit_id  = unit,
        .chan     = channel,
        .atten    = atten,
        .bitwidth = ADC_BITWIDTH,
    };
    ret = adc_cali_create_scheme_curve_fitting(&cfg, &handle);
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cfg = {
        .unit_id  = unit,
        .atten    = atten,
        .bitwidth = ADC_BITWIDTH,
    };
    ret = adc_cali_create_scheme_line_fitting(&cfg, &handle);
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "ADC calibration enabled");
        return true;
    }
    ESP_LOGW(TAG, "ADC calibration unavailable (%s); using raw-count estimate",
             esp_err_to_name(ret));
    return false;
}

#if LED_ENABLED
static rmt_channel_handle_t s_led_rmt_chan;
static rmt_encoder_handle_t s_led_rmt_encoder;
#endif

static void led_init(void)
{
#if LED_ENABLED
    rmt_tx_channel_config_t chan_cfg = {
        .clk_src           = RMT_CLK_SRC_DEFAULT,
        .gpio_num           = LED_GPIO,
        .mem_block_symbols  = 64,
        .resolution_hz      = 10 * 1000 * 1000,  /* 10 MHz -> 100 ns/tick */
        .trans_queue_depth  = 4,
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&chan_cfg, &s_led_rmt_chan));

    rmt_bytes_encoder_config_t enc_cfg = {
        .bit0 = {
            .level0 = 1, .duration0 = WS2812_T0H_NS / 100,
            .level1 = 0, .duration1 = WS2812_T0L_NS / 100,
        },
        .bit1 = {
            .level0 = 1, .duration0 = WS2812_T1H_NS / 100,
            .level1 = 0, .duration1 = WS2812_T1L_NS / 100,
        },
        .flags.msb_first = 1,
    };
    ESP_ERROR_CHECK(rmt_new_bytes_encoder(&enc_cfg, &s_led_rmt_encoder));

    ESP_ERROR_CHECK(rmt_enable(s_led_rmt_chan));
#endif
}

#if LED_ENABLED
static void led_write_grb(uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t pixel[3] = { g, r, b };  /* WS2812 wants GRB order on the wire. */
    rmt_transmit_config_t tx_cfg = {
        .loop_count = 0,
    };
    ESP_ERROR_CHECK(rmt_transmit(s_led_rmt_chan, s_led_rmt_encoder,
                                 pixel, sizeof(pixel), &tx_cfg));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(s_led_rmt_chan, portMAX_DELAY));
    esp_rom_delay_us(WS2812_RESET_US);
}
#endif

/* on == true lights the LED (LED_ON_R/G/B); off drives all channels to 0. */
static void led_set(bool on)
{
#if LED_ENABLED
    if (on) {
        led_write_grb(LED_ON_R, LED_ON_G, LED_ON_B);
    } else {
        led_write_grb(0, 0, 0);
    }
#else
    (void)on;
#endif
}

/* Level to drive for "load off", given the active-low input. */
#if LOAD_SWITCH_ACTIVE_LOW
#define LOAD_SWITCH_OFF_LEVEL  1
#else
#define LOAD_SWITCH_OFF_LEVEL  0
#endif

static void load_switch_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << LOAD_SWITCH_GPIO,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    gpio_set_level(LOAD_SWITCH_GPIO, LOAD_SWITCH_OFF_LEVEL);   /* start off */
}

/* on == true closes the switch (current flows to the load). */
static void load_switch_set(bool on)
{
#if LOAD_SWITCH_ACTIVE_LOW
    gpio_set_level(LOAD_SWITCH_GPIO, on ? 0 : 1);
#else
    gpio_set_level(LOAD_SWITCH_GPIO, on ? 1 : 0);
#endif
}

static void pi_handshake_init(void)
{
    gpio_config_t out = {
        .pin_bit_mask = 1ULL << PI_SHUTDOWN_REQ_GPIO,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&out));
    gpio_set_level(PI_SHUTDOWN_REQ_GPIO, 0);

    /* Pull-down so an unpowered Pi (floating line) reads as LOW / not alive. */
    gpio_config_t in = {
        .pin_bit_mask = 1ULL << PI_ALIVE_GPIO,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&in));
}

static float read_vbat(void);

/* Voltage Vbat must exceed to latch CAR ON, given the lowest Vbat seen since
 * CAR OFF (0 if none). */
static float restart_threshold_v(float off_min_v)
{
    float on_v = CAR_ON_THRESHOLD_V + CAR_STATE_HYST_V;
    if (off_min_v + RESTART_RISE_V > on_v) {
        on_v = off_min_v + RESTART_RISE_V;
    }
    return on_v;
}

/*
 * Ask the Pi to shut down, wait until it reports it is down (GPIO5 LOW for
 * PI_ALIVE_LOW_MS) or PI_SHUTDOWN_TIMEOUT_MS expires, then cut its power and
 * clear the request line. Blocks for up to PI_SHUTDOWN_TIMEOUT_MS.
 *
 * If the car turns back on while waiting (Vbat > restart_v), the shutdown is
 * cancelled: GPIO10 goes LOW and the Pi's power is left on. Returns true if
 * the Pi's power was cut, false if the shutdown was cancelled.
 */
static bool pi_shutdown_and_power_off(float restart_v)
{
    ESP_LOGI(TAG, "Requesting Pi shutdown (GPIO%d HIGH)", PI_SHUTDOWN_REQ_GPIO);
    gpio_set_level(PI_SHUTDOWN_REQ_GPIO, 1);

    const TickType_t start = xTaskGetTickCount();
    const TickType_t timeout = pdMS_TO_TICKS(PI_SHUTDOWN_TIMEOUT_MS);
    int low_ms = 0;
    bool pi_down = false;

    while (xTaskGetTickCount() - start < timeout) {
        if (read_vbat() > restart_v) {
            gpio_set_level(PI_SHUTDOWN_REQ_GPIO, 0);
            ESP_LOGI(TAG, "Car back ON after %lu ms; shutdown cancelled, GPIO%d LOW",
                     (unsigned long)pdTICKS_TO_MS(xTaskGetTickCount() - start),
                     PI_SHUTDOWN_REQ_GPIO);
            return false;
        }

        if (gpio_get_level(PI_ALIVE_GPIO) == 0) {
            low_ms += PI_ALIVE_POLL_MS;
            if (low_ms >= PI_ALIVE_LOW_MS) {
                pi_down = true;
                break;
            }
        } else {
            low_ms = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(PI_ALIVE_POLL_MS));
    }

    if (pi_down) {
        ESP_LOGI(TAG, "Pi reported shutdown after %lu ms",
                 (unsigned long)pdTICKS_TO_MS(xTaskGetTickCount() - start));
    } else {
        ESP_LOGW(TAG, "Pi shutdown timed out after %d ms; cutting power anyway",
                 PI_SHUTDOWN_TIMEOUT_MS);
    }

    load_switch_set(false);
    gpio_set_level(PI_SHUTDOWN_REQ_GPIO, 0);
    ESP_LOGI(TAG, "Pi power cut, GPIO%d LOW", PI_SHUTDOWN_REQ_GPIO);
    return true;
}

static void adc_init(void)
{
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &s_adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten    = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc_handle, ADC_CHANNEL, &chan_cfg));

    s_cali_enabled = adc_calibration_init(ADC_UNIT, ADC_CHANNEL, ADC_ATTEN,
                                          &s_cali_handle);
}

/* ---------------- MPU6050 ---------------- */

static esp_err_t imu_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(s_imu_dev, buf, sizeof(buf), 100 /* ms */);
}

static esp_err_t imu_read_regs(uint8_t reg, uint8_t *dst, size_t len)
{
    return i2c_master_transmit_receive(s_imu_dev, &reg, 1, dst, len, 100 /* ms */);
}

static void imu_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .clk_source                   = I2C_CLK_SRC_DEFAULT,
        .i2c_port                     = IMU_I2C_PORT,
        .scl_io_num                   = IMU_SCL_GPIO,
        .sda_io_num                   = IMU_SDA_GPIO,
        .glitch_ignore_cnt            = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &s_i2c_bus));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = IMU_ADDR,
        .scl_speed_hz    = IMU_I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_i2c_bus, &dev_cfg, &s_imu_dev));

    uint8_t who = 0;
    esp_err_t ret = imu_read_regs(MPU6050_REG_WHO_AM_I, &who, 1);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "MPU6050 not responding (%s); gyro line disabled",
                 esp_err_to_name(ret));
        s_imu_ok = false;
        return;
    }
    ESP_LOGI(TAG, "MPU6050 WHO_AM_I = 0x%02X", who);

    /* Wake from sleep, use the X gyro as the clock reference. */
    ESP_ERROR_CHECK(imu_write_reg(MPU6050_REG_PWR_MGMT_1, 0x01));
    vTaskDelay(pdMS_TO_TICKS(50));
    /* DLPF ~44 Hz, 1 kHz internal sample rate. */
    ESP_ERROR_CHECK(imu_write_reg(MPU6050_REG_CONFIG, 0x03));
    /* Sample-rate divider -> 100 Hz. */
    ESP_ERROR_CHECK(imu_write_reg(MPU6050_REG_SMPLRT_DIV, 9));
    /* Gyro FS = +/-250 deg/s, accel FS = +/-2 g. */
    ESP_ERROR_CHECK(imu_write_reg(MPU6050_REG_GYRO_CONFIG, 0x00));
    ESP_ERROR_CHECK(imu_write_reg(MPU6050_REG_ACCEL_CONFIG, 0x00));

    s_imu_ok = true;
}

/* Reads gyro rates in deg/s. Returns false if the bus read failed. */
static bool imu_read_gyro_dps(float *gx, float *gy, float *gz)
{
    uint8_t b[14];
    if (imu_read_regs(MPU6050_REG_ACCEL_XOUT_H, b, sizeof(b)) != ESP_OK) {
        return false;
    }
    /* Bytes 8..13 are GYRO_{X,Y,Z}OUT_H/L (accel is 0..5, temp is 6..7). */
    int16_t rx = (int16_t)((b[8]  << 8) | b[9]);
    int16_t ry = (int16_t)((b[10] << 8) | b[11]);
    int16_t rz = (int16_t)((b[12] << 8) | b[13]);

    *gx = rx / MPU6050_GYRO_SENS_LSB_PER_DPS;
    *gy = ry / MPU6050_GYRO_SENS_LSB_PER_DPS;
    *gz = rz / MPU6050_GYRO_SENS_LSB_PER_DPS;
    return true;
}

/* Returns the averaged voltage at the ADC pin, in millivolts. */
static int read_adc_pin_mv(void)
{
    int raw_accum = 0;
    for (int i = 0; i < SAMPLES_PER_READING; i++) {
        int raw = 0;
        ESP_ERROR_CHECK(adc_oneshot_read(s_adc_handle, ADC_CHANNEL, &raw));
        raw_accum += raw;
    }
    int raw_avg = raw_accum / SAMPLES_PER_READING;

    int mv;
    if (s_cali_enabled) {
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(s_cali_handle, raw_avg, &mv));
    } else {
        /* Fallback: assume 12-bit full scale maps to ~3100 mV at 12 dB atten. */
        mv = (raw_avg * 3100) / 4095;
    }
    return mv;
}

/* Returns the reconstructed battery voltage, in volts. */
static float read_vbat(void)
{
    return (read_adc_pin_mv() / 1000.0f) * DIVIDER_SCALE + VIN_CAL_OFFSET_V;
}

void app_main(void)
{
    ESP_LOGI(TAG, "Power Controller starting");
    ESP_LOGI(TAG, "Divider scale = %.3f (R1=%.0f, R2=%.0f)",
             DIVIDER_SCALE, DIVIDER_R1_OHMS, DIVIDER_R2_OHMS);

    led_init();
    load_switch_init();
    pi_handshake_init();
    adc_init();
    imu_init();

    bool car_on = false;
    led_set(car_on);
    load_switch_set(car_on);

    float baseline_v = 0.0f;      /* slow average of Vbat while CAR ON       */
    bool drop_pending = false;    /* Vbat fell DROP_TRIGGER_V below baseline */
    TickType_t drop_start = 0;
    float off_min_v = 0.0f;       /* lowest Vbat since CAR OFF (0 at boot)   */

    while (1) {
        int pin_mv = read_adc_pin_mv();
        float vin = (pin_mv / 1000.0f) * DIVIDER_SCALE + VIN_CAL_OFFSET_V;

        /* Hysteresis: need to cross above (thr + hyst) to latch CAR ON, and
         * below (thr - hyst) to latch CAR OFF. */
        if (!car_on) {
            if (vin < off_min_v) {
                off_min_v = vin;
            }
            if (vin > restart_threshold_v(off_min_v)) {
                car_on = true;
                led_set(true);
                load_switch_set(true);
                baseline_v = vin;
                drop_pending = false;
            }
        } else {
            bool stop = false;

            if (vin < CAR_ON_THRESHOLD_V - CAR_STATE_HYST_V) {
                stop = true;  /* fallback: battery already below resting level */
            } else if (drop_pending) {
                if (vin >= baseline_v - DROP_RECOVER_V) {
                    drop_pending = false;
                    ESP_LOGI(TAG, "Voltage dip recovered (%.2f V, baseline %.2f V); "
                             "load dip, not engine stop", vin, baseline_v);
                } else if (xTaskGetTickCount() - drop_start >=
                           pdMS_TO_TICKS(DROP_CONFIRM_MS)) {
                    stop = true;
                    ESP_LOGI(TAG, "Voltage drop held for %d ms (%.2f V, baseline "
                             "%.2f V); engine stopped", DROP_CONFIRM_MS, vin,
                             baseline_v);
                }
            } else if (vin < baseline_v - DROP_TRIGGER_V) {
                drop_pending = true;
                drop_start = xTaskGetTickCount();
                ESP_LOGI(TAG, "Voltage drop detected (%.2f V, baseline %.2f V); "
                         "confirming", vin, baseline_v);
            }

            /* Baseline is frozen while a drop is being confirmed. */
            if (!drop_pending) {
                baseline_v += DROP_BASELINE_ALPHA * (vin - baseline_v);
            }

            if (stop) {
                car_on = false;
                drop_pending = false;
                off_min_v = vin;
                led_set(false);
                if (!pi_shutdown_and_power_off(restart_threshold_v(off_min_v))) {
                    /* Car came back on mid-shutdown: Pi power was never cut. */
                    car_on = true;
                    led_set(true);
                }
            }
        }

        const char *car_state = car_on ? "CAR ON" : "CAR OFF";
        const char *sanity = "";
        if (vin < VIN_MIN_V) {
            sanity = "  (!) below expected range";
        } else if (vin > VIN_MAX_V) {
            sanity = "  (!) above expected range";
        }

        ESP_LOGI(TAG, "ADC pin = %d mV  |  Vbat = %.2f V  |  %s  |  LOAD %s%s",
                 pin_mv, vin, car_state, car_on ? "ON" : "OFF", sanity);

        if (s_imu_ok) {
            float gx, gy, gz;
            if (imu_read_gyro_dps(&gx, &gy, &gz)) {
                ESP_LOGI(TAG, "Gyro   = %+7.2f  %+7.2f  %+7.2f  deg/s (x y z)",
                         gx, gy, gz);
            } else {
                ESP_LOGW(TAG, "Gyro   = read failed");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(READING_PERIOD_MS));
    }
}
