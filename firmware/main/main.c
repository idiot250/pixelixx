#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "driver/usb_serial_jtag.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"

/* -------------------------------------------------------------------------- */
/* Hardware configuration                                                     */
/* -------------------------------------------------------------------------- */
#define PIXELP_PI               3.14159265358979323846f

#define MATRIX_WIDTH            16
#define MATRIX_HEIGHT           16
#define LED_COUNT               (MATRIX_WIDTH * MATRIX_HEIGHT)
#define LED_GPIO                GPIO_NUM_7

#define I2S_BCLK_GPIO           GPIO_NUM_4
#define I2S_WS_GPIO             GPIO_NUM_5
#define I2S_DATA_GPIO           GPIO_NUM_6

#define ENCODER_A_GPIO          GPIO_NUM_8
#define ENCODER_B_GPIO          GPIO_NUM_9
#define ENCODER_SW_GPIO         GPIO_NUM_10
#define MODE_BUTTON_GPIO        GPIO_NUM_11

#define SAMPLE_RATE             16000
#define FFT_SIZE                256
#define FFT_BANDS               16
#define USB_SYNC_0              0xA5
#define USB_SYNC_1              0x5A
#define USB_PAYLOAD_BYTES       (FFT_SIZE * 2)

/* Set this to 0 for a progressive-row panel instead of a serpentine panel. */
#define MATRIX_SERPENTINE       1

/* Firmware brightness range. Adjust after confirming the LED supply rating. */
#define BRIGHTNESS_DEFAULT      12
#define BRIGHTNESS_MIN          2
#define BRIGHTNESS_MAX          32
#define BRIGHTNESS_STEP         2

static const char *TAG = "pixelp";

/* -------------------------------------------------------------------------- */
/* Shared state                                                                */
/* -------------------------------------------------------------------------- */
typedef enum {
    MODE_WAVE = 0,
    MODE_SPECTRUM,
    MODE_BUBBLES,
    MODE_FIELD,
    MODE_COUNT
} visual_mode_t;

typedef enum {
    SOURCE_MIC = 0,
    SOURCE_USB,
} audio_source_t;

typedef struct {
    float level;                    /* Smoothed 0..1 amplitude envelope */
    float bands[FFT_BANDS];         /* Smoothed 0..1 frequency-band levels */
} audio_features_t;

static volatile visual_mode_t g_mode = MODE_WAVE;
static volatile audio_source_t g_source = SOURCE_MIC;
static volatile uint8_t g_brightness = BRIGHTNESS_DEFAULT;

static led_strip_handle_t g_led_strip;
static i2s_chan_handle_t g_i2s_rx;
static bool g_mic_ready;
static bool g_usb_ready;
static audio_features_t g_features;
static float g_rms_reference = 700.0f;
static float g_peak_height[FFT_BANDS];

/* -------------------------------------------------------------------------- */
/* Small utilities                                                             */
/* -------------------------------------------------------------------------- */
static float clampf(float x, float lo, float hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

static uint8_t clamp_u8(int x)
{
    if (x < 0) return 0;
    if (x > 255) return 255;
    return (uint8_t)x;
}

static float average_bands(int first, int last)
{
    float sum = 0.0f;
    for (int i = first; i <= last; ++i) sum += g_features.bands[i];
    return sum / (float)(last - first + 1);
}

static void hsv_to_rgb(float hue, float saturation, float value,
                       uint8_t *r, uint8_t *g, uint8_t *b)
{
    hue = fmodf(hue, 360.0f);
    if (hue < 0.0f) hue += 360.0f;
    saturation = clampf(saturation, 0.0f, 1.0f);
    value = clampf(value, 0.0f, 1.0f);

    float c = value * saturation;
    float h = hue / 60.0f;
    float x = c * (1.0f - fabsf(fmodf(h, 2.0f) - 1.0f));
    float m = value - c;
    float rr = 0, gg = 0, bb = 0;

    if (h < 1)      { rr = c; gg = x; }
    else if (h < 2) { rr = x; gg = c; }
    else if (h < 3) { gg = c; bb = x; }
    else if (h < 4) { gg = x; bb = c; }
    else if (h < 5) { rr = x; bb = c; }
    else            { rr = c; bb = x; }

    *r = clamp_u8((int)((rr + m) * 255.0f));
    *g = clamp_u8((int)((gg + m) * 255.0f));
    *b = clamp_u8((int)((bb + m) * 255.0f));
}

static inline int matrix_index(int x, int y)
{
#if MATRIX_SERPENTINE
    if (y & 1) x = MATRIX_WIDTH - 1 - x;
#endif
    return y * MATRIX_WIDTH + x;
}

static void set_pixel(int x, int y, uint8_t r, uint8_t g, uint8_t b)
{
    if ((unsigned)x >= MATRIX_WIDTH || (unsigned)y >= MATRIX_HEIGHT) return;
    const uint16_t brightness = g_brightness;
    r = (uint8_t)(((uint16_t)r * brightness) / 255U);
    g = (uint8_t)(((uint16_t)g * brightness) / 255U);
    b = (uint8_t)(((uint16_t)b * brightness) / 255U);
    (void)led_strip_set_pixel(g_led_strip, matrix_index(x, y), r, g, b);
}

static void set_hsv(int x, int y, float hue, float sat, float val)
{
    uint8_t r, g, b;
    hsv_to_rgb(hue, sat, val, &r, &g, &b);
    set_pixel(x, y, r, g, b);
}

static void clear_matrix(void)
{
    (void)led_strip_clear(g_led_strip);
}

/* -------------------------------------------------------------------------- */
/* Peripheral initialization                                                  */
/* -------------------------------------------------------------------------- */
static esp_err_t init_leds(void)
{
    led_strip_config_t strip_cfg = {
        .strip_gpio_num = LED_GPIO,
        .max_leds = LED_COUNT,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB,
        .led_model = LED_MODEL_WS2812,
        .flags.invert_out = false,
    };
    led_strip_rmt_config_t rmt_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
        .mem_block_symbols = 64,
        .flags.with_dma = false,
    };
    esp_err_t err = led_strip_new_rmt_device(&strip_cfg, &rmt_cfg, &g_led_strip);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LED strip initialization failed: %s", esp_err_to_name(err));
        return err;
    }
    return led_strip_clear(g_led_strip);
}

static esp_err_t init_microphone(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = 4;
    chan_cfg.dma_frame_num = FFT_SIZE;

    esp_err_t err = i2s_new_channel(&chan_cfg, NULL, &g_i2s_rx);
    if (err != ESP_OK) return err;

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        /* INMP441 sends a 24-bit sample in a 32-bit I2S slot. Read both slots;
           L/R tied low selects the left slot, which is the first word. */
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_BCLK_GPIO,
            .ws = I2S_WS_GPIO,
            .dout = I2S_GPIO_UNUSED,
            .din = I2S_DATA_GPIO,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };

    err = i2s_channel_init_std_mode(g_i2s_rx, &std_cfg);
    if (err != ESP_OK) return err;
    return i2s_channel_enable(g_i2s_rx);
}

static esp_err_t init_usb_audio(void)
{
    usb_serial_jtag_driver_config_t cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    cfg.rx_buffer_size = 4096;
    cfg.tx_buffer_size = 512;
    return usb_serial_jtag_driver_install(&cfg);
}

static void init_controls(void)
{
    const uint64_t mask = (1ULL << ENCODER_A_GPIO) |
                          (1ULL << ENCODER_B_GPIO) |
                          (1ULL << ENCODER_SW_GPIO) |
                          (1ULL << MODE_BUTTON_GPIO);
    gpio_config_t cfg = {
        .pin_bit_mask = mask,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&cfg));
}

/* -------------------------------------------------------------------------- */
/* Input capture                                                               */
/* -------------------------------------------------------------------------- */
static bool capture_microphone(int16_t samples[FFT_SIZE])
{
    if (!g_mic_ready) {
        memset(samples, 0, FFT_SIZE * sizeof(samples[0]));
        return false;
    }

    int32_t stereo[FFT_SIZE * 2];
    size_t bytes_read = 0;
    esp_err_t err = i2s_channel_read(g_i2s_rx, stereo, sizeof(stereo),
                                     &bytes_read, pdMS_TO_TICKS(60));
    if (err != ESP_OK || bytes_read < sizeof(stereo)) {
        memset(samples, 0, FFT_SIZE * sizeof(samples[0]));
        return false;
    }

    for (int i = 0; i < FFT_SIZE; ++i) {
        /* INMP441's 24 valid bits are left-aligned in the 32-bit slot. */
        samples[i] = (int16_t)(stereo[2 * i] >> 16);
    }
    return true;
}

/* USB audio packet: A5 5A, uint16 sequence, then 256 signed little-endian
   16-bit mono PCM samples. Parser state is retained across USB reads. */
typedef enum {
    USB_PARSE_SYNC0,
    USB_PARSE_SYNC1,
    USB_PARSE_SEQ0,
    USB_PARSE_SEQ1,
    USB_PARSE_PAYLOAD,
} usb_parse_state_t;

static usb_parse_state_t s_usb_state = USB_PARSE_SYNC0;
static uint8_t s_usb_payload[USB_PAYLOAD_BYTES];
static uint16_t s_usb_payload_pos;
static uint8_t s_usb_seq_low;
static uint16_t s_usb_sequence;

static bool usb_parser_feed(uint8_t byte, int16_t samples[FFT_SIZE])
{
    switch (s_usb_state) {
    case USB_PARSE_SYNC0:
        if (byte == USB_SYNC_0) s_usb_state = USB_PARSE_SYNC1;
        break;
    case USB_PARSE_SYNC1:
        if (byte == USB_SYNC_1) s_usb_state = USB_PARSE_SEQ0;
        else s_usb_state = (byte == USB_SYNC_0) ? USB_PARSE_SYNC1 : USB_PARSE_SYNC0;
        break;
    case USB_PARSE_SEQ0:
        s_usb_seq_low = byte;
        s_usb_state = USB_PARSE_SEQ1;
        break;
    case USB_PARSE_SEQ1:
        s_usb_sequence = (uint16_t)s_usb_seq_low | ((uint16_t)byte << 8);
        s_usb_payload_pos = 0;
        s_usb_state = USB_PARSE_PAYLOAD;
        break;
    case USB_PARSE_PAYLOAD:
        s_usb_payload[s_usb_payload_pos++] = byte;
        if (s_usb_payload_pos >= USB_PAYLOAD_BYTES) {
            memcpy(samples, s_usb_payload, USB_PAYLOAD_BYTES);
            s_usb_state = USB_PARSE_SYNC0;
            (void)s_usb_sequence; /* Reserved for packet-loss diagnostics. */
            return true;
        }
        break;
    }
    return false;
}

static bool capture_usb_audio(int16_t samples[FFT_SIZE])
{
    if (!g_usb_ready) {
        memset(samples, 0, FFT_SIZE * sizeof(samples[0]));
        return false;
    }

    uint8_t chunk[64];
    const int64_t deadline = esp_timer_get_time() + 25000; /* 25 ms */

    while (esp_timer_get_time() < deadline) {
        int n = usb_serial_jtag_read_bytes(chunk, sizeof(chunk), pdMS_TO_TICKS(10));
        for (int i = 0; i < n; ++i) {
            if (usb_parser_feed(chunk[i], samples)) return true;
        }
    }

    /* No complete packet arrived. Render silence rather than freezing the UI. */
    memset(samples, 0, FFT_SIZE * sizeof(samples[0]));
    return false;
}

/* -------------------------------------------------------------------------- */
/* Audio analysis: Hann window + in-place radix-2 FFT                          */
/* -------------------------------------------------------------------------- */
static void fft_256(float real[FFT_SIZE], float imag[FFT_SIZE])
{
    unsigned j = 0;
    for (unsigned i = 1; i < FFT_SIZE; ++i) {
        unsigned bit = FFT_SIZE >> 1;
        while (j & bit) {
            j ^= bit;
            bit >>= 1;
        }
        j ^= bit;
        if (i < j) {
            float tr = real[i]; real[i] = real[j]; real[j] = tr;
            float ti = imag[i]; imag[i] = imag[j]; imag[j] = ti;
        }
    }

    for (unsigned len = 2; len <= FFT_SIZE; len <<= 1) {
        const float angle = -2.0f * (float)PIXELP_PI / (float)len;
        const float wlen_r = cosf(angle);
        const float wlen_i = sinf(angle);
        for (unsigned i = 0; i < FFT_SIZE; i += len) {
            float wr = 1.0f, wi = 0.0f;
            for (unsigned k = 0; k < len / 2; ++k) {
                const unsigned u = i + k;
                const unsigned v = i + k + len / 2;
                const float vr = real[v] * wr - imag[v] * wi;
                const float vi = real[v] * wi + imag[v] * wr;
                real[v] = real[u] - vr;
                imag[v] = imag[u] - vi;
                real[u] += vr;
                imag[u] += vi;
                const float next_wr = wr * wlen_r - wi * wlen_i;
                wi = wr * wlen_i + wi * wlen_r;
                wr = next_wr;
            }
        }
    }
}

static void analyze_audio(const int16_t samples[FFT_SIZE])
{
    float real[FFT_SIZE];
    float imag[FFT_SIZE] = {0};
    double sum_sq = 0.0;

    for (int i = 0; i < FFT_SIZE; ++i) {
        const float s = (float)samples[i] / 32768.0f;
        sum_sq += (double)s * (double)s;
        const float window = 0.5f - 0.5f * cosf((2.0f * (float)PIXELP_PI * i) / (FFT_SIZE - 1));
        real[i] = s * window;
    }

    const float rms = sqrtf((float)(sum_sq / FFT_SIZE)) * 32768.0f;
    if (rms > g_rms_reference) g_rms_reference += (rms - g_rms_reference) * 0.08f;
    else g_rms_reference += (rms - g_rms_reference) * 0.004f;
    if (g_rms_reference < 450.0f) g_rms_reference = 450.0f;
    const float raw_level = clampf(rms / (g_rms_reference * 1.25f), 0.0f, 1.0f);
    g_features.level = g_features.level * 0.55f + raw_level * 0.45f;

    fft_256(real, imag);

    const int min_bin = 2;   /* 125 Hz at 16 kHz / 256 */
    const int max_bin = 110; /* Below Nyquist; upper bands cover most audible detail. */
    for (int band = 0; band < FFT_BANDS; ++band) {
        const float ratio = (float)max_bin / (float)min_bin;
        int first = (int)floorf(min_bin * powf(ratio, (float)band / FFT_BANDS));
        int last = (int)ceilf(min_bin * powf(ratio, (float)(band + 1) / FFT_BANDS)) - 1;
        if (first < 1) first = 1;
        if (last < first) last = first;
        if (last > FFT_SIZE / 2 - 1) last = FFT_SIZE / 2 - 1;

        float peak = 0.0f;
        for (int bin = first; bin <= last; ++bin) {
            const float magnitude = sqrtf(real[bin] * real[bin] + imag[bin] * imag[bin]);
            const float normalized = magnitude / (FFT_SIZE * 0.25f);
            if (normalized > peak) peak = normalized;
        }
        float target = clampf(powf(peak * 2.3f, 0.78f), 0.0f, 1.0f);
        if (target > g_features.bands[band]) {
            g_features.bands[band] = g_features.bands[band] * 0.48f + target * 0.52f;
        } else {
            g_features.bands[band] = g_features.bands[band] * 0.84f + target * 0.16f;
        }
    }
}

/* -------------------------------------------------------------------------- */
/* Visualizations                                                              */
/* -------------------------------------------------------------------------- */
static void render_wave(const int16_t samples[FFT_SIZE], float palette, float t)
{
    (void)t;
    for (int x = 0; x < MATRIX_WIDTH; ++x) {
        const int sample_index = x * (FFT_SIZE / MATRIX_WIDTH);
        const float amplitude = (float)samples[sample_index] / 32768.0f;
        const int center_y = 8 - (int)lrintf(amplitude * 6.5f);
        const float intensity = 0.28f + fabsf(amplitude) * 0.72f;
        const float hue = palette + x * 2.6f;
        set_hsv(x, center_y, hue, 0.68f, intensity);
        set_hsv(x, center_y - 1, hue + 5.0f, 0.80f, intensity * 0.36f);
        set_hsv(x, center_y + 1, hue - 5.0f, 0.80f, intensity * 0.36f);
    }
}

static void render_spectrum(float palette)
{
    for (int x = 0; x < MATRIX_WIDTH; ++x) {
        const float value = g_features.bands[x];
        const float height = value * 15.0f;
        if (height > g_peak_height[x]) g_peak_height[x] = height;
        else g_peak_height[x] = fmaxf(0.0f, g_peak_height[x] - 0.16f);

        const int whole = (int)height;
        for (int row = 0; row <= whole && row < MATRIX_HEIGHT; ++row) {
            const int y = MATRIX_HEIGHT - 1 - row;
            const float vertical = 0.30f + 0.70f * ((float)row / 15.0f);
            set_hsv(x, y, palette + x * 3.2f + row * 1.25f,
                    0.72f, vertical);
        }

        const int peak_y = MATRIX_HEIGHT - 1 - (int)g_peak_height[x];
        set_hsv(x, peak_y, palette + x * 3.2f + 12.0f, 0.25f, 1.0f);
    }
}

typedef struct {
    float x, y, radius, hue;
    bool active;
} bubble_t;

static bubble_t s_bubbles[7];
static int64_t s_last_bubble_us;

static void render_bubbles(float palette, float now_s)
{
    const float energy = g_features.level;
    if (energy > 0.16f && esp_timer_get_time() - s_last_bubble_us >
        (int64_t)(170000.0f - energy * 90000.0f)) {
        for (int i = 0; i < (int)(sizeof(s_bubbles) / sizeof(s_bubbles[0])); ++i) {
            if (!s_bubbles[i].active) {
                s_bubbles[i].x = (float)(esp_random() % MATRIX_WIDTH);
                s_bubbles[i].y = (float)(esp_random() % MATRIX_HEIGHT);
                s_bubbles[i].radius = 0.5f;
                s_bubbles[i].hue = palette + (float)(esp_random() % 38);
                s_bubbles[i].active = true;
                s_last_bubble_us = esp_timer_get_time();
                break;
            }
        }
    }

    for (int i = 0; i < (int)(sizeof(s_bubbles) / sizeof(s_bubbles[0])); ++i) {
        bubble_t *b = &s_bubbles[i];
        if (!b->active) continue;
        b->radius += 0.055f + energy * 0.20f;
        const float life = clampf(1.0f - b->radius / 8.8f, 0.0f, 1.0f);
        const int min_x = (int)floorf(b->x - b->radius - 1);
        const int max_x = (int)ceilf(b->x + b->radius + 1);
        const int min_y = (int)floorf(b->y - b->radius - 1);
        const int max_y = (int)ceilf(b->y + b->radius + 1);
        for (int y = min_y; y <= max_y; ++y) {
            for (int x = min_x; x <= max_x; ++x) {
                const float dx = (float)x - b->x;
                const float dy = (float)y - b->y;
                const float d = sqrtf(dx * dx + dy * dy);
                const float ring = fabsf(d - b->radius);
                if (ring < 0.85f) {
                    const float value = life * (1.0f - ring / 0.85f) * 0.95f;
                    set_hsv(x, y, b->hue + now_s * 3.0f, 0.58f, value);
                }
            }
        }
        if (b->radius > 9.0f) b->active = false;
    }
}

static float gaussian(float x, float y, float cx, float cy, float spread)
{
    const float dx = x - cx;
    const float dy = y - cy;
    return expf(-(dx * dx + dy * dy) / spread);
}

static void render_frequency_field(float palette, float t)
{
    const float low = average_bands(0, 4);
    const float mid = average_bands(5, 10);
    const float high = average_bands(11, 15);

    const float low_x = 3.2f + 1.8f * sinf(t * 0.47f);
    const float low_y = 5.0f + 2.0f * cosf(t * 0.39f);
    const float mid_x = 11.7f + 2.0f * cosf(t * 0.34f + 1.1f);
    const float mid_y = 10.0f + 2.0f * sinf(t * 0.43f);
    const float high_x = 7.7f + 3.2f * sinf(t * 0.28f + 2.0f);
    const float high_y = 7.4f + 3.0f * cosf(t * 0.31f + 0.6f);

    for (int y = 0; y < MATRIX_HEIGHT; ++y) {
        for (int x = 0; x < MATRIX_WIDTH; ++x) {
            const float a = low * gaussian((float)x, (float)y, low_x, low_y, 22.0f);
            const float b = mid * gaussian((float)x, (float)y, mid_x, mid_y, 28.0f);
            const float c = high * gaussian((float)x, (float)y, high_x, high_y, 18.0f);
            const float total = a + b + c;
            const float value = clampf(0.035f + total * 1.22f, 0.0f, 1.0f);
            const float hue = palette + (a * 3.0f + b * 26.0f + c * 51.0f) /
                                         (total + 0.001f);
            const float saturation = clampf(0.92f - total * 0.45f, 0.10f, 0.82f);
            set_hsv(x, y, hue, saturation, value);
        }
    }
}

static void render_frame(const int16_t samples[FFT_SIZE])
{
    const float t = (float)esp_timer_get_time() / 1000000.0f;
    const float palette = 220.0f + 18.0f * sinf(t * 0.12f);
    clear_matrix();

    switch (g_mode) {
    case MODE_WAVE:     render_wave(samples, palette, t); break;
    case MODE_SPECTRUM: render_spectrum(palette); break;
    case MODE_BUBBLES:  render_bubbles(palette, t); break;
    case MODE_FIELD:    render_frequency_field(palette, t); break;
    default:            render_wave(samples, palette, t); break;
    }
    (void)led_strip_refresh(g_led_strip);
}

/* -------------------------------------------------------------------------- */
/* Encoder + buttons                                                           */
/* Encoder press: toggle MIC/USB audio. Mode button: cycle visualizations.     */
/* Encoder rotation: adjust brightness.                                       */
/* -------------------------------------------------------------------------- */
static int read_encoder_state(void)
{
    return (gpio_get_level(ENCODER_A_GPIO) << 1) | gpio_get_level(ENCODER_B_GPIO);
}

static void controls_task(void *arg)
{
    (void)arg;
    static const int8_t transition[16] = {
         0, -1,  1,  0,
         1,  0,  0, -1,
        -1,  0,  0,  1,
         0,  1, -1,  0
    };

    int previous_encoder = read_encoder_state();
    int quarter_steps = 0;
    int mode_last_raw = 1, mode_stable = 1;
    int source_last_raw = 1, source_stable = 1;
    int64_t mode_change_us = 0, source_change_us = 0;

    while (1) {
        const int current_encoder = read_encoder_state();
        const int transition_index = (previous_encoder << 2) | current_encoder;
        quarter_steps += transition[transition_index];
        previous_encoder = current_encoder;

        if (quarter_steps >= 4) {
            if (g_brightness + BRIGHTNESS_STEP <= BRIGHTNESS_MAX)
                g_brightness += BRIGHTNESS_STEP;
            quarter_steps = 0;
        } else if (quarter_steps <= -4) {
            if (g_brightness >= BRIGHTNESS_MIN + BRIGHTNESS_STEP)
                g_brightness -= BRIGHTNESS_STEP;
            quarter_steps = 0;
        }

        const int mode_raw = gpio_get_level(MODE_BUTTON_GPIO);
        if (mode_raw != mode_last_raw) {
            mode_last_raw = mode_raw;
            mode_change_us = esp_timer_get_time();
        }
        if ((esp_timer_get_time() - mode_change_us) > 35000 && mode_stable != mode_raw) {
            mode_stable = mode_raw;
            if (mode_stable == 0) {
                g_mode = (visual_mode_t)(((int)g_mode + 1) % MODE_COUNT);
                ESP_LOGI(TAG, "visual mode: %d", (int)g_mode);
            }
        }

        const int source_raw = gpio_get_level(ENCODER_SW_GPIO);
        if (source_raw != source_last_raw) {
            source_last_raw = source_raw;
            source_change_us = esp_timer_get_time();
        }
        if ((esp_timer_get_time() - source_change_us) > 35000 && source_stable != source_raw) {
            source_stable = source_raw;
            if (source_stable == 0) {
                g_source = (g_source == SOURCE_MIC) ? SOURCE_USB : SOURCE_MIC;
                ESP_LOGI(TAG, "audio source: %s", g_source == SOURCE_MIC ? "microphone" : "USB system audio");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* -------------------------------------------------------------------------- */
/* Main                                                                        */
/* -------------------------------------------------------------------------- */
void app_main(void)
{
    ESP_LOGI(TAG, "Starting PixelP firmware");
    ESP_ERROR_CHECK(init_leds());
    init_controls();

    esp_err_t usb_err = init_usb_audio();
    g_usb_ready = (usb_err == ESP_OK);
    if (g_usb_ready) ESP_LOGI(TAG, "USB Serial/JTAG audio input ready");
    else ESP_LOGE(TAG, "USB audio driver init failed: %s", esp_err_to_name(usb_err));

    esp_err_t mic_err = init_microphone();
    g_mic_ready = (mic_err == ESP_OK);
    if (g_mic_ready) ESP_LOGI(TAG, "INMP441 I2S microphone ready");
    else ESP_LOGW(TAG, "Microphone unavailable (%s); USB input can still be used", esp_err_to_name(mic_err));

    xTaskCreate(controls_task, "pixelp_controls", 3072, NULL, 5, NULL);

    int16_t samples[FFT_SIZE] = {0};
    int64_t last_log_us = esp_timer_get_time();

    while (1) {
        bool captured = false;
        if (g_source == SOURCE_MIC) captured = capture_microphone(samples);
        else captured = capture_usb_audio(samples);

        if (!captured) {
            /* Keep the animation alive if an input is unavailable, without
               spinning continuously while waiting for that input. */
            memset(samples, 0, sizeof(samples));
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        analyze_audio(samples);
        render_frame(samples);

        if (esp_timer_get_time() - last_log_us > 5000000) {
            ESP_LOGI(TAG, "mode=%d source=%s brightness=%u level=%.2f",
                     (int)g_mode,
                     g_source == SOURCE_MIC ? "mic" : "usb",
                     (unsigned)g_brightness,
                     (double)g_features.level);
            last_log_us = esp_timer_get_time();
        }
    }
}
