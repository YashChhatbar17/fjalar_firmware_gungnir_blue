#include <zephyr/kernel.h>
#include <zephyr/drivers/display.h>
#include <zephyr/display/cfb.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>

#include "tracker.h"
#include "display.h"
#include "sensors.h"

LOG_MODULE_REGISTER(app_display, CONFIG_APP_DISPLAY_LOG_LEVEL);

#define DRAWING_THREAD_PRIORITY 6
#define DRAWING_THREAD_STACK_SIZE 1024

K_THREAD_STACK_DEFINE(drawing_thread_stack, DRAWING_THREAD_STACK_SIZE);
struct k_thread drawing_thread_data;
k_tid_t drawing_thread_id;
void drawing_thread(tracker_t *tracker, void *p2, void *p3);

uint8_t font_width;
uint8_t font_height;

const struct device *display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));

void init_display(tracker_t *tracker) {
    drawing_thread_id = k_thread_create(
		&drawing_thread_data,
		drawing_thread_stack,
		K_THREAD_STACK_SIZEOF(drawing_thread_stack),
		(k_thread_entry_t) drawing_thread,
		tracker, NULL, NULL,
		DRAWING_THREAD_PRIORITY, 0, K_NO_WAIT
	);
}

void next_frame(tracker_t *tracker) {
    tracker->current_frame = (tracker->current_frame + 1) % FRAME_MAX;
}

void drawing_thread(tracker_t *tracker, void *p2, void *p3) {
	const struct device *dev;
	uint16_t x_res;
	uint16_t y_res;
	uint16_t rows;
	uint8_t ppt;

	dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
	if (!device_is_ready(dev)) {
		LOG_ERR("Device %s not ready\n", dev->name);
		return;
	}

	if (display_set_pixel_format(dev, PIXEL_FORMAT_MONO10) != 0) {
		if (display_set_pixel_format(dev, PIXEL_FORMAT_MONO01) != 0) {
			LOG_ERR("Failed to set required pixel format");
			return;
		}
	}

	LOG_INF("Initialized %s\n", dev->name);

	if (cfb_framebuffer_init(dev)) {
		LOG_ERR("Framebuffer initialization failed!\n");
		return;
	}

	cfb_framebuffer_clear(dev, true);

	display_blanking_off(dev);

	x_res = cfb_get_display_parameter(dev, CFB_DISPLAY_WIDTH);
	y_res = cfb_get_display_parameter(dev, CFB_DISPLAY_HEIGHT);
	rows = cfb_get_display_parameter(dev, CFB_DISPLAY_ROWS);
	ppt = cfb_get_display_parameter(dev, CFB_DISPLAY_PPT);

	for (int idx = 0; idx < 42; idx++) {
		if (cfb_get_font_size(dev, idx, &font_width, &font_height)) {
			break;
		}
		cfb_framebuffer_set_font(dev, idx);
		LOG_INF("font width %d, font height %d\n",
		       font_width, font_height);
	}

	LOG_INF("x_res %d, y_res %d, ppt %d, rows %d, cols %d\n",
	       x_res,
	       y_res,
	       ppt,
	       rows,
	       cfb_get_display_parameter(dev, CFB_DISPLAY_COLS));

	cfb_framebuffer_invert(dev);

	cfb_set_kerning(dev, 3);

    //init gpio
    const struct gpio_dt_spec touch_sw = GPIO_DT_SPEC_GET(DT_ALIAS(sw1), gpios);
    gpio_pin_configure_dt(&touch_sw, GPIO_INPUT);

    enum screen_frames last_frame = tracker->current_frame;
    draw_frame(tracker, last_frame);
    int ret;
    while (true) {
        ret = gpio_pin_get_dt(&touch_sw);
        if (last_frame != tracker->current_frame || ret) {
            enum screen_frames frame = tracker->current_frame;
            draw_frame(tracker, frame);
            last_frame = frame;
            tracker->current_frame = last_frame;
        }
        k_msleep(100);
    }
}

const static char* state_to_string(enum flight_state state) {
    switch(state) {
        default:
            return "goon";
            // LOG_ERR("Invalid flight state");
    }
}

void draw_frame(tracker_t *tracker, enum screen_frames frame) {
    LOG_DBG("Draw frame");
    cfb_framebuffer_clear(display_dev, true);

    uint8_t buf[128];
    int len;
    int i = 0;
    int x_offset = 10;
    LOG_ERR("hej? %d", tracker->current_frame);
    switch (frame) {
        case FRAME_INFO:
            LOG_ERR("chud");
            i = 3;
            len = snprintk(buf, sizeof(buf), "info");
            cfb_draw_text(display_dev, buf, x_offset, i++ * font_height);
            break;
        case FRAME_TRACKING:
            i = 3;
            len = snprintk(buf, sizeof(buf), "tracking");
            cfb_draw_text(display_dev, buf, x_offset, i++ * font_height);
            len = snprintk(buf, sizeof(buf), "rocket");
            cfb_draw_text(display_dev, buf, x_offset, i++ * font_height);
            len = snprintk(buf, sizeof(buf), "lat: %f°", tracker->rocket.lat);
            cfb_draw_text(display_dev, buf, x_offset, i++ * font_height);
            len = snprintk(buf, sizeof(buf), "lon: %f°", tracker->rocket.lon);
            cfb_draw_text(display_dev, buf, x_offset, i++ * font_height);
            len = snprintk(buf, sizeof(buf), "tracker");
            cfb_draw_text(display_dev, buf, x_offset, i++ * font_height);
            len = snprintk(buf, sizeof(buf), "lat: %f°", tracker->latitude);
            cfb_draw_text(display_dev, buf, x_offset, i++ * font_height);
            len = snprintk(buf, sizeof(buf), "lon: %f°", tracker->longitude);
            cfb_draw_text(display_dev, buf, x_offset, i++ * font_height);

            // float distance = haversine_distance(tracker->telemetry.latitude, tracker->telemetry.longitude, tracker->latitude, tracker->longitude);
            // len = snprintk(buf, sizeof(buf), "distance: %fm", distance);
            // cfb_draw_text(display_dev, buf, x_offset, i++ * font_height);
            break;
        case FRAME_TELEMETRY:
            i = 3;
            len = snprintk(buf, sizeof(buf), "telemetry");
            cfb_draw_text(display_dev, buf, x_offset, i++ * font_height);
            // len = snprintk(buf, sizeof(buf), "vel: %fm/s", tracker->telemetry.velocity);
            // cfb_draw_text(display_dev, buf, x_offset, i++ * font_height);
            // len = snprintk(buf, sizeof(buf), "az: %fm/s2", tracker->telemetry.az);
            // cfb_draw_text(display_dev, buf, x_offset, i++ * font_height);
            // len = snprintk(buf, sizeof(buf), "p1:%d p2:%d p3:%d", tracker->telemetry.pyro1_connected, tracker->telemetry.pyro2_connected, tracker->telemetry.pyro3_connected);
            // cfb_draw_text(display_dev, buf, x_offset, i++ * font_height);
            // len = snprintk(buf, sizeof(buf), "rssi: %ddBm", tracker->local_rssi);
            // cfb_draw_text(display_dev, buf, x_offset, i++ * font_height);
            // len = snprintk(buf, sizeof(buf), "state: %s", state_to_string(tracker->telemetry.flight_state));
            // cfb_draw_text(display_dev, buf, x_offset, i++ * font_height);
            // len = snprintk(buf, sizeof(buf), "flash: %f%%", tracker->telemetry.flash_address / (float) 0x8000000 * 100 * 8);
            // cfb_draw_text(display_dev, buf, x_offset, i++ * font_height);
            // len = snprintk(buf, sizeof(buf), "volt: %fV", tracker->telemetry.battery);
            // cfb_draw_text(display_dev, buf, x_offset, i++ * font_height);
            break;
        case FRAME_GET_READY:
            i = 3;
            len = snprintk(buf, sizeof(buf), "BECOME READY?");
            cfb_draw_text(display_dev, buf, x_offset, i++ * font_height);
            len = snprintk(buf, sizeof(buf), "state: %s", state_to_string(tracker->rocket.state));
            cfb_draw_text(display_dev, buf, x_offset, i++ * font_height);
            break;
        case FRAME_ENTER_INIT:
            i = 3;
            len = snprintk(buf, sizeof(buf), "START INIT?");
            cfb_draw_text(display_dev, buf, x_offset, i++ * font_height);
            len = snprintk(buf, sizeof(buf), "state: %s", state_to_string(tracker->rocket.state));
            cfb_draw_text(display_dev, buf, x_offset, i++ * font_height);
            break;
        case FRAME_MAX:
            break;
    }
    int e = cfb_framebuffer_finalize(display_dev);
    if (e) {
        LOG_ERR("framebuffer could not be finalied: %d", e);
    }
}
