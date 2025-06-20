// SPDX-FileCopyrightText: 2024 Cesanta Software Limited
// SPDX-License-Identifier: GPL-2.0-only or commercial
// Generated by Mongoose Wizard, https://mongoose.ws/wizard/

// Default mock implementation of the API callbacks

#include "mongoose_glue.h"
static struct leds s_leds = {false};
void glue_get_leds(struct leds *data) {
  *data = s_leds;  // Sync with your device
}
void glue_set_leds(struct leds *data) {
  s_leds = *data; // Sync with your device
}

static struct settings s_settings = {42, false, 10};
void glue_get_settings(struct settings *data) {
  *data = s_settings;  // Sync with your device
}
void glue_set_settings(struct settings *data) {
  s_settings = *data; // Sync with your device
}

static struct motor s_motor = {5, 52, 1500};
void glue_get_motor(struct motor *data) {
  *data = s_motor;  // Sync with your device
}
void glue_set_motor(struct motor *data) {
  s_motor = *data; // Sync with your device
}

static struct state s_state = {"1.0.0", 42, 102, 42, 42};
void glue_get_state(struct state *data) {
  *data = s_state;  // Sync with your device
}

void *glue_ota_begin_firmware_update(char *file_name, size_t total_size) {
  bool ok = mg_ota_begin(total_size);
  MG_DEBUG(("%s size %lu, ok: %d", file_name, total_size, ok));
  return ok ? (void *) 1 : NULL;
}
bool glue_ota_end_firmware_update(void *context) {
  mg_timer_add(&g_mgr, 500, 0, (void (*)(void *)) (void *) mg_ota_end, context);
  return true;
}
bool glue_ota_write_firmware_update(void *context, void *buf, size_t len) {
  MG_DEBUG(("ctx: %p %p/%lu", context, buf, len));
  return mg_ota_write(buf, len);
}

static uint64_t s_action_timeout_reboot;  // Time when reboot ends
bool glue_check_reboot(void) {
  return s_action_timeout_reboot > mg_now(); // Return true if reboot is in progress
}
void glue_start_reboot(struct mg_str params) {
  MG_DEBUG(("Passed parameters: [%.*s]", params.len, params.buf));
  s_action_timeout_reboot = mg_now() + 1000; // Start reboot, finish after 1 second
}

static struct read s_read = {false};
void glue_get_read(struct read *data) {
  *data = s_read;  // Sync with your device
}
void glue_set_read(struct read *data) {
  s_read = *data; // Sync with your device
}
