#pragma once
#include <stdatomic.h>
#include <esp_err.h>
#include <esp_camera.h>

typedef struct {
  camera_fb_t* fb;
  atomic_int refcount;
} camera_fb_rc_t;

esp_err_t camera_start();
QueueHandle_t camera_subscribe();
void camera_unsubscribe(QueueHandle_t queue);
void fb_drop(camera_fb_rc_t* fb);
