
#include "freertos/FreeRTOS.h"
#include <freertos/queue.h>
#include "freertos/task.h"
#include <esp_log.h>
#include "camera.h"

#define FB_COUNT 4
#define MAX_CLIENTS 4

typedef struct {
  uint8_t subscribe;
  TaskHandle_t task;
  QueueHandle_t queue;
} camera_ctrl_req_t;

static const char *TAG = "camera";

static QueueHandle_t Qs[MAX_CLIENTS];
static QueueHandle_t camera_reqs;
static uint32_t active;

static camera_config_t camera_config = {
  .pin_pwdn = 32,
  .pin_reset = -1,
  .pin_xclk = 0,
  .pin_sscb_sda = 26,
  .pin_sscb_scl = 27,

  .pin_d7 = 35,
  .pin_d6 = 34,
  .pin_d5 = 39,
  .pin_d4 = 36,
  .pin_d3 = 21,
  .pin_d2 = 19,
  .pin_d1 = 18,
  .pin_d0 = 5,
  .pin_vsync = 25,
  .pin_href = 23,
  .pin_pclk = 22,

  //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
  .xclk_freq_hz = 20000000,
  .ledc_timer = LEDC_TIMER_0,
  .ledc_channel = LEDC_CHANNEL_0,

  .pixel_format = PIXFORMAT_JPEG, //YUV422,GRAYSCALE,RGB565,JPEG
  .frame_size = FRAMESIZE_VGA,    //QQVGA-UXGA Do not use sizes above QVGA when not JPEG

  .jpeg_quality = 12, //0-63 lower number means higher quality
  .fb_count = FB_COUNT,       //if more than one, i2s runs in continuous mode. Use only with JPEG
  .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

void fb_drop(camera_fb_rc_t* fb) {
  if(atomic_fetch_sub(&fb->refcount, 1) == 1) {
    esp_camera_fb_return(fb->fb);
    free(fb);
  }
}

QueueHandle_t camera_subscribe() {
  camera_ctrl_req_t req;
  req.subscribe = 1;
  req.task = xTaskGetCurrentTaskHandle();
  req.queue = NULL;

  camera_ctrl_req_t *p = &req;
  xQueueSend(camera_reqs, &p, portMAX_DELAY);
  printf("sent\n");
  ulTaskNotifyTake(false, portMAX_DELAY);
  printf("got\n");
  assert(req.queue != NULL);
  return req.queue;
}

void camera_unsubscribe(QueueHandle_t queue) {
  camera_ctrl_req_t req;
  req.subscribe = 0;
  req.task = xTaskGetCurrentTaskHandle();
  req.queue = queue;
  camera_ctrl_req_t *p = &req;
  xQueueSend(camera_reqs, &p, portMAX_DELAY);
}

static void camera_task(void *p) {
  for(;;) {
    camera_ctrl_req_t *req;
    while(xQueueReceive(camera_reqs, &req, 0)) {
      if(req->subscribe) {
        int available_idx = -1;
        for(int i = 0; i < MAX_CLIENTS; i++) {
          if(!(active & (1U << i))) {
            available_idx = i;
            active |= 1U << i;
            break;
          }
        }

        assert(available_idx >= 0);
        req->queue = Qs[available_idx];
        xTaskNotifyGive(req->task);
      } else {
        bool found = false;
        for(int i = 0; i < MAX_CLIENTS; i++) {
          if(req->queue == Qs[i]) {
            active &= ~(1U << i);

            camera_fb_rc_t* rc = NULL;
            while(xQueueReceive(req->queue, &rc, 0)) {
              fb_drop(rc);
            }
            found = true;
            break;
          }
        }

        assert(found);
      }
    }

    camera_fb_t *fb = esp_camera_fb_get();
    if(!fb) {
      printf("no buffer\r\n");
      continue;
    }
    assert(fb);

    camera_fb_rc_t* rc = malloc(sizeof(*rc));
    rc->fb = fb;
    atomic_store(&rc->refcount, 1);

    for(int i = 0; i < MAX_CLIENTS; i++) {
      if(active & (1U << i)) {
        atomic_fetch_add(&rc->refcount, 1);
        if(!xQueueSend(Qs[i], &rc, 0)) {
          fb_drop(rc);
        }
      }
    }
    fb_drop(rc);
  }
}

esp_err_t camera_start() {
  esp_err_t err = esp_camera_init(&camera_config);
  if(err != ESP_OK) {
    ESP_LOGE(TAG, "Camera Init Failed");
    return err;
  }

  camera_reqs = xQueueCreate(MAX_CLIENTS, sizeof(camera_ctrl_req_t*));
  assert(camera_reqs);

  for(int i = 0; i < MAX_CLIENTS; i++) {
    Qs[i] = xQueueCreate(1, sizeof(camera_fb_rc_t*));
    assert(Qs[i]);
  }

  BaseType_t res = xTaskCreate(camera_task, "cam_task", 4096, NULL, 1, NULL);
  assert(res);
  return ESP_OK;
}
