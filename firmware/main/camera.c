#define MAX_CLIENTS 4

#include "freertos/FreeRTOS.h"
#include <freertos/queue.h>
#include "freertos/task.h"
#include <esp_log.h>
#include "camera.h"

static const char *TAG = "camera";

#define BOARD_ESP32CAM_AITHINKER
#define FB_COUNT 4

#define BOARD_WROVER_KIT 1

// WROVER-KIT PIN Map
#ifdef BOARD_WROVER_KIT

#define CAM_PIN_PWDN -1  //power down is not used
#define CAM_PIN_RESET -1 //software reset will be performed
#define CAM_PIN_XCLK 21
#define CAM_PIN_SIOD 26
#define CAM_PIN_SIOC 27

#define CAM_PIN_D7 35
#define CAM_PIN_D6 34
#define CAM_PIN_D5 39
#define CAM_PIN_D4 36
#define CAM_PIN_D3 19
#define CAM_PIN_D2 18
#define CAM_PIN_D1 5
#define CAM_PIN_D0 4
#define CAM_PIN_VSYNC 25
#define CAM_PIN_HREF 23
#define CAM_PIN_PCLK 22

#endif

// ESP32Cam (AiThinker) PIN Map
#ifdef BOARD_ESP32CAM_AITHINKER

#define CAM_PIN_PWDN 32
#define CAM_PIN_RESET -1 //software reset will be performed
#define CAM_PIN_XCLK 0
#define CAM_PIN_SIOD 26
#define CAM_PIN_SIOC 27

#define CAM_PIN_D7 35
#define CAM_PIN_D6 34
#define CAM_PIN_D5 39
#define CAM_PIN_D4 36
#define CAM_PIN_D3 21
#define CAM_PIN_D2 19
#define CAM_PIN_D1 18
#define CAM_PIN_D0 5
#define CAM_PIN_VSYNC 25
#define CAM_PIN_HREF 23
#define CAM_PIN_PCLK 22

#endif

QueueHandle_t Qs[MAX_CLIENTS];


void fb_drop(camera_fb_rc_t* fb) {
  if(atomic_fetch_sub(&fb->refcount, 1) == 1) {
    esp_camera_fb_return(fb->fb);
    free(fb);
  }
}

typedef struct {
  uint8_t subscribe;
  TaskHandle_t task;

  QueueHandle_t queue;
} camera_ctrl_req_t;

QueueHandle_t camera_reqs;


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

uint32_t active;
void camera_task(void *p) {
  camera_reqs = xQueueCreate(MAX_CLIENTS, sizeof(camera_ctrl_req_t*));

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
static camera_config_t camera_config = {
  .pin_pwdn = CAM_PIN_PWDN,
  .pin_reset = CAM_PIN_RESET,
  .pin_xclk = CAM_PIN_XCLK,
  .pin_sscb_sda = CAM_PIN_SIOD,
  .pin_sscb_scl = CAM_PIN_SIOC,

  .pin_d7 = CAM_PIN_D7,
  .pin_d6 = CAM_PIN_D6,
  .pin_d5 = CAM_PIN_D5,
  .pin_d4 = CAM_PIN_D4,
  .pin_d3 = CAM_PIN_D3,
  .pin_d2 = CAM_PIN_D2,
  .pin_d1 = CAM_PIN_D1,
  .pin_d0 = CAM_PIN_D0,
  .pin_vsync = CAM_PIN_VSYNC,
  .pin_href = CAM_PIN_HREF,
  .pin_pclk = CAM_PIN_PCLK,

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

static esp_err_t init_camera()
{
  //initialize the camera
  esp_err_t err = esp_camera_init(&camera_config);
  if (err != ESP_OK)
  {
    ESP_LOGE(TAG, "Camera Init Failed");
    return err;
  }

  return ESP_OK;
}

esp_err_t camera_start() {
  esp_err_t err = esp_camera_init(&camera_config);
  if(err != ESP_OK) {
    ESP_LOGE(TAG, "Camera Init Failed");
    return err;
  }

  for(int i = 0; i < MAX_CLIENTS; i++) {
    Qs[i] = xQueueCreate(1, sizeof(camera_fb_rc_t*));
    assert(Qs[i]);
  }

  BaseType_t res = xTaskCreate(camera_task, "cam_task", 4096, NULL, 1, NULL);
  assert(res);
  return ESP_OK;
}
