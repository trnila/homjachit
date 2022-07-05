#include <unistd.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <sys/socket.h>
#include "camera.h"
#include "stream.h"

atomic_int clients_count;
    char s[1024];
    char str[128];
#define MAX_CLIENTS 4
int clients[MAX_CLIENTS];
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

typedef struct {
  int socket;
  QueueHandle_t queue;
} stream_params_t;

stream_params_t params[MAX_CLIENTS];

void stream_task(void *p) {
  stream_params_t *params = p;

  read(params->socket, s, sizeof(s));
  printf("client accepted\n");

  const char *str = "HTTP/1.0 200 OK\r\nContent-Type: multipart/x-mixed-replace;boundary=" PART_BOUNDARY "\r\nConnection: close\r\n";
  write(params->socket, str, strlen(str));

  QueueHandle_t queue = camera_subscribe();
  bool connected = true;
  while(connected) {
    camera_fb_rc_t* rc = NULL;
    BaseType_t ret = xQueueReceive(queue, &rc, portMAX_DELAY);

    char hdr[256];
    int len = snprintf(hdr, sizeof(hdr), "\r\n--" PART_BOUNDARY "\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", rc->fb->len);

    if(write(params->socket, hdr, len) != len) {
      connected = false;
    } else {
      if(write(params->socket, rc->fb->buf, rc->fb->len) != rc->fb->len) {
        connected = false;
      }
    }

    fb_drop(rc);
  }

  close(params->socket);
  camera_unsubscribe(queue);
  vTaskDelete(NULL);
}

void server_task(void* p) {
  struct sockaddr_in addr;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_family = AF_INET;
  addr.sin_port = htons(81);

  int sock = socket(AF_INET, SOCK_STREAM, 0);
  assert(sock >= 0);

  int err = bind(sock, (struct sockaddr*) &addr, sizeof(addr));
  assert(err == 0);

  err = listen(sock, 1);
  assert(err == 0);

  for(;;) {
    int client_sock = accept(sock, NULL, NULL);
    if(client_sock >= 0) {
      if(atomic_load(&clients_count) >= MAX_CLIENTS) {
        close(client_sock);
      } else {
        int idx = atomic_load(&clients_count);
        stream_params_t *p = &params[idx];
        p->socket = client_sock;
        BaseType_t ret = xTaskCreate(stream_task, "client", 4096, p, 1, NULL);
        assert(ret);
        atomic_fetch_add(&clients_count, 1);
      }
    }
  }
/*
      bool ok = false;
      for(int i = 0; i < MAX_CLIENTS; i++) {
        if(clients[i] == -1) {
          assert(err == 0);
          clients[i] = client_sock;
          ok = true;
          // read request
          read(client_sock, s, sizeof(s));
          printf("client accepted %d\n", clients[i]);

          const char *str = "HTTP/1.0 200 OK\r\nContent-Type: multipart/x-mixed-replace;boundary=" PART_BOUNDARY "\r\nConnection: close\r\n";
          write(client_sock, str, strlen(str));
          break;
        }
      }

      if(!ok) {
        printf("not accepted\n");
        close(client_sock);
      }
    }

    camera_fb_t * fb = NULL;
    fb = esp_camera_fb_get();
    assert(fb);

    int len = snprintf(str, sizeof(str), "\r\n--" PART_BOUNDARY "\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", fb->len);

    for(int i = 0; i < MAX_CLIENTS; i++) {
      if(clients[i] >= 0) {
        //printf("%d %d\n", i, clients[i]);
        if(write(clients[i], str, len) != len) {
          close(clients[i]);
          clients[i] = -1;
        }
        if(write(clients[i], fb->buf, fb->len) != fb->len) {
          close(clients[i]);
          clients[i] = -1;
        }
      }
    }
    esp_camera_fb_return(fb);
//    ESP_LOGI(TAG, "tick");
  }
  */
}

esp_err_t stream_start() {
  BaseType_t res = xTaskCreate(server_task, "stream_task", 4096, NULL, 1, NULL);
  assert(res);

  return ESP_OK;
}
