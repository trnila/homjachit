#include <unistd.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <sys/socket.h>
#include "camera.h"
#include "stream.h"

#define MAX_CLIENTS 4
#define PART_BOUNDARY "123456789000000000000987654321"

atomic_int clients_count;
char s[1024];
char str[128];
int clients[MAX_CLIENTS];

void stream_task(void *p) {
  int client_socket = p;

  read(client_socket, s, sizeof(s));
  printf("client accepted\n");

  const char *str = "HTTP/1.0 200 OK\r\nContent-Type: multipart/x-mixed-replace;boundary=" PART_BOUNDARY "\r\nConnection: close\r\n";
  write(client_socket, str, strlen(str));

  QueueHandle_t queue = camera_subscribe();
  bool connected = true;
  while(connected) {
    camera_fb_rc_t* rc = NULL;
    BaseType_t ret = xQueueReceive(queue, &rc, portMAX_DELAY);
    assert(ret);

    char hdr[256];
    int len = snprintf(hdr, sizeof(hdr), "\r\n--" PART_BOUNDARY "\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", rc->fb->len);

    if(write(client_socket, hdr, len) != len) {
      connected = false;
    } else {
      if(write(client_socket, rc->fb->buf, rc->fb->len) != rc->fb->len) {
        connected = false;
      }
    }

    fb_drop(rc);
  }

  close(client_socket);
  camera_unsubscribe(queue);
  atomic_fetch_sub(&clients_count, 1);
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
        BaseType_t ret = xTaskCreate(stream_task, "client", 4096, client_sock, 1, NULL);
        assert(ret);
        atomic_fetch_add(&clients_count, 1);
      }
    }
  }
}

esp_err_t stream_start() {
  BaseType_t res = xTaskCreate(server_task, "stream_task", 4096, NULL, 1, NULL);
  assert(res);

  return ESP_OK;
}
