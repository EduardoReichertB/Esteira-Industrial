// Esteira Industrial (ESP32 + FreeRTOS) — 4 tasks com touch ISR
// - ENC_SENSE (periódica 5 ms) -> notifica SPD_CTRL
// - SPD_CTRL (hard RT) -> controle PI simulado, trata HMI (soft) se solicitado
// - SORT_ACT (hard RT, evento Touch B) -> aciona "desviador"
// - SAFETY_TASK (hard RT, evento Touch D) -> E-stop
// Compilado com ESP-IDF 5.x

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/touch_sens.h"

//novos includes:
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "esp_event.h"
#include "esp_sntp.h"
#include <time.h>
#include <sys/time.h>
#include <inttypes.h>

#define TAG "ESTEIRA"

//VARIAVEIS PARA A TASK TIME
static const char *TAG_TIME = "TIME_TASK";

#define TIME_PERIOD_MS       1000
#define TIME_DEADLINE_US     1000000
#define TIME_STACK           3072
#define TIME_PRIORITY        1

static portMUX_TYPE time_mux = portMUX_INITIALIZER_UNLOCKED;

static int64_t last_sntp_sync_us = -1;
static uint32_t sntp_sync_generation = 0;

static uint32_t time_total = 0;
static uint32_t time_deadline_misses = 0;
static int64_t time_c_max_us = 0;

#define MONITOR_PROTOCOL_NAME "UDP"

// Mapeamento para usar wi-fi
static const char *TAG_WIFI = "TCP_SERVER";
#define WIFI_SSID "FAAR_2G"
#define WIFI_PASS "paoCOMmanteiga"
#define TCP_PORT 7000 // qualquer número entre 0 e 65535

#define PC_IP   "192.168.1.109"   // IP do PC - use ifconfig (Linux) ou ipconfig (Windows) para achar o IP
#define PC_PORT 7070 // qualquer número entre 0 e 65535

// ====== Mapeamento dos touch pads ======
#define TP_OBJ   TOUCH_PAD_NUM7   // Touch B -> detecção de objeto (desvio)
#define TP_HMI   TOUCH_PAD_NUM8   // Touch C -> HMI/telemetria
#define TP_ESTOP TOUCH_PAD_NUM9   // Touch D -> E-stop

// ====== Periodicidade, prioridades, stack ======
#define ENC_T_MS        5
#define PRIO_ESTOP      5 // RM: 3 DM: 5 CS: 5
#define PRIO_ENC        4 // RM: 5 DM: 4 CS: 4
#define PRIO_CTRL       3 // RM: 4 DM: 3 CS: 3
#define PRIO_SORT       2 // RM: 3 DM: 2 CS: 3
#define PRIO_MON        1 // prioridade monitor
#define STK             3072

// ====== Deadlines (do enunciado) ======
#define DEADLINE_ENC_US       5000    // 5 ms
#define DEADLINE_CTRL_US     10000    // 10 ms
#define DEADLINE_SORT_US     10000    // 10 ms
#define DEADLINE_SAFETY_US    5000    // 5 ms
#define DEADLINE_MONITOR_US 500000    // 500 ms
#define MONITOR_PERIOD_MS      500

// comandos dentro do monitor
typedef enum {
  CMD_SORT,
  CMD_ESTOP,
  CMD_STATUS,
  CMD_PING
}monitor_cmd_t;

// tipo para mensagem do monitor
typedef struct {
  char payload[256];
}monitor_msg_t;

// PARA MONITOR
static QueueHandle_t qMonitorTx = NULL;  //fila para mandar os log
static QueueHandle_t qMonitorCmd = NULL; //fila de comando

static TaskHandle_t hMonitor = NULL;
static TaskHandle_t hNet = NULL;

// Estatísticas temporais da MONITOR_TASK
static uint32_t monitor_total = 0;
static uint32_t monitor_misses = 0;
static int64_t monitor_c_max_us = 0;
static int64_t monitor_r_max_us = 0;

// ====== Handles/IPC ======
static TaskHandle_t hENC = NULL, hCTRL = NULL, hSORT = NULL, hSAFE = NULL;

// Notificação ENC->CTRL (direct-to-task notification)
static TaskHandle_t hCtrlNotify = NULL;

// Fila de eventos para o SORT_ACT
typedef struct {
    int64_t t_evt_us;   // timestamp do toque (para logging/latência)
} sort_evt_t;
static QueueHandle_t qSort = NULL;

// Semáforos para E-stop e HMI
static SemaphoreHandle_t semEStop = NULL;
static SemaphoreHandle_t semHMI   = NULL;

// Estado "simulado" da esteira
typedef struct {
  float rpm;
  float pos_mm;
  float set_rpm;
}belt_state_t;

typedef struct{
  int misses_enc;
  int total_enc;
  int misses_spd;
  int total_spd;
  int misses_sort;
  int total_sort;
  int misses_stop;
  int total_stop;
}misses_t;

misses_t misses = {0};

static belt_state_t g_belt = { .rpm = 0.f, .pos_mm = 0.f, .set_rpm = 120.0f };

// variaveis para conseguir o tempo ao tocar e fazer algo
static volatile int64_t tempo_touch_b = 0;
static volatile int64_t tempo_touch_c = 0;
static volatile int64_t tempo_touch_d = 0;

//registra um instante
static void time_sync_notification_cb(struct timeval *tv)
{
  (void)tv;

  int64_t now_us = esp_timer_get_time();

  portENTER_CRITICAL(&time_mux);
  last_sntp_sync_us = now_us;
  sntp_sync_generation++;
  portEXIT_CRITICAL(&time_mux);

  ESP_LOGI(TAG_TIME, "Tempo sincronizado via SNTP");
}

static void time_sntp_init(void){
    // Fuso horário de Brasília: UTC-3
    setenv("TZ", "BRT-3", 1);
    tzset();

    // Callback chamado quando o relógio é sincronizado
    esp_sntp_set_time_sync_notification_cb(
      time_sync_notification_cb
    );

    // Servidores NTP
    esp_sntp_setservername(0, "a.st1.ntp.br");
    esp_sntp_setservername(1, "b.st1.ntp.br");
    esp_sntp_setservername(2, "pool.ntp.org");

    // Configura e inicia o SNTP
    esp_sntp_setoperatingmode(ESP_SNTP_OPMODE_POLL);
    esp_sntp_init();

    ESP_LOGI(TAG_TIME, "SNTP iniciado; aguardando sincronizacao");
}

static void time_task(void *arg)
{
    (void)arg;

    TickType_t next = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(TIME_PERIOD_MS);

    uint32_t last_generation = 0;
    uint32_t cycles_since_sync = 0;

    for (;;) {
        int64_t t_start = esp_timer_get_time();

        // Captura hora atual do sistema.
        struct timeval tv;
        gettimeofday(&tv, NULL);

        struct tm timeinfo = {0};
        localtime_r(&tv.tv_sec, &timeinfo);

        char timestamp[40];

        strftime(timestamp, sizeof(timestamp),
                 "%Y-%m-%d %H:%M:%S", &timeinfo);

        // Lê o estado da última sincronização SNTP.
        int64_t sync_us;
        uint32_t generation;

        portENTER_CRITICAL(&time_mux);
        sync_us = last_sntp_sync_us;
        generation = sntp_sync_generation;
        portEXIT_CRITICAL(&time_mux);

        bool synced = (generation > 0);

        if (generation != last_generation) {
            last_generation = generation;
            cycles_since_sync = 0;
        } else if (synced) {
            cycles_since_sync++;
        }

        // Milissegundos da hora civil.
        int milliseconds = (int)(tv.tv_usec / 1000);

        // Medição do tempo de execução C.
        int64_t t_end = esp_timer_get_time();
        int64_t C_us = t_end - t_start;

        time_total++;

        if (C_us > time_c_max_us) {
            time_c_max_us = C_us;
        }

        if (C_us > TIME_DEADLINE_US) {
            time_deadline_misses++;

            ESP_LOGW(TAG_TIME,
                     "Deadline excedido: C=%" PRId64 " us",
                     C_us);
        }

        // Tempo monotônico desde a última sincronização.
        int64_t sync_age_us = -1;

        if (synced) {
            sync_age_us = t_end - sync_us;
        }

        ESP_LOGI(TAG_TIME,
                 "hora=%s.%03d synced=%s "
                 "ciclos_desde_sync=%" PRIu32 " "
                 "sync_age_ms=%" PRId64 " "
                 "C_us=%" PRId64,
                 timestamp,
                 milliseconds,
                 synced ? "sim" : "nao",
                 cycles_since_sync,
                 sync_age_us >= 0 ? sync_age_us / 1000 : -1,
                 C_us);

        vTaskDelayUntil(&next, period);
    }
}

static void monitor_process_command(monitor_cmd_t cmd)
{
  switch (cmd) {
  case CMD_SORT: {
    sort_evt_t ev = {
      .t_evt_us = esp_timer_get_time()
    };

    if (xQueueSend(qSort, &ev, 0) != pdTRUE) {
      ESP_LOGW(TAG_WIFI, "MONITOR: fila SORT cheia");
    }
    break;
  }
  case CMD_ESTOP:
    // Mesmo mecanismo de sinalização usado pela tarefa de segurança.
    xSemaphoreGive(semEStop);
    break;
  case CMD_STATUS:
    // O estado será enviado na próxima mensagem periódica.
    break;
  case CMD_PING:
    // O tratamento da resposta PING/RTT fica na tarefa de rede.
    break;
  }
}

static void monitor_task(void *arg)
{
  TickType_t next = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(MONITOR_PERIOD_MS);

  monitor_total = 0;
  monitor_misses = 0;
  monitor_c_max_us = 0;
  monitor_r_max_us = 0;

  for (;;) {
    // Início da ativação: usado para medir resposta e execução.
    const int64_t t_release = esp_timer_get_time();
    const int64_t t_start = esp_timer_get_time();

    // 1. Processa os comandos pendentes sem bloquear.
    monitor_cmd_t cmd;

    while (xQueueReceive(qMonitorCmd, &cmd, 0) == pdTRUE) {
      monitor_process_command(cmd);
    }

    // 2. Coleta o estado e prepara a telemetria.
    monitor_msg_t msg;

    struct timeval tv;
    gettimeofday(&tv, NULL);

    struct tm timeinfo;
    localtime_r(&tv.tv_sec, &timeinfo);

    char timestamp[40];

    strftime(timestamp, sizeof(timestamp), "%Y-%m-%dT%H:%M:%S", &timeinfo);

    int milliseconds = (int)(tv.tv_usec / 1000);

    int64_t unix_ms = (int64_t)tv.tv_sec * 1000 + tv.tv_usec / 1000;

    static uint32_t seq = 0;

    int n = snprintf(
      msg.payload,
      sizeof(msg.payload),
      "{\"type\":\"telemetry\","
      "\"seq\":%" PRIu32 ","
      "\"protocol\":\"%s\","
      "\"timestamp\":\"%s.%03d\","
      "\"unix_ms\":%" PRId64 ","
      "\"mono_us\":%" PRId64 ","
      "\"rpm\":%.2f,"
      "\"pos_mm\":%.2f,"
      "\"set_rpm\":%.2f,"
      "\"enc_miss\":%d,"
      "\"spd_miss\":%d,"
      "\"sort_miss\":%d,"
      "\"safety_miss\":%d}",
      seq++,
      MONITOR_PROTOCOL_NAME,
      timestamp,
      milliseconds,
      unix_ms,
      t_start,
      (double)g_belt.rpm,
      (double)g_belt.pos_mm,
      (double)g_belt.set_rpm,
      misses.misses_enc,
      misses.misses_spd,
      misses.misses_sort,
      misses.misses_stop
    );

    // 3. Enfileira a mensagem sem esperar espaço na fila.
    if (n > 0 && n < sizeof(msg.payload)) {
      if (xQueueSend(qMonitorTx, &msg, 0) != pdTRUE) {
        ESP_LOGW(TAG_WIFI, "MONITOR: fila de telemetria cheia");
      }
    }

    // 4. Mede C e verifica o deadline.
    const int64_t t_end = esp_timer_get_time();
    const int64_t C_us = t_end - t_start;
    const int64_t R_us = t_end - t_release;

    monitor_total++;

    if (C_us > monitor_c_max_us) {
      monitor_c_max_us = C_us;
    }

    if (R_us > monitor_r_max_us) {
      monitor_r_max_us = R_us;
    }

    if (R_us > DEADLINE_MONITOR_US) {
      monitor_misses++;
      ESP_LOGW(TAG_WIFI, "MONITOR DEADLINE: R=%lld us, C=%lld us", (long long)R_us, (long long)C_us);
    }

    // 5. Espera até a próxima ativação periódica.
    vTaskDelayUntil(&next, period);
  }
}

static void udp_task(void *arg){
  int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);

  if (sock < 0) {
    ESP_LOGE(TAG_WIFI, "Falha ao criar socket UDP");
    vTaskDelete(NULL);
    return;
  }

  struct sockaddr_in local = {0};
  local.sin_family = AF_INET;
  local.sin_port = htons(PC_PORT);
  local.sin_addr.s_addr = htonl(INADDR_ANY);

  if (bind(sock, (struct sockaddr *)&local, sizeof(local)) < 0) {
    ESP_LOGE(TAG_WIFI, "Falha no bind UDP");
    close(sock);
    vTaskDelete(NULL);
    return;
  }

  struct timeval timeout = {
    .tv_sec = 0,
    .tv_usec = 20000
  };
  setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

  struct sockaddr_in pc = {0};
  pc.sin_family = AF_INET;
  pc.sin_port = htons(PC_PORT);
  inet_pton(AF_INET, PC_IP, &pc.sin_addr.s_addr);

  char rx[256];

  for (;;) {
    // Envia a telemetria pendente sem bloquear esperando a fila.
    monitor_msg_t msg;
    if (xQueueReceive(qMonitorTx, &msg, 0) == pdTRUE) {
      sendto(sock, msg.payload, strlen(msg.payload), 0, (struct sockaddr *)&pc, sizeof(pc));
    }

    // Recebe comandos. O timeout limita o tempo bloqueado.
    struct sockaddr_in source;
    socklen_t source_len = sizeof(source);

    int len = recvfrom(sock, rx, sizeof(rx) - 1, 0, (struct sockaddr *)&source, &source_len);

    if (len > 0) {
      rx[len] = '\0';

      monitor_cmd_t cmd;
      bool valid = true;

      if (strcmp(rx, "SORT") == 0) {
        cmd = CMD_SORT;
      }else if (strcmp(rx, "ESTOP") == 0) {
        cmd = CMD_ESTOP;
      }else if (strcmp(rx, "STATUS") == 0) {
        cmd = CMD_STATUS;
      }else if (strncmp(rx, "PING", 4) == 0) {
        // PING é tratado separadamente para preservar o ID.
        int64_t now = esp_timer_get_time();
        char reply[128];
        snprintf(reply, sizeof(reply), "{\"type\":\"PONG\",\"ts_us\":%lld,\"echo\":\"%.32s\"}", (long long)now, rx);

        sendto(sock, reply, strlen(reply), 0, (struct sockaddr *)&source, source_len);
        valid = false;
      }else {
        valid = false;
        ESP_LOGW(TAG_WIFI, "Comando UDP desconhecido: %s", rx);
      }

      if (valid && xQueueSend(qMonitorCmd, &cmd, 0) != pdTRUE) {
        ESP_LOGW(TAG_WIFI, "Fila de comandos cheia");
      }
    }
  }
}

//Configura WiFi da ESP32
static void wifi_init(void) {
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_sta();
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  wifi_config_t wifi_config = {
    .sta = {
      .threshold.authmode = WIFI_AUTH_WPA2_PSK,
    },
  };
  strcpy((char*)wifi_config.sta.ssid, WIFI_SSID);
  strcpy((char*)wifi_config.sta.password, WIFI_PASS);
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());
  ESP_LOGI(TAG_WIFI, "Conectando ao WiFi...");
  ESP_ERROR_CHECK(esp_wifi_connect());
}

static void tcp_task(void *arg){
  int listen_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);

  if (listen_fd < 0) {
    ESP_LOGE(TAG_WIFI, "Falha ao criar socket TCP");
    vTaskDelete(NULL);
    return;
  }

  int reuse = 1;
  setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

  struct sockaddr_in addr = {0};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(TCP_PORT);
  addr.sin_addr.s_addr = htonl(INADDR_ANY);

  if (bind(listen_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0 ||
    listen(listen_fd, 1) < 0) {
    ESP_LOGE(TAG_WIFI, "Falha ao iniciar servidor TCP");
    close(listen_fd);
    vTaskDelete(NULL);
    return;
  }

  ESP_LOGI(TAG_WIFI, "Servidor TCP ativo na porta %d", TCP_PORT);

  for (;;) {
    struct sockaddr_in source;
    socklen_t source_len = sizeof(source);
    int client = accept(listen_fd, (struct sockaddr *)&source, &source_len);

    if (client < 0) {
      continue;
    }

    struct timeval timeout = {
      .tv_sec = 0,
      .tv_usec = 20000
    };

    setsockopt(client, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    ESP_LOGI(TAG_WIFI, "Cliente TCP conectado");

    const char *hello = "{\"type\":\"connected\"}\n";
    send(client, hello, strlen(hello), 0);
    char rx[256];
    for (;;) {
      // Transmite mensagens pendentes.
      monitor_msg_t msg;

      if (xQueueReceive(qMonitorTx, &msg, 0) == pdTRUE) {
        char line[300];

        int n = snprintf(line, sizeof(line), "%s\n", msg.payload);
        if (n <= 0 || n >= sizeof(line) || send(client, line, n, 0) < 0) {
          break;
        }
      }

      // Recebe comandos sem espera indefinida.
      int len = recv(client, rx, sizeof(rx) - 1, 0);
      if (len > 0) {
        rx[len] = '\0';

        // Este exemplo pressupõe um comando por recv().
        // Para uso robusto, implemente enquadramento por '\n'
        // e acumule dados recebidos em um buffer.
        char *newline = strpbrk(rx, "\r\n");
        if (newline != NULL) {
          *newline = '\0';
        }

        monitor_cmd_t cmd;
        bool valid = true;

        if (strcmp(rx, "SORT") == 0) {
          cmd = CMD_SORT;
        }else if (strcmp(rx, "ESTOP") == 0) {
          cmd = CMD_ESTOP;
        }else if (strcmp(rx, "STATUS") == 0) {
          cmd = CMD_STATUS;
        }else if (strncmp(rx, "PING", 4) == 0) {
          char reply[128];

          snprintf(reply, sizeof(reply), "{\"type\":\"PONG\",\"echo\":\"%.32s\"}\n", rx);

          if (send(client, reply, strlen(reply), 0) < 0) {
            break;
          }

          valid = false;
        }else {
          valid = false;
          ESP_LOGW(TAG_WIFI, "Comando TCP desconhecido: %s", rx);
        }

        if (valid && xQueueSend(qMonitorCmd, &cmd, 0) != pdTRUE) {
          ESP_LOGW(TAG_WIFI, "Fila de comandos cheia");
        }
      }else if (len == 0) {
        // O PC encerrou a conexão.
        break;
      }else if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
        // Erro real de recepção.
        break;
      }

      vTaskDelay(pdMS_TO_TICKS(1));
    }

    shutdown(client, SHUT_RDWR);
    close(client);
    ESP_LOGI(TAG_WIFI, "Cliente TCP desconectado");
  }
}

// ====== Util: busy loop previsível (~WCET) ======
static inline void cpu_tight_loop_us(uint32_t us)
{
  int64_t start = esp_timer_get_time();
  while ((esp_timer_get_time() - start) < us) {
    __asm__ __volatile__("nop");
  }
}

// ====== ENC_SENSE (periódica 5 ms): estima velocidade/posição ======
static void task_enc_sense(void *arg)
{
  TickType_t next = xTaskGetTickCount();
  const TickType_t T = pdMS_TO_TICKS(ENC_T_MS);

  misses.misses_enc = 0;
  misses.total_enc = 1;
  int ciclos = 0;

  for (;;) {
    misses.total_enc++;
    int64_t t0 = esp_timer_get_time();
    // Dinâmica simulada: aproxima rpm do setpoint, integra posição
    float err = g_belt.set_rpm - g_belt.rpm;
    g_belt.rpm += 0.05f * err;             // aproximação lenta para simular inércia
    g_belt.pos_mm += (g_belt.rpm / 60.0f) * (ENC_T_MS / 1000.0f) * 100.0f; // 100 mm por rev (exemplo)

    // Carga determinística ~0.6 ms
    cpu_tight_loop_us(600);

    // Notifica controle
    if (hCtrlNotify){
      xTaskNotifyGive(hCtrlNotify);
    }

    ciclos++;
    if(ciclos > 2000){
      ESP_LOGI(TAG, "Enc_MISS=%d Spd_MISS=%d Sort_MISS=%d Stop_MISS=%d", misses.misses_enc, misses.misses_spd, misses.misses_sort, misses.misses_stop);
      ESP_LOGI(TAG, "Enc_TOTAL=%d Spd_TOTAL=%d Sort_TOTAL=%d Stop_TOTAL=%d", misses.total_enc, misses.total_spd, misses.total_sort, misses.total_stop);
      ESP_LOGI(TAG, "Enc_PCT=%.2f Spd_PCT=%.2f Sort_PCT=%.2f Stop_PCT=%.2f",
          ((float)misses.misses_enc / misses.total_enc) * 100.0f,
          ((float)misses.misses_spd / misses.total_spd) * 100.0f,
          ((float)misses.misses_sort / misses.total_sort) * 100.0f,
          ((float)misses.misses_stop / misses.total_stop) * 100.0f
      );
      ciclos = 0;
    }

    int64_t total_enc = esp_timer_get_time() - t0;

    if(total_enc > DEADLINE_ENC_US){
      ESP_LOGI(TAG, "ENC_SENSE DEADLINE! total=%lld microsegundos (us)", total_enc);
      misses.misses_enc++;
    }else{
      //ESP_LOGI(TAG, "total=%lld us", total_enc);
    }

    vTaskDelayUntil(&next, T);
  }
}

// ====== SPD_CTRL (encadeada): controle PI simulado + HMI (soft) ======
static void task_spd_ctrl(void *arg)
{
  // um controlador PI minimalista e simulado
  float kp = 0.4f, ki = 0.1f, integ = 0.f;

  misses.misses_spd = 0;
  misses.total_spd = 1;

  for (;;) {

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // acorda após ENC_SENSE
    misses.total_spd++;

    int64_t t0 = esp_timer_get_time();

    // Controle (hard): alvo é g_belt.set_rpm, atuando em g_belt.rpm (simulado)
    float err = g_belt.set_rpm - g_belt.rpm;
    integ += err * (ENC_T_MS / 1000.0f);
    float u = kp * err + ki * integ;
    // "Aplicar" u: aqui apenas ajustamos o setpoint ligeiramente (mock)
    g_belt.set_rpm += 0.1f * u;

    // Carga determinística ~1.2 ms
    cpu_tight_loop_us(1200);

    // Trecho não-crítico (soft): se HMI solicitada, imprime e retorna rápido
    if (xSemaphoreTake(semHMI, 0) == pdTRUE) {
      ESP_LOGI(TAG, "HMI: rpm=%.1f set=%.1f pos=%.1fmm",g_belt.rpm, g_belt.set_rpm, g_belt.pos_mm);
      cpu_tight_loop_us(400); // ~0.4 ms (soft)
    }

    int64_t total_spd = esp_timer_get_time() - t0;

    if(total_spd > DEADLINE_CTRL_US){
      ESP_LOGI(TAG, "HMI/Telemetria DEADLINE! total=%lld microsegundos (us)", total_spd);
      misses.misses_spd++;
    }else{
      //ESP_LOGI(TAG, "SPD CTRL total=%lld", total_spd);
    }
  }
}

// ====== SORT_ACT (evento Touch B): aciona "desviador" no tempo certo ======
static void task_sort_act(void *arg)
{
  sort_evt_t ev;
  misses.misses_sort = 0;
  misses.total_sort = 1;
  for (;;) {
    if (xQueueReceive(qSort, &ev, portMAX_DELAY) == pdTRUE) {
      misses.total_sort++;
      int64_t t0 = esp_timer_get_time();
      int64_t tempo_touch = t0 - ev.t_evt_us; //é o tempo que levou para tocar e entrar na função

      // Preparação de janela crítica (ex.: 0.7 ms)
      cpu_tight_loop_us(700);

      // Aqui você acionaria GPIO/solenóide no "instante"
      // Nesta demo, apenas logamos o tempo gasto:
      int64_t dt = esp_timer_get_time() - t0;

      int64_t total_b = dt + tempo_touch;

      ESP_LOGI(TAG, "SORT_ACT: desviador acionado, exec=%lld microsegundos (us), toque=%lld microssegundos(us), total=%lld microssegundos (us)", dt, tempo_touch, total_b);

      if(total_b > DEADLINE_SORT_US){
        ESP_LOGI(TAG, "Sort_Act DEADLINE! total=%lld microssegundos (us)", total_b);
        misses.misses_sort++;
      }
    }
  }
}

// ====== SAFETY_TASK (evento Touch D): E-stop ======
static void task_safety(void *arg)
{
  misses.misses_stop = 0;
  misses.total_stop = 1;

  for (;;) {
    if (xSemaphoreTake(semEStop, portMAX_DELAY) == pdTRUE) {
      misses.total_stop++;
      int64_t t0 = esp_timer_get_time();
      int64_t tempo_touch_stop = t0 - tempo_touch_d;

      // Ação crítica: zera "PWM", trava atuadores, sinaliza alarme (~0.9 ms)
      g_belt.set_rpm = 0.f;
      cpu_tight_loop_us(900);

      int64_t dt = esp_timer_get_time() - t0;

      int64_t total_d = tempo_touch_stop + dt;

      ESP_LOGI(TAG, "ESTOP: parada de emergência, exec=%lld microsegundos (us), toque=%lld microssegundos (us), total=%lld microssegundos (us)", dt, tempo_touch_stop, total_d);

      if(total_d > DEADLINE_SAFETY_US){
        ESP_LOGI(TAG, "ESTOP DEADLINE! total=%lld microssegundos (us)", total_d);
        misses.misses_stop++;
      }
    }
  }
}

//callback dos touch
static bool IRAM_ATTR touch_hmi_callback(touch_sensor_handle_t handle, const touch_active_event_data_t *event, void *user_data){
  BaseType_t hp = pdFALSE;

  switch(event->chan_id){
    case 7:{
      sort_evt_t ev7 = {.t_evt_us = esp_timer_get_time()}; //passa o tempo em q foi tocado
      xQueueSendFromISR(qSort, &ev7, &hp);
      break;
    }
    case 8: {
      xSemaphoreGiveFromISR(semHMI, &hp);
      break;
    }
    case 9:{
      tempo_touch_d = esp_timer_get_time(); //guarda o tempo na variavel global criada
      xSemaphoreGiveFromISR(semEStop, &hp);
      break;
    }
    default:{break;}
  }

  return (hp == pdTRUE);
}

static bool IRAM_ATTR touch_inactive_cb(touch_sensor_handle_t h, const touch_inactive_event_data_t *ev, void *ctx)
{
  return false;
}

// ====== app_main ======
void app_main(void)
{
  ESP_ERROR_CHECK(nvs_flash_init());
  wifi_init();

  // Filas da MONITOR_TASK e da tarefa de rede
  qMonitorTx = xQueueCreate(10, sizeof(monitor_msg_t));
  qMonitorCmd = xQueueCreate(10, sizeof(monitor_cmd_t));

  if (qMonitorTx == NULL || qMonitorCmd == NULL) {
    ESP_LOGE(TAG, "Falha ao criar filas do monitoramento");
    abort();
  }

  // criar os mutex e a fila
  semEStop = xSemaphoreCreateBinary();
  semHMI   = xSemaphoreCreateBinary();

  qSort = xQueueCreate(10, sizeof(sort_evt_t));

  touch_sensor_handle_t sens_handle = NULL;
  // sample configuration
  touch_sensor_sample_config_t sample_cfg[1] = {
    TOUCH_SENSOR_V1_DEFAULT_SAMPLE_CONFIG(5.0f, TOUCH_VOLT_LIM_L_0V5, TOUCH_VOLT_LIM_H_1V7),
  };
  touch_sensor_config_t touch_cfg = TOUCH_SENSOR_DEFAULT_BASIC_CONFIG(1, sample_cfg);

  //canal do touch HMI/Telemetria
  touch_channel_handle_t ch_b = NULL;
  touch_channel_handle_t ch_c = NULL;
  touch_channel_handle_t ch_d = NULL;

  touch_event_callbacks_t cbs = {
    .on_active = touch_hmi_callback,
    .on_inactive = touch_inactive_cb,
  };
  touch_channel_config_t chan_cfg = {
    .abs_active_thresh = {1000},
    .charge_speed = TOUCH_CHARGE_SPEED_7,
    .init_charge_volt = TOUCH_INIT_CHARGE_VOLT_DEFAULT,
    .group = TOUCH_CHAN_TRIG_GROUP_BOTH
  };
  ESP_ERROR_CHECK(touch_sensor_new_controller(&touch_cfg, &sens_handle));
  // Configura o filtro (obrigatório antes de registrar callbacks)
  touch_sensor_filter_config_t filter_cfg = TOUCH_SENSOR_DEFAULT_FILTER_CONFIG();
  ESP_ERROR_CHECK(touch_sensor_config_filter(sens_handle, &filter_cfg));

  ESP_ERROR_CHECK(touch_sensor_new_channel(sens_handle, 7, &chan_cfg, &ch_b));
  ESP_ERROR_CHECK(touch_sensor_new_channel(sens_handle, 8, &chan_cfg, &ch_c));
  ESP_ERROR_CHECK(touch_sensor_new_channel(sens_handle, 9, &chan_cfg, &ch_d));

  ESP_ERROR_CHECK(touch_sensor_register_callbacks(sens_handle, &cbs, NULL));
  ESP_ERROR_CHECK(touch_sensor_enable(sens_handle));
  ESP_ERROR_CHECK(touch_sensor_start_continuous_scanning(sens_handle));

  // Cria tasks (afinidade no core 0 para reduzir jitter, opcional)
  xTaskCreatePinnedToCore(task_safety,   "SAFETY",   STK, NULL, PRIO_ESTOP, &hSAFE, 0);
  xTaskCreatePinnedToCore(task_enc_sense,"ENC_SENSE",STK, NULL, PRIO_ENC,   &hENC,  0);
  xTaskCreatePinnedToCore(task_spd_ctrl, "SPD_CTRL", STK, NULL, PRIO_CTRL,  &hCTRL, 0);
  xTaskCreatePinnedToCore(task_sort_act, "SORT_ACT", STK, NULL, PRIO_SORT,  &hSORT, 0);

  //novas tasks:
  // Criar MONITOR_TASK periódica
  BaseType_t ok = xTaskCreate(
    monitor_task,
    "MONITOR_TASK",
    4096,
    NULL,
    PRIO_MON,
    &hMonitor
  );

  if (ok != pdPASS) {
    ESP_LOGE(TAG, "Falha ao criar MONITOR_TASK");
    abort();
  }

  // Criar a tarefa de comunicação escolhida
  #if MONITOR_USE_TCP
    ok = xTaskCreate(tcp_task, "NET_TCP", 4096, NULL, 2, &hNet);
  #else
    ok = xTaskCreate(udp_task, "NET_UDP", 4096, NULL, 2, &hNet);
  #endif

  if (ok != pdPASS) {
    ESP_LOGE(TAG, "Falha ao criar tarefa de rede");
    abort();
  }

  time_sntp_init();

  BaseType_t tempo = xTaskCreate(
    time_task,
    "TIME_TASK",
    TIME_STACK,
    NULL,
    TIME_PRIORITY,
    NULL
  );

  if (ok != pdPASS) {
    ESP_LOGE(TAG_TIME, "Falha ao criar TIME_TASK");
  }

  hCtrlNotify = hCTRL;
}
