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

#define TAG "ESTEIRA"

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
#define STK             3072

// ====== Deadlines (do enunciado) ======
#define DEADLINE_ENC_US      5500    // 5 ms
#define DEADLINE_CTRL_US    10000    // 10 ms
#define DEADLINE_SORT_US    10000    // 10 ms
#define DEADLINE_SAFETY_US   5000    // 5 ms

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

  hCtrlNotify = hCTRL;
}