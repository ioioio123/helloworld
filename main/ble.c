#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"

#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

#include "board_config.h"
#include "app_state.h"
#include "weld.h"
#include "ble.h"

static const char *TAG = "ble";

#define DEVICE_NAME "WELDER-C3"

/* ---------------- UUID ---------------- */
/* 服务 1a2b3c4d-0001-4a5b-8c9d-0e1f2a3b4c5d
 * 遥测 ...-0002-...   包络 ...-0003-...   控制 ...-0004-... */
static const ble_uuid128_t s_svc_uuid = BLE_UUID128_INIT(
    0x5d, 0x4c, 0x3b, 0x2a, 0x1f, 0x0e, 0x9d, 0x8c,
    0x5b, 0x4a, 0x01, 0x00, 0x4d, 0x3c, 0x2b, 0x1a);

static const ble_uuid128_t s_tele_uuid = BLE_UUID128_INIT(
    0x5d, 0x4c, 0x3b, 0x2a, 0x1f, 0x0e, 0x9d, 0x8c,
    0x5b, 0x4a, 0x02, 0x00, 0x4d, 0x3c, 0x2b, 0x1a);

static const ble_uuid128_t s_env_uuid = BLE_UUID128_INIT(
    0x5d, 0x4c, 0x3b, 0x2a, 0x1f, 0x0e, 0x9d, 0x8c,
    0x5b, 0x4a, 0x03, 0x00, 0x4d, 0x3c, 0x2b, 0x1a);

static const ble_uuid128_t s_ctl_uuid = BLE_UUID128_INIT(
    0x5d, 0x4c, 0x3b, 0x2a, 0x1f, 0x0e, 0x9d, 0x8c,
    0x5b, 0x4a, 0x04, 0x00, 0x4d, 0x3c, 0x2b, 0x1a);

/* ---------------- 状态 ---------------- */
static uint16_t s_conn    = BLE_HS_CONN_HANDLE_NONE;
static uint8_t  s_addr_type;
static uint16_t s_tele_h;
static uint16_t s_env_h;
static uint16_t s_ctl_h;

static char    s_tele_buf[80];
static uint8_t s_env_buf[ENV_BUCKETS];
static uint32_t s_last_env_seq;

/* ---------------- 工具 ---------------- */

static void refresh_telemetry(void)
{
    app_state_t *s = app_state_lock();
    uint16_t peak   = s->env_peak;
    float    temp   = s->temp_c;
    uint32_t pulse  = s->pulse_us;
    uint32_t count  = s->weld_count;
    bool     locked = s->temp_locked;
    bool     weld   = s->welding;
    app_state_unlock();

    int status = locked ? 2 : (weld ? 1 : 0);
    snprintf(s_tele_buf, sizeof(s_tele_buf), "W,%u,%d,%lu,%lu,%d\n",
             (unsigned)peak,
             (int)(temp * 10.0f),
             (unsigned long)pulse,
             (unsigned long)count,
             status);
}

static void notify_chr(uint16_t handle, const void *data, uint16_t len)
{
    if (s_conn == BLE_HS_CONN_HANDLE_NONE || handle == 0) {
        return;
    }
    struct os_mbuf *om = ble_hs_mbuf_from_flat(data, len);
    if (om == NULL) {
        return;
    }
    /* 没订阅时返回错误，属正常情况，忽略即可 */
    ble_gatts_notify_custom(s_conn, handle, om);
}

void ble_notify_envelope(const uint16_t *env, int buckets)
{
    if (buckets > ENV_BUCKETS) {
        buckets = ENV_BUCKETS;
    }
    for (int i = 0; i < buckets; i++) {
        /* 12bit ADC -> 8bit 显示，保留相对形状 */
        s_env_buf[i] = (uint8_t)((uint32_t)env[i] * 255u / 4095u);
    }
    notify_chr(s_env_h, s_env_buf, (uint16_t)buckets);
}

/* ---------------- 控制命令 ---------------- */

static void handle_command(const char *cmd)
{
    if (cmd == NULL || cmd[0] == '\0') {
        return;
    }
    switch (cmd[0]) {
    case 'F':
    case 'f':
        /* 与脚踏完全相同的入口，冷却/超温/看门狗一律生效 */
        weld_request_trigger();
        ESP_LOGI(TAG, "远程触发请求");
        break;

    case 'P':
    case 'p': {
        uint32_t req = (uint32_t)strtoul(cmd + 1, NULL, 10);
        weld_set_pulse_us(req);
        ESP_LOGI(TAG, "远程设脉宽 %lu us -> 实际 %lu us",
                 (unsigned long)req, (unsigned long)weld_get_pulse_us());
        refresh_telemetry();
        break;
    }

    case '?':
        refresh_telemetry();
        ESP_LOGI(TAG, "遥测: %s", s_tele_buf);
        break;

    default:
        ESP_LOGW(TAG, "未知命令: %s", cmd);
        break;
    }
}

/* ---------------- GATT ---------------- */

static int gatt_access(uint16_t conn_handle, uint16_t attr_handle,
                       struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        if (attr_handle != s_ctl_h) {
            return BLE_ATT_ERR_UNLIKELY;
        }
        char buf[33];
        uint16_t len = 0;
        if (ble_hs_mbuf_to_flat(ctxt->om, buf, sizeof(buf) - 1, &len) != 0) {
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }
        buf[len] = '\0';
        /* 去掉尾部换行 */
        while (len > 0 && (buf[len - 1] == '\n' || buf[len - 1] == '\r')) {
            buf[--len] = '\0';
        }
        handle_command(buf);
        return 0;
    }

    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        if (attr_handle == s_tele_h) {
            return (os_mbuf_append(ctxt->om, s_tele_buf, strlen(s_tele_buf)) == 0)
                   ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        if (attr_handle == s_env_h) {
            return (os_mbuf_append(ctxt->om, s_env_buf, sizeof(s_env_buf)) == 0)
                   ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
    }

    return BLE_ATT_ERR_UNLIKELY;
}

static const struct ble_gatt_svc_def s_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &s_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid       = &s_tele_uuid.u,
                .access_cb  = gatt_access,
                .flags      = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &s_tele_h,
            },
            {
                .uuid       = &s_env_uuid.u,
                .access_cb  = gatt_access,
                .flags      = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &s_env_h,
            },
            {
                .uuid       = &s_ctl_uuid.u,
                .access_cb  = gatt_access,
                .flags      = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
                .val_handle = &s_ctl_h,
            },
            { 0 },
        },
    },
    { 0 },
};

/* ---------------- GAP ---------------- */

static void ble_advertise(void);

static int gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_conn = event->connect.conn_handle;
            ESP_LOGI(TAG, "手机已连接, conn=%d", s_conn);
        } else {
            ESP_LOGW(TAG, "连接失败 status=%d", event->connect.status);
            ble_advertise();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "连接断开 reason=0x%x", event->disconnect.reason);
        s_conn = BLE_HS_CONN_HANDLE_NONE;
        ble_advertise();
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        ble_advertise();
        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
        ESP_LOGI(TAG, "订阅 attr=%d notify=%d", event->subscribe.attr_handle,
                 event->subscribe.cur_notify);
        return 0;

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "MTU 协商 = %d", event->mtu.value);
        return 0;

    default:
        return 0;
    }
}

static void ble_advertise(void)
{
    struct ble_hs_adv_fields fields;
    struct ble_gap_adv_params params;
    const char *name;
    int rc;

    memset(&fields, 0, sizeof(fields));
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    name = ble_svc_gap_device_name();
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "设置广播数据失败 rc=%d", rc);
        return;
    }

    memset(&params, 0, sizeof(params));
    params.conn_mode = BLE_GAP_CONN_MODE_UND;
    params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(s_addr_type, NULL, BLE_HS_FOREVER, &params, gap_event, NULL);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGE(TAG, "启动广播失败 rc=%d", rc);
    } else {
        ESP_LOGI(TAG, "广播中，设备名 %s", DEVICE_NAME);
    }
}

static void on_sync(void)
{
    ble_hs_util_ensure_addr(0);
    ble_hs_id_infer_auto(0, &s_addr_type);
    ble_advertise();
}

static void on_reset(int reason)
{
    ESP_LOGW(TAG, "BLE 协议栈复位 reason=%d", reason);
}

static void host_task(void *param)
{
    ESP_LOGI(TAG, "BLE host 任务启动");
    nimble_port_run();              /* 只在 nimble_port_stop() 后返回 */
    nimble_port_freertos_deinit();
}

/* ---------------- 遥测任务 ---------------- */

static void tele_task(void *arg)
{
    for (;;) {
        refresh_telemetry();
        notify_chr(s_tele_h, s_tele_buf, (uint16_t)strlen(s_tele_buf));

        /* 包络有更新就整条推一次。先在锁内拷一份，再发通知，
         * 避免持锁期间做 BLE 内存分配。 */
        static uint16_t env_copy[ENV_BUCKETS];
        uint32_t seq;
        bool need_push = false;

        app_state_t *s = app_state_lock();
        seq = s->env_seq;
        if (seq != s_last_env_seq) {
            memcpy(env_copy, s->env, sizeof(env_copy));
            need_push = true;
        }
        app_state_unlock();

        if (need_push) {
            s_last_env_seq = seq;
            ble_notify_envelope(env_copy, ENV_BUCKETS);
        }

        vTaskDelay(pdMS_TO_TICKS(100));     /* 10 Hz */
    }
}

/* ---------------- 初始化 ---------------- */

void ble_init(void)
{
    int rc = nimble_port_init();
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init 失败 rc=%d", rc);
        return;
    }

    ble_hs_cfg.reset_cb          = on_reset;
    ble_hs_cfg.sync_cb           = on_sync;
    ble_hs_cfg.store_status_cb   = ble_store_util_status_rr;
    /* 不做配对绑定：焊接机是本地设备，加了配对反而挡住快速连接。
     * 若需要，把 sm_io_cap / sm_bonding 打开即可。 */
    ble_hs_cfg.sm_io_cap         = BLE_HS_IO_NO_INPUT_OUTPUT;
    ble_hs_cfg.sm_bonding        = 0;
    ble_hs_cfg.sm_mitm           = 0;
    ble_hs_cfg.sm_sc             = 0;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    rc = ble_gatts_count_cfg(s_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_count_cfg 失败 rc=%d", rc);
        return;
    }
    rc = ble_gatts_add_svcs(s_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_add_svcs 失败 rc=%d", rc);
        return;
    }

    rc = ble_svc_gap_device_name_set(DEVICE_NAME);
    if (rc != 0) {
        ESP_LOGE(TAG, "设置设备名失败 rc=%d", rc);
    }

    /* 不做配对绑定，因此不需要持久化存储（v6.1 也不再提供 ble_store_config_init） */
    nimble_port_freertos_init(host_task);

    xTaskCreate(tele_task, "ble_tele", 3072, NULL, 4, NULL);

    ESP_LOGI(TAG, "BLE 初始化完成，设备名 %s", DEVICE_NAME);
}
