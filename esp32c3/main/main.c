/*
 * ESP32-C3 BLE Mesh 节点：Generic OnOff Server + Vendor Server。
 *
 * 硬件定义：
 *   GPIO4：LED 输出
 *   GPIO5：蜂鸣器输出
 *
 * Vendor 指令 Payload：
 *   Byte0 type：0x01 LED，0x02 Buzzer
 *   Byte1 mode：0x00 关闭，0x01 常亮/常响，0x02 5Hz 闪烁/鸣响
 *
 * 本节点开启 PB-ADV/PB-GATT，nRF Mesh 可扫描到未配网设备并完成配网。
 */

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "nvs_flash.h"

#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_generic_model_api.h"
#include "esp_ble_mesh_local_data_operation_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_provisioning_api.h"

#define TAG "MESH_NODE_C3"

#define CID_ESPRESSIF          0x02E5
#define VENDOR_MODEL_ID_SERVER 0x0001
#define VENDOR_MODEL_OP_SET    ESP_BLE_MESH_MODEL_OP_3(0x01, CID_ESPRESSIF)
#define VENDOR_MODEL_OP_STATUS ESP_BLE_MESH_MODEL_OP_3(0x02, CID_ESPRESSIF)

#define DEVICE_TYPE_LED        0x01
#define DEVICE_TYPE_BUZZER     0x02

#define DEVICE_MODE_OFF        0x00
#define DEVICE_MODE_ON         0x01
#define DEVICE_MODE_BLINK      0x02

#define LED_GPIO               GPIO_NUM_4
#define BUZZER_GPIO            GPIO_NUM_5
#define BUZZER_ACTIVE_LOW      1
#define BLINK_TOGGLE_MS        100
#define DEFAULT_TTL            7
#define PROV_START_DELAY_MS    1200
#define PROV_RETRY_DELAY_MS    2000
#define PROV_REQUEST_TIMEOUT_MS 5000

typedef struct {
    uint8_t type;
    gpio_num_t gpio;
    TimerHandle_t timer;
    uint8_t mode;
    uint8_t level;
} device_state_t;

static uint8_t dev_uuid[16] = {0xdd, 0xdd};
static uint16_t primary_addr = ESP_BLE_MESH_ADDR_UNASSIGNED;
static TaskHandle_t prov_enable_task_handle;
static bool prov_enable_active;
static bool prov_enable_requesting;
static TickType_t prov_enable_request_tick;
static uint32_t prov_enable_attempts;
static bool vendor_publish_ready;

static void publish_device_status(uint8_t type);
static void start_prov_enable_task(void);

static device_state_t led_device = {
    .type = DEVICE_TYPE_LED,
    .gpio = LED_GPIO,
    .timer = NULL,
    .mode = DEVICE_MODE_OFF,
    .level = 0,
};

static device_state_t buzzer_device = {
    .type = DEVICE_TYPE_BUZZER,
    .gpio = BUZZER_GPIO,
    .timer = NULL,
    .mode = DEVICE_MODE_OFF,
    .level = 0,
};

static esp_ble_mesh_cfg_srv_t config_server = {
    .net_transmit = ESP_BLE_MESH_TRANSMIT(2, 20),
#if defined(CONFIG_BLE_MESH_RELAY)
    .relay = ESP_BLE_MESH_RELAY_ENABLED,
#else
    .relay = ESP_BLE_MESH_RELAY_NOT_SUPPORTED,
#endif
    .relay_retransmit = ESP_BLE_MESH_TRANSMIT(2, 20),
    .beacon = ESP_BLE_MESH_BEACON_ENABLED,
#if defined(CONFIG_BLE_MESH_GATT_PROXY_SERVER)
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_ENABLED,
#else
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_NOT_SUPPORTED,
#endif
#if defined(CONFIG_BLE_MESH_FRIEND)
    .friend_state = ESP_BLE_MESH_FRIEND_ENABLED,
#else
    .friend_state = ESP_BLE_MESH_FRIEND_NOT_SUPPORTED,
#endif
    .default_ttl = DEFAULT_TTL,
};

ESP_BLE_MESH_MODEL_PUB_DEFINE(onoff_pub, 2 + 3, ROLE_NODE);
static esp_ble_mesh_gen_onoff_srv_t onoff_server = {
    .rsp_ctrl = {
        .get_auto_rsp = ESP_BLE_MESH_SERVER_RSP_BY_APP,
        .set_auto_rsp = ESP_BLE_MESH_SERVER_RSP_BY_APP,
    },
};

ESP_BLE_MESH_MODEL_PUB_DEFINE(vendor_srv_pub, 3 + 3, ROLE_NODE);
static esp_ble_mesh_model_op_t vendor_srv_op[] = {
    ESP_BLE_MESH_MODEL_OP(VENDOR_MODEL_OP_SET, 2),
    ESP_BLE_MESH_MODEL_OP_END,
};

static esp_ble_mesh_model_t root_models[] = {
    ESP_BLE_MESH_MODEL_CFG_SRV(&config_server),
    ESP_BLE_MESH_MODEL_GEN_ONOFF_SRV(&onoff_pub, &onoff_server),
};

static esp_ble_mesh_model_t vendor_models[] = {
    ESP_BLE_MESH_VENDOR_MODEL(CID_ESPRESSIF, VENDOR_MODEL_ID_SERVER,
                              vendor_srv_op, &vendor_srv_pub, NULL),
};

static esp_ble_mesh_elem_t elements[] = {
    ESP_BLE_MESH_ELEMENT(0, root_models, vendor_models),
};

static esp_ble_mesh_comp_t composition = {
    .cid = CID_ESPRESSIF,
    .element_count = ARRAY_SIZE(elements),
    .elements = elements,
};

static esp_ble_mesh_prov_t provision = {
    .uuid = dev_uuid,
    .output_size = 0,
    .output_actions = 0,
    .input_size = 0,
    .input_actions = 0,
};

static device_state_t *get_device(uint8_t type)
{
    if (type == DEVICE_TYPE_LED) {
        return &led_device;
    }
    if (type == DEVICE_TYPE_BUZZER) {
        return &buzzer_device;
    }
    return NULL;
}

static void set_device_level(device_state_t *dev, uint8_t level)
{
    dev->level = level ? 1 : 0;

#if BUZZER_ACTIVE_LOW
    if (dev->type == DEVICE_TYPE_BUZZER) {
        gpio_set_level(dev->gpio, dev->level ? 0 : 1);
        return;
    }
#endif

    gpio_set_level(dev->gpio, dev->level);
}

static void blink_timer_cb(TimerHandle_t timer)
{
    device_state_t *dev = (device_state_t *)pvTimerGetTimerID(timer);

    if (dev == NULL || dev->mode != DEVICE_MODE_BLINK) {
        return;
    }

    set_device_level(dev, !dev->level);
}

static void hardware_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_GPIO) | (1ULL << BUZZER_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    led_device.timer = xTimerCreate("led_blink", pdMS_TO_TICKS(BLINK_TOGGLE_MS),
                                    pdTRUE, &led_device, blink_timer_cb);
    buzzer_device.timer = xTimerCreate("buzzer_blink", pdMS_TO_TICKS(BLINK_TOGGLE_MS),
                                       pdTRUE, &buzzer_device, blink_timer_cb);
    configASSERT(led_device.timer != NULL);
    configASSERT(buzzer_device.timer != NULL);

    set_device_level(&led_device, 0);
    set_device_level(&buzzer_device, 0);
}

void update_device_state(uint8_t type, uint8_t mode)
{
    device_state_t *dev = get_device(type);

    if (dev == NULL) {
        ESP_LOGW(TAG, "未知硬件类型 0x%02x", type);
        return;
    }

    if (mode > DEVICE_MODE_BLINK) {
        ESP_LOGW(TAG, "未知控制模式 0x%02x", mode);
        return;
    }

    dev->mode = mode;
    xTimerStop(dev->timer, 0);

    switch (mode) {
    case DEVICE_MODE_OFF:
        set_device_level(dev, 0);
        break;
    case DEVICE_MODE_ON:
        set_device_level(dev, 1);
        break;
    case DEVICE_MODE_BLINK:
        set_device_level(dev, 1);
        xTimerChangePeriod(dev->timer, pdMS_TO_TICKS(BLINK_TOGGLE_MS), 0);
        xTimerStart(dev->timer, 0);
        break;
    default:
        break;
    }

    ESP_LOGI(TAG, "硬件状态更新 type=0x%02x mode=0x%02x level=%u",
             type, mode, dev->level);
    publish_device_status(type);
}

static esp_err_t bluetooth_init(void)
{
    esp_err_t err;

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    err = esp_bt_controller_init(&bt_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "蓝牙控制器初始化失败: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "蓝牙控制器使能失败: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_bluedroid_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Bluedroid 初始化失败: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_bluedroid_enable();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Bluedroid 使能失败: %s", esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}

static void ble_mesh_get_dev_uuid(uint8_t *uuid)
{
    memcpy(uuid + 2, esp_bt_dev_get_address(), BD_ADDR_LEN);
}

static void prov_enable_task(void *arg)
{
    TickType_t delay_ticks = pdMS_TO_TICKS(PROV_START_DELAY_MS);

    (void)arg;

    while (!prov_enable_active && !esp_ble_mesh_node_is_provisioned()) {
        vTaskDelay(delay_ticks);

        if (prov_enable_active || esp_ble_mesh_node_is_provisioned()) {
            break;
        }

        if (prov_enable_requesting) {
            TickType_t elapsed = xTaskGetTickCount() - prov_enable_request_tick;
            if (elapsed < pdMS_TO_TICKS(PROV_REQUEST_TIMEOUT_MS)) {
                continue;
            }
            ESP_LOGW(TAG, "PB-ADV enable request timed out, retrying");
            prov_enable_requesting = false;
        }

        prov_enable_attempts++;
        ESP_LOGI(TAG, "Starting PB-ADV provisioning, attempt=%" PRIu32, prov_enable_attempts);

        esp_err_t err = esp_ble_mesh_node_prov_enable(ESP_BLE_MESH_PROV_ADV);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "esp_ble_mesh_node_prov_enable failed: %s", esp_err_to_name(err));
            delay_ticks = pdMS_TO_TICKS(PROV_RETRY_DELAY_MS);
            continue;
        }

        prov_enable_requesting = true;
        prov_enable_request_tick = xTaskGetTickCount();
        delay_ticks = pdMS_TO_TICKS(PROV_RETRY_DELAY_MS);
    }

    prov_enable_task_handle = NULL;
    vTaskDelete(NULL);
}

static void start_prov_enable_task(void)
{
    if (prov_enable_task_handle != NULL || prov_enable_active ||
        esp_ble_mesh_node_is_provisioned()) {
        return;
    }

    BaseType_t ok = xTaskCreate(prov_enable_task, "prov_enable", 4096, NULL,
                                tskIDLE_PRIORITY + 2, &prov_enable_task_handle);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "Failed to create provisioning enable task");
    }
}

static void publish_device_status(uint8_t type)
{
    device_state_t *dev = get_device(type);
    if (dev == NULL) {
        return;
    }

    if (!vendor_publish_ready) {
        return;
    }

    uint8_t payload[3] = {dev->type, dev->mode, dev->level};
    esp_err_t err = esp_ble_mesh_model_publish(&vendor_models[0], VENDOR_MODEL_OP_STATUS,
                                               sizeof(payload), payload, ROLE_NODE);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Vendor 状态主动上报失败: %s", esp_err_to_name(err));
    }
}

static void reply_vendor_status(esp_ble_mesh_msg_ctx_t *ctx, uint8_t type)
{
    device_state_t *dev = get_device(type);
    if (dev == NULL) {
        return;
    }

    uint8_t payload[3] = {dev->type, dev->mode, dev->level};
    esp_err_t err = esp_ble_mesh_server_model_send_msg(&vendor_models[0], ctx,
                                                       VENDOR_MODEL_OP_STATUS,
                                                       sizeof(payload), payload);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Vendor 状态响应失败: %s", esp_err_to_name(err));
    }
}

static void handle_onoff_get(esp_ble_mesh_model_t *model, esp_ble_mesh_msg_ctx_t *ctx)
{
    uint8_t onoff = led_device.level;
    esp_ble_mesh_server_model_send_msg(model, ctx, ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_STATUS,
                                       sizeof(onoff), &onoff);
}

static void handle_onoff_set(esp_ble_mesh_model_t *model, esp_ble_mesh_msg_ctx_t *ctx,
                             esp_ble_mesh_server_recv_gen_onoff_set_t *set)
{
    onoff_server.state.onoff = set->onoff ? 1 : 0;
    update_device_state(DEVICE_TYPE_LED, onoff_server.state.onoff ? DEVICE_MODE_ON : DEVICE_MODE_OFF);

    if (ctx->recv_op == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET) {
        esp_ble_mesh_server_model_send_msg(model, ctx, ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_STATUS,
                                           sizeof(onoff_server.state.onoff),
                                           &onoff_server.state.onoff);
    }

    esp_ble_mesh_model_publish(model, ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_STATUS,
                               sizeof(onoff_server.state.onoff),
                               &onoff_server.state.onoff, ROLE_NODE);
}

static void provisioning_cb(esp_ble_mesh_prov_cb_event_t event,
                            esp_ble_mesh_prov_cb_param_t *param)
{
    switch (event) {
    case ESP_BLE_MESH_PROV_REGISTER_COMP_EVT:
        ESP_LOGI(TAG, "Mesh 注册完成 err=%d", param->prov_register_comp.err_code);
        break;
    case ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT:
        prov_enable_requesting = false;
        prov_enable_active = (param->node_prov_enable_comp.err_code == ESP_OK);
        if (!prov_enable_active) {
            ESP_LOGW(TAG, "PB-ADV provisioning enable failed, err=%d",
                     param->node_prov_enable_comp.err_code);
            start_prov_enable_task();
        }
        ESP_LOGI(TAG, "节点配网广播已开启 err=%d", param->node_prov_enable_comp.err_code);
        break;
    case ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT:
        ESP_LOGI(TAG, "配网链路打开 bearer=%s",
                 param->node_prov_link_open.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
        break;
    case ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT:
        ESP_LOGI(TAG, "配网链路关闭 reason=0x%02x", param->node_prov_link_close.reason);
        break;
    case ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT:
        primary_addr = param->node_prov_complete.addr;
        prov_enable_active = false;
        prov_enable_requesting = false;
        ESP_LOGI(TAG, "节点配网完成 addr=0x%04x net_idx=0x%03x iv_index=0x%08" PRIx32,
                 primary_addr, param->node_prov_complete.net_idx,
                 param->node_prov_complete.iv_index);
        break;
    case ESP_BLE_MESH_NODE_PROV_RESET_EVT:
        ESP_LOGI(TAG, "节点配网信息已重置，重新开启 PB-ADV/PB-GATT");
        prov_enable_active = false;
        prov_enable_requesting = false;
        start_prov_enable_task();
        break;
    case ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT:
        ESP_LOGI(TAG, "未配网设备名设置完成 err=%d",
                 param->node_set_unprov_dev_name_comp.err_code);
        break;
    default:
        break;
    }
}

static void config_server_cb(esp_ble_mesh_cfg_server_cb_event_t event,
                             esp_ble_mesh_cfg_server_cb_param_t *param)
{
    if (event != ESP_BLE_MESH_CFG_SERVER_STATE_CHANGE_EVT) {
        return;
    }

    switch (param->ctx.recv_op) {
    case ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD:
        ESP_LOGI(TAG, "收到 AppKey Add net_idx=0x%04x app_idx=0x%04x",
                 param->value.state_change.appkey_add.net_idx,
                 param->value.state_change.appkey_add.app_idx);
        break;
    case ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND:
        ESP_LOGI(TAG, "模型绑定 AppKey elem=0x%04x cid=0x%04x model=0x%04x",
                 param->value.state_change.mod_app_bind.element_addr,
                 param->value.state_change.mod_app_bind.company_id,
                 param->value.state_change.mod_app_bind.model_id);
        break;
    case ESP_BLE_MESH_MODEL_OP_MODEL_SUB_ADD:
        ESP_LOGI(TAG, "模型订阅组地址 elem=0x%04x sub=0x%04x cid=0x%04x model=0x%04x",
                 param->value.state_change.mod_sub_add.element_addr,
                 param->value.state_change.mod_sub_add.sub_addr,
                 param->value.state_change.mod_sub_add.company_id,
                 param->value.state_change.mod_sub_add.model_id);
        break;
    case ESP_BLE_MESH_MODEL_OP_MODEL_PUB_SET:
        if (param->value.state_change.mod_pub_set.company_id == CID_ESPRESSIF &&
            param->value.state_change.mod_pub_set.model_id == VENDOR_MODEL_ID_SERVER) {
            vendor_publish_ready = true;
        }
        ESP_LOGI(TAG, "模型发布地址设置 elem=0x%04x pub=0x%04x cid=0x%04x model=0x%04x",
                 param->value.state_change.mod_pub_set.element_addr,
                 param->value.state_change.mod_pub_set.pub_addr,
                 param->value.state_change.mod_pub_set.company_id,
                 param->value.state_change.mod_pub_set.model_id);
        break;
    default:
        break;
    }
}

static void generic_server_cb(esp_ble_mesh_generic_server_cb_event_t event,
                              esp_ble_mesh_generic_server_cb_param_t *param)
{
    ESP_LOGI(TAG, "Generic Server event=0x%02x op=0x%04" PRIx32 " src=0x%04x dst=0x%04x",
             event, param->ctx.recv_op, param->ctx.addr, param->ctx.recv_dst);

    switch (event) {
    case ESP_BLE_MESH_GENERIC_SERVER_RECV_GET_MSG_EVT:
        if (param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_GET) {
            handle_onoff_get(param->model, &param->ctx);
        }
        break;
    case ESP_BLE_MESH_GENERIC_SERVER_RECV_SET_MSG_EVT:
        if (param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET ||
            param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK) {
            handle_onoff_set(param->model, &param->ctx, &param->value.set.onoff);
        }
        break;
    case ESP_BLE_MESH_GENERIC_SERVER_STATE_CHANGE_EVT:
        if (param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET ||
            param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK) {
            update_device_state(DEVICE_TYPE_LED,
                                param->value.state_change.onoff_set.onoff ? DEVICE_MODE_ON : DEVICE_MODE_OFF);
        }
        break;
    default:
        break;
    }
}

static void custom_model_cb(esp_ble_mesh_model_cb_event_t event,
                            esp_ble_mesh_model_cb_param_t *param)
{
    switch (event) {
    case ESP_BLE_MESH_MODEL_OPERATION_EVT:
        if (param->model_operation.opcode == VENDOR_MODEL_OP_SET &&
            param->model_operation.length >= 2) {
            uint8_t type = param->model_operation.msg[0];
            uint8_t mode = param->model_operation.msg[1];

            ESP_LOGI(TAG, "收到 Vendor 指令 src=0x%04x dst=0x%04x type=0x%02x mode=0x%02x",
                     param->model_operation.ctx->addr,
                     param->model_operation.ctx->recv_dst, type, mode);
            update_device_state(type, mode);
            reply_vendor_status(param->model_operation.ctx, type);
        }
        break;
    case ESP_BLE_MESH_MODEL_PUBLISH_COMP_EVT:
        if (param->model_publish_comp.err_code != ESP_OK) {
            ESP_LOGW(TAG, "模型发布完成但返回错误 %d", param->model_publish_comp.err_code);
        }
        break;
    case ESP_BLE_MESH_MODEL_SEND_COMP_EVT:
        if (param->model_send_comp.err_code != ESP_OK) {
            ESP_LOGW(TAG, "模型发送完成但返回错误 %d", param->model_send_comp.err_code);
        }
        break;
    default:
        break;
    }
}

static esp_err_t ble_mesh_init(void)
{
    esp_err_t err;

    esp_ble_mesh_register_prov_callback(provisioning_cb);
    esp_ble_mesh_register_config_server_callback(config_server_cb);
    esp_ble_mesh_register_generic_server_callback(generic_server_cb);
    esp_ble_mesh_register_custom_model_callback(custom_model_cb);

    err = esp_ble_mesh_init(&provision, &composition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BLE Mesh 初始化失败: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_ble_mesh_set_unprovisioned_device_name("C3-Mesh-Node");
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "设置未配网设备名失败: %s", esp_err_to_name(err));
    }

    err = ESP_OK;
    start_prov_enable_task();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "开启节点配网失败: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "BLE Mesh 节点初始化完成，等待 nRF Mesh 或网关配网");
    return ESP_OK;
}

void app_main(void)
{
    esp_err_t err;

    ESP_LOGI(TAG, "ESP32-C3 BLE Mesh 节点启动");
    hardware_init();

    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    err = bluetooth_init();
    if (err != ESP_OK) {
        return;
    }

    ble_mesh_get_dev_uuid(dev_uuid);
    err = ble_mesh_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BLE Mesh 启动失败");
    }
}
