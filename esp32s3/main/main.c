/*
 * ESP32-S3 BLE Mesh 网关：Provisioner + GATT Proxy + UART0 上位机桥接。
 *
 * 串口协议：
 *   PC -> 网关：0xAA + addr(高字节) + addr(低字节) + type + mode + 0x55
 *   网关 -> PC：文本行 FB,addr,state 或 GW,status，便于上位机和串口工具同时查看。
 *
 * Mesh 逻辑：
 *   addr == 0xC000 时按组地址转发，同时尝试调用 esp_ble_mesh_model_publish；
 *   addr 为单播地址时定点发送 Vendor 指令，节点按 type/mode 控制硬件。
 */

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_generic_model_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_provisioning_api.h"

#define TAG "MESH_GATEWAY"

#define CID_ESPRESSIF          0x02E5
#define VENDOR_MODEL_ID_CLIENT 0x0001
#define VENDOR_MODEL_OP_SET    ESP_BLE_MESH_MODEL_OP_3(0x01, CID_ESPRESSIF)
#define VENDOR_MODEL_OP_STATUS ESP_BLE_MESH_MODEL_OP_3(0x02, CID_ESPRESSIF)

#define UART_PORT              UART_NUM_0
#define UART_BAUD_RATE         115200
#define UART_RX_BUF_SIZE       256

#define GATEWAY_LED_GPIO       GPIO_NUM_1
#define GROUP_ADDR             0xC000
#define PROV_OWN_ADDR          0x0001
#define PROV_START_ADDR        0x0002
#define APP_KEY_IDX            0x0000
#define APP_KEY_FILL           0x12
#define MSG_SEND_TTL           7
#define MSG_TIMEOUT_MS         4000
#define PROV_ENABLE_DELAY_MS   1500
#define PROV_ENABLE_RETRY_MS   3000
#define PROV_ENABLE_TIMEOUT_MS 5000

typedef struct {
    uint16_t addr;
    uint8_t type;
    uint8_t mode;
} uart_command_t;

typedef enum {
    PARSER_WAIT_HEAD = 0,
    PARSER_ADDR_H,
    PARSER_ADDR_L,
    PARSER_TYPE,
    PARSER_MODE,
    PARSER_WAIT_TAIL,
} uart_parser_state_t;

typedef enum {
    NODE_CFG_IDLE = 0,
    NODE_CFG_WAIT_ONOFF_BIND,
    NODE_CFG_WAIT_VENDOR_BIND,
    NODE_CFG_WAIT_VENDOR_SUB,
    NODE_CFG_WAIT_VENDOR_PUB,
    NODE_CFG_DONE,
} node_config_step_t;

typedef struct {
    uint8_t uuid[16];
    uint16_t unicast;
    uint8_t elem_num;
    node_config_step_t config_step;
    bool configured;
} mesh_node_info_t;

static uint8_t dev_uuid[16] = {0xdd, 0xdd};
static mesh_node_info_t nodes[CONFIG_BLE_MESH_MAX_PROV_NODES];
static uint16_t net_idx = ESP_BLE_MESH_KEY_PRIMARY;
static uint16_t app_idx = APP_KEY_IDX;
static uint8_t app_key[16];
static uint8_t vendor_tid;
static bool mesh_ready;
static bool local_app_key_requested;
static bool local_app_key_ready;
static bool vendor_client_bound;
static bool provisioner_scan_enabled;
static bool provisioner_scan_requesting;
static TickType_t provisioner_scan_request_tick;
static TaskHandle_t provisioner_scan_task_handle;
static uint32_t provisioner_scan_attempts;

static esp_ble_mesh_client_t config_client;
static esp_ble_mesh_client_t onoff_client;
static esp_ble_mesh_client_t vendor_client;

static void start_provisioner_scan_task(void);
static void add_local_app_key_after_scan_enabled(void);
static void mark_local_app_key_ready(uint16_t ready_app_idx);

ESP_BLE_MESH_MODEL_PUB_DEFINE(vendor_cli_pub, 3 + 3, ROLE_PROVISIONER);

static const esp_ble_mesh_client_op_pair_t vendor_op_pair[] = {
    {VENDOR_MODEL_OP_SET, VENDOR_MODEL_OP_STATUS},
};

static esp_ble_mesh_model_op_t vendor_cli_op[] = {
    ESP_BLE_MESH_MODEL_OP(VENDOR_MODEL_OP_STATUS, 3),
    ESP_BLE_MESH_MODEL_OP_END,
};

static esp_ble_mesh_cfg_srv_t config_server = {
    .net_transmit = ESP_BLE_MESH_TRANSMIT(2, 20),
    .relay = ESP_BLE_MESH_RELAY_DISABLED,
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
    .default_ttl = MSG_SEND_TTL,
};

static esp_ble_mesh_model_t root_models[] = {
    ESP_BLE_MESH_MODEL_CFG_SRV(&config_server),
    ESP_BLE_MESH_MODEL_CFG_CLI(&config_client),
    ESP_BLE_MESH_MODEL_GEN_ONOFF_CLI(NULL, &onoff_client),
};

static esp_ble_mesh_model_t vendor_models[] = {
    ESP_BLE_MESH_VENDOR_MODEL(CID_ESPRESSIF, VENDOR_MODEL_ID_CLIENT, vendor_cli_op,
                              &vendor_cli_pub, &vendor_client),
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
    .prov_uuid = dev_uuid,
    .prov_unicast_addr = PROV_OWN_ADDR,
    .prov_start_address = PROV_START_ADDR,
    .prov_attention = 0x00,
    .prov_algorithm = 0x00,
    .prov_pub_key_oob = 0x00,
    .prov_static_oob_val = NULL,
    .prov_static_oob_len = 0x00,
    .flags = 0x00,
    .iv_index = 0x00,
};

static void set_gateway_led(bool on)
{
    gpio_set_level(GATEWAY_LED_GPIO, on ? 1 : 0);
}

static void gateway_led_forwarding_pulse(void)
{
    for (int i = 0; i < 3; i++) {
        set_gateway_led(false);
        vTaskDelay(pdMS_TO_TICKS(80));
        set_gateway_led(true);
        vTaskDelay(pdMS_TO_TICKS(80));
    }
}

static void gateway_led_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << GATEWAY_LED_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    set_gateway_led(false);
}

static void uart_init_bridge(void)
{
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, UART_RX_BUF_SIZE, 0, 0, NULL, 0));
}

static void uart_send_gateway_status(const char *status)
{
    char line[32];
    int len = snprintf(line, sizeof(line), "GW,%s\r\n", status);
    if (len > 0) {
        uart_write_bytes(UART_PORT, line, len);
    }
}

static void uart_send_gateway_addr_status(const char *status, uint16_t addr)
{
    char line[40];
    int len = snprintf(line, sizeof(line), "GW,%s,%04X\r\n", status, addr);
    if (len > 0) {
        uart_write_bytes(UART_PORT, line, len);
    }
}

static void uart_send_feedback(uint16_t node_addr, uint8_t state)
{
    char line[24];
    int len = snprintf(line, sizeof(line), "FB,%04X,%02X\r\n", node_addr, state);
    if (len > 0) {
        uart_write_bytes(UART_PORT, line, len);
    }
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

static void provisioner_scan_task(void *arg)
{
    TickType_t delay_ticks = pdMS_TO_TICKS(PROV_ENABLE_DELAY_MS);

    (void)arg;

    while (!provisioner_scan_enabled) {
        vTaskDelay(delay_ticks);

        if (provisioner_scan_enabled) {
            break;
        }

        if (provisioner_scan_requesting) {
            TickType_t elapsed = xTaskGetTickCount() - provisioner_scan_request_tick;
            if (elapsed < pdMS_TO_TICKS(PROV_ENABLE_TIMEOUT_MS)) {
                continue;
            }
            ESP_LOGW(TAG, "Provisioner scan enable request timed out, retrying");
            provisioner_scan_requesting = false;
        }

        provisioner_scan_attempts++;
        ESP_LOGI(TAG, "Starting provisioner PB-ADV scan, attempt=%" PRIu32,
                 provisioner_scan_attempts);

        esp_err_t err = esp_ble_mesh_provisioner_prov_enable(ESP_BLE_MESH_PROV_ADV);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Provisioner scan enable failed immediately: %s",
                     esp_err_to_name(err));
            delay_ticks = pdMS_TO_TICKS(PROV_ENABLE_RETRY_MS);
            continue;
        }

        provisioner_scan_requesting = true;
        provisioner_scan_request_tick = xTaskGetTickCount();
        delay_ticks = pdMS_TO_TICKS(PROV_ENABLE_RETRY_MS);
    }

    provisioner_scan_task_handle = NULL;
    vTaskDelete(NULL);
}

static void start_provisioner_scan_task(void)
{
    if (provisioner_scan_task_handle != NULL || provisioner_scan_enabled) {
        return;
    }

    BaseType_t ok = xTaskCreate(provisioner_scan_task, "prov_scan", 4096, NULL,
                                tskIDLE_PRIORITY + 2, &provisioner_scan_task_handle);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "Failed to create provisioner scan task");
    }
}

static void add_local_app_key_after_scan_enabled(void)
{
    if (local_app_key_requested) {
        return;
    }

    local_app_key_requested = true;

    if (esp_ble_mesh_provisioner_get_local_app_key(net_idx, app_idx) != NULL) {
        ESP_LOGI(TAG, "Provisioner 本地 AppKey 已存在，直接绑定本地模型");
        mark_local_app_key_ready(app_idx);
        return;
    }

    esp_err_t err = esp_ble_mesh_provisioner_add_local_app_key(app_key, net_idx, app_idx);
    if (err != ESP_OK) {
        local_app_key_requested = false;
        ESP_LOGE(TAG, "添加本地 AppKey 失败: %s", esp_err_to_name(err));
    }
}

static void bind_local_models_to_app_key(void)
{
    esp_err_t err = esp_ble_mesh_provisioner_bind_app_key_to_local_model(
        PROV_OWN_ADDR, app_idx, ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_CLI,
        ESP_BLE_MESH_CID_NVAL);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "绑定 Generic OnOff Client AppKey 失败: %s", esp_err_to_name(err));
    }

    err = esp_ble_mesh_provisioner_bind_app_key_to_local_model(
        PROV_OWN_ADDR, app_idx, VENDOR_MODEL_ID_CLIENT, CID_ESPRESSIF);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "绑定 Vendor Client AppKey 失败: %s", esp_err_to_name(err));
    }
}

static void mark_local_app_key_ready(uint16_t ready_app_idx)
{
    app_idx = ready_app_idx;
    local_app_key_ready = true;
    bind_local_models_to_app_key();
}

static mesh_node_info_t *find_node(uint16_t unicast)
{
    for (size_t i = 0; i < ARRAY_SIZE(nodes); i++) {
        if (nodes[i].unicast != ESP_BLE_MESH_ADDR_UNASSIGNED &&
            nodes[i].unicast <= unicast &&
            unicast < nodes[i].unicast + nodes[i].elem_num) {
            return &nodes[i];
        }
    }
    return NULL;
}

static mesh_node_info_t *store_node(const uint8_t uuid[16], uint16_t unicast, uint8_t elem_num)
{
    for (size_t i = 0; i < ARRAY_SIZE(nodes); i++) {
        if (!memcmp(nodes[i].uuid, uuid, sizeof(nodes[i].uuid))) {
            nodes[i].unicast = unicast;
            nodes[i].elem_num = elem_num;
            nodes[i].config_step = NODE_CFG_IDLE;
            nodes[i].configured = false;
            return &nodes[i];
        }
    }

    for (size_t i = 0; i < ARRAY_SIZE(nodes); i++) {
        if (nodes[i].unicast == ESP_BLE_MESH_ADDR_UNASSIGNED) {
            memcpy(nodes[i].uuid, uuid, sizeof(nodes[i].uuid));
            nodes[i].unicast = unicast;
            nodes[i].elem_num = elem_num;
            nodes[i].config_step = NODE_CFG_IDLE;
            nodes[i].configured = false;
            return &nodes[i];
        }
    }

    return NULL;
}

static void fill_common(esp_ble_mesh_client_common_param_t *common,
                        esp_ble_mesh_model_t *model,
                        uint32_t opcode,
                        uint16_t dst)
{
    memset(common, 0, sizeof(*common));
    common->opcode = opcode;
    common->model = model;
    common->ctx.net_idx = net_idx;
    common->ctx.app_idx = app_idx;
    common->ctx.addr = dst;
    common->ctx.send_ttl = MSG_SEND_TTL;
    common->msg_timeout = MSG_TIMEOUT_MS;
}

static esp_err_t send_config_app_key_add(uint16_t dst)
{
    esp_ble_mesh_client_common_param_t common;
    esp_ble_mesh_cfg_client_set_state_t set_state = {0};

    fill_common(&common, config_client.model, ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD, dst);
    set_state.app_key_add.net_idx = net_idx;
    set_state.app_key_add.app_idx = app_idx;
    memcpy(set_state.app_key_add.app_key, app_key, sizeof(app_key));

    return esp_ble_mesh_config_client_set_state(&common, &set_state);
}

static esp_err_t send_model_app_bind(uint16_t dst, uint16_t element_addr,
                                     uint16_t model_id, uint16_t company_id)
{
    esp_ble_mesh_client_common_param_t common;
    esp_ble_mesh_cfg_client_set_state_t set_state = {0};

    fill_common(&common, config_client.model, ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND, dst);
    set_state.model_app_bind.element_addr = element_addr;
    set_state.model_app_bind.model_app_idx = app_idx;
    set_state.model_app_bind.model_id = model_id;
    set_state.model_app_bind.company_id = company_id;

    return esp_ble_mesh_config_client_set_state(&common, &set_state);
}

static esp_err_t send_vendor_sub_add(uint16_t dst, uint16_t element_addr)
{
    esp_ble_mesh_client_common_param_t common;
    esp_ble_mesh_cfg_client_set_state_t set_state = {0};

    fill_common(&common, config_client.model, ESP_BLE_MESH_MODEL_OP_MODEL_SUB_ADD, dst);
    set_state.model_sub_add.element_addr = element_addr;
    set_state.model_sub_add.sub_addr = GROUP_ADDR;
    set_state.model_sub_add.model_id = VENDOR_MODEL_ID_CLIENT;
    set_state.model_sub_add.company_id = CID_ESPRESSIF;

    return esp_ble_mesh_config_client_set_state(&common, &set_state);
}

static esp_err_t send_vendor_pub_set(uint16_t dst, uint16_t element_addr)
{
    esp_ble_mesh_client_common_param_t common;
    esp_ble_mesh_cfg_client_set_state_t set_state = {0};

    fill_common(&common, config_client.model, ESP_BLE_MESH_MODEL_OP_MODEL_PUB_SET, dst);
    set_state.model_pub_set.element_addr = element_addr;
    set_state.model_pub_set.publish_addr = PROV_OWN_ADDR;
    set_state.model_pub_set.publish_app_idx = app_idx;
    set_state.model_pub_set.cred_flag = false;
    set_state.model_pub_set.publish_ttl = MSG_SEND_TTL;
    set_state.model_pub_set.publish_period = 0;
    set_state.model_pub_set.publish_retransmit = ESP_BLE_MESH_PUBLISH_TRANSMIT(1, 50);
    set_state.model_pub_set.model_id = VENDOR_MODEL_ID_CLIENT;
    set_state.model_pub_set.company_id = CID_ESPRESSIF;

    return esp_ble_mesh_config_client_set_state(&common, &set_state);
}

static esp_err_t send_vendor_command(uint16_t dst, uint8_t type, uint8_t mode, bool need_rsp)
{
    uint8_t payload[3] = {type, mode, vendor_tid++};
    esp_ble_mesh_msg_ctx_t ctx = {
        .net_idx = net_idx,
        .app_idx = app_idx,
        .addr = dst,
        .send_ttl = MSG_SEND_TTL,
    };

    return esp_ble_mesh_client_model_send_msg(vendor_client.model, &ctx, VENDOR_MODEL_OP_SET,
                                              sizeof(payload), payload, MSG_TIMEOUT_MS, need_rsp,
                                              ROLE_PROVISIONER);
}

static void forward_command(const uart_command_t *cmd)
{
    if (!mesh_ready) {
        ESP_LOGW(TAG, "Mesh 尚未就绪，丢弃串口指令");
        uart_send_gateway_status("BUSY");
        return;
    }

    ESP_LOGI(TAG, "串口指令 addr=0x%04x type=0x%02x mode=0x%02x",
             cmd->addr, cmd->type, cmd->mode);
    gateway_led_forwarding_pulse();

    if (cmd->addr == GROUP_ADDR) {
        esp_err_t err = send_vendor_command(GROUP_ADDR, cmd->type, cmd->mode, false);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "组播发送失败: %s", esp_err_to_name(err));
        }
        return;
    }

    if (!ESP_BLE_MESH_ADDR_IS_UNICAST(cmd->addr)) {
        ESP_LOGW(TAG, "非法目标地址 0x%04x", cmd->addr);
        return;
    }

    esp_err_t err = send_vendor_command(cmd->addr, cmd->type, cmd->mode, true);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "单播发送失败: %s", esp_err_to_name(err));
    }
}

static bool uart_parser_feed(uint8_t byte, uart_command_t *cmd)
{
    static uart_parser_state_t state = PARSER_WAIT_HEAD;
    static uart_command_t current = {0};

    switch (state) {
    case PARSER_WAIT_HEAD:
        if (byte == 0xAA) {
            memset(&current, 0, sizeof(current));
            state = PARSER_ADDR_H;
        }
        break;
    case PARSER_ADDR_H:
        current.addr = (uint16_t)byte << 8;
        state = PARSER_ADDR_L;
        break;
    case PARSER_ADDR_L:
        current.addr |= byte;
        state = PARSER_TYPE;
        break;
    case PARSER_TYPE:
        current.type = byte;
        state = PARSER_MODE;
        break;
    case PARSER_MODE:
        current.mode = byte;
        state = PARSER_WAIT_TAIL;
        break;
    case PARSER_WAIT_TAIL:
        state = PARSER_WAIT_HEAD;
        if (byte == 0x55) {
            *cmd = current;
            return true;
        }
        if (byte == 0xAA) {
            state = PARSER_ADDR_H;
        }
        break;
    default:
        state = PARSER_WAIT_HEAD;
        break;
    }

    return false;
}

static void uart_rx_task(void *arg)
{
    uint8_t data[64];
    uart_command_t cmd;

    while (true) {
        int len = uart_read_bytes(UART_PORT, data, sizeof(data), pdMS_TO_TICKS(100));
        for (int i = 0; i < len; i++) {
            if (uart_parser_feed(data[i], &cmd)) {
                forward_command(&cmd);
            }
        }
    }
}

static void recv_unprov_adv_pkt(uint8_t uuid[16], uint8_t addr[BD_ADDR_LEN],
                                esp_ble_mesh_addr_type_t addr_type,
                                uint16_t oob_info,
                                esp_ble_mesh_prov_bearer_t bearer)
{
    esp_ble_mesh_unprov_dev_add_t add_dev = {0};

    memcpy(add_dev.addr, addr, BD_ADDR_LEN);
    add_dev.addr_type = addr_type;
    memcpy(add_dev.uuid, uuid, 16);
    add_dev.oob_info = oob_info;
    add_dev.bearer = bearer;

    esp_err_t err = esp_ble_mesh_provisioner_add_unprov_dev(
        &add_dev,
        ADD_DEV_RM_AFTER_PROV_FLAG | ADD_DEV_START_PROV_NOW_FLAG | ADD_DEV_FLUSHABLE_DEV_FLAG);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "加入未配网设备队列失败: %s", esp_err_to_name(err));
    }
}

static void provisioning_cb(esp_ble_mesh_prov_cb_event_t event,
                            esp_ble_mesh_prov_cb_param_t *param)
{
    switch (event) {
    case ESP_BLE_MESH_PROV_REGISTER_COMP_EVT:
        ESP_LOGI(TAG, "Mesh 注册完成 err=%d", param->prov_register_comp.err_code);
        break;
    case ESP_BLE_MESH_PROVISIONER_PROV_ENABLE_COMP_EVT:
        provisioner_scan_requesting = false;
        provisioner_scan_enabled = (param->provisioner_prov_enable_comp.err_code == ESP_OK);
        ESP_LOGI(TAG, "Provisioner scan enable complete err=%d",
                 param->provisioner_prov_enable_comp.err_code);
        if (provisioner_scan_enabled) {
            uart_send_gateway_status("SCAN");
            add_local_app_key_after_scan_enabled();
        } else {
            start_provisioner_scan_task();
        }
        break;
    case ESP_BLE_MESH_PROVISIONER_RECV_UNPROV_ADV_PKT_EVT:
        ESP_LOGI(TAG, "发现未配网设备 UUID=%s",
                 bt_hex(param->provisioner_recv_unprov_adv_pkt.dev_uuid, 16));
        recv_unprov_adv_pkt(param->provisioner_recv_unprov_adv_pkt.dev_uuid,
                            param->provisioner_recv_unprov_adv_pkt.addr,
                            param->provisioner_recv_unprov_adv_pkt.addr_type,
                            param->provisioner_recv_unprov_adv_pkt.oob_info,
                            param->provisioner_recv_unprov_adv_pkt.bearer);
        break;
    case ESP_BLE_MESH_PROVISIONER_PROV_COMPLETE_EVT: {
        uint16_t unicast = param->provisioner_prov_complete.unicast_addr;
        uint8_t elem_num = param->provisioner_prov_complete.element_num;
        uint16_t node_idx = param->provisioner_prov_complete.node_idx;
        char name[16];

        ESP_LOGI(TAG, "节点配网完成 index=%u addr=0x%04x elems=%u",
                 node_idx, unicast, elem_num);
        uart_send_gateway_addr_status("PROV", unicast);
        snprintf(name, sizeof(name), "Node-%u", node_idx + 1);
        esp_ble_mesh_provisioner_set_node_name(node_idx, name);
        store_node(param->provisioner_prov_complete.device_uuid, unicast, elem_num);
        send_config_app_key_add(unicast);
        break;
    }
    case ESP_BLE_MESH_PROVISIONER_ADD_LOCAL_APP_KEY_COMP_EVT:
        ESP_LOGI(TAG, "Provisioner 本地 AppKey 添加完成 err=%d",
                 param->provisioner_add_app_key_comp.err_code);
        if (param->provisioner_add_app_key_comp.err_code == ESP_OK) {
            mark_local_app_key_ready(param->provisioner_add_app_key_comp.app_idx);
        } else if (esp_ble_mesh_provisioner_get_local_app_key(net_idx, app_idx) != NULL) {
            ESP_LOGW(TAG, "AppKey 添加返回错误，但本地已有 AppKey，继续绑定模型");
            mark_local_app_key_ready(app_idx);
        } else {
            local_app_key_requested = false;
        }
        break;
    case ESP_BLE_MESH_PROVISIONER_BIND_APP_KEY_TO_MODEL_COMP_EVT:
        ESP_LOGI(TAG, "Provisioner 本地模型绑定 AppKey 完成 elem=0x%04x cid=0x%04x model=0x%04x err=%d",
                 param->provisioner_bind_app_key_to_model_comp.element_addr,
                 param->provisioner_bind_app_key_to_model_comp.company_id,
                 param->provisioner_bind_app_key_to_model_comp.model_id,
                 param->provisioner_bind_app_key_to_model_comp.err_code);
        if (param->provisioner_bind_app_key_to_model_comp.err_code == ESP_OK &&
            param->provisioner_bind_app_key_to_model_comp.company_id == CID_ESPRESSIF &&
            param->provisioner_bind_app_key_to_model_comp.model_id == VENDOR_MODEL_ID_CLIENT) {
            vendor_client_bound = true;
            mesh_ready = local_app_key_ready;
            set_gateway_led(mesh_ready);
            if (mesh_ready) {
                uart_send_gateway_status("READY");
            }
        }
        break;
    case ESP_BLE_MESH_PROVISIONER_PROV_LINK_OPEN_EVT:
        ESP_LOGI(TAG, "Provisioning link open bearer=%s",
                 param->provisioner_prov_link_open.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
        break;
    case ESP_BLE_MESH_PROVISIONER_PROV_LINK_CLOSE_EVT:
        ESP_LOGI(TAG, "Provisioning link close reason=0x%02x",
                 param->provisioner_prov_link_close.reason);
        break;
    default:
        break;
    }
}

static void config_client_cb(esp_ble_mesh_cfg_client_cb_event_t event,
                             esp_ble_mesh_cfg_client_cb_param_t *param)
{
    uint32_t opcode = param->params->opcode;
    uint16_t addr = param->params->ctx.addr;
    mesh_node_info_t *node = find_node(addr);

    ESP_LOGI(TAG, "Config Client event=0x%02x opcode=0x%04" PRIx32 " addr=0x%04x err=0x%02x",
             event, opcode, addr, param->error_code);
    if (param->error_code || node == NULL) {
        return;
    }

    if (event != ESP_BLE_MESH_CFG_CLIENT_SET_STATE_EVT) {
        return;
    }

    switch (opcode) {
    case ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD:
        node->config_step = NODE_CFG_WAIT_ONOFF_BIND;
        send_model_app_bind(addr, node->unicast, ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_SRV,
                            ESP_BLE_MESH_CID_NVAL);
        break;
    case ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND:
        if (!node->configured) {
            if (node->config_step == NODE_CFG_WAIT_ONOFF_BIND) {
                node->config_step = NODE_CFG_WAIT_VENDOR_BIND;
                send_model_app_bind(addr, node->unicast, VENDOR_MODEL_ID_CLIENT, CID_ESPRESSIF);
            } else if (node->config_step == NODE_CFG_WAIT_VENDOR_BIND) {
                node->config_step = NODE_CFG_WAIT_VENDOR_SUB;
                send_vendor_sub_add(addr, node->unicast);
            }
        }
        break;
    case ESP_BLE_MESH_MODEL_OP_MODEL_SUB_ADD:
        if (node->config_step == NODE_CFG_WAIT_VENDOR_SUB) {
            node->config_step = NODE_CFG_WAIT_VENDOR_PUB;
            send_vendor_pub_set(addr, node->unicast);
        }
        break;
    case ESP_BLE_MESH_MODEL_OP_MODEL_PUB_SET:
        node->config_step = NODE_CFG_DONE;
        node->configured = true;
        ESP_LOGI(TAG, "节点 0x%04x 配置更新完成", addr);
        uart_send_gateway_addr_status("NODE_READY", addr);
        break;
    default:
        break;
    }
}

static void generic_client_cb(esp_ble_mesh_generic_client_cb_event_t event,
                              esp_ble_mesh_generic_client_cb_param_t *param)
{
    if (param->error_code != ESP_OK || param->params == NULL) {
        return;
    }

    if (param->params->opcode == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET ||
        param->params->opcode == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_GET) {
        uart_send_feedback(param->params->ctx.addr,
                           param->status_cb.onoff_status.present_onoff);
    }

    ESP_LOGI(TAG, "Generic Client event=0x%02x addr=0x%04x onoff=%u",
             event, param->params->ctx.addr,
             param->status_cb.onoff_status.present_onoff);
}

static void custom_model_cb(esp_ble_mesh_model_cb_event_t event,
                            esp_ble_mesh_model_cb_param_t *param)
{
    switch (event) {
    case ESP_BLE_MESH_MODEL_OPERATION_EVT:
        if (param->model_operation.opcode == VENDOR_MODEL_OP_STATUS &&
            param->model_operation.length >= 3) {
            uint16_t node_addr = param->model_operation.ctx->addr;
            uint8_t type = param->model_operation.msg[0];
            uint8_t mode = param->model_operation.msg[1];
            uint8_t state = (uint8_t)((type << 4) | (mode & 0x0f));
            uart_send_feedback(node_addr, state);
            ESP_LOGI(TAG, "Vendor 状态 node=0x%04x type=0x%02x mode=0x%02x",
                     node_addr, type, mode);
        }
        break;
    case ESP_BLE_MESH_CLIENT_MODEL_RECV_PUBLISH_MSG_EVT:
        if (param->client_recv_publish_msg.opcode == VENDOR_MODEL_OP_STATUS &&
            param->client_recv_publish_msg.length >= 3) {
            uint16_t node_addr = param->client_recv_publish_msg.ctx->addr;
            uint8_t type = param->client_recv_publish_msg.msg[0];
            uint8_t mode = param->client_recv_publish_msg.msg[1];
            uint8_t state = (uint8_t)((type << 4) | (mode & 0x0f));
            uart_send_feedback(node_addr, state);
        }
        break;
    case ESP_BLE_MESH_CLIENT_MODEL_SEND_TIMEOUT_EVT:
        ESP_LOGW(TAG, "Vendor 指令超时 opcode=0x%06" PRIx32 " dst=0x%04x",
                 param->client_send_timeout.opcode,
                 param->client_send_timeout.ctx ? param->client_send_timeout.ctx->addr : 0);
        break;
    default:
        break;
    }
}

static esp_err_t ble_mesh_init(void)
{
    esp_err_t err;

    memset(nodes, 0, sizeof(nodes));
    memset(app_key, APP_KEY_FILL, sizeof(app_key));
    net_idx = ESP_BLE_MESH_KEY_PRIMARY;
    app_idx = APP_KEY_IDX;

    vendor_client.op_pair_size = ARRAY_SIZE(vendor_op_pair);
    vendor_client.op_pair = vendor_op_pair;
    mesh_ready = false;
    local_app_key_requested = false;
    local_app_key_ready = false;
    vendor_client_bound = false;

    esp_ble_mesh_register_prov_callback(provisioning_cb);
    esp_ble_mesh_register_config_client_callback(config_client_cb);
    esp_ble_mesh_register_generic_client_callback(generic_client_cb);
    esp_ble_mesh_register_custom_model_callback(custom_model_cb);

    err = esp_ble_mesh_init(&provision, &composition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BLE Mesh 初始化失败: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_ble_mesh_client_model_init(&vendor_models[0]);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Vendor Client 初始化失败: %s", esp_err_to_name(err));
        return err;
    }

    err = ESP_OK;
    start_provisioner_scan_task();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Provisioner 使能失败: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "BLE Mesh Provisioner/GATT Proxy 初始化完成");
    return ESP_OK;
}

void app_main(void)
{
    esp_err_t err;

    ESP_LOGI(TAG, "ESP32-S3 BLE Mesh 网关启动");

    gateway_led_init();
    uart_init_bridge();

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
        return;
    }

    xTaskCreate(uart_rx_task, "uart_rx_task", 4096, NULL, 8, NULL);
}
