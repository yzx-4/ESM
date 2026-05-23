/*
 * 这段示例按当前 IDF 头文件里实际存在的 API 重写。
 * 结论先写在前面：
 *   1. MCPWM 的 ETM 事件可以用公开 API 获取。
 *   2. ADC continuous 模式可以用公开 API 配置并启动。
 *   3. 但当前这版 IDF 没有公开的 ADC ETM task 创建 API，
 *      所以下面只保留“可确认存在”的部分。
 */

#include "driver/mcpwm_prelude.h"
#include "driver/mcpwm_etm.h"
#include "esp_adc/adc_continuous.h"
#include "esp_etm.h"

static adc_continuous_handle_t s_adc_handle = NULL;

#define EXAMPLE_ADC_UNIT        ADC_UNIT_1
#define EXAMPLE_ADC_CONV_MODE   ADC_CONV_SINGLE_UNIT_1
#define EXAMPLE_ADC_OUTPUT_TYPE  ADC_DIGI_OUTPUT_FORMAT_TYPE1

static adc_channel_t adc_channels[] = {
    ADC_CHANNEL_0,
    ADC_CHANNEL_1,
};

#define EXAMPLE_ADC_CHANNEL_NUM   (sizeof(adc_channels) / sizeof(adc_channels[0]))

static void configure_adc_continuous(void)
{
    adc_continuous_handle_cfg_t adc_handle_cfg = {
        .max_store_buf_size = 1024,
        .conv_frame_size = EXAMPLE_ADC_CHANNEL_NUM * SOC_ADC_DIGI_DATA_BYTES_PER_CONV,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_handle_cfg, &s_adc_handle));

    adc_digi_pattern_config_t adc_pattern[EXAMPLE_ADC_CHANNEL_NUM] = {0};
    for (int i = 0; i < EXAMPLE_ADC_CHANNEL_NUM; i++) {
        adc_pattern[i].atten = ADC_ATTEN_DB_11;
        adc_pattern[i].channel = adc_channels[i];
        adc_pattern[i].unit = EXAMPLE_ADC_UNIT;
        adc_pattern[i].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
    }

    adc_continuous_config_t adc_cfg = {
        .pattern_num = EXAMPLE_ADC_CHANNEL_NUM,
        .adc_pattern = adc_pattern,
        .sample_freq_hz = 10000,
        .conv_mode = EXAMPLE_ADC_CONV_MODE,
        .format = EXAMPLE_ADC_OUTPUT_TYPE,
    };
    ESP_ERROR_CHECK(adc_continuous_config(s_adc_handle, &adc_cfg));
    ESP_ERROR_CHECK(adc_continuous_start(s_adc_handle));
}

static void configure_mcpwm_etm_event(mcpwm_timer_handle_t timer)
{
    esp_etm_event_handle_t mcpwm_etm_event = NULL;
    esp_etm_channel_handle_t etm_chan = NULL;
    esp_etm_channel_config_t etm_chan_cfg = {
        .flags = {
            .allow_pd = 0,
        },
    };

    /* 公开 API 里存在的是 comparator ETM event。若你要“中心点触发”，
     * 通常做法是把 comparator compare 值设置到 PWM 半周期位置。
     */
    mcpwm_oper_handle_t oper = NULL;
    mcpwm_cmpr_handle_t cmpr = NULL;
    mcpwm_operator_config_t oper_cfg = {
        .group_id = 0,
    };
    mcpwm_comparator_config_t cmpr_cfg = {
        .flags.update_cmp_on_tez = true,
    };
    mcpwm_cmpr_etm_event_config_t etm_event_cfg = {
        .event_type = MCPWM_CMPR_ETM_EVENT_EQUAL,
    };

    ESP_ERROR_CHECK(mcpwm_new_operator(&oper_cfg, &oper));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &cmpr_cfg, &cmpr));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(cmpr, 500));
    ESP_ERROR_CHECK(mcpwm_comparator_new_etm_event(cmpr, &etm_event_cfg, &mcpwm_etm_event));

    ESP_ERROR_CHECK(esp_etm_new_channel(&etm_chan_cfg, &etm_chan));

    /* 当前 IDF 版本没有公开的 ADC ETM task 创建 API。
     * 这里先保留 ETM 连接位置，真实 ADC 硬件触发需要后续用你工程里的封装补上。
     */
    ESP_ERROR_CHECK(esp_etm_channel_connect(etm_chan, mcpwm_etm_event, NULL));
    ESP_ERROR_CHECK(esp_etm_channel_enable(etm_chan));
}

void example_setup(void)
{
    configure_adc_continuous();
    /* timer 需要由上层实际创建后传入，这里只展示真实存在的 API 组合。 */
}


/*下面是这个电机项目通信线程与电机控制线程用队列交换数据的的框架，数据的形式。
命令接收用 while(xQueueReceive(..., 0)) 确保清空队列，只留最新一条，避免过时命令堆积。
反馈发送用 xQueueOverwrite（或队列长度为1时的 xQueueSend），绝不阻塞控制线程。
三环在一个任务内分频运行，数据共享零拷贝
*/
typedef enum {
    CMD_MODE_NONE = 0,
    CMD_MODE_TORQUE,    // 力控（电流控制）
    CMD_MODE_VELOCITY,  // 速度控制
    CMD_MODE_POSITION   // 位置控制
} control_mode_t;

typedef struct {
    control_mode_t mode;        // 当前生效的控制模式
    uint8_t u_bus;              // 由于esp32的adc太少，供电电压也需要可能需要上位机给
    float target_torque;        // 目标力矩（Iq 电流值，A）
    float target_velocity;      // 目标速度（rad/s 或 rpm）
    float target_position;      // 目标位置（机械角度，rad）
    float torque_limit;         // 力矩限制值
} control_command_t;
typedef struct {
    float actual_torque;        // 当前实际力矩（由 Iq 估算）
    float actual_velocity;      // 当前实际速度
    float actual_position;      // 当前实际位置
    float iq_measured;          // 当前 q 轴电流
    float vbus_voltage;         // 母线电压
    uint8_t fault_code;         // 故障码（过流、过温等）
} motor_feedback_t;