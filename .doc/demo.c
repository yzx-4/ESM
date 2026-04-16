/*  这是一个mcpwm触发adc采样的部分配置，由于adc转换需要一定时间，
三相连续采并不一定很准，直接采两相再计算是否更好
*/

adc_continuous_handle_t adc_handle;
adc_continuous_handle_cfg_t adc_config = {
    .max_store_buf_size = 1024,
    .conv_frame_size = 512,
};
adc_continuous_new_handle(&adc_config, &adc_handle);

adc_continuous_config_t adc_dig_config = {
    .conv_mode = ADC_CONV_SINGLE_UNIT_1,
    .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
    .pattern_num = 2,  // 两个通道：U相和V相
    .adc_pattern = patterns,  // 预先设置好的通道序列
    .sample_freq_hz = 1000000, // 设为最大值（如1MHz），让ADC硬件以最快速度准备就绪，实际转换由触发信号决定
    .trigger_measure = ADC_CONTINUOUS_TRIG_MODE_MCPWM, // 关键：触发源为MCPWM
    .mcpwm_timer = MCPWM_TIMER_0, // 选择哪个MCPWM定时器
    .mcpwm_cmpr_event = MCPWM_CMPR_EVENT_ACTION_ON_LOW, // 在比较值匹配时触发
};
adc_continuous_config(adc_handle, &adc_dig_config);

