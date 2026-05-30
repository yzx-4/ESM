## 项目简介
ESP32C5的有感FOC，3环控制

## 开发环境
- 主控芯片：ESP32C5/ESP32(内存足够，spi_flash,psram根据情况选)
- 开发框架：ESP-IDF
- 开发工具：VSCode + clangd

## 常用命令
1. 编译
idf.py build

2. 烧录
idf.py flashss

3. 监视串口日志
idf.py monitor