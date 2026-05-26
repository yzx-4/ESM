# ESP32 嵌入式项目
基于 ESP-IDF 开发的嵌入式工程项目

## 项目简介
本项目用于 ESP32 芯片的功能开发与调试，包含外设驱动、业务逻辑、调试日志等模块。

## 开发环境
- 主控芯片：ESP32
- 开发框架：ESP-IDF
- 开发工具：VSCode + clangd

## 常用命令
```bash
# 编译
idf.py build

# 烧录
idf.py flash

# 监视串口日志
idf.py monitor