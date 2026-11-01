# ESP32-S3 + NRF24L01 RF 链路质量探测器

用 2 或 3 个 NRF24L01 在 2.4GHz 上测量链路丢包/重传，检测有人/物体穿过
链路，显示在 ST7735 160x128 上。它是 **RF 运动探测器**，不是真雷达：
没有相位/TOF，不能测距、测速、成像。

> 独立固件，和 `esp32-s3-st7735-sdr` 的 SDR 固件分开。NRF 和 SDR 都占用
> 2.4GHz，不能同时跑；本固件不初始化 WiFi/PHY。

## 接线

ST7735 和按键沿用 SDR 的接法：

| ST7735 | ESP32-S3 |
|---|---|
| SCK | GPIO12 |
| MOSI | GPIO11 |
| CS | GPIO10 |
| DC | GPIO9 |
| RST | GPIO8 |
| BL | GPIO7 |
| VCC/GND | 3V3/GND |

| 按键 | ESP32-S3 |
|---|---|
| K1 / UP | GPIO4 |
| K2 / DOWN | GPIO5 |
| K3 / OK | GPIO6 |
| K4 / BACK | GPIO14 |

NRF24L01 使用**独立 SPI3**，不要和屏幕的 SPI2 混在一起：

| NRF 信号 | 模块 A | 模块 B | 模块 C |
|---|---|---|---|
| VCC | 3V3 | 3V3 | 3V3 |
| GND | GND | GND | GND |
| SCK | GPIO13 | GPIO13 | GPIO13 |
| MOSI | GPIO15 | GPIO15 | GPIO15 |
| MISO | GPIO16 | GPIO16 | GPIO16 |
| CSN | GPIO17 | GPIO18 | GPIO21 |
| CE | GPIO38 | GPIO39 | GPIO40 |
| IRQ | 不接 | 不接 | 不接 |

- 只接 2 个模块时，接 A、B，C 的 CSN/CE 不用接；固件会自动只做 AB 链路。
- 模块引脚以丝印标签为准（GND/VCC/CE/CSN/SCK/MOSI/MISO/IRQ），不要按图片编号。
- 每个模块 VCC/GND 并 **10µF + 100nF**，尽量靠近模块。
- 必须 3.3V，不要接 5V。
- 三根天线尽量拉开 10–20cm，不要并排贴在一起。
- GPIO39/40 是 JTAG 引脚；不调试 JTAG 时当普通 GPIO 用没问题。

## 构建

需要 ESP-IDF v5.4.2：

```bash
cd esp32-s3-nrf-radar
idf.py -B build -DIDF_TARGET=esp32s3 build
idf.py -B build -p /dev/cu.usbserial-XXXX flash
```

组件管理器已关闭，`esp_lcd_st7735` 和所有依赖都在 `components/` 内，编译不需要联网。

## 操作

| 按键 | 功能 |
|---|---|
| K1 | 通道 +1（70..90，默认 80 = 2480MHz） |
| K2 | 通道 −1 |
| K3 | 重新校准基线 |
| K4 | 暂停 / 继续 |

- 上电后先做约 20 帧基线校准，屏幕显示 `CALIBRATING`。
- 三条链路 AB / AC / BC 轮流测量；哪条链路被扰动，对应行会掉质量/丢包。
- 底部历史条显示最近约 160 帧的链路质量。
- 任一链路明显偏离基线 → `MOTION`。
- 没有检测到 NRF 模块 → `NO NRF`。

## 指标

- 每链路每帧发 16 个包，250kbps，auto-ACK，8 字节 payload。
- 丢包率 = 未收到包数 / 16。
- 重传数来自 `OBSERVE_TX.ARC_CNT` 累加。
- 运动判据：丢包比基线高 15% 以上，或重传数超过基线 2 倍 + 8。
- 无运动时基线缓慢跟随环境。

## 已知限制

- 不是真雷达：不能测距、测速、成像。
- 3 个模块只能做“哪条链路被扰动”的粗略判断，不是多基地雷达。
- NRF24L01 没有数值 RSSI，只有 1-bit RPD；本固件用丢包/重传，更灵敏。
- 2.4GHz 易受 WiFi/蓝牙/微波炉干扰；换通道或重新校准。
- NRF 与 SDR 不能同时工作；需要 SDR 时刷回另一个固件。
- PA+LNA 模块电流更大，建议单独 3.3V LDO 和大电容。

## 目录

```
main/nrf24.c/.h      NRF24L01 SPI 驱动
main/radar.c/.h      3 链路调度、基线、运动判据
main/st7735.c/.h     ST7735 160x128 CRT 绿色 UI
main/main.c          app_main + 任务
components/          esp_lcd_st7735（MIT）
```
