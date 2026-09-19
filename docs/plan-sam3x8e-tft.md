# SAM3X8E + Adafruit 2.8" TFT Touch Shield v2 —— 实施计划

目标：让 xuanwu 能跑 Arduino Due + Adafruit 2.8" TFT Touch Shield v2（ILI9341 + FT6206）的真实固件，
从 Arduino 自带例程一路到 `Arduino_RT-Thread` + `RTT-GUI` 的复杂 GUI 例程；同时把片上外设的建模
方式泛化，使后续支持其他厂商 MCU 是"加一个寄存器适配器 + 一份 YAML"，而不是重写驱动层。

> 本文是活的文档：每完成一项就把 `[ ]` 改成 `[x]`，并在 §8 记录实测数据。

## 进度

- [x] M6.0 事件调度器 + 批量推进（吞吐 0.85 → 138.6 M instr/s，见 §8）
- [x] M6.0 收尾：SPI 每字节成本用真实 ILI9341 固件验收（并入 M6.2/Rung 3，见 §8）
- [x] M6.1 GPIO / SPI / I2C 行为核泛化（`peripherals/`，SAM 的 PIO/SPI 已改为薄适配层）
- [x] M6.2 SAM3X8E 外设补齐（TWI0/1、SPI 完善、PMC/EEFC、地址图审计、micros 验证 —— 见 §3 M6.2）
- [x] M7.1 显示子系统（ILI9341 + DisplaySurface + headless/pygame；见 §3 M7.1 的落地说明）
- [ ] M7.2 触摸（FT6206 + 输入源）
- [ ] M7.3 存储（SD SPI + FAT 镜像 + 工具）
- [ ] M7.4 器件层泛化与子项目化
- [ ] M8 固件阶梯 Rung 0–9
- [x] Rung 3 里程碑：Adafruit `graphicstest` 跑完并留下正确帧缓冲（见 §8）

## 1. 研究结论

### 1.1 硬件接口：SPI + I2C，没有并口

Adafruit 2.8" TFT Touch Shield v2（v2 = 电容版）不是老款 8 位并口屏：

| 信号 | Due 引脚 | 说明 |
|---|---|---|
| ILI9341 SPI | MOSI 75 / MISO 74 / SCK 76 | SPI 头 |
| ILI9341 CS | **D10（GPIO）** | 不是硬件 NPCS |
| ILI9341 DC | **D9（GPIO）** | 命令/数据 |
| ILI9341 RST | 接板子 RESET | 库用软复位即可 |
| microSD CS | **D4（GPIO）** | SPI 模式 SD |
| FT6206 | **SDA/SCL（I2C）** | 电容触摸，地址 0x38 |
| 触摸 IRQ | D7，默认未连 | 库用轮询（`G_MODE` 写入被注释掉） |

参考：[Zephyr shield 文档](https://raw.githubusercontent.com/zephyrproject-rtos/zephyr/refs/heads/main/boards/shields/adafruit_2_8_tft_touch_v2/doc/index.rst)、
Adafruit learn 页面。

因此**不需要 8/16 位并口总线**；并口只在接口设计里留位置，供将来其他板子/库使用。

### 1.2 两个库要什么（读源码结论）

`Arduino_RT-Thread/src/rtconfig.h` 中 `CONFIG_USING_ADAFRUIT_TFT_CAPACITIVE` +
`ARDUINO_SAM_DUE` 这一段，正是为这块 shield 写的，接线与 §1.1 完全一致：
`CONFIG_ILI_CS_PIN=10`、`CONFIG_ILI_DC_PIN=9`、`CONFIG_SD_CS_PIN=4`、
`CONFIG_USING_IIC1`（`Wire1`）、`CONFIG_FT6206_INT_PIN=7`、`CONFIG_USING_MODULE=1`。

| 库文件 | 依赖的硬件行为 | xuanwu 现状 |
|---|---|---|
| `drv_spi.cpp` + `<SPI.h>` | `SPI.transfer()` 轮询 `SPI_SR.TDRE` / 读 `SPI_RDR`；`SPI_CSR` 模式与分频；写 `TDR` | SPI 模型有，但逐字节转发、`SR` 语义不全 |
| `drv_spiili.cpp` | ILI9341 命令/数据流，CS/DC 为 **GPIO** | 缺 ILI9341 器件模型 |
| `drv_spisd.cpp` | **自带 SPI 模式 SD 协议**（CMD0/8/55/41/58/16/17/24/13/9/10/12，R1/R3/R7/R2，0xFE 数据令牌），块设备 → DFS/elmfat | 缺 SD 卡模型与 FAT 镜像 |
| `drv_iic_ft6206.cpp` | `Wire`：`beginTransmission/write/endTransmission` + `requestFrom(0x38, 14)`；寄存器 0x00/0x01/0x02/0x80/0xA3(=0x06)/0xA4/0xA6/0xA8(=0x11) | **完全没有 I2C** |
| `rtt.cpp` | `sysTickHook()` → `rt_tick_increase()`（Arduino SAM core 在 `SysTick_Handler` 中回调） | SysTick 有；需验证 `micros()` 依赖的寄存器 |
| `rtgui/image_*.c` | 从文件系统读 `/pic/logo.bmp` | 依赖 SD + FAT |

要点：

- SAM 的 `Wire` 是**纯轮询**实现（`TWI_WaitTransferComplete` 等循环读 `TWI_SR`），不需要中断即可工作，
  TWI 模型因此简单得多。
- `Wire` 在 Due 上是 **TWI1**（`variant.h: WIRE_INTERFACE TWI1`，引脚 20/21，基址 `0x40090000`）；
  `Wire1` 是 TWI0（引脚 70/71，`0x4008C000`）。库配置用 `CHANNEL 1` = `Wire1`，而 shield 的 SDA/SCL 在 20/21。
  **不猜**：两个 TWI 都建模，用未认领 MMIO 报告判定固件实际访问哪个基址。
- `CONFIG_USING_MODULE=1` 是 RT-Thread 的动态模块加载器（运行时链接 `.mo`），超出本计划范围：
  构建固件时用我们自己的 `rtconfig.h` 副本把它关掉。

### 1.3 性能：必须先解决

实测（`stm32f411` + `Blink_m4.ino.elf`，300k 指令，三次取最小）：

```
现状（中断引擎 + SysTick 逐指令钩子）      351.0 ms    0.85 M instr/s
同一设备，摘掉全部 Unicorn 钩子              1.7 ms  176.29 M instr/s
```

**逐指令 Python 钩子代价 207×**。GUI 负载量级：320×240 RGB565 全屏 = 153600 字节，
驱动写一字节约 12 条指令 → 约 180 万条指令/帧，现状约 1.8 s/帧，不可用。
摘掉钩子后还要解决**逐字节桥调用**（153600 次 Python 调用 ≈ 0.2–0.5 s/帧）。

## 2. 设计原则

### 2.1 三层结构

```
① 行为核（厂商无关）        src/xuanwu/peripherals/
   SpiController / I2cController / GpioPort / SdCardSpi / Scheduler
② 寄存器适配器（厂商相关）  src/xuanwu/arch/vendor/<vendor>/
   一张 REGISTERS 表 + 把寄存器位映射到行为核的 hook
③ 芯片描述（数据）          src/xuanwu/data/chips/**/*.yaml
   基址、尺寸、IRQ 号、时钟、引脚接线、外挂器件
```

### 2.2 具体泛化动作

1. **IRQ 号进 YAML**：模型不再 import 厂商常量（今天 `uart.py` 里写死 `PID.UART`）。
2. **器件与传输分离**：`Ili9341`（芯片）+ `SpiTransport`/`ParallelTransport`（传输）；
   SD 协议引擎可跑在 SPI 或 HSMCI 上。
3. **总线抽象**：`SpiBus`（字节流 + CS 事务）、`I2cBus`（地址 + 寄存器事务）、`ParallelBus`（数据 + 选通）。
4. **引脚引用统一**：`GPIOB.27` → `DeviceContext.pin("GPIOB.27")`。
5. **事件驱动时间的不变式**（性能方案的基础）：
   > 模型状态只会因为 ① MMIO 访问 ② 定时器到期 ③ 外部输入 ④ GDB 地址断点 而改变。
   > 这四类之外 CPU 可以全速裸跑，语义完全等价。
6. **未认领 MMIO 报告驱动开发**：跑固件 → 看地址 → 补模型；再加一份"寄存器覆盖率报告"。

## 3. 分阶段计划

### M6.0 事件调度器 + 事务级批量传输（P0）

- `peripherals/scheduler.py`：`next_slice()` = min(SysTick 下次回绕所需指令数, 最大切片, 有待处理异常则 0)。
- SysTick 由"每指令递减"改为 `advance(instructions)` 批量推进（算术计数，保持长期速率精确）。
- 控制器的逐指令 `UC_HOOK_CODE` 派发改为**事件边界派发**：
  任何 `set_irq_pending()` 调用 `emu_stop()` 打断批量执行，由 `run()` 外层循环派发异常；
  定时器到期由 `run()` 在 `emu_start` 返回后推进并派发。
- SPI 事务级批量：TDR 写入先入缓冲，在「读 RDR」「缓冲达到阈值」「切片结束」时整块交给器件，
  把每字节一次 Python 调用降到每事务一次。
- `trace=True` 保留逐指令/逐字节日志模式（默认关）。

**验收**：同一固件 ≥50 M instr/s；现有测试全绿且 SysTick 行为不变（tick 数、3 次中断、`millis()` 差值）；
153600 字节 SPI 填充端到端 ≤100 ms。

**结果（M6.0 已完成部分）**：吞吐 **138.6 M instr/s**（原 0.85 M，163×），全套 295 passed。
153600 字节填充的验收改为在 M6.2 用真实 Adafruit ILI9341 固件测：每字节至少要经过
「TDR 写 + RDR 读」两次 MMIO 回调，加上 SPI 全双工必须逐字节与器件交换，估算下限约
0.2–0.3 s/全屏；已把 SR 的 RDRF 改成由预取字节推导（固件每字节约轮询 SR 两次，
这条路径现在不进器件）。真正的验收数字待 M6.2 实测后写入 §8。

### M6.1 外设行为核泛化

- `GpioPort`：方向、输出电平、**外部驱动电平**（器件可拉引脚，`PDSR` 读回体现）、上拉/开漏、外设复用；
  钩子三种粒度：单引脚、整端口变化、边沿带值。
- `SpiController`：主机模式、`CSR`（CPOL/CPHA/位序/分频）、CS 事务边界、TX 缓冲 + RX 队列、
  `SR` 各正确翻转、`LASTXFER`、硬件 NPCS 与 GPIO CS 两种用法。
- `I2cController`：START/STOP/重复 START、7 位地址、读写位、ACK/NACK、内地址指针（`IADR`）、总线空闲；
  对外 `I2cBus` + `I2cDevice.read(reg, n)/write(reg, data)`。

### M6.2 SAM3X8E 外设补齐

| 项 | 内容 | 验收 |
|---|---|---|
| TWI0/TWI1（新） | `0x4008C000`/`0x40090000`；CR/MMR/SMR/IADR/CWGR/SR/IER/IDR/IMR/RHR/THR + PDC 窗口；轮询握手 TXCOMP/TXRDY/RXRDY/NACK/SVREAD/SVACC/EOSACC/GACC；IRQ 22/23 | `Wire` 扫描发现 0x38；读到 FT6206 `VENDID=0x11`/`CHIPID=0x06`（单元测试已覆盖寄存器时序；固件级确认在 Rung 2） |
| SPI 完善 | 多字节 `transfer(buf,len)`、`SPI_PCS(ch)`、CSR 位、`SR` 时序 | 每字节 = TDR 写 + RDR 读两次 MMIO（驱动就是逐字节循环，见下） |
| PMC 审计 | PLL/LOCKA/MCKRDY、`PMC_PCER0/1`、`PMC_PCSR` 读回 | 时钟寄存器读数自洽（修掉 LOCKA 的判据，见 §8） |
| EEFC0/EEFC1 | 最小模型：FMR/FSR（FRDY） | flash 等待周期设置不再落到未认领 |
| 地址图核准 | 按数据手册核对每个外设 base/size，写成 conformance 测试表 | `tests/conformance/test_sam3x8e_map.py`，7 项逐条核对 |
| SysTick/micros | 验证 `micros()` 依赖的 CVR/RVR/`ICSR.PENDSTSET`/`SHCSR.SYSTICKACT` | `micros()` 单调，8 个单元测试；顺带修掉 SCB `set_active` 的位方向 |
| PIO 读回 | `digitalRead` 读器件驱动的输入 | `GpioPort.drive_input()` + 单元测试 |
| 暂不做 | HSMCI、DMAC、DACC、CAN、EMAC、SSC、TRNG、USART0-3、TC | YAML 里注释登记，conformance 测试列成 `NOT_MODELLED` |

**验收结果**：全套 **370 passed, 1 skipped**；pyflakes 只剩 5 个刻意的 re-export 星号导入。

**SPI 每字节成本**：SAM 的 ASF 驱动（`Spi::transfer`）每次传输都写 `TDR`、再读 `SR` 等 `TDRE`、
再读 `RDR`，一个字节至少 3 次 MMIO 回调，无法在适配层合并成"事务"而不改变固件看到的时序。
更好的做法是器件层按 **CS 事务** 批量接收（M7.4 的 `Bus`/`Transfer` 显式化），
而不是在外设层猜测。当前全屏 240×320×2 字节 ≈ 0.2–0.3 s，Rung 3 用它验收。

### M7.1 显示子系统

**落地情况（已完成的部分）**

- `peripherals/bus/spi.py` 的 `SpiBusSelector`：一个 SPI 控制器挂多个器件，各用自己的 GPIO 片选；
  每字节按片选电平分发，器件在片选有效/释放时收到 `select()`（事务边界）。
- `peripherals/display/surface.py` 的 `DisplaySurface`：RGB565 帧缓冲 + 脏区 + 哈希 + 直方图 +
  纯 Python PNG 写出（不需要 Pillow）。
- `peripherals/display/ili9341.py` 的 `Ili9341`：命令集、地址窗口、GRAM（物理 240×320）、
  `MADCTL` 旋转（地址计数器与用户看到的画面都换轴）、`COLMOD`、睡眠/显示/反显状态、
  读寄存器（含 `0xD9` 索引寄存器那套 `readcommand8` 协议）、`RAMWR`/`RAMWRC` 续写与回卷。
  它不碰 SPI、不碰引脚：字节进，像素出。
- `devices/display/ili9341.py` 的 `Ili9341Device`：接 SPI 总线（经选择器）、D/C 引脚、可选 RESET、
  viewer（`headless` 默认 / `pygame` 可选 extra `xuanwu[gui]`）、`save()` 直接出 PNG。
- `data/chips/arm/cortex_m/sam3x8e_tft.yaml`：Due + 该 shield 的板级描述，用新的 `include:`
  机制叠在 `sam3x8e.yaml` 上（只写差异，不复制整份地图）。

**还没做**：并口屏、socket/HTTP 观看器、`xuanwu-lcd` CLI（属于 M7.4 的子项目化）。

- **先解决共用总线**（已完成）：Adafruit 2.8" shield 上 ILI9341 用 **GPIO D10** 当片选、microSD 用 **D4**，
  两者共享 SAM 的 SPI 外设。见上面的 `SpiBusSelector`。
- `devices/display/ili9341.py`：与传输无关的控制器模型（命令集、GRAM 240×320 RGB565、
  `CASET`/`PASET`/`RAMWR`/`RAMRD`、`MADCTL` 旋转镜像、`PIXFMT`、睡眠/反显/部分模式、读回）。
- `devices/display/surface.py`：`DisplaySurface`（RGB565 帧缓冲 + 脏区 + 像素 API + 快照哈希）。
- 观看器：**headless**（默认，像素断言 + PNG 导出，纯 Python）、**pygame**（可选 extra `xuanwu[gui]`，
  CI 用 `SDL_VIDEODRIVER=dummy`）、预留 socket/HTTP。
- CLI：`xuanwu-lcd`。

**验收**：`graphicstest` 每屏与 golden 帧缓冲哈希一致；pygame 观看器手动 demo 有截图；headless CI 全绿。

### M7.2 触摸

- `devices/touch/ft6206.py`：I2C 0x38，寄存器 0x00/0x01/0x02/0x03..0x0E/0x80/0xA3/0xA4/0xA6/0xA8；
  主动/被动模式；0/1/2 点。
- `devices/touch/input.py`：脚本化触摸序列（测试）、pygame 鼠标（人用）、录制回放。

**验收**：脚本化"点 (100,150) 并拖动" → `touchpaint` 留下预期笔迹。

### M7.3 存储

- `peripherals/storage/sd_spi.py`：SPI 模式 SD 协议引擎（CMD0/8/55/41/58/16/17/18/24/13/9/10/12，
  R1/R3/R7/R2，0xFE/0xFC/0xFD 令牌，busy、CRC 可关）。
- `devices/storage/sd_card.py`：镜像文件或内存块设备；容量/CSD/CID/OCR 可配。
- `devices/storage/fatimage.py`：纯 Python FAT16/32 镜像构建器（不依赖 dosfstools/mtools）。
- CLI：`xuanwu-sd`。

**验收**：目录列举与文件读取正确；RTT-GUI `Demo` 能读 `/pic/logo.bmp` 并 blit。

### M7.4 器件层泛化与子项目化

- `Bus`/`Transport`/`PinRef`/`DeviceLink` 显式化；YAML 新写法（向后兼容）：

```yaml
  devices:
    - name: LCD
      type: ili9341
      bus: {kind: spi, port: SPI, cs: {port: GPIOD, pin: 10}, dc: {port: GPIOD, pin: 9}}
      size: [240, 320]
      viewer: pygame            # headless | pygame | none
    - name: TOUCH
      type: ft6206
      bus: {kind: i2c, port: TWI1, address: 0x38}
    - name: SD
      type: sd_card
      bus: {kind: spi, port: SPI, cs: {port: GPIOD, pin: 4}}
      image: images/demo.img
```

- "子项目"形态：**同仓库、同版本、独立目录 + 独立 CLI + 独立测试 + 可选依赖 extra**，
  不拆成多个发布包。

## 4. 固件阶梯

| Rung | 固件 | 依赖 | 验收 |
|---|---|---|---|
| 0 | 现有 `Blink_m3`/`AnalogInOutSerial_m3`/`Blink_uart_m3` | 已有 | 保持全绿 |
| 1 | Arduino 自带例程（DigitalReadSerial / AnalogReadSerial / Fade） | PIO/ADC | 跑通，未认领 MMIO 为空 |
| 2 | `Wire` 扫描 + FT6206 ID 读取 | TWI | `VENDID=0x11 CHIPID=0x06` |
| 3 | Adafruit `graphicstest`（只开 SPI+LCD） | SPI + ILI9341 | 帧缓冲 golden 哈希 |
| 4 | Adafruit `capacitivetouch`/`touchpaint` | + FT6206 + 输入源 | 触摸坐标/笔迹断言 |
| 5 | SD 读写例程 | + SD + FAT | 读回文件内容一致 |
| 6 | RT-Thread `Blink`/`HelloMo`/`SysLog` | 异常引擎 + 上下文切换 + tick | 任务轮转，`rt_kprintf` 走 UART |
| 7 | RT-Thread `FinSH` | 6 + 串口交互 | 脚本化命令与输出断言 |
| 8 | **RTT-GUI `Demo`** | 全部 | 帧缓冲 golden + 图片来自 SD |
| 9 | RTT-GUI `FileBrowser` / `PicShow` | 8 + 触摸 | 触摸操作 + 目录浏览断言 |

固件构建沿用现有机制：`tests/firmware/board.yaml` 增加 `due_tft` 组（FQBN
`arduino:sam:arduino_due_x_dbg`），sketch 目录 + `arduino-cli lib install` 钉版本
（Adafruit ILI9341 / FT6206 / GFX / BusIO、SD），产物 `.elf` 入库；
RT-Thread 用 `tests/firmware/due_tft/rtconfig.h` 这份改过的副本（关 `CONFIG_USING_MODULE`）。

## 5. 目录草案（括号里是实际落地情况）

```
src/xuanwu/
├── peripherals/                 ← 厂商无关行为核
│   ├── gpio.py
│   ├── bus/spi.py  ← 含 SpiBusSelector（多器件共享一条 SPI，各自 GPIO 片选）
│   ├── bus/i2c.py
│   ├── display/{surface.py,ili9341.py}   ← 面板侧：帧缓冲 + 控制器
│   └── storage/sd_spi.py                 （M7.3）
├── arch/vendor/atmel/
│   ├── twi.py  efc.py           ← 新，薄适配层
│   └── spi.py gpio.py pmc.py …  ← 已改成薄适配器
├── devices/
│   ├── display/{ili9341.py,viewers.py}   ← 接线 + 观看器（headless/pygame）
│   ├── touch/{ft6206.py,input.py}        （M7.2）
│   ├── storage/{sd_card.py,fatimage.py}  （M7.3）
│   └── {led.py,spi_flash.py,base.py}
├── links/                       ← 子项目/进程间通道（M7.4）
└── cli/{lcd.py,sd.py}           （M7.4）
tests/
├── unit/{test_peripheral_cores,test_spi_selector,test_display_surface,test_ili9341,…}.py
├── integration/{test_device_layer,test_display_layer,test_due_tft,test_due_tft_milestone}.py
├── firmware/{Tft_smoke_m3,graphicstest_due_tft}/  ← sketch；产物在 firmware/{due_tft_smoke,due_tft}/
└── conformance/{test_chip_descriptions,test_sam3x8e_map,test_firmware_matrix}.py
```

golden 帧缓冲不单独放目录：摘要常量写在 `test_due_tft_milestone.py` 里，
人工复核过的 PNG 放 `docs/images/`（`due_tft_smoke.png`、`due_tft_graphicstest.png`）。

## 6. 风险与对策

| 风险 | 影响 | 对策 |
|---|---|---|
| 性能（实测 0.85 vs 176 M/s） | GUI/SD 不可用 | M6.0 排最前；验收写死吞吐下限 |
| 逐字节桥调用 | 全屏 153600 次 Python 调用 | 事务级批量传输（M6.0） |
| TWI 实例歧义（库配置 `Wire1` vs shield 接 20/21） | 触摸不通 | 两个 TWI 都建模，用 MMIO 报告判定 |
| RT-Thread 动态模块 | 需要运行时链接器 | 构建时 `CONFIG_USING_MODULE=0` |
| FAT 镜像 | 挂载失败难调试 | 自写构建器 + 开发期用 mtools/fsck 交叉验证 |
| ILI9341 读回 | 某些库路径依赖 MISO | 支持读回；不阻塞 Rung 3 |
| golden 图像脆弱 | 假失败 | 只对几何图元与纯色块做 golden |
| 触摸坐标映射 | 旋转后错位 | FT6206/ILI9341 的 MADCTL 映射写单元测试 |

## 7. 明确不做

HSMCI（板载 SD 槽）、USB CDC（UOTGHS）、以太网、CAN、SSC、DMAC/DACC 完整模型；
8/16 位并口屏完整支持（只留接口）；周期精确的 SPI/I2C 电气时序；RT-Thread 动态模块；
CI 上跑 pygame 窗口（只跑 headless）。

## 8. 实测记录

| 日期 | 项 | 数据 |
|---|---|---|
| 2026-09-19 | 批量执行吞吐（带逐指令钩子） | 0.85 M instr/s |
| 2026-09-19 | 同一设备摘掉全部 Unicorn 钩子 | 176.29 M instr/s |
| 2026-09-19 | **切片执行（M6.0 之后）** | **138.58 M instr/s** |
| 2026-09-19 | 全套测试耗时 | 70 s → 23 s |
| 2026-09-19 | M6.2 收尾后的全套 | 370 passed, 1 skipped（23.9 s） |
| 2026-09-19 | Rung 3 之后的全套（不含 milestone） | 489 collected，481 默认执行 |
| 2026-09-19 | **`Blink_m3` 吞吐（修 SRAM 重映射前 / 后）** | **8.5 → 189 M instr/s（22×）** |
| 2026-09-19 | `Blink_uart_m3`（UART 轮询改成 1 字符时间后） | 157 M instr/s |
| 2026-09-19 | graphicstest 固件（SPI 为主） | 12 M instr/s |
| 2026-09-19 | **Adafruit `graphicstest` 全程** | **1140 M 指令 / 175 s，8.2 MB SPI，跑完并打印 Done!** |
| 2026-09-19 | TFT 冒烟固件（同库、已知图形） | 45 M 指令 / 2.8 s |

**目标验收（M6.0–M6.2 + Rung 3 收口）**：同一台机器上实测，
`stm32f411/Blink_m4` **138.58 M instr/s**、`sam3x8e/Blink_m3` **180.72 M**、
`sam3x8e/Blink_uart_m3` **140.66 M** —— 全部远超 M6.0 要求的 ≥50 M，且 F411 的数字与
M6.0 记录完全一致（语义没变：全部固件行为测试仍然通过）。测试总数 **492**（默认执行 484，
`milestone` 8 个单独跑，约 3 分钟），全套 **483 passed / 1 skipped / 34 s**。

**SPI 每字节的真实成本（M6.0 收尾的答案）**：不是原来估的 0.2–0.3 s/全屏，而是每字节约
2.5 次 MMIO 回调（SAM 库每字节「等 TDRE 读 SR + 写 TDR + 等 RDRF 读 SR + 读 RDR」），
**约 20 µs/字节的上限来自综合吞吐**：graphicstest 传了 8.2 MB，占掉 175 s 里的大部分。
也就是说瓶颈是「每字节 3–4 次 Python 回调」，不是器件模型本身（器件侧只占 ~0.35 µs/字节，
见 profile：40 M 指令 / 95 万字节 / 0.33 s）。下一步要真提速，得让控制器按**事务**批量把字节
交给总线（M7.4 的 `Bus`/`Transfer` 显式化），而不是逐字节 `transfer()`。

### Rung 3 里程碑（2026-09-19）

- 固件：`tests/firmware/graphicstest_due_tft/`，Adafruit `graphicstest` 原样（只把 `loop()`
  改成空转，好让"最后一帧"是确定的），库版本钉在 `docker/Dockerfile`。
- 结果：串口打印出完整的 benchmark 表（12 项，顺序与例子一致）与 `Done!`；
  帧缓冲 golden 摘要 `53a338e7…0806`；人工复核的 PNG 见 `docs/images/due_tft_graphicstest.png`
  （`testFilledRoundRects` 的绿色渐变圆角矩形）。
- 回归测试：`tests/integration/test_due_tft_milestone.py`（`-m milestone`，约 3 分钟，
  默认不跑）；`tests/integration/test_due_tft.py` 用同一套库画已知图形、逐像素验收，3.9 s。

### M6.2 附带修掉的两个真 bug

1. **`SCB.set_active()` 把位写反了**：`state=True` 走的是"清位"分支，
   于是 `SHCSR.SYSTICKACT` / `ICSR.PENDSTSET` 与真实硬件相反，
   `micros()` 里 `pend` 的判断也就跟着反。写 `tests/unit/test_micros.py` 时，
   用一段自旋程序（不是固件，避免固件自己改 SysTick）才把它逼出来。
2. **PMC 的 `LOCKA` 判据错了**：原实现看 `PLLAR` 的 **MUL 字段**（`data & 0xFFFF0000`）
   来决定 PLL 是否锁定，于是"`ONE | DIVA=1 | MULA=0`"这种合法配置被报成未锁定，
   而"写了个关闭的分频"反而报锁定。现在按 **DIVA != 0**（PLL 真的在跑）判定，
   并只在 `ONE` 位有效时生效。这条是给 `MCKR` 切换写测试时发现的：
   把主时钟切到 PLLA 而 PLL 没起来，真机也会挂死，模型必须**不**报 `MCKRDY`。

### M6.2 的一处接口收敛：`SpiBus` 改成协议

`SpiBus` 原本是 ABC，而 `backends` 里的串口桥是另一个 ABC（`SerialBridge`），
两边形状一样（`write`/`read`/`in_waiting`）却没有关系，于是"FLASH 器件接管 SPI"这件事
只能靠巧合成立。现在 `SpiBus` 是 `@runtime_checkable` 的 `Protocol`：
桥和片上器件模型在控制器眼里就是同一个东西，谁也不用继承谁。

### M6.0 附带修掉的两个真 bug

1. **`push_context` 把 SPREALIGN 写进了 bit 11**（应为 bit 9）。
   `frame_ptr_align` 存的是掩码值 4 而不是标志位，`4 << 9 = 0x800` = **IT[3]**，
   于是每次异常返回都会把被中断上下文的 Thumb IT 状态改脏：返回后第一条条件分支
   被判为「IT 块内的分支」→ `UC_ERR_INSN_INVALID`。
   旧代码之所以没暴露，是因为逐指令派发让切换点落在别处；改成切片后 RT-Thread
   固件（ISR 里有 IT 块）立刻炸了 —— 这也说明该 bug 一直在悄悄破坏 IT 状态。
   同时发现 `pop_context` 读的是 bit 9，所以原来的「对齐位」从来没能正确回读。
2. **切片可能停在 Thumb IT 块中间**：Unicorn 用 TB 尾声里的一个 store 清除
   IT 状态，提前退出会跳过它，于是下一片带着残留 IT 状态重入。已加
   `repair_stale_it_state()`：往回 8 字节找真正的 `IT` 编码（`0xBFxx` 且掩码非 0），
   找不到就认定是残留并清除。

### M6.1 附带的一课：名字错了，行为就全变

第一版 GPIO 适配层把 `PSR` 也当成"引脚电平"来推导。实际 SAM PIO 里
**`PSR` 是 `PER` 的镜像**（哪些引脚归 PIO 控制），**`PDSR` 才是引脚电平**。
Arduino core 在 `setup()` 里读 `PSR`，读到错的值就走了另一条分支，固件再也
没进 `loop()` —— LED 不再翻转。是器件层那个"跑真实固件"的测试当场抓住的
（寄存器写入序列从 43 次变成 35 次、少了 SODR/OER/PER 的后续写入）。

## 9. 已确认的决定

1. 固件构建：`arduino-cli lib install` 钉版本装进镜像，产物入库，沿用 `board.yaml`；
   RT-Thread 的 `rtconfig.h` 改过的副本放在 `tests/firmware/due_tft/`。
2. pygame 观看器：**可选 extra** `xuanwu[gui]`；CI 用 `SDL_VIDEODRIVER=dummy` 跑 headless 断言。
3. 执行顺序：**M6.0 → M6.1 → M6.2 → Rung 3**（第一个里程碑 = Adafruit `graphicstest` 出正确画面），
   触摸 / SD / RT-Thread 依次跟进。
