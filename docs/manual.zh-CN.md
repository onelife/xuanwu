# xuanwu 使用手册

本手册面向「想用它跑固件、调外设、加芯片」的人。文中的每条命令和每段输出都在项目的开发容器里实际跑过。

- [0. 30 秒了解 xuanwu](#0-30-秒了解-xuanwu)
- [1. 安装与环境](#1-安装与环境)
- [2. 第一个例子：把一个固件跑起来](#2-第一个例子把一个固件跑起来)
- [3. 核心 API](#3-核心-api)
- [4. 串口：把 UART 接到主机](#4-串口把-uart-接到主机)
- [5. 用 GDB 调试](#5-用-gdb-调试)
- [6. semihosting：让 firmware 直接 printf](#6-semihosting让-firmware-直接-printf)
- [7. 浮点（FPU）](#7-浮点fpu)
- [8. 外部器件（LED、SPI Flash）](#8-外部器件ledspi-flash)
- [9. 构建测试固件（Arduino CLI）](#9-构建测试固件arduino-cli)
- [10. 跑测试](#10-跑测试)
- [11. 加一颗新芯片](#11-加一颗新芯片)
- [12. 排错 FAQ](#12-排错-faq)
- [13. 附录](#13-附录)

---

## 0. 30 秒了解 xuanwu

xuanwu 是一个 **MCU 模拟器**：CPU 交给 [Unicorn](https://github.com/unicorn-engine/unicorn)，反汇编交给 [Capstone](https://www.capstone-engine.org/)，其余的——内存映射、外设寄存器模型、Cortex-M 异常/中断引擎、GDB stub、串口桥、semihosting、外部器件——都是这个项目自己实现的。

它**不是** QEMU 那样的全系统模拟器，也**不是**周期精确的仿真器。它的目标是：

| 能做 | 做不到 |
|---|---|
| 直接跑真实的 Arduino/CMSIS 固件（`.elf`/`.hex`/`.bin`） | 周期精确的时序（SysTick 按指令数近似，见 7.1） |
| 用 YAML 描述一颗新芯片的内存图和寄存器块 | 模拟芯片的模拟部分（ADC 噪声、USB 协议栈） |
| 用 Python 直接读写外设寄存器、观察固件行为 | 门级/总线级时序（AHB/APB 等待周期） |
| 用 `gdb-multiarch` 打断点、单步、看寄存器 | `CPACR` 控制 FPU 使能（Unicorn 限制） |
| 把 UART/SPI 接到主机（socat pty / TCP / 内存回环） | 多核、Cache、MPU |

一次典型会话长这样：

```console
$ python examples/blinky.py
chip      : stm32f411
firmware  : Blink_m4.ino.elf
reset     : pc=0x08001cd4  msp=0x20020000
...内存映射表...
after 500,000 instructions: pc=0x08001e74  sp=0x2001ffe0
```

---

## 1. 安装与环境

### 1.1 用开发容器（推荐）

镜像里已经装好了 Python 3.12、unicorn/capstone、`socat`、`gdb-multiarch`、`pytest`/`pyflakes`，以及 `arduino-cli`（含 Arduino Due / MKR Zero / Nucleo-F767ZI / 通用 STM32F4 的 core 与工具链），开箱即可跑测试和构建固件。

```bash
# 在仓库根目录执行（仓库根 = 本 README 所在目录）
docker compose -f docker/docker-compose.yml up -d --build
docker exec -it xuanwu_container bash
```

进入容器后，工作目录是 `/home/onelife/xuanwu`，仓库在 `/home/onelife/xuanwu/xuanwu`（也就是宿主机的仓库，直接挂载进来的）：

```bash
cd /home/onelife/xuanwu/xuanwu
python -m pytest tests -q            # 全部测试
python examples/blinky.py            # 第一个例子
```

> 镜像里没有系统的 `arm-none-eabi-gcc`：固件由各 Arduino core 自带的工具链编译（见第 9 节），手写的裸机固件也会自动找到那份工具链。

### 1.2 直接安装到本机

```bash
pip install .                        # 只装运行时依赖
pip install -r requirements-dev.txt  # 加上 pytest / pyflakes / setuptools / wheel
```

> 最后两个不是可选项：测试套件会直接执行 `setup.py` 和 `pip wheel`（`tests/unit/test_packaging.py`），而 `python:3.12` 之类的干净镜像里没有 `setuptools`——`pip install .` 是在隔离环境里构建的，不会把它装进 site-packages。

| 依赖 | 用途 | 版本要求 |
|---|---|---|
| `unicorn` | CPU 模拟（执行指令、内存钩子、异常回调） | `>= 2.1` |
| `capstone` | 反汇编（`show_inst` 与调试输出） | `>= 5.0` |
| `pyyaml` | 读取芯片描述与固件清单 | — |
| `pyelftools` | 加载 ELF（可选） | — |
| Python | — | 3.9+ |

Windows/macOS 上不需要 `socat`：串口桥默认会退化成 TCP（见第 4 节）。

### 1.3 确认装好了

```python
import sys; sys.path.insert(0, "src")   # 从源码树运行时
from xuanwu.chips import list_chips
print(list_chips())
```

```console
['sam3x8e', 'stm32f411']
```

`list_chips()` 列出的是**随包发布**的芯片描述；也可以直接给 YAML 路径。

---

## 2. 第一个例子：把一个固件跑起来

```bash
python examples/blinky.py                 # STM32F411 + Blink_m4
python examples/blinky.py sampled         # SAM3X8E（需要 socat，或用 TCP 桥）
python examples/blinky.py blinky 2000000  # 指定执行的指令条数
```

输出（节选，完整输出里还有一张内存映射表）：

```console
chip      : stm32f411
firmware  : Blink_m4.ino.elf
reset     : pc=0x08001cd4  msp=0x20020000

Start        End          Permission   Description
0x08000000 ~ 0x08080000   rwx          [Flash: Main                     ]
0x20000000 ~ 0x20020000   rwx          [SRAM                            ]
0x40000000 ~ 0x40100000   rw-          [PERIF_BB (<= 0x42000000)        ]
0xe0000000 ~ 0xe0010000   rw-          [PPB (IO)                        ]
...
after 500,000 instructions: pc=0x08001e74  sp=0x2001ffe0
```

自己写一份等价的脚本只要十来行：

```python
import sys
sys.path.insert(0, "src")
from xuanwu import XuanWu

device = XuanWu("stm32f411", "tests/firmware/stm32f411/Blink_m4.ino.elf")
device.reset()                                  # 从向量表取 msp/pc，复位所有外设
print(f"reset: pc=0x{device.reg.pc:08x} msp=0x{device.reg.msp:08x}")

device.run(count=100_000)                       # 执行 10 万条指令
print(f"after 100000 instructions: pc=0x{device.reg.pc:08x}")
print("r0..r3:", [f"0x{device.reg.read(n):08x}" for n in ("r0", "r1", "r2", "r3")])
print("systick:", f"{device.hw.perif['systick'].elapsed_ms:.3f} ms")
```

```console
reset: pc=0x08001cd4 msp=0x20020000
after 100000 instructions: pc=0x080013ac
r0..r3: ['0x00000001', '0x00000001', '0x40020800', '0x00000001']
systick: 1.000 ms
```

---

## 3. 核心 API

### 3.1 构造 `XuanWu`

```python
from xuanwu import XuanWu

device = XuanWu(
    chip,                      # 芯片名（"stm32f411"）或 YAML 路径
    code,                      # 固件：.elf / .hex / .bin
    rsp=False,                 # True 或端口号：启动 GDB stub
    hardware_options=None,     # 覆盖各 core 外设的构造参数，如 {"bridge": "tcp"}
    semihosting=True,          # True / False / SemiHosting 实例
)
```

| 参数 | 说明 |
|---|---|
| `chip` | 随包芯片名，或指向 `*.yaml` 的路径 |
| `code` | 固件路径；ELF 会按 LMA/VMA 搬运 `.data` |
| `rsp` | `True` 用默认端口 6666，给整数则用该端口；`run()` 会阻塞在 GDB 服务上 |
| `hardware_options` | 透传给每个 `type: core` 外设的构造参数，常用于换串口桥 |
| `semihosting` | `True` 处理 `BKPT 0xAB`；传 `SemiHosting(output=...)` 可捕获输出；`False` 则当作普通断点 |

### 3.2 `device` 上有什么

| 属性 | 类型 | 用途 |
|---|---|---|
| `device.box` | `unicorn.Uc` | 底层 Unicorn 实例（加自己的钩子时用） |
| `device.dasm` | `capstone.Cs` | 反汇编器 |
| `device.reg` | `RegisterController` | 按名字读写寄存器：`read("r0")`、`write("pc", ...)`、`pc_t` |
| `device.mem` | `MemoryController` | `read/read_word/write`、`show_map()`、`unclaimed_accesses()` |
| `device.hw` | `ArmHardwareController` | `hw.perif["gpio"]`… 异常与中断引擎 |
| `device.dev` | `DeviceController` | 外部器件：`dev["LED"]`、`dev.names()` |
| `device._chip` | `dict` | 载入后的芯片描述 |

### 3.3 `reset()` 与 `run()`

```python
device.reset()                    # 复位外设 + 从 0x0/0x4 取 msp/pc
device.run(count=1000)            # 执行 1000 条指令
device.run(until=0x08001234)      # 一直跑到某个地址
device.run()                      # 不限条数（配 rsp 时表示「服务 GDB，阻塞」）
```

`count` 是**指令条数**（不是毫秒），`count=0` 表示不限。只给 `until` 时，Unicorn 会在 PC 命中该地址时停下——**在目标地址上的那条指令执行之前**。

### 3.4 读写内存、寄存器、看反汇编

```python
device.reg.write("r0", 0x1234)
device.reg.read("r0")             # 0x1234
device.reg.read("control")        # CONTROL 寄存器
device.reg.read("s0")             # 浮点寄存器（S0-S31 / D0-D15 / Q0-Q15 都可按名访问）

device.mem.read_word(0x20000000)          # 读一个字（走 MMIO 路由）
device.mem.read(0x20000000, 16)           # 读 16 字节
device.mem.write(0x2001F000, b"\x00\xbf") # 写字节串

device.show_inst(-2, 4)                   # 以当前 PC 为中心反汇编并打印读写到的寄存器
```

`show_inst` 的输出会带 `[R]`/`[M]` 标注（read/modify）的寄存器值，定位「这条指令到底动了什么」很快。

### 3.5 未认领的 MMIO：固件卡住时先看它

固件访问了某个外设寄存器，但没有任何模型认领它时，模拟器**不会报错**，只是悄悄读写后备缓冲区。这类访问会被记下来：

```bash
python examples/unclaimed_io.py
```

```console
Address      Size    Count
0x40007000   4       3
0x40023c00   1       1
0x40023c00   4       9
3 unclaimed address(es)

after 200,000 instructions, grouped for a work list:
  0x40007000  4 byte(s)  x3      TIM
  0x40023c00  1 byte(s)  x1      FLASH interface
  0x40023c00  4 byte(s)  x9      FLASH interface
```

拿到地址，对照参考手册，然后照着第 11 节把它补上即可。这就是「为什么固件跑不起来」的第一诊断手段。

---

## 4. 串口：把 UART 接到主机

模拟的 UART/SPI 通过 **bridge（桥）** 暴露给主机。选择方式有两种：芯片 YAML 里的 `bridge:` 字段，或运行时用 `hardware_options={"bridge": ...}` 覆盖。

| `bridge` | 行为 | 适用 |
|---|---|---|
| `auto`（默认） | 有 `socat` 就用 `socat`，否则用 `tcp` | 想少配置 |
| `socat` | 建一对 pty，主机端就是 `peer_hint` 那个 `/dev/pts/N` | Linux/macOS，想用 `screen`/`cat` |
| `tcp` | 模拟器监听 `tcp://host:port`，任意程序连上来 | 跨平台、跨机器 |
| `loopback` | 纯内存，不占用任何主机资源 | 测试 |
| `none` | 没有主机端（给器件层接管，如 SPI Flash） | 器件层 |

### 4.1 完整例子：读固件打印的内容

```bash
python examples/uart_bridge.py
```

```console
UART bridge : tcp://127.0.0.1:43731
--- captured from the simulated serial port ---
millis0 = 1
millis1 = 11
millis0 = 11
millis1 = 21
millis0 = 21
--- millis() advanced by 10 ms across two delay(5) calls ---
```

它做的事就三步（完整代码见 `examples/uart_bridge.py`）：

```python
device = XuanWu("sam3x8e", str(FIRMWARE), hardware_options={"bridge": "tcp"})
device.reset()

hint = device.hw.perif["uart"].peer_hint     # 'tcp://127.0.0.1:43731'
host, port = hint[len("tcp://"):].rsplit(":", 1)

with socket.create_connection((host, int(port)), timeout=5) as peer:
    peer.settimeout(0.2)
    for _ in range(...):
        device.run(count=2_000_000)          # 让固件跑一会儿
        chunk = peer.recv(4096)              # 再把它写出来的字节收走
```

关键点：**模拟与收数据是交替进行的**。`device.run()` 是阻塞的，所以要么像上面这样切片执行，要么把 `device.run()` 放到线程里（`tests/integration/test_gdb_stub.py` 里有现成写法）。

### 4.2 用 socat 手工接上去

```python
device = XuanWu("sam3x8e", fw, hardware_options={"bridge": "socat"})
print(device.hw.perif["uart"].peer_hint)     # /dev/pts/5
```

另开一个终端：`cat /dev/pts/5`（或 `screen /dev/pts/5 115200`）就能看到固件输出。波特率在 YAML 的 `baudrate:` 里，模拟器不做时序限速。

### 4.3 测试里用 loopback

```python
device = XuanWu("sam3x8e", fw, hardware_options={"bridge": "loopback"})
bridge = device.hw.perif["uart"]._bridge
bridge.feed(b"Z")                            # 假装主机发来一个字节
device.run(count=200_000)                    # 固件轮询并读走它
assert bridge.in_waiting == 0
```

---

## 5. 用 GDB 调试

### 5.1 启动

```python
device = XuanWu("stm32f411", "firmware.elf", rsp=6666)   # 或 rsp=True（默认 6666）
device.reset()
device.run()                                            # 阻塞，服务 GDB
```

```console
$ gdb-multiarch -q firmware.elf
(gdb) target remote 127.0.0.1:6666
```

stub 会通过 `qXfer:features:read` 下发 Cortex-M 的 target description，所以**不必先告诉 GDB 架构**；带上 ELF 只是为了符号。

### 5.2 一个真实会话

```
(gdb) set pagination off
(gdb) file tests/firmware/stm32f411/fpu_test/fpu_test.elf
(gdb) target remote 127.0.0.1:6666
main () at .../fpu_test.c:96
96      int main(void) {
(gdb) info registers r0 pc xpsr
r0             0x0                 0
pc             0x800011c           0x800011c <main>
xpsr           0x41000000          1090519040
(gdb) info registers d0 d15 fpscr
d0             3.5126997609638368e-303 (raw 0x012345673fc00000)
d15            0                   (raw 0x0000000000000000)
fpscr          0x0                 0
(gdb) p/x $d0
$1 = 0x12345673fc00000
```

芯片描述里声明了 `FPU` 时，target description 里会多出 `d0`-`d15` 和 `fpscr`，所以浮点寄存器可以直接看。没有 FPU 的芯片（如 `sam3x8e`）则只报 17 个核心寄存器。

### 5.3 支持的报文

| 报文 | 含义 |
|---|---|
| `g` / `p` / `P` | 读写（全部/单个）寄存器 |
| `m` / `X` | 读写内存 |
| `s` / `c` | 单步 / 继续 |
| `Z0`/`z0` | 软件断点（命中后模拟器停在被断指令之前） |
| `Z2`/`Z3` | 写/读观察点 |
| `qXfer:features:read` | target description（寄存器布局） |
| `QStartNoAckMode` / `D` / `vKill` | 杂项 |

> stub 一次只服务**一个**连接：如果已经有一个客户端连着，新的连接会排队。测试里要连两次时，先把第一个关掉。

---

## 6. semihosting：让 firmware 直接 printf

Arm semihosting 是「没有操作系统时的系统调用约定」：guest 执行 `BKPT 0xAB`，把请求码放 r0、参数放 r1，由调试器（这里是模拟器）代劳。于是**不需要 UART、不需要串口桥、不需要任何接线**，固件就能打印。

```bash
python examples/semihosting.py
```

```console
the guest printed: 'hello from semihosting\n'
pc after the trap: 0x2001f006 (the BKPT is behind us)
pc after the parked loop ran: 0x2001f006 (it branches to itself)
```

核心就三行：

```python
import io
from xuanwu.backends import SemiHosting

stream = io.StringIO()
device = XuanWu("stm32f411", fw, semihosting=SemiHosting(output=stream))
device.reset()
device.mem.write(0x2001F000, PROGRAM)      # 见 examples/semihosting.py 的 8 字节程序
device.reg.pc_t = 0x2001F000               # pc_t 会自动置上 Thumb 位
device.run(count=3)                        # movs, adr, bkpt —— 陷阱被处理并跳过
print(stream.getvalue())
```

已实现的操作：`SYS_WRITEC`、`SYS_WRITE0`、`SYS_WRITE`、`SYS_READC`、`SYS_ISTTY`、`SYS_CLOCK`、`SYS_TIME`、`SYS_ERRNO`、`SYS_ISERROR`、`SYS_FLEN`、`SYS_SEEK`、`SYS_TICKFREQ`、`SYS_ELAPSED`、`SYS_GET_CMDLINE`、`SYS_HEAPINFO`、`SYS_EXIT`、`SYS_EXIT_EXTENDED`。其它请求会记一条日志并返回 `-1`。

让自己的固件用上（GCC）：

```bash
arm-none-eabi-gcc --specs=rdimon.specs ... -lrdimon     # 半主机版 newlib
```

没有半主机库时，也可以自己写一行 `__asm__ volatile("bkpt 0xab")`。

> 只有 `BKPT 0xAB` 被当成半主机调用；别的 immediate 仍然是普通断点/故障，`semihosting=False` 可整体关闭。

---

## 7. 浮点（FPU）

### 7.1 打开 FPU

芯片描述里声明这一块即可：

```yaml
    - FPU:            # Cortex-M4F 的 FP 扩展寄存器
        type: core
        base: 0xE000EF30
        size: 0x1C
```

声明之后会有三件事生效：

1. `FPCCR`/`FPCAR`/`FPDSCR`/`MVFR0`/`MVFR1`/`MVFR2` 有模型（固件启动时常读它们）；
2. 异常入栈时，若 `CONTROL.FPCA` 已置位，会**额外压入 0x48 字节的浮点帧**（`S0`-`S15`+`FPSCR`+保留字），返回时对应弹出；
3. GDB target description 里多出 `d0`-`d15`/`fpscr`。

### 7.2 例子

```bash
python examples/fpu.py
```

```console
--- with the floating-point frame stacked
    magic=0x58575546  add=1 mul=1 cvt=1  interrupts=3
    MVFR0=0x10110021  MVFR1=0x11000011  FPCCR=0xc0000000
    values held in s0-s3: [('a', 1.5), ('b', 2.25), ('c', -3.75), ('d', 100.0)]
    simulated time: 0.20 ms over 20000 cycles
--- with stacking disabled (the values are destroyed)
    magic=0x58575546  add=1 mul=1 cvt=1  interrupts=3
    MVFR0=0x10110021  MVFR1=0x11000011  FPCCR=0xc0000000
    values held in s0-s3: [('a', 9.0), ('b', 9.0), ('c', 8.0), ('d', 8.0)]
```

第二段是**负对照**：故意不让异常压浮点帧，四个值立刻被中断处理函数留下的垃圾覆盖。这是 `tests/integration/test_fpu_stacking.py` 的判据——只有两段都对，才说明浮点帧真的在压栈/弹栈。

### 7.3 已知限制

- **`CPACR` 不 gate FPU**：Unicorn 无论 `CPACR` 怎么写都会执行 VFP 指令，也不看 CPU 型号（M0 也照跑）。它没有 VFP 指令级钩子，所以 `NOCP` UsageFault 无法低成本模拟。
- **不实现 lazy stacking**：`FPCCR.LSPACT` 恒读 0。

### 7.4 时间基准（SysTick）

`arch/cortex_m/systick.py` 是模型里唯一的时钟，由每次执行指令驱动：每条指令让计数减 `cycles_per_instruction`（默认 1，即「一条指令一个周期」的近似）。一个周期是 `RVR + 1` 个 cycle，`CALIB.TENMS` 由芯片 YAML 的 `clock` 推导：

```yaml
    - SYSTICK:
        type: core
        base: 0xE000E010
        size: 0x10
        clock: 100000000     # 100 MHz，STM32F411
```

`SysTick.cycles` / `.ticks` / `.elapsed_ms` 暴露仿真时间；测试应该断言它们，而不是墙钟时间。Unicorn 不报告指令耗时，所以这始终是近似值：`millis()` 走的是**仿真时间**。

---

## 8. 外部器件（LED、SPI Flash）

芯片描述里可以声明「板子上还接了什么」：

```yaml
  devices:
    - name: LED
      type: led
      port: GPIOB
      pin: 27
    - name: FLASH
      type: spi_flash
      port: SPI
      cs: {port: GPIOC, pin: 26, active_low: true}
```

`led` 观察一个 GPIO 引脚，`spi_flash` 则**接管 SPI 的字节流**（`peripheral.bridge = 自己`），所以 SPI 不再需要 socat/TCP。

```bash
python examples/devices.py
```

```console
devices declared by the chip description:
  LED
  FLASH

GPIOB.27 high -> LED lit: True, transitions: 1
GPIOB.27 low  -> LED lit: False, transitions: 2

flash JEDEC ID: ef4018 (Winbond W25Q128: ef4018)
flash size: 0x100000 bytes, first byte: 0xff
after page program, flash[0x40:0x46] = b'xuanwu'
read back over the bus:            b'xuanwu'
```

在 Python 里用：

```python
device.dev.names()          # ['LED', 'FLASH']
device.dev["LED"].state     # True / False / None
device.dev["LED"].transitions
device.dev["FLASH"].read_memory(0x40, 6)
```

加一个新器件类型：在 `src/xuanwu/devices/` 里实现 `Device`（`attach()` 拿到 `DeviceContext`，用 `ctx.peripheral("GPIOB")` 找外设），然后注册进 `devices/__init__.py` 的 `BUILDIN`。

---

## 9. 构建测试固件（Arduino CLI）

### 9.1 为什么用 Arduino CLI

- 每块板子的 core **自带匹配的工具链**，镜像里不需要再装发行版的 `gcc-arm-none-eabi`；
- 固件来源是 sketch（`.ino`），改一行就能重编，不用维护手写的链接脚本；
- `board.yaml` 把「sketch → 板子 FQBN → 芯片描述」写清楚，谁都能复现。

当前矩阵（`tests/firmware/board.yaml`）：

| 目录 | 板子 | FQBN | 芯片描述 |
|---|---|---|---|
| `stm32f411` | Generic STM32F4 (F411CEUx) | `STMicroelectronics:stm32:GenF4:pnum=GENERIC_F411CEUX` | `stm32f411` |
| `sam3x8e` | Arduino Due (Programming Port) | `arduino:sam:arduino_due_x_dbg` | `sam3x8e` |
| `mkrzero` | Arduino MKR Zero（SAMD21，Cortex-M0+） | `arduino:samd:mkrzero` | 还没有（构建但测试不跑） |
| `nucleo_f767zi` | ST Nucleo-F767ZI（STM32F767，Cortex-M7） | `STMicroelectronics:stm32:Nucleo_144:pnum=NUCLEO_F767ZI` | 还没有（构建但测试不跑） |

### 9.2 用脚本重建

```console
$ python tests/firmware/build.py --list
name             chip         output           board
stm32f411        stm32f411    stm32f411        Generic STM32F4 series (STM32F411CEUx)
                                               STMicroelectronics:stm32:GenF4:pnum=GENERIC_F411CEUX
sam3x8e          sam3x8e      sam3x8e          Arduino Due (Programming Port)
                                               arduino:sam:arduino_due_x_dbg
mkrzero          -            mkrzero          Arduino MKR Zero
                                               arduino:samd:mkrzero
nucleo_f767zi    -            nucleo_f767zi    ST Nucleo-F767ZI
                                               STMicroelectronics:stm32:Nucleo_144:pnum=NUCLEO_F767ZI

$ python tests/firmware/build.py sam3x8e
arduino-cli: arduino-cli  Version: 1.5.1 ...
== sam3x8e (Arduino Due (Programming Port))
  $ arduino-cli compile --fqbn arduino:sam:arduino_due_x_dbg --output-dir .../tests/firmware/sam3x8e ...
  ok: Blink_uart_m3.ino.bin, Blink_uart_m3.ino.elf, Blink_uart_m3.ino.map, ...

built 1 firmware image(s)
```

构建产物写到「输出目录」（就是测试扫描的目录），`.elf`/`.bin`/`.hex` 会提交进仓库，`.map` 与 `*.with_bootloader.*` 被忽略。

### 9.3 不用脚本，手敲 arduino-cli

```bash
# 一次性：装 core（镜像里已经装好）
arduino-cli config add board_manager.additional_urls \
    https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json
arduino-cli core update-index
arduino-cli core install arduino:sam
arduino-cli core install arduino:samd
arduino-cli core install STMicroelectronics:stm32@3.0.0

# 编译（sketch 目录必须与 .ino 同名）
arduino-cli compile \
    --fqbn arduino:sam:arduino_due_x_dbg \
    --output-dir tests/firmware/sam3x8e \
    --export-binaries \
    tests/firmware/Blink_uart_m3
```

找板子的 FQBN：

```bash
arduino-cli board listall | grep -i due       # Arduino Due (Programming Port) -> arduino:sam:arduino_due_x_dbg
arduino-cli board details -b STMicroelectronics:stm32:Nucleo_144 | grep -i 767
```

### 9.4 加一块新板子

1. 建 `tests/firmware/<SketchName>/<SketchName>.ino`（目录名必须与文件名一致）；
2. 在 `board.yaml` 里加一条：`name` / `board` / `fqbn` / `chip` / `sketch` / `output`，没有芯片描述就留空 `chip:` 并写 `note:` 说明缺什么；
3. `python tests/firmware/build.py <name>`；
4. `pytest tests/conformance -q` —— `test_firmware_matrix.py` 会检查 sketch 是否存在、FQBN 是否合法、输出目录是否唯一、以及每个 `.elf` 是否都被某条声明覆盖。

> `chip:` 是权威来源，因为**加载地址会撞车**：STM32F411 与 STM32F767 的 flash 都在 `0x08000000`。加载地址只在没有声明时作为兜底。

### 9.5 手写的裸机固件

`tests/firmware/stm32f411/fpu_test/` 不走 Arduino：它需要自己控制 FPU 与异常帧。用现成脚本构建：

```bash
bash tests/firmware/stm32f411/fpu_test/build.sh
```

```console
using /home/onelife/.arduino15/packages/STMicroelectronics/tools/xpack-arm-none-eabi-gcc/14.2.1-1.1/bin/arm-none-eabi-gcc (14.2.1)
   text    data     bss     dec     hex filename
    568       0      96     664     298 .../fpu_test.elf
built .../fpu_test.elf
```

脚本按 `$CC` → `PATH` 上的 `arm-none-eabi-gcc` → Arduino core 自带工具链的顺序查找，所以容器里不装系统 Arm 编译器也能重建。

---

## 10. 跑测试

```bash
pytest tests -q                       # 全部
pytest tests/unit -q                  # 单元测试（快，不需要 socat）
pytest tests/conformance -q           # 芯片描述 + 固件矩阵的一致性检查
pytest tests/integration -q           # 真跑固件：UART、GDB、semihosting、器件、例子脚本
pytest tests -q -m "not slow"         # 跳过较慢的
pytest tests -q -k fpu                # 只跑与 fpu 相关的
```

markers：`integration`（要跑真实固件）、`conformance`（校验 YAML/清单）、`slow`（构建 wheel、跑例子脚本之类）。

几件值得知道的事：

- **例子脚本本身也是测试**（`tests/integration/test_examples.py`）：本手册里的每个例子都会被真实执行一遍，所以文档不会悄悄过期。
- **conformance 里有固件矩阵检查**：新加的 `.elf` 如果没有被 `board.yaml` 覆盖，测试会失败而不是被静默忽略。
- **`pyflakes`**：`python -m pyflakes src/xuanwu tests` 目前只剩 5 个 re-export 模块的星号导入告警（`arch/__init__.py`、`armv7m.py`、`atmel_sam.py`、`stm_stm.py`、`config/__init__.py`）——这是刻意的再导出写法，pyflakes 无法判断。

---

## 11. 加一颗新芯片

最小可用的新增 = 一份 YAML：

```yaml
chip:
  name: stm32f103        # 前三个字母决定厂商族：stm -> arch/vendor/st
  arch: arm
  mode: cortex_m
  peripherals:
    - "Flash":
        type: memory
        base: 0x08000000
        size: 0x20000
    - SRAM:
        type: memory
        base: 0x20000000
        size: 0x5000
    - PPB:
        type: peripheral
        base: 0xE0000000
        size: 0x10000
    - SCID: {type: core, base: 0xE000E000, size: 0x10, interrupt_lines: 7}
    - SYSTICK: {type: core, base: 0xE000E010, size: 0x10, clock: 72000000}
    - NVIC: {type: core, base: 0xE000E100, size: 0x3F0, interrupt_lines: 7, priority_bits: 4}
    - SCB: {type: core, base: 0xE000ED00, size: 0x40, cpuid: 0x410FC231}
```

放到 `src/xuanwu/data/chips/arm/cortex_m/stm32f103.yaml`，然后：

```bash
pytest tests/conformance -q                      # 秒级校验
python -c "import sys; sys.path.insert(0,'src'); from xuanwu import XuanWu; \
  d = XuanWu('stm32f103', 'firmware.elf'); d.reset(); d.mem.show_map()"
```

缺的外设模型照着 `arch/vendor/st/gpio.py` 写一个，注册进 `arch/vendor/st/__init__.py` 的 `BUILDIN`。细节见 [`docs/add-a-chip.md`](add-a-chip.md)。

---

## 12. 排错 FAQ

**`ModuleNotFoundError: No module named 'distutils'` / `pkg_resources`**
旧版 unicorn/capstone 的问题。`requirements.txt` 要求 `unicorn>=2.1`、`capstone>=5.0`；不要降版本，也不要再钉 `setuptools<81`。

**`XwSerialBridgeError` / 找不到 socat**
芯片 YAML 用了 `bridge: socat`（或 `auto` 在没有 socat 的机器上退化失败）。改成 `bridge: tcp`，或装 socat，或在运行时覆盖：`XuanWu(..., hardware_options={"bridge": "tcp"})`。

**`UcError: Invalid instruction (UC_ERR_INSN_INVALID)`**
两种常见原因：
1. 固件不是给这颗芯片编的（加载地址/指令集不对）；
2. 代码里手动写了偶数 PC —— Unicorn 把 PC 的 bit 0 当作指令集选择位，写了偶数就会切到 ARM 态。用 `device.reg.pc_t = addr`（自动置位）而不是 `reg.write("pc", addr)`。

**固件跑着跑着就停了，PC 不动**
先 `python examples/unclaimed_io.py`：多半是在轮询一个还没建模的寄存器位。

**GDB 报 `Truncated register 16 in remote 'g' packet`**
说明 target description 没被接受。stub 现在会下发完整的 `<target>` 文档；如果你自己改过 `src/xuanwu/data/gdb/features/`，注意**只能下发 `<target>` 包裹的文档**，光给 `<feature>` GDB 会静默忽略。

**`arduino-cli: command not found` / `no arm-none-eabi-gcc found`**
用开发容器（第 1 节），或按 9.3 自己装 core。

**端口被占用（`rsp=6666` 或 TCP 桥）**
换端口：`rsp=16666`，或 `hardware_options={"bridge": "tcp", "port": 0}` 让系统分配。测试里用 `free_port()` 的做法（见 `tests/integration/test_gdb_stub.py`）。

**跑得很慢**
`run(count=N)` 的 N 是指令数；固件里 `delay(1000)` 在 100 MHz + 一条指令一个周期下就是 1 亿条指令。要么少跑点，要么调大芯片 YAML 里 `cycles_per_instruction`（这会让「仿真时间」跑得更快，是精度换速度）。

---

## 13. 附录

### 13.1 目录结构

```
src/xuanwu/
├── xuanwu.py            XuanWu 门面：内存图、外设、装载、stub
├── chips.py             芯片名/路径解析
├── memory.py            内存映射、MMIO 路由、位带别名、未认领访问统计
├── register.py          寄存器名 -> Unicorn 寄存器号（含 pc_t）
├── loader.py            ELF / Intel HEX / binary
├── rsp.py               GDB remote serial protocol
├── device.py            外部器件装载
├── devices/             器件模型：led、spi_flash
├── arch/
│   ├── base.py          寄存器表 + ArmHardwareBase
│   ├── cortex_m/        内核外设（SCID/SysTick/NVIC/SCB/CP/DBG/DWT/FPU）+ 异常引擎
│   └── vendor/          st/ 与 atmel/ 两家外设模型
├── backends/            串口桥（socat/tcp/loopback/none）与 semihosting
├── config/              路径、日志、常量
└── data/                芯片描述（chips/）与 GDB target description（gdb/）
tests/                   unit / conformance / integration + firmware（含构建脚本与清单）
examples/                可直接运行的例子（也是测试）
docs/                    架构、加芯片、调试、本手册
docker/                  Dockerfile 与 docker-compose.yml
```

### 13.2 环境变量

| 变量 | 作用 |
|---|---|
| `DEBUG=1` | 打开 debug 日志，并写入 `$XDG_STATE_HOME/xuanwu/xuanwu.log` |
| `ARDUINO_CLI` | 指定 `arduino-cli` 可执行文件（默认取 PATH 上的） |
| `ARDUINO_DIRECTORIES_DATA` | Arduino core 与工具链的数据目录 |

### 13.3 已知限制（诚实清单）

- FPU：`CPACR` 不 gate、不实现 lazy stacking（见 7.3）；
- SysTick 按指令推进，不是真实周期；`millis()` 是仿真时间；
- `ArmHardwareNvic` 之外的多数外设模型只实现固件实际用到的寄存器语义：PWM 不产生中断、ADC 返回常数、SPI 按字节；
- 外部器件只有 `led` 与 `spi_flash`，没有 I2C 总线；
- 未实现的芯片：MKR Zero（SAMD21/M0+）与 Nucleo-F767ZI（STM32F767/M7）已有可复现的固件，缺芯片描述——这正是下一颗芯片的入口。

### 13.4 相关文档

| 文档 | 内容 |
|---|---|
| [`README.md`](../README.md) | 项目概览、快速开始、支持芯片 |
| [`docs/architecture.md`](architecture.md) | 分层、内存路由、外设框架、异常引擎、GDB、时间基准 |
| [`docs/add-a-chip.md`](add-a-chip.md) | 加一颗新芯片的完整流程与 YAML 字段参考 |
| [`docs/debugging.md`](debugging.md) | 日志、状态检查、GDB、串口桥、常见故障表 |
| [`tests/firmware/README.md`](../tests/firmware/README.md) | 固件目录布局与构建方式 |
| [`CHANGELOG.md`](../CHANGELOG.md) | 每个版本改了什么、为什么 |
