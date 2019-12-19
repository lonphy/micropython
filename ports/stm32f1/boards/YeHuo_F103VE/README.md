# 野火指南者-Mini STM32F103VET

野火指南者-Mini STM32F103VET 的 MicroPython 开发板定义文件

### 构建与烧写固件:

* 克隆MicroPython stmf103 (https://github.com/lonphy/micropython)
> 自行安装arm-none-eabi-gcc工具 与 Python环境

```bash
git clone https://github.com/lonphy/micropython
cd micropython
git checkout feature-f103
make -C mpy-cross
cd ports/stm32f1
make BOARD=YeHuo_F103VE
```

####  使用串口或调试器下载

将build-YeHuo_F103VE目录下的 firmware.bin/hex/elf 通过熟悉的工具下载到MCU中即可

> 注意, 烧录完， 会自动进行枚举 SD卡(如果已插入, 需要同时按住按键K2)或SPI Flash, 

使用开发板的USB Mini口(J11)连接电脑，打开串口工具开始你的repl之旅