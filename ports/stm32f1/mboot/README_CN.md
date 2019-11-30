Mboot - MicroPython 启动加载器
===============================

`Mboot` 是一个STM32 MCU 定制启动加载器, 支持 STM32F1 系列。 它可在`FS外设`上提供一个标准`USB DFU`接口,
也可以从文件系统载入和烧写 `.dfu.gz` 格式固件。

如何使用
----------

1. 使用`mpconfigboard.mk`和`mpconfigboard.h`配置板子。
    例如, 配置时一定要在`mpconfigboard.mk`文件中确保有以下行:

    ```makefile
    LD_FILES = boards/MPY_ST103/mpy_st103.ld boards/common_bl.ld
    TEXT0_ADDR = 0x08020000
    ```

    在`mpconfigboard.h`文件中包含 (建议放到文件最后):

    ```c
    // 启动加载器配置
    #define MBOOT_I2C_PERIPH_ID (1)
    #define MBOOT_I2C_SCL       (pin_B8)
    #define MBOOT_I2C_SDA       (pin_B9)
    ```

    配置一个管脚来强制进入启动加载器(示例):

    ```c
    #define MBOOT_BOOTPIN_PIN    (pin_A0)
    #define MBOOT_BOOTPIN_PULL   (MP_HAL_PIN_PULL_UP)
    #define MBOOT_BOOTPIN_ACTIVE (0)
    ```
 
    Mboot 支持通过DFU和I2C接口烧写外部Nor Flash。Nor Flash通过FSM被映射到一个地址范围
    要配置它使用以下选项(按需编辑):

    ```c
    #define MBOOT_NORFLASH_ADDR                     (0x64000000)
    #define MBOOT_NORFLASH_BYTE_SIZE                (16 * 1024 * 1024)
    #define MBOOT_NORFLASH_LAYOUT                   "/0x64000000/64*128Kg"
    ```

    这里假设已定义Nor Flash 片选引脚。
    `MBOOT_NORFLASH_LAYOUT`值会被`USB DFU` 实用程序使用， 并且必须定义Nor Flash布局。
    注意布局描述中的页数量不能大于99(值必须是两位数)，所以报告的页大小(上面的`128Kg`)必须足够大以保障页数量是2位数。

    另外布局可以指定为多个小节类似`32*16Kg,32*16Kg`, 这种情况下`MBOOT_NORFLASH_ERASE_BLOCKS_PER_PAGE`必须更改为`16/4`以匹配`16Kg`。

    Mboot 目前支持通过FSMC外扩一个Nor Flash设备

    打开从文件系统载入固件功能, 使用:

    ```c
    #define MBOOT_FSLOAD (1)
    ```

2. 像往常一样构建主板的主应用固件。

3. 构建 mboot 使用:
    ```bash
    $ cd mboot
    $ make BOARD=<board-id>
    ```

    以上命令会为mboot生成一个**DFU**文件。  可用`USB DFU`编程命令部署(它将被放在Flash: 0x08000000处):
    
    ```bash
    $ make BOARD=<board-id> deploy
    ```

4. 复位板子同时按住用户按键直到3个LED点亮, 然后再松开用户按键。 LED0会以1次/秒的速度闪烁表示已经进入mboot。

5. 使用`USB DFU`或`I2C`下载固件. 脚本`mboot.py`展示了如何与`I2C`启动加载器接口通讯。  
    它应该运行在通过`I2C`连接的目标pyboard板上。

从应用代码进入Mboot
------------------------------------

从运行中的应用进入Mboot使用以下步骤:

1. `r0`寄存器载入`0x70ad0000`. 低7位是可行的I2C地址。

2. 把`0x08000000`处的值载入`MSP`。

3. 跳转到`0x08000004`处的地址。

通过将这些数据存储在RAM的特殊区域，可以将额外的数据从应用程序传给Mboot。
这个区域的开始地址存储在0x08000000(它将指向Mboot的堆栈之后)。这里最大可存储1024字节。

数据在这个区域是一个元素的序列.每个元素都有如下的形式:

    <type:uint8_t> <len:uint8_t> <payload...>

`type`和`len`类型是字节, `payload` 可以有0或更多字节, `len` 必须是`payload`的字节长度。

序列的最后一个元素必须是结束元素:

* END: type=1, len=0

注意: `MicroPython`的 `machine.bootloader()`方法 执行上面的步骤1-3, 并可接受一个可选的附加`bytes`参数传递到Mboot。

从文件系统载入应用固件
----------------------------------

要使Mboot从文件系统加载固件，并且自动烧写, 必须传递附加参数(参考上面)表明文件系统在哪里以及要烧写的文件名,
参数示例:

* MOUNT: type=2, len=10, payload=(<mount-point:u8> <fs-type:u8> <base-addr:u32> <byte-len:u32>)

* FSLOAD: type=3, len=1+n, payload=(<mount-point:u8> <filename...>)

`u32` 是无符号32位小端数字。

要载入的固件必须是一个 gzip格式的DfuSe文件(.dfu.gz)

提供的`fwupdate.py`脚本包含一堆帮助函数， 用于进入保证Mboot时数据正确性，同时更新Mboot自身

示例: MPY-ST103 上的 Mboot
-------------------------

PYBv1.x默认不使用mboot, 但为这些板子提供了完整的mboot配置来演示它是如何工作和测试的。
要在这些板子上构建和部署mboot，与普通构建过程的惟一区别是传递`USE_MBOOT=1`给`make`

PYBv1.0详细说明(PYBv1.1 使用PYBV11代替PYBV10):

1. 确保pyboard处于工厂DFU模式(BOOT0连到3V3后上电), 构建mboot并部署(在stm32f1/mboot目录):

    ```bash
    $ make BOARD=PYBV10 USE_MBOOT=1 clean all deploy
    ```

    完成后，mboot就烧写到了板子上。

2. 现在按住用户键的同时，按复位键, 直到蓝色LED点亮，再松开用户键, 使板子进入mboot模式,
    红色LED以1次/秒的速度闪烁表示已进入mboot. 然后构建应用固件并部署(在stm32f1/目录):

    ```bash
    $ make BOARD=PYBV10 USE_MBOOT=1 clean all deploy
    ```

    欧了, 应用已经烧写好， 并且立即引导

在 PYBv1.x 上， 无 mboot 的flash布局是:

    0x08000000  0x08004000      0x08020000
    | ISR text  | filesystem    | rest of MicroPython firmware

在 PYBv1.x 上, 有 mboot 的flash布局是:

    0x08000000  0x08004000      0x08020000
    | mboot     | filesystem    | ISR and full MicroPython firmware

注意，当进入/离开mboot配置时，文件系统完好无损， 所以它的内容会被保存下来。
