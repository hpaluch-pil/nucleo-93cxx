# How to read flash 93C86 to memory

Here is an attempt how to read contents
of [93C86] serial flash into RAM of
[STM NUCLEO-F767ZI] development board.
Please read my [Getting started with ST NUCLEO F767ZI Board]
for development setup instructions.

Known bugs limitations:
* Usable in debugger only (it reads data from flash to array and loops forever)
* only 16-bit flash data organization is currently supported.

## Source tree

The `GPIO_IOToggle/` is versioned as overlay (only changed files there)
of target directory:

```
STM32Cube_FW_F7_V1.14.0\Projects\STM32F767ZI-Nucleo\Examples\GPIO\GPIO_IOToggle\ 
```

For development you need to download and
extract `en.stm32cubef7.zip` from [STM32CubeF7]:

## Connecting  93C86 to Nucleo

Here is table how [93C86] should be connected to [STM NUCLEO-F767ZI]:

|Nucleo Conn.|Nucleo PIN|Nucleo Func|93C86 PIN|
|------------|----------|-----------|---------|
|CN8|7|+3V3|8 Vcc|
|CN8|11|GND|5 GND (Vss)|
|CN10|31|PB0^1|1 CS|
|CN10|2|PF13|2 CLK|
|CN10|8|PF14|3 D (input)|
|CN10|12|PF15|4 Q (output)|

-----
1) The PB0 is also connected to green LED LD1.
   But it is OK because there is Operational Amplifier on PB0 with high impedance
   (no need to worry about CS signal load).


[93C86]: https://www.microchip.com/wwwproducts/en/93C86
[STM NUCLEO-F767ZI]: https://www.st.com/content/st_com/en/products/evaluation-tools/product-evaluation-tools/mcu-eval-tools/stm32-mcu-eval-tools/stm32-mcu-nucleo/nucleo-f767zi.html
[Getting started with ST NUCLEO F767ZI Board]: https://github.com/hpaluch/hpaluch.github.io/wiki/Getting-started-with-ST-NUCLEO-F767ZI-Board
[STM32CubeF7]: https://www.st.com/en/embedded-software/stm32cubef7.html
