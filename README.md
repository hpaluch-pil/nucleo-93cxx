# How to read EEPROM 93LC66C to memory

Here is an attempt how to read contents
of [93LC66C] serial EEPROM (see also [93LC66C PDF]) into RAM of
[STM NUCLEO-F767ZI] development board.
Please read my [Getting started with ST NUCLEO F767ZI Board]
for development setup instructions.

Known bugs and limitations:
* Usable in debugger only (it reads data from EEPROM to array and loops forever)
* only 16-bit EEPROM data organization is currently supported 
  (support for 8-bit data is on the way, but not yet tested)

## Source tree

The `GPIO_IOToggle/` is versioned as overlay (only changed files there)
of target directory:

```
STM32Cube_FW_F7_V1.14.0\Projects\STM32F767ZI-Nucleo\Examples\GPIO\GPIO_IOToggle\ 
```

For development you need to download and
extract `en.stm32cubef7.zip` from [STM32CubeF7]:

## Connecting 93LC66C to Nucleo

Here is table how [93LC66C] should be connected to [STM NUCLEO-F767ZI]:

|Nucleo Conn.|Nucleo PIN|Nucleo Func|93LC66C PIN|
|------------|----------|-----------|-----------|
|CN8|7|+3V3|8 Vcc|
|CN8|11|GND|5 GND (Vss)|
|CN10|31|PB0 <sup>1</sup>|1 CS|
|CN10|2|PF13|2 CLK|
|CN10|8|PF14|3 D (input)|
|CN10|12|PF15|4 Q (output)|

-----
 1) The PB0 is also connected to green LED LD1.
    But it is OK because there is U7 -  [Op-Amp TSV631] connected to PB0 with high impedance
    input (no need to worry about CS signal load).
    See [STM32 Nucleo-144 boards], `Figure 20. Extension connectors` for details.

Current schematic is (made in ExpressSCH 6.1.4  - part of ExpressPCB freeware):

![STM NUCLEO-F767ZI with 93LCxx EEPROM](https://github.com/hpaluch-pil/nucleo-93cxx/blob/master/ExpressPCB/nucleo-w-93lc.png?raw=true)


[93LC66C]: https://www.microchip.com/wwwproducts/en/93LC66C
[93LC66C PDF]: http://ww1.microchip.com/downloads/en/DeviceDoc/21795E.pdf
[STM NUCLEO-F767ZI]: https://www.st.com/content/st_com/en/products/evaluation-tools/product-evaluation-tools/mcu-eval-tools/stm32-mcu-eval-tools/stm32-mcu-nucleo/nucleo-f767zi.html
[Getting started with ST NUCLEO F767ZI Board]: https://github.com/hpaluch/hpaluch.github.io/wiki/Getting-started-with-ST-NUCLEO-F767ZI-Board
[STM32CubeF7]: https://www.st.com/en/embedded-software/stm32cubef7.html
[Op-Amp TSV631]: https://www.st.com/en/amplifiers-and-comparators/tsv631.html
[STM32 Nucleo-144 boards]: https://www.st.com/content/ccc/resource/technical/document/user_manual/group0/26/49/90/2e/33/0d/4a/da/DM00244518/files/DM00244518.pdf/jcr:content/translations/en.DM00244518.pdf
