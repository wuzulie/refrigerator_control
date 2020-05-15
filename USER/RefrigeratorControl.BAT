SET PATH=d:\Keil_v5\ARM\ARMCC\Bin;C:\ProgramData\Oracle\Java\javapath;D:\AiThinkerIDE_V1.0\msys32\opt\arm-nono-eabi-gcc\bin;D:\AiThinkerIDE_V1.0\msys32\opt\xtensa-lx106-elf\bin;D:\AiThinkerIDE_V1.0\msys32\opt\xtensa-esp32-elf\bin;d:\AiThinkerIDE_V1.0\msys32\mingw32\bin;d:\AiThinkerIDE_V1.0\msys32\usr\bin;D:\AiThinkerIDE_V1.0\eclipse\eclipse.exe;D:\AiThinkerIDE_V1.0\msys32\opt\arm-nono-eabi-gcc\bin;D:\AiThinkerIDE_V1.0\msys32\opt\xtensa-lx106-elf\bin;D:\AiThinkerIDE_V1.0\msys32\opt\xtensa-esp32-elf\bin;d:\AiThinkerIDE_V1.0\msys32\mingw32\bin;d:\AiThinkerIDE_V1.0\msys32\usr\bin;D:\AiThinkerIDE_V1.0\eclipse\eclipse.exe;D:\AiThinkerIDE_V1.0\eclipse\jre\bin;C:\Windows\system32;C:\Windows;C:\Windows\system32\wbem;C:\Program Files\Apache Software Foundation\Tomcat 9.0\bin;e:\MATLAB\R2018b\runtime\win64;e:\MATLAB\R2018b\bin;;d:\Microsoft VS Code\bin
SET CPU_TYPE=STM32F103C8
SET CPU_VENDOR=STMicroelectronics
SET UV2_TARGET=RefrigeratorControl
SET CPU_CLOCK=0x00B71B00
"d:\Keil_v5\ARM\ARMCC\Bin\ArmAsm" --Via "..\output\startup_stm32f10x_md._ia"
"d:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\output\main.__i"
"d:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\output\stm32f10x_it.__i"
"d:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\output\stm32f10x_gpio.__i"
"d:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\output\stm32f10x_rcc.__i"
"d:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\output\stm32f10x_usart.__i"
"d:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\output\stm32f10x_adc.__i"
"d:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\output\stm32f10x_tim.__i"
"d:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\output\stm32f10x_exti.__i"
"d:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\output\misc.__i"
"d:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\output\stm32f10x_fsmc.__i"
"d:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\output\stm32f10x_dma.__i"
"d:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\output\core_cm3.__i"
"d:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\output\system_stm32f10x.__i"
"d:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\output\usart1.__i"
"d:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\output\adc.__i"
"d:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\output\sys.__i"
"d:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\output\timer.__i"
"d:\Keil_v5\ARM\ARMCC\Bin\ArmLink" --Via "..\Output\NTC.lnp"
"d:\Keil_v5\ARM\ARMCC\Bin\fromelf.exe" "..\Output\NTC.axf" --i32combined --output "..\Output\NTC.hex"
