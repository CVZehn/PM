SET PATH=D:\keil\ARM\ARMCLANG\Bin;C:\Program Files (x86)\Common Files\Oracle\Java\javapath;C:\WINDOWS\system32;C:\WINDOWS;C:\WINDOWS\System32\Wbem;C:\WINDOWS\System32\WindowsPowerShell\v1.0\;C:\Program Files\Common Files\Autodesk Shared\;C:\Users\Zehn\.dnx\bin;C:\Program Files\Microsoft DNX\Dnvm\;E:\MATLABE\runtime\win64;E:\MATLABE\bin;C:\Program Files (x86)\NVIDIA Corporation\PhysX\Common;C:\WINDOWS\system32;C:\WINDOWS;C:\WINDOWS\System32\Wbem;C:\WINDOWS\System32\WindowsPowerShell\v1.0\;C:\WINDOWS\System32\OpenSSH\;D:\Git\cmd;C:\Program Files\Microsoft\Web Platform Installer\;D:\bin;C:\Users\Administrator\AppData\Local\Microsoft\WindowsApps;D:\Microsoft VS Code\bin
SET ARMCC6_ASMOPT=--diag_suppress=9931
SET ARMCC6_CCOPT=--diag_suppress=9931
SET ARMCC6_LINKOPT=--diag_suppress=9931
SET CPU_TYPE=STM32F103C8
SET CPU_VENDOR=STMicroelectronics
SET UV2_TARGET=pm
SET CPU_CLOCK=0x02DC6C00
"D:\keil\ARM\ARMCLANG\Bin\ArmClang.exe" @"pm\main.__i"
"D:\keil\ARM\ARMCLANG\Bin\ArmClang.exe" @"pm\usb_device.__i"
"D:\keil\ARM\ARMCLANG\Bin\ArmClang.exe" @"pm\usbd_conf.__i"
"D:\keil\ARM\ARMCLANG\Bin\ArmClang.exe" @"pm\usbd_desc.__i"
"D:\keil\ARM\ARMCLANG\Bin\ArmClang.exe" @"pm\usbd_cdc_if.__i"
"D:\keil\ARM\ARMCLANG\Bin\ArmClang.exe" @"pm\stm32f1xx_it.__i"
"D:\keil\ARM\ARMCLANG\Bin\ArmClang.exe" @"pm\stm32f1xx_hal_msp.__i"
"D:\keil\ARM\ARMCLANG\Bin\ArmClang.exe" @"pm\stm32f1xx_hal_gpio_ex.__i"
"D:\keil\ARM\ARMCLANG\Bin\ArmClang.exe" @"pm\stm32f1xx_ll_gpio.__i"
"D:\keil\ARM\ARMCLANG\Bin\ArmClang.exe" @"pm\stm32f1xx_hal_pcd.__i"
"D:\keil\ARM\ARMCLANG\Bin\ArmClang.exe" @"pm\stm32f1xx_hal_pcd_ex.__i"
"D:\keil\ARM\ARMCLANG\Bin\ArmClang.exe" @"pm\stm32f1xx_ll_usb.__i"
"D:\keil\ARM\ARMCLANG\Bin\ArmClang.exe" @"pm\stm32f1xx_hal_tim.__i"
"D:\keil\ARM\ARMCLANG\Bin\ArmClang.exe" @"pm\stm32f1xx_hal_tim_ex.__i"
"D:\keil\ARM\ARMCLANG\Bin\ArmClang.exe" @"pm\stm32f1xx_hal.__i"
"D:\keil\ARM\ARMCLANG\Bin\ArmClang.exe" @"pm\stm32f1xx_hal_rcc.__i"
"D:\keil\ARM\ARMCLANG\Bin\ArmClang.exe" @"pm\stm32f1xx_hal_rcc_ex.__i"
"D:\keil\ARM\ARMCLANG\Bin\ArmClang.exe" @"pm\stm32f1xx_hal_gpio.__i"
"D:\keil\ARM\ARMCLANG\Bin\ArmClang.exe" @"pm\stm32f1xx_hal_dma.__i"
"D:\keil\ARM\ARMCLANG\Bin\ArmClang.exe" @"pm\stm32f1xx_hal_cortex.__i"
"D:\keil\ARM\ARMCLANG\Bin\ArmClang.exe" @"pm\stm32f1xx_hal_pwr.__i"
"D:\keil\ARM\ARMCLANG\Bin\ArmClang.exe" @"pm\stm32f1xx_hal_flash.__i"
"D:\keil\ARM\ARMCLANG\Bin\ArmClang.exe" @"pm\stm32f1xx_hal_flash_ex.__i"
"D:\keil\ARM\ARMCLANG\Bin\ArmClang.exe" @"pm\stm32f1xx_ll_rcc.__i"
"D:\keil\ARM\ARMCLANG\Bin\ArmClang.exe" @"pm\stm32f1xx_ll_utils.__i"
"D:\keil\ARM\ARMCLANG\Bin\ArmClang.exe" @"pm\stm32f1xx_ll_exti.__i"
"D:\keil\ARM\ARMCLANG\Bin\ArmAsm" --Via "pm\startup_stm32f103xb._ia"
"D:\keil\ARM\ARMCLANG\Bin\ArmClang.exe" @"pm\system_stm32f1xx.__i"
"D:\keil\ARM\ARMCLANG\Bin\ArmClang.exe" @"pm\delay.__i"
"D:\keil\ARM\ARMCLANG\Bin\ArmClang.exe" @"pm\sys.__i"
"D:\keil\ARM\ARMCLANG\Bin\ArmClang.exe" @"pm\usbd_core.__i"
"D:\keil\ARM\ARMCLANG\Bin\ArmClang.exe" @"pm\usbd_ctlreq.__i"
"D:\keil\ARM\ARMCLANG\Bin\ArmClang.exe" @"pm\usbd_ioreq.__i"
"D:\keil\ARM\ARMCLANG\Bin\ArmClang.exe" @"pm\usbd_cdc.__i"
"D:\keil\ARM\ARMCLANG\Bin\ArmLink" --Via "pm\pm.lnp"
"D:\keil\ARM\ARMCLANG\Bin\fromelf.exe" "pm\pm.axf" --i32combined --output "pm\pm.hex"
