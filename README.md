# stm32f439-usb-host-fatfs

stm32f439 CUBEMX generated project with Usb_Host and FatFs to access a USB Pen Drive.<br>

Using embedded software package STM32F4 v1.28.3 with classic Middleware versions ST USB Host Lib V3.5.1. and FatFs VR0.12c (ST modified 2023-08-18).<br>
This supports the NUCLEO-F439ZI board.<br>

Debug the project.<br>
Add Appli_state and USBWriteOpEnable to the Live Expressions Watch Window.<br>
Appli_state will be APPLICATION_DISCONNECT initially when the USB Pen Drive is not connected.<br>
On Connecting the USB Pen Drive the Appli_state changes to APPLICATION_READY.<br>
From the Watch Window set USBWriteOpEnable to 1. It will be immediately set back to 0 indicating sucessful operation.<br>
Unplug the USB Pen Drive and the Appli_state changes back to APPLICATION_DISCONNECT.<br>
Plug-in the USB Pen drive into the PC and check it's contents as  - TEMP.txt file containing the line "Welcome to EmbeTronicX!!!"<br>

https://embetronicx.com/tutorials/microcontrollers/stm32/stm32-usb-host-msc-connect-pendrive-to-stm32/
Based on the link above : <br>
a. USB_HOST and FATFS configuration <br>
b. ETX_MSC_ProcessUsbDevice() <br>
