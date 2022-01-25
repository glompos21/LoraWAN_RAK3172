# A simple project to connect a RAK3172 over LoraWAN

The project use the autogenerated files from the STM32CubeMX (lora_fromioc_RAK3172.ioc) and replaced
the files on LoRAWAN/Target with the files provided by Rakwireless for [Low Level Development](https://docs.rakwireless.com/Product-Categories/WisDuo/RAK3172-Module/Low-Level-Development/#overview)



 ## 

Core/Inc/main.c
Remove static from all functions

Core/Inc/platform.h
Comment line32 // #define USE_BSP_DRIVER

Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver/radio.c
Replace it with the included or comment all the DBG functions

Replace files from RAK3172LowLevelDevelopmentSourceFiles:
lora_app.c -> LoRaWAN/App/lora_app.c
radio_conf.h, radio_board_if.h, radio_board_if.c -> LoRaWAN/Target