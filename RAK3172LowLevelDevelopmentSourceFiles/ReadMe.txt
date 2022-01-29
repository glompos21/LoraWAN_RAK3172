radio.c (Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver/radio.c)
Commented all the DBG functions


lora_app.c (LoRaWAN/App/lora_app.c)
Commented SYS_LED, BSP_LED etc

Alteration from original RAK file radio_conf.h
line 49: //#define DBG_GPIO_RADIO_RX(set_rst) DBG_GPIO_##set_rst##_LINE(DGB_LINE1_PORT, DGB_LINE1_PIN);
line 54: //#define DBG_GPIO_RADIO_TX(set_rst) DBG_GPIO_##set_rst##_LINE(DGB_LINE2_PORT, DGB_LINE2_PIN);


