if(CONFIG_HEBALL_CMD_HANDLER OR CONFIG_HEBALL_BLE_PERIPHERAL)
    target_sources(app PRIVATE src/zmk_kscan_he_api.c)
endif()

if(CONFIG_HEBALL_CMD_HANDLER)
    target_sources(app PRIVATE src/cmd_handler.c)
endif()

if(CONFIG_HEBALL_BLE_PERIPHERAL)
    target_sources(app PRIVATE src/heball_ble_peripheral.c)
endif()

if(CONFIG_HEBALL_BLE_CENTRAL)
    target_sources(app PRIVATE src/heball_ble_central.c)
endif()
