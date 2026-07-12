target_sources_ifdef(CONFIG_APP_BLE_CORE app PRIVATE ble_core.c)
target_sources_ifdef(CONFIG_APP_ADV app PRIVATE adv.c)
zephyr_include_directories(.)
