# heatpumpir 组件 - 编译所有空调协议供 standalone_heatpump_ir 使用
file(GLOB PROTOCOL_SRCS "*.cpp")
list(FILTER PROTOCOL_SRCS EXCLUDE REGEX ".*examples.*")
idf_component_register(SRCS ${PROTOCOL_SRCS}
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_gpio esp_driver_ledc esp_rom esp_timer)
