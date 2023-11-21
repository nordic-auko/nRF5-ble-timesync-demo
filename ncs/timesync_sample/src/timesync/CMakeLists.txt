#
# Copyright (c) 2023 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

# target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/time_sync.c)
zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR})

target_sources_ifdef(CONFIG_SOC_SERIES_NRF52X
	app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/time_sync_nrf52.c)
target_sources_ifdef(CONFIG_SOC_SERIES_NRF53X
	app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/time_sync_nrf53.c)
