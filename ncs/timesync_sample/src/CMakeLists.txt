#
# Copyright (c) 2022 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

target_sources(app PRIVATE
  main.c
  timesync/time_sync.c
)

zephyr_library_include_directories(timesync)
