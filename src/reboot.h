// Comment by Teig 7Feb2019
//     This file is copied from ../lib_tsn-master/lib_tsn/src/util/reboot.h
//     Source is at https://github.com/xmos/lib_tsn/blob/master/lib_tsn/src/util/reboot.h
//     It was mentioned by "akp" on https://www.xcore.com/viewtopic.php?f=44&t=7065&e=1&view=unread#unread "Restarting a statKIT form SW?"
//     THIS CODE REBOOTS AN xCORE-200 but not a startKIT
//     The whole library was not imported

// Copyright (c) 2013-2017, XMOS Ltd, All rights reserved
#ifndef _reboot_h_
#define _reboot_h_

void device_reboot(void);

#endif
