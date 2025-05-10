// Comment by Teig 7Feb2019
//     This file is copied from ../lib_tsn-master/lib_tsn/src/util/reboot.xc
//     Source is at https://github.com/xmos/lib_tsn/blob/master/lib_tsn/src/util/reboot.xc
//     It was mentioned by "akp" on https://www.xcore.com/viewtopic.php?f=44&t=7065&e=1&view=unread#unread "Restarting a statKIT form SW?"
//     THIS CODE REBOOTS AN xCORE-200 but not a startKIT
//     The whole library was not imported

// Copyright (c) 2013-2017, XMOS Ltd, All rights reserved
#include <xs1.h>
#include <platform.h>
#include "reboot.h"

/* Reboots XMOS device by writing to the PLL config register */
void device_reboot(void)
{
    unsigned int pllVal;
    unsigned int localTileId = get_local_tile_id();
    unsigned int tileId;
    unsigned int tileArrayLength;

    asm volatile ("ldc %0, tile.globound":"=r"(tileArrayLength));

    /* Reset all remote tiles */
    for(int i = 0; i < tileArrayLength; i++)
    {
        /* Cannot cast tileref to unsigned */
        tileId = get_tile_id(tile[i]);

        /* Do not reboot local tile yet */
        if (localTileId != tileId)
        {
            read_sswitch_reg(tileId, 6, pllVal);
            pllVal &= 0x7FFFFFFF; // Mask the XS2 no reset bit
            write_sswitch_reg_no_ack(tileId, 6, pllVal);
        }
    }

    /* Finally reboot this tile */
    read_sswitch_reg(localTileId, 6, pllVal);
    pllVal &= 0x7FFFFFFF;
    write_sswitch_reg_no_ack(localTileId, 6, pllVal);

}
