/*
 * _version.h
 *
 *  Created on: 15. aug. 2018
 *      Author: teig
 */


#ifndef VERSION_H_
#define VERSION_H_

// SHOULD THE LENGTH OF THESE NEED TO CHANGE THEN THE STRING THEY ARE COPIED INTO MUST BE MODIFIED
//
#define XTIMECOMPOSER_VERSION_STR "14.4.1"

#define RFM69_CLIENT_VERSION_STR "0.8.93"
#define RFM69_CLIENT_VERSION_NUM    0893
//          NEXT                change button ports to buffered
//          NEXT                Check if "defines_adafruit.h" is needed
//          NEXT                Consider button_states_t not button_state_t
// 0893     10May2025           Making ready for exporting to GitHub. New .gitignore and README.md
// 0893     27Jan2020           random_light_change_cnt is new, used in SCREEN_LIGHT
//                              VERSION_OF_APP_PAYLOAD_03 also handled and printed in SCREEN_RADIO_DEBUG_TRANS
// 0892     20Nov2019           No code change, just testing a green Exibel USB cable (Ok here, also with IceCore, I had not discovered 1441, 1431 1421 etc.)
// 0891     12Sep2019           SCREEN_RX_DISPLAY_OVERSIKT and SCREEN_WELCOME now also have a ±
// 0890     13May2019           i2c_internal_commands_if broken up into itself plus i2c_internal_commands_if
//                              THIS COST ONE chanend:            ## -> so not interesting for
//                              Constraints[0]: C:8/7 T:10/7 C:32/14 M:58844 S:5516 C:45940 D:7388
//                              Constraints[1]: C:8/1 T:10/1 C:32/0  M:2952  S:348  C:2112  D:492
// 0889     12May2019           Even more functions
//                              Constraints[0]: C:8/7 T:10/7 C:32/13 M:58532 S:5500 C:45652 D:7380
//                              Constraints[1]: C:8/1 T:10/1 C:32/0  M:2952  S:348  C:2112  D:492
// 0888     12May2019           Functions instead of inline: i2c_internal_mcp23008_init and i2c_internal_mcp23008_poll_button
// 0887     12May2019           USB_WATCHDOG_AND_RELAY_BOX error handing and recovery
// 0886     11May2019           relay_button_state_e different sequence
// 0885     11May2019           relay_button_state_e expanded with more states for buttons on USB_WATCHDOG_AND_RELAY_BOX
// 0884     11May2019 RFM69=012 If 10 seconds timout timed out and cleared with right button then enter SCREEN_WELCOME
// 0883     11May2019           Button on USB_WATCHDOG_AND_RELAY_BOX handled
// 0882     04May2019           USB_WATCHDOG_AND_RELAY_BOX: Blinking with mcp23008 red and green LEDS every second
// 0881     03May2019           mcp23008 works initial rudimentary test
// 0880     03May2019           CLIENT_ALLOW_SESSION_TYPE_TRANS==1 removed, now only 0 and 2 allowed in lib_rfm69_xc
//                              Some mcp23008 i2c chip code added (skeletons)
// 0879     27Feb2019           Was too hard to compare sending here with sending from the aquarium-sw on the startKIT, even if I shot some scope pictures.
//                              Going back to reception
// 0878     27Feb2019           Going on testing. SCREEN_WELCOME for TX is new. Generated some scope pictures, like SDS00050.png etc.
// 0877     27Feb2019           Now all i_radio-calls include ..iff_asynch versions.
//                                  WORKS/works(*)/FAILS              W    W   W   w   w   w
//                                  CLIENT_ALLOW_SESSION_TYPE_TRANS   0    1   1   1   1   0
//                                  TRANS_ASYNCH_WRAPPED              0    0   1   1   0   0
//                                  I_RADIO_ANY none:                 0    0   0   0   0   0
//                                  SLAVE/MASTER                      ---SLAVE--   --MASTER-
//                                  (*) I could see sending as extra receptions on the AQUARIUM startKIT when
//                                      extra IRQ pulses were seen. See 2019 09 27 A...png file
// 0876     24Feb2019           Using working encrypt16_iff_asynch.
// 0875     22Feb2019           Also compiling for IS_MYTARGET_MASTER (not tested)
// 0874     22Feb2019           Now g_radio_log_value works on eXplorerKIT. Almost finished TRANS=1 and TRANS ASYNCH_WRAPPED=1 with both zero kept
// 0873     21Feb2019           Lots of changes with no TRANS=1 and TRANS ASYNCH_WRAPPED=1 with both zero kept. Sync with RFM69_DRIVER_VERSION_STR "0.9.20"
// 0872     13Feb2019           SCREEN_AQUARIUM_ERROR_BITS and SCREEN_DEBUG modified
// 0871     11Feb2019           start_time_trans1 set
// 0870     11Feb2019           Data to and from _trans functions completely changed
// 0869     10Feb2019           Common data structure timing_transx_t now used with do_sessions_trans2to3
// 0868     09Feb2019           trans2_timed_out is global are new
// 0866     09Feb2018           Call to do_sessions_trans2to3 is new
// 0865     07Feb2019           Use of get_radio_log_value and i_radio.get_radio_log_value_ptr is new
// 0864     07Feb2019           Files reboot.h and reboot.xc were added. Not used yet
// 0863     06Feb2019           Undoing RACL=001
// 0862     06Feb2019 RACLI=001 testing with code to restart, see https://www.xcore.com/viewtopic.php?f=44&t=7065. See reboot above which solves the problem for xCORE-200
// 0861     05Feb2019 RFM69=011 Works with that value with getting RSSI and _trans1, _trans2 and _trans3
// 0860     29Jan2019           Logging format. Works with lib_rfm69
// 0859     29Jan2019           Logging format
// 0858     28Jan2019           CAR_KEY code removed (some comments kept)
// 0857     27Jan2019           SCREEN_AQUARIUM_BOX_INTERNALS changed
// 0856     27Jan2019           100 and 1000 ms seemed ok. Going back to 2000, which I guess would only handle a _real_ problem. No log from RFM69 library any more
// 0855     27Jan2019           Just testing with IRQ_HIGH_MAX_TIME_MILLIS = 100
// 0854     27Jan2019           Just testing with IRQ_HIGH_MAX_TIME_MILLIS = 1000
// 0853     27Jan2019 RFM69=008 IRQ_interrupt_task new and used. Just one-way from IRQ handler, like it used to
//          27Jan2019           Below was probably last use of IRQ_detect_and_poll_task_2
// 0852     26Jan2019           SCREEN_VOLTAGES -> SCREEN_AQUARIUM_BOX_INTERNALS and new contents
// 0851     24Jan2019           More logging. IRQ_HIGH_MAX_TIME_MILLIS 1000
// 0850     23Jan2019           More logging. IRQ_HIGH_MAX_TIME_MILLIS 2000
// 0849     23Jan2019           Some rewrite, should be minor
// 0848     23Jan2019           IRQ_HIGH_MAX_TIME_MILLIS from 1200 to 2000
// 0847     23Jan2019           c_irq_update_e new name to make deadlock situation shown, and no read_irq_val_and_tick_state call then
// 0846     23Jan2019 RFM69=007 I think I noticed a situation where IRQ was correctly low but it did not do any RX of messages
// 0845     23Jan2019           Doing reset of more values in reset_values
// 0844     22Jan2019           IRQ_detect_and_poll_task is new, but now even using IRQ_detect_and_poll_task_2 with chanend
// 0843     21Jan2019           Just testing that it takes RFM69_DRIVER_VERSION_STR "0.8.07" plus a change in SCREEN_DEBUG
// 0842     20Jan2019           SCREEN_DEBUG now displays from aquarium
// 0841                         See "2019 01 15 A Log light amount change ver 0841 changes too often around 21.00.txt"
// 0841     15Jan2019           VERSION_OF_APP_PAYLOAD_01 and VERSION_OF_APP_PAYLOAD_02 handled separately
// 0839     12Jan2019 RFM69=006 More advanced reception from both from MASTER_ID_AQUARIUM and MASTER_ID_BLACK_BOARD
// 0838     10Jan2019           Display_screen forgotten
// 0837     10Jan2019 RFM69=005 Allowing reception from both from MASTER_ID_AQUARIUM and MASTER_ID_BLACK_BOARD
// 0836     12Dec2018           RX_context.allow_10_sek_timeout is new
// 0835     07Dec2018           buttons_state was not init enough, so first left button to dark did not work
// 0834     05Dec2018           SCREEN_HJELP is new
// 0833     05Dec2018           Direction of next screen now "up" or "down". Changes with keys: Hold center in while left is pushed in and out
// 0832     03Dec2018           i_blink_and_watchdog.blink_pulse_ok only when RXTX_context.receiveDone sin IRQ has separate LED
// 0831     02Dec2018           Just a new number after two important logs with different settings
// 0830     01Dec2018           messageRadioCRC16AppCRC32Errs_IRQ (in LIB), num_bothCRCerrs removed
// 0829     01Dec2018           _USERMAKEFILE_LIB_RFM69_XC_SEMANTICS_DO_INTERMEDIATE_RECEIVEDONE=0 just to test FLASHed version!
//                              ultimateIRQclear_cnt in this case counts, also when FLASHed. The good thing is that with this solution it's not down to RF69_MODE_STANDBY
//                              between every reception and _maybe_ we will see more packages and lose fewer
// 0828     30Nov2018           i_radio.uspi_ultimateIRQclear(); is new
//                              When I run this with the debugger and lots of printouts, ultimateIRQclear_cnt counts a lot!
//                              The same code ran as FLASHed shows NO increments over 14 hours at least
// 0827     29Nov2018 RFM69=004 Button_Task updated
// 0826     27Nov2018           Just testing with _USERMAKEFILE_LIB_RFM69_XC_SEMANTICS_DO_INTERMEDIATE_RECEIVEDONE=1 again to see if IRQ stuck still happens
//                              Observe that we treat all messages with IRQ with _USERMAKEFILE_LIB_RFM69_XC_SEMANTICS_DO_CRC_ERR_NO_IRQ=0
// 0825     27Nov2018           debug_state is new. _USERMAKEFILE_LIB_RFM69_XC_SEMANTICS_DO_CRC_ERR_NO_IRQ=0 again
//                    RFM69=003 solved with debug_mode_0_1 with IOF_BUTTON_RIGHT. It went on when this was pressed
// 0824     26Nov2018           _USERMAKEFILE_LIB_RFM69_XC_SEMANTICS_DO_CRC_ERR_NO_IRQ=1 again, the only change. Not able to see IRQ error
// 0823     26Nov2018           Did not help to MUL by 5 below. Now new par layout in main. Still ERROR_BITNUM_RF_IRQFLAGS2_FIFONOTEMPTY, but some times that's ok!
// 0822     26Nov2018           WAIT_FOR_REGISTER_VALUE_MS * 5 for ERROR_BITNUM_RF_IRQFLAGS2_FIFONOTEMPTY?
// 0821     26Nov2018           Reading error bits also on right button. ERROR_BITNUM_RF_IRQFLAGS2_FIFONOTEMPTY triggers
// 0820     26Nov2018           SCREEN_DEBUG better
// 0819     26Nov2018           SCREEN_DEBUG is new
//                              SEMANTICS_DO_CRC_ERR_NO_IRQ now to be set with _USERMAKEFILE_LIB_RFM69_XC_SEMANTICS_DO_CRC_ERR_NO_IRQ
//                              RFM69=003 IRQ error aqain, first time seen in with FLASH_EXPLORER_BOX.
// 0818     25Nov2018           SCREEN_STATISTICS_2 and compiled with SEMANTICS_DO_CRC_ERR_NO_IRQ=0 in lib_rfm69_xc
//                              I did see that the IRQ error with IRQ LED on always happened once when I was using the display (now call this RFM69=003)
//                              There were lots of CRC16 and even both CRC16 and CRC32 errors over the night
// 0817     24Nov2018           New screen SCREEN_STATISTICS and more statistics
// 0815     23Nov2018           u_to_str_lm is new
// 0814     22Nov2018           New light and heating regulating names of text constants
// 0813     21Nov2108           Error screen and showing when a screen has got new data (with '*' and '+')
// 0812     21Nov2018           An error in SCREEN_4_MAX_NA_MIN
// "0.8.11" 20Nov2018           display_screen_store_values is new
// "0.8.10" 20Nov2018           Display now toggles on and off with left button etc.
// "0.8.9"  20Nov2018
//                    RFM69=002 MAX and MIN values now like 100 for % and 99 for temp, to make display nicer
// "0.8.8"  19Nov2018           All LEDs are black (except IRQ on board) of screen is black
// "0.8.7"  19Nov2018           feed_watchdog is new etc. Plus a blank screen
// "0.8.6"  19Nov2018           RX_context.appSeqCnt_prev and calculations around this fixed (had been compromised by some code cut and paste)
// "0.8.5"  18Nov2018           SCREEN_4_MAX_NA_MIN is new (with Watt etc.)
// "0.8.4"  18Nov2018           Helper functions to convert and print pluss _USERMAKEFILE_LIB_RFM69_XC_SEMANTICS_DO_INTERMEDIATE_RECEIVEDONE=0 again
// "0.8.2"  13Nov2018           IRQ and timeout handling now separate functions in _Aquarium_rfm69_client.xc
// "0.8.1"  02Nov2018           DEG C and WATT ALSO DISPLAYED
// "0.7.2"  29Oct2018           THIS VERSION SHOWS AQUARIUM TIME AND WATER TEMP IN THE DISPLAY! YESS!
//                    RFM69=001 The IRQ line that keeps high after IRQ 7 (messagePacketLenErr_IRQ) is worked on. Testing at the moment
//                    --"--     This tag is also seen in the "rfm69_commprot.h" code
// "0.7.1"  07Jun2018           _commprot.h -> renamed to rfm69_commprot.h
//                              _globals.h  -> renamed to rfm69_globals.h and new _globals.h made
// "0.7.0"  06Jun2018           File changes before moving to lib_fsm69_xc
//                              RFM69.h           -> contents moved to rfm69_xc.xc then renamed to rfm69_xc.h
//                              RFM69_interface.h -> contents moved to rfm69_xc.h then file obsoleted
//                              RFM69.xc          -> renamed to rfm69_xc.xc
//                              RFM69.xc          -> renamed to rfm69_xc.xc
//                              spi_driver.h      -> renamed to rfm69_spi_driver.h
//                              spi_driver.xc     -> renamed to rfm69_spi_driver.xc
//                              CRC.h             -> renamed to rfm69_crc.h and uint32_t moved go _globals.h
//                              CRC.xc            -> renamed to rfm69_crc.xc
//                              RFM69_registers.h -> renamed to rfm69_registers.h
// "0.6.0"  05May2018           SEMANTICS_DO_LOOP_FOR_RF_IRQFLAGS2_PACKETSENT is new
// "0.5.5"  18Apr2018           This is a long-live number during development
// "0.5.3"  03Apr2018           SHARED_ID etc are new
// "0.5.2"                      File _commprot.h is new etc
// "0.5.1"  28Feb2018           more advanced handling
// "0.5.0"  26Feb2018           testing some changes that Maxim initiated
//          RFM69=000           First testing with two boards
//          28Jun2020 RFM69=000 To get xflash to work:
//                              XCORE-200-EXPLORER.xn (xTIMEcomposer 14.4.1)
//                              See https://www.teigfam.net/oyvind/home/technology/098-my-xmos-notes/#ticket_xflash_1441_of_xcore-200_explorer_board_warnings
//                                  <Device NodeId="0" Tile="0" Class="SQIFlash" Name="bootFlash" Type="S25LQ016B" PageSize="256" SectorSize="4096" NumPages="8192">
//                                  replaced with
//                                  <Device NodeId="0" Tile="0" Class="SQIFlash" Name="bootFlash" Type="S25LQ016B">

#endif /* VERSION_H_ */

