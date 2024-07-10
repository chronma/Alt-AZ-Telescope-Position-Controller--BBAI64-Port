// This file defines the GPIO port addresses and PRU address
// Should work for all BB, but wireless versions and video pins will not let some pru pins be set to PRU mode
// I.E.  Config-pin P9.29 pruout will fail on BBG wireless
/* modify /boot/uEnv.txt example turn off video to free up some P8 pins

#disable_uboot_overlay_emmc=1
disable_uboot_overlay_video=1  (remove # to make active)
#disable_uboot_overlay_audio=1
#disable_uboot_overlay_wireless=1
#disable_uboot_overlay_adc=1
linnux cmd to see pinmux
ls /sys/devices/platform/ocp/ | grep pinmux*
*/

/* P8 PRG_PRU0 (avoid boot pins) (first 9 and mode 0 for pinmux)
 * P8_3 - PRG1_PRU0_GPO19   P9_28 - PRG0_PRU0_GPO0
 * P8_7 - PRG1_PRU0_GPO14   P9_29 - PRG0_PRU0_GPO10
 * P8_8 - PRG1_PRU0_GPO13   P9_30 - PRG0_PRU0_GPO1
 * P8_9 - PRG1_PRU0_GPO16   P9_31 - PRG0_PRU0_GPO9
 * P8_10 - PRG1_PRU0_GPO15  P9_33 - PRG0_PRU0_GPO7
 * P8_15 - PRG0_PRU0_GPO18  P9_35 - PRG0_PRU0_GPO12
 * P8_17 - PRG0_PRU0_GPO02  P9_36 - PRG0_PRU0_GPO13
 * P8_18 - PRG0_PRU0_GPO03  P9_37 - PRG0_PRU0_GPO14
 * P8_22 - PRG1_PRU0_GPO04  P9_38 - PRG0_PRU0_GPO15
 * P8_26 - PRG0_PRU0_GPO08  P9_39 - PRG0_PRU0_GPO11
 * P8_16 - PRG0_PRU0_GPO19
 */


// R30 output bits / R31 input bits on pru0 (BBG default PINMUX )
#define P9_28   (1UL<<0) // a1 pulse pruout
#define P9_29   (1UL<<10) // a1 dir  pruout
#define P9_30   (1UL<<1) // a2 pulse pruout
#define P9_31   (1UL<<9) // a2 dir pruout
#define P9_33   (1UL<<7) // a3 pulse pruout
#define P9_35   (1UL<<12) // a3 dir pruout
#define P9_36   (1UL<<13) // a1 limit pruin
#define P9_37   (1UL<<14) // a2 limit pruin
#define P9_38   (1UL<<15) // a3 limit pruin
#define P9_39   (1UL<<11) // stop
#define P8_16   (1UL<<19) // test output (square wave at input check read frequency)







