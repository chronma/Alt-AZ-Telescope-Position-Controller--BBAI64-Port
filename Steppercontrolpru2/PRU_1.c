/*Redone for BBAI64
 * CJM 6/24
 *
 *
 * Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *  * Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdint.h>
#include <pru_cfg.h>
#include <pru_ctrl.h>
#include <pru_iep.h>
#include "resource_table_empty.h"
#include "pru0_pinmap.h"

/* Mapping Constant Table (CT) registers to variables */
// volatile far uint8_t CT_MCSPI0 __attribute__((cregister("MCSPI0", near), peripheral));

#ifndef PRU_SRAM
#define PRU_SRAM __far __attribute__((cregister("PRU_SHAREDMEM", near)))
#endif

#ifndef PRU_DMEM0
#define PRU_DMEM0 far __attribute__((cregister("PRU0_DMEM_0",  near)))
#endif

#ifndef PRU_DMEM1
#define PRU_DMEM1 far __attribute__((cregister("PRU0_DMEM_1",  near)))
#endif

#define BitVal(data,y)     ( (data>>y) & 1)       /** Return Data.Y value   **/
#define SetBit(data,y)     data |= (1 << y)       /** Set Data.Y   to 1     **/
#define ClearBit(data,y)   data &= ~(1 << y)      /** Clear Data.Y to 0     **/
#define TogleBit(data,y)   (data ^=BitVal(y))     /** Togle Data.Y  value   **/
#define Togle(data)        (data =~data )         /** Togle Data value      **/

// ************************************************************************
/* NOTE:  Allocating shared_freq_x to PRU Shared Memory means that other PRU cores on
 *        the same subsystem must take care not to allocate data to that memory.
 *        Users also cannot rely on where in shared memory these variables are placed
 *        so accessing them from another PRU core or from the ARM is an undefined behavior.
*/

// PRU_SRAM volatile uint32_t shared_0; (reference declaration)
//PRU_DMEM0 volatile uint32_t shared_2; (reference declaration)

/* NOTE:  Here we pick where in memory to store shared_5.  The stack and
 *        heap take up the first 0x200 words, so we must start after that.
 *        Since we are hardcoding where things are stored we can share
 *        this between the PRUs and the ARM.
*/
// Skip the first 0x200 bytes of DRAM since the Makefile allocates
// 0x100 for the STACK and 0x100 for the HEAP.


PRU_SRAM volatile uint32_t *pru0_command_bits = (uint32_t*)(0x000); // buffer for command bits not sure how to size (malloc?)
PRU_SRAM volatile uint32_t *pru_0_axis_databuffer = (uint32_t*)(0x100); // buffer for axis data place holder

// PRU0 Axis Data Buffer

//  [0]A1steps [1]A1PulseLength (nanoseconds)
//  [2]A2steps [3]A2PulseLength
//  [4]A3steps [5]A3PulseLength
//  [6]A1adsteps
//  [7]A2adsteps
//  [8]A3adsteps
//  [9]A1steps@reset command
//  [10]A2steps@reset command
//  [11]A3steps@reset command
//
// Set up commandbits at offset 000 (dmem)

/* PRU0

 * bit 0 - 0=ok to write new values for axis 1 (pru0)
 * bit 1 - 0=ok to write new values for axis 2 (pru0)
 * bit 2 - 0=ok to write new values for axis 3 (pru0)
 * bit 3 - Axis 1 direction
 * bit 4 - Axis 2 direction
 * bit 5 - Axis 3 direction
 * bit 6 - Axis 1 limit input set
 * bit 7 - Axis 2 limit input set
 * bit 8 - Axis 3 limit input set
 * bit 9 - lock Axis 1 (arm program cannot write new instruction until cleared (set to 0)
 * bit 10 - lock Axis 2 (arm program cannot write new instruction until cleared (set to 0)
 * bit 11 - lock Axis 3 (arm program cannot write new instruction until cleared (set to 0)
 * bit 12 - reset pru 0
 * bit
  */


void reset_iep(void)  //https://catch22eu.github.io/website/beaglebone/beaglebone-pru-c/
{                     //https://e2e.ti.com/support/processors-group/processors/f/processors-forum/995643/ct_iep-tmr_glb_sts_bit-cnt_ovf-behavior-not-working-as-expected
     // Set counter to 0
     //CT_IEP0.TMR_CNT = 0x0;
     CT_IEP0.count_reg0 =0;
     // clear over flow
     //CT_IEP0.TMR_GLB_STS_bit.CNT_OVF = 0x1;
     CT_IEP0.global_status_reg_bit.cnt_ovf = 1;
     // Enable counter
     //CT_IEP0.TMR_GLB_CFG = 0x11;
     CT_IEP0.global_cfg_reg_bit.cnt_enable = 1;
}

int read_iep(void)
{
     // Return counter content
     //return CT_IEP0.TMR_CNT;
     return CT_IEP0.count_reg0;
}

void reset_pru(unsigned int A1pulsesatreset, unsigned int A2pulsesatreset, unsigned int A3pulsesatreset)
{
        pru_0_axis_databuffer[9]=A1pulsesatreset;
        pru_0_axis_databuffer[10]=A2pulsesatreset;
        pru_0_axis_databuffer[11]=A3pulsesatreset;
        CT_PRU0_CTRL.CONTROL_bit.SOFT_RST_N=0;
        CT_PRU0_CTRL.CONTROL_bit.RESTART=0;

}


// *************************************************************************

/* PRCM Registers */ //not sure how these are used yet
//#define CM_PER_BASE ((volatile uint8_t *)(0x44E00000))
//#define SPI0_CLKCTRL  (0x4C)
//#define ON (0x2)
//#define MCSPI0_MODULCTRL (*((volatile uint32_t*)(&CT_MCSPI0 + 0x128)))

/* PRU-to-ARM interrupt */
//#define PRU_ARM_INTERRUPT (19+16)

volatile register uint32_t __R30;
volatile register uint32_t __R31;

int main(void)
{

    /*****************************************************************/
    /* Access PRU peripherals using Constant Table & PRU header file */
    /*****************************************************************/

    /* Clear SYSCFG[STANDBY_INIT] to enable OCP master port */
    //CT_CFG.SYSCFG_bit.STANDBY_INIT = 0;

    /* Read IEPCLK[OCP_EN] for IEP clock source */
    //result = CT_CFG.IEPCLK_bit.OCP_EN;


    /*****************************************************************/
    /* Access PRU Shared RAM using Constant Table                    */
    /*****************************************************************/

    /* C28 defaults to 0x00000000, we need to set bits 23:8 to 0x0100 in order to have it point to 0x00010000    */
    CT_PRU0_CTRL.CONSTANT_TABLE_PROG_PTR_0_bit.C28_POINTER = 0x0100;
    // set up

    uint8_t member_number =0;
    pru0_command_bits[0]=0; //make sure nothing starts moving at start
    uint32_t A1Steps=0;
    uint32_t A2Steps=0;
    uint32_t A3Steps=0;
    uint32_t A1pulselength=0;
    uint32_t A2pulselength=0;
    uint32_t A3pulselength=0;

    short cmd_bit_holder=0;
    uint32_t A1stepcounter=0;
    uint32_t A2stepcounter=0;
    uint32_t A3stepcounter=0;
    uint32_t A1time=0;
    uint32_t A2time=0;
    uint32_t A3time=0;
    uint32_t A1adsteps=0;
    uint32_t A1acounter=0;
    uint32_t A1startdecel=0;
    uint32_t A2adsteps=0;
    uint32_t A2acounter=0;
    uint32_t A2startdecel=0;
    uint32_t A3adsteps=0;
    uint32_t A3acounter=0;
    uint32_t A3startdecel=0;

    for (member_number=0; member_number <16; member_number ++){ // zero out 16 words of the data buffer helps to trouble shoot
        pru_0_axis_databuffer[member_number]=0x0000000000000000;
    }

    reset_iep(); // initialize IEP

    while (1) {
    // command bit 12 resets program this will be tied to e-stop possibly end limits.

    if (BitVal(pru0_command_bits[0],12)==1) {  // resets pru1 and restarts program
        ClearBit(pru0_command_bits[0],12);

        reset_pru(A1stepcounter,A2stepcounter,A3stepcounter);
        }

    // check for stop P9_39  PRUIN

    if((__R31&P9_39)) { // bit 8 stop input
                SetBit(pru0_command_bits[0],8);
                reset_pru(A1stepcounter,A2stepcounter,A3stepcounter);
    }

    // set up axis 1  P9_29 direction pin, P9_28 pulse
    if (BitVal(pru0_command_bits[0],0)==1) {  //check bit 0 if its ok to set axis 4 data
                    A1Steps=pru_0_axis_databuffer[0]*2; // two pulses is on wavelength
                    A1pulselength=(pru_0_axis_databuffer[1]/2); // pulse+pulse = step
                    A1adsteps= pru_0_axis_databuffer[6];
                    A1startdecel= A1Steps-A1adsteps;
                    A1acounter=A1adsteps;
                    pru_0_axis_databuffer[0]=0;
                    pru_0_axis_databuffer[1]=0;
                    pru_0_axis_databuffer[6]=0;
                    if (BitVal(pru0_command_bits[0],3)==0) {// set direction right away
                    __R30 &= ~P9_29; // P9_29 is Axis 1 Direction
                    } else {
                    __R30 |= P9_29;
                        }
                    ClearBit(pru0_command_bits[0],0);
                    SetBit(cmd_bit_holder,0);
                    SetBit(pru0_command_bits[0],9); // send message to arm program to not write new commands to axis4
    }
       // set up axis 2  P9_31 direction pin, P9_30 pulse
    if (BitVal(pru0_command_bits[0],1)==1) {  //check bit 0 if its ok to set axis 5 data
                    A2Steps=pru_0_axis_databuffer[2]*2; // two pulses is on wavelength
                    A2pulselength=(pru_0_axis_databuffer[3]/2); // pulse+pulse = step
                    A2adsteps= pru_0_axis_databuffer[7];
                    A2startdecel= A2Steps-A2adsteps;
                    A2acounter=A2adsteps;
                    pru_0_axis_databuffer[2]=0;
                    pru_0_axis_databuffer[3]=0;
                    pru_0_axis_databuffer[7]=0;

                    if (BitVal(pru0_command_bits[0],4)==0) {// set direction right away
                    __R30 &= ~P9_31; // P9_31 is Axis 2 Direction
                    } else {
                    __R30 |= P9_31;
                    }
                     ClearBit(pru0_command_bits[0],1);
                     SetBit(cmd_bit_holder,1);
                     SetBit(pru0_command_bits[0],10); // send message to arm program to not write new commands to axis2
               }

        // set up axis 3  P9_35 direction pin, P9_33 pulse
        if (BitVal(pru0_command_bits[0],2)==1) {  //check bit 0 if its ok to set axis 6 data
                     A3Steps=pru_0_axis_databuffer[4]*2; // two pulses is on wavelength;;
                     A3pulselength=(pru_0_axis_databuffer[5]/2); // pulse+pulse = step
                     A3adsteps= pru_0_axis_databuffer[8];
                     A3startdecel= A3Steps-A3adsteps;
                     A3acounter=A3adsteps;
                     pru_0_axis_databuffer[4]=0;
                     pru_0_axis_databuffer[5]=0;
                     pru_0_axis_databuffer[8]=0;
                     if (BitVal(pru0_command_bits[0],5)==0) {// set direction right away
                     __R30 &= ~P9_35; // P9_35 is Axis 6 Direction
                     } else {
                     __R30 |= P9_35;
                     }
                     ClearBit(pru0_command_bits[0],2);
                     SetBit(cmd_bit_holder,2);
                     SetBit(pru0_command_bits[0],11); // send message to arm program to not write new commands to axis3
                 }



       while (cmd_bit_holder!=0) {

           // check end limits P9_36, P9_37, P9_38  PRUIN

               if((__R31&P9_36)) { //bit 2 - Axis 1 limit input set
                   SetBit(pru0_command_bits[0],6);
                   reset_pru(A1stepcounter,A2stepcounter,A3stepcounter);
               }
               if((__R31&P9_37)) { //bit 3 - Axis 2 limit input set
                       SetBit(pru0_command_bits[0],7);
                       reset_pru(A1stepcounter,A2stepcounter,A3stepcounter);
               }
               if((__R31&P9_38)) { //bit 4 - Axis 3 limit input set
                           SetBit(pru0_command_bits[0],8);
                           reset_pru(A1stepcounter,A2stepcounter,A3stepcounter);
               }


           // Axis 1 pulse generator
        if (A1Steps > 0) {

                    if (A1stepcounter==0){
                        A1time=0;
                    }

                    if ((read_iep()-A1time)>= A1pulselength*A1acounter){ // toggle the pulse every iep time interval that equals pulse length
                            __R30 ^= P9_28;
                            A1time=read_iep();  // not perfect when the timer rolls over may get a weird pulse (timer rolls over every 21 sec or so)
                            A1stepcounter++;
                            if (A1stepcounter<A1adsteps){
                                if (A1acounter>0) {
                                    A1acounter--;
                                }
                            }
                            else if (A1stepcounter<A1startdecel){
                            A1acounter=1;
                            }
                            else {
                            A1acounter++;
                            }

                    }
                    if (A1stepcounter==A1Steps){
                        A1stepcounter=0;
                        A1Steps=0;
                        __R30 &= ~(P9_28);
                        ClearBit(cmd_bit_holder,0);
                        ClearBit(pru0_command_bits[0],9); // send message to arm program ok write new commands to axis1
                    }
        }
        else {
            ClearBit(cmd_bit_holder,0);
            ClearBit(pru0_command_bits[0],9); // send message to arm program ok write new commands to axis1
        }
        // Axis 2 pulse generator
        if (A2Steps > 0) {

                    if (A2stepcounter==0){
                        A2time=0;
                    }

                    if ((read_iep()-A2time)>= A2pulselength*A2acounter){ // toggle the pulse every iep time interval that equals pulse length
                        __R30 ^= P9_30;
                        A2time=read_iep();  // not perfect when the timer rolls over may get a weird pulse (timer rolls over every 21 sec or so)
                        A2stepcounter++;
                        if (A2stepcounter<A2adsteps){
                            if (A2acounter>0) {
                                A2acounter--;
                            }
                        }
                        else if (A2stepcounter<A2startdecel){
                            A2acounter=1;
                        }
                        else {
                             A2acounter++;
                        }

                    }
                    if (A2stepcounter==A2Steps){
                        A2stepcounter=0;
                        A2Steps=0;
                        __R30 &= ~(P9_30);
                        ClearBit(cmd_bit_holder,1);
                        ClearBit(pru0_command_bits[0],10); // send message to arm program ok write new commands to axis2
                    }}
        else {
            ClearBit(cmd_bit_holder,1);
            ClearBit(pru0_command_bits[0],10); // send message to arm program ok write new commands to axis2
        }
        // Axis 3 pulse generator (P9_33)
                if (A3Steps > 0) {

                    if (A3stepcounter==0){
                        A3time=0;

                    }

                    if ((read_iep()-A3time)>= A3pulselength*A3acounter){ // toggle the pulse every iep time interval that equals pulse length
                        __R30 ^= P9_33;
                        A3time=read_iep();
                        A3stepcounter++;
                        if (A3stepcounter<A3adsteps){
                            if (A3acounter>0) {
                                A3acounter--;
                            }
                        }
                        else if (A3stepcounter<A3startdecel){
                            A3acounter=1;
                        }
                        else {
                              A3acounter++;
                        }

                    }
                    if (A3stepcounter==A3Steps){
                        A3stepcounter=0;
                        A3Steps=0;
                        __R30 &= ~(P9_33);
                        ClearBit(cmd_bit_holder,2);
                        ClearBit(pru0_command_bits[0],11); // send message to arm program ok write new commands to axis3
                    }}
                else {
                    ClearBit(cmd_bit_holder,2);
                    ClearBit(pru0_command_bits[0],11); // send message to arm program ok write new commands to axis3
        }

    }
       __delay_cycles(1000000); // poll memory every 1/200th second
       __R30^= P8_16;  // test heartbeat
       pru_0_axis_databuffer[13]=read_iep();
       if((__R31&P9_36)) { //bit 2 - Axis 1 limit input set
                          SetBit(pru0_command_bits[0],6);
                          pru_0_axis_databuffer[12]=read_iep();
                          reset_iep();
       }


} //end of interupt loop
} //end of main
