/*
 * J721E_PRU0.cmd
 *
 * Example Linker command file for linking programs built with the C compiler
 * on J721E PRU0 cores
 *
 * Copyright (C) 2019-2021 Texas Instruments Incorporated - https://www.ti.com/
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *	* Redistributions of source code must retain the above copyright
 *	  notice, this list of conditions and the following disclaimer.
 *
 *	* Redistributions in binary form must reproduce the above copyright
 *	  notice, this list of conditions and the following disclaimer in the
 *	  documentation and/or other materials provided with the
 *	  distribution.
 *
 *	* Neither the name of Texas Instruments Incorporated nor the names of
 *	  its contributors may be used to endorse or promote products derived
 *	  from this software without specific prior written permission.
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

-cr		/* Link using C conventions */

/* Specify the System Memory Map */
MEMORY
{
      PAGE 0:
	/* 12 KB PRU Instruction RAM */
	PRU_IMEM	: org = 0x00000000 len = 0x00001000

      PAGE 1:
	/* Data RAMs */
	/* 8 KB PRU Data RAM 0; use only the first 4 KB for PRU0 and reserve
	 * the second 4 KB for RTU0 and Tx_PRU0 */
	PRU0_DMEM_0	: org = 0x00000000 len = 0x00001000	CREGISTER=24
	/* 8 KB PRU Data RAM 1; reserved completely for Slice1 cores - PRU1,
	 * RTU1 and Tx_PRU1; do not use for any Slice0 cores */
	PRU0_DMEM_1	: org = 0x00002000 len = 0x00000500	CREGISTER=25
	/* NOTE: Custom split of the second 4 KB of ICSS Data RAMs 0 and 1
	 * split equally between the corresponding RTU and Tx_PRU cores in
	 * each slice */
	RTU0_DMEM_0	: org = 0x00001000 len = 0x00000800
	TX_PRU0_DMEM_0	: org = 0x00001800 len = 0x00000800
	RTU0_DMEM_1	: org = 0x00003000 len = 0x00000800
	TX_PRU0_DMEM_1	: org = 0x00003800 len = 0x00000800

      PAGE 2:
	/* C28 needs to be programmed to point to SHAREDMEM, default is 0 */
	/* 64 KB PRU Shared RAM */
	PRU_SHAREDMEM	: org = 0x00010000 len = 0x00010000	CREGISTER=28

	/* Internal Peripherals */
	/* NOTE: Use full INTC length instead of 0x200 to match the pruIntc
	 * structure definition in pru_intc.h, ignoring the second Constant
	 * Register #6 that starts at 0x200 offset within INTC */
	PRU_INTC	: org = 0x00020000 len = 0x00001504	CREGISTER=0
	PRU_IEP1	: org = 0x0002F000 len = 0x00000100	CREGISTER=1
	PRU_IEP1_0x100	: org = 0x0002F100 len = 0x0000021C	CREGISTER=2
	PRU_ECAP	: org = 0x00030000 len = 0x00000060	CREGISTER=3
	PRU_CFG		: org = 0x00026000 len = 0x00000100	CREGISTER=4
	PRU_CFG_0x100	: org = 0x00026100 len = 0x00000098	CREGISTER=5
	/* XXX: This value is part of INTC space, and is therefore commented
	 * out as it conflicts with PRU_INTC size above. Using this requires
	 * splitting up the pruIntc structure and CT_INTC variable from
	 * pru_intc.h */
	/*PRU_INTC_0x200: org = 0x00020200 len = 0x00001304	CREGISTER=6*/
	PRU_UART	: org = 0x00028000 len = 0x00000038	CREGISTER=7
	PRU_IEP0_0x100	: org = 0x0002E400 len = 0x0000021C	CREGISTER=8
	MII_G_RT	: org = 0x00033000 len = 0x00000C18	CREGISTER=9
	TM_CFG_PRU0	: org = 0x0002A000 len = 0x0000004C	CREGISTER=10
	PRU0_CTRL	: org = 0x00022000 len = 0x00000088	CREGISTER=11
	/* FIXME: PA_STATS_QRAM and CRAM assigned random sizes of 0x100 */
	PA_STATS_QRAM	: org = 0x00027000 len = 0x00000100	CREGISTER=12
	PA_STATS_CRAM	: org = 0x0002C000 len = 0x00000100	CREGISTER=13
	MII_MDIO	: org = 0x00032400 len = 0x00000090	CREGISTER=21
	PRU_RTU_RAT0	: org = 0x00008000 len = 0x00000854	CREGISTER=22
	PRU_IEP0	: org = 0x0002E000 len = 0x00000400	CREGISTER=26
	MII_RT		: org = 0x00032000 len = 0x0000024C	CREGISTER=27

	/* External Regions */
	/* Random length 0x1000 assigned to the below regions */
	RSVD14		: org = 0x00024800 len = 0x00000100	CREGISTER=14
	RSVD15		: org = 0x60000000 len = 0x00001000	CREGISTER=15
	RSVD16		: org = 0x70000000 len = 0x00001000	CREGISTER=16
	RSVD17		: org = 0x80000000 len = 0x00001000	CREGISTER=17
	RSVD18		: org = 0x90000000 len = 0x00001000	CREGISTER=18
	RSVD19		: org = 0xA0000000 len = 0x00001000	CREGISTER=19
	RSVD20		: org = 0xB0000000 len = 0x00001000	CREGISTER=20
	RSVD23		: org = 0xC0000000 len = 0x00001000	CREGISTER=23
	/* Random length 0x10000 (max len value) assigned to programmable C29-31*/
	RSVD29		: org = 0xD0000000 len = 0x00010000	CREGISTER=29
	RSVD30		: org = 0xE0000000 len = 0x00010000	CREGISTER=30
	RSVD31		: org = 0xF0000000 len = 0x00010000	CREGISTER=31
}

/* Specify the sections allocation into memory */
SECTIONS {
	/* Forces _c_int00 to the start of PRU IRAM. Not necessary when loading
	   an ELF file, but useful when loading a binary */
	//.text:_c_int00*	>  0x0, PAGE 0

	.text		>  PRU_IMEM, PAGE 0
	.stack		>  PRU0_DMEM_0, PAGE 1
	.bss		>  PRU0_DMEM_0, PAGE 1
	.cio		>  PRU0_DMEM_0, PAGE 1
	.data		>  PRU0_DMEM_0, PAGE 1
	.switch		>  PRU0_DMEM_0, PAGE 1
	.sysmem		>  PRU0_DMEM_0, PAGE 1
	.cinit		>  PRU0_DMEM_0, PAGE 1
	.rodata		>  PRU0_DMEM_0, PAGE 1
	.rofardata	>  PRU0_DMEM_0, PAGE 1
	.farbss		>  PRU0_DMEM_0, PAGE 1
	.fardata	>  PRU0_DMEM_0, PAGE 1
	/* Ensure resource_table section is aligned on 8-byte address for
	   ARMv8 (64-bit) kernel */
	.resource_table : ALIGN (8) >  PRU0_DMEM_0, PAGE 1
}
