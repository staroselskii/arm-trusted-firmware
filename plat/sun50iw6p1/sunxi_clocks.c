/*
 * Copyright (c) 2016, ARM Limited and Contributors. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of ARM nor the names of its contributors may be used
 * to endorse or promote products derived from this software without specific
 * prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <debug.h>
#include <mmio.h>
#include <ccmu.h>
#include "sunxi_private.h"

#define PLL_CPUX_1008MHZ    0x2900
#define PLL_CPUX_816MHZ     0x2100
#define PLL_CPUX_408MHZ     0x1000

#define PLL_LOCK_TIME	    (0x2UL << 24)

static void mmio_clrsetbits32(uintptr_t addr, uint32_t mask, uint32_t bits)
{
	uint32_t regval = mmio_read_32(addr);

	regval &= ~mask;
	regval |= bits;
	mmio_write_32(addr, regval);
}

static void mmio_setbits32(uintptr_t addr, uint32_t bits)
{
	uint32_t regval = mmio_read_32(addr);

	regval |= bits;
	mmio_write_32(addr, regval);
}

/* TODO (prt): we should have a timeout and return an error/success... */
static int pll_wait_until_stable(uintptr_t addr)
{
	while ((mmio_read_32(addr) & PLL_STABLE_BIT) != PLL_STABLE_BIT) {
		/* spin */
	}

	return 0;
}

int sunxi_setup_clocks(uint16_t socid)
{
	uint32_t reg;

	/* Avoid reprogramming PERIPH0 if not necessary */
	reg = mmio_read_32(CCMU_PLL_PERIPH0_CTRL_REG);
	if ((reg & 0x0ffff) != 0x6300)		/* is not at 600 MHz? */
		mmio_write_32(CCMU_PLL_PERIPH0_CTRL_REG, 0xa0006300);

	/* Set up dividers (suitable for the target clock frequency)
	   and switch CPUX (and thus AXI & APB) to the LOSC24 clock */
	mmio_write_32(CCMU_CPUX_AXI_CFG_REG, ( CPUX_SRCSEL_OSC24M |
					       APB_CLKDIV(4) |
					       AXI_CLKDIV(3) ));
	udelay(20);

	/* Set to 816MHz, but don't enable yet. */
	mmio_write_32(CCMU_PLL_CPUX_CTRL_REG, PLL_CPUX_816MHZ | PLL_LOCK_TIME);

	/* Enable PLL_CPUX again */
	mmio_setbits32(CCMU_PLL_CPUX_CTRL_REG, PLL_ENABLE_BIT);
	/* Wait until the PLL_CPUX becomes stable */
	pll_wait_until_stable(CCMU_PLL_CPUX_CTRL_REG);

	/* Wait another 20us, because Allwinner does so... */
	udelay(20);

	/* Switch AXI clock back to PLL_CPUX, dividers are set up already. */
	mmio_clrsetbits32(CCMU_CPUX_AXI_CFG_REG,
			  CPUX_SRCSEL_MASK, CPUX_SRCSEL_PLLCPUX);

	/* Wait 1000us, because Allwiner does so... */
	udelay(1000);

	/* AHB1/2 = PERIPH0 / (1 * 3) = 200MHz */
	mmio_write_32(CCMU_AHB1_AHB2_CFG_REG, 0x03000002);
	/* AHB3 = PERIPH0 / (2 * 1) = 300MHz    TODO: Really? Check! */
//	mmio_write_32(CCMU_AHB3_CFG_REG,      0x03000100);
	/* APB1 = PERIPH0 / (2 * 3) = 100MHz */
	mmio_write_32(CCMU_APB1_CFG_REG,      0x03000102);
	/* APB2 =>  24 MHz */
	mmio_write_32(CCMU_APB2_CFG_REG,      0x00000000);

	return 0;
}
