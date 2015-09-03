/*
 * Copyright (c) 2015, ARM Limited and Contributors. All rights reserved.
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

#include <arch.h>
#include <arch_helpers.h>
#include <arm_gic_common.h>
#include <arm_gicv3.h>
#include <assert.h>
#include <bl_common.h>
#include <debug.h>
#include <interrupt_mgmt.h>
#include "arm_gicv3_private.h"

static const arm_gicv3_driver_data_t *driver_data;
static unsigned int gicv2_compat;

/*******************************************************************************
 * This functon initialises the ARM GICv3 driver with provided platform inputs.
 ******************************************************************************/
void arm_gicv3_driver_init(arm_gicv3_driver_data_t *plat_driver_data)
{
	unsigned int gic_version;

	assert(plat_driver_data);
	assert(plat_driver_data->gicd_base);
	assert(plat_driver_data->gicr_base);
	assert(plat_driver_data->rdistif_num);
	assert(plat_driver_data->rdistif_base_addrs);

	/*
	 * The platform should provide a list of atleast one type of
	 * interrupts
	 */
	assert(plat_driver_data->g0_interrupt_array ||
	       plat_driver_data->g1s_interrupt_array);

	/*
	 * If there are no interrupts of a particular type, then the number of
	 * interrupts of that type should be 0 and vice-versa.
	 */
	assert(plat_driver_data->g0_interrupt_array ?
	       plat_driver_data->g0_interrupt_num :
	       plat_driver_data->g0_interrupt_num == 0);
	assert(plat_driver_data->g1s_interrupt_array ?
	       plat_driver_data->g1s_interrupt_num :
	       plat_driver_data->g1s_interrupt_num == 0);

	/* Ensure that the GIC supports the system register interface. */
	assert((read_id_aa64pfr0_el1() >> ID_AA64PFR0_GIC_SHIFT) &
	       ID_AA64PFR0_GIC_MASK);

	/* Ensure that the GIC is version 3.0 */
	gic_version = gicd_read_pidr2(plat_driver_data->gicd_base);
	gic_version >>=	PIDR2_ARCH_REV_SHIFT;
	gic_version &= PIDR2_ARCH_REV_MASK;
	assert(gic_version == ARCH_REV_GICV3);

	/*
	 * Find out whether the GIC supports the GICv2 compatibility mode. The
	 * ARE_S bit resets to 0 if supported
	 */
	gicv2_compat = gicd_read_ctlr(plat_driver_data->gicd_base);
	gicv2_compat >>= CTLR_ARE_S_SHIFT;
	gicv2_compat = !(gicv2_compat & CTLR_ARE_S_MASK);

	/*
	 * Find the base address of each implemented Redistributor interface.
	 * The number of interfaces should be equal to the number of CPUs in the
	 * system. The memory for saving these addresses has to be allocated by
	 * the platform port
	 */
	arm_gicv3_rdistif_base_addrs_probe(plat_driver_data->rdistif_base_addrs,
					   plat_driver_data->rdistif_num,
					   plat_driver_data->gicr_base,
					   plat_driver_data->mpidr_to_core_pos);

	driver_data = plat_driver_data;

	INFO("ARM GICV3 driver initialised %s legacy support\n",
	     gicv2_compat ? "with" : "without");
}

/*******************************************************************************
 * This function initialises the GIC distributor interface based upon the data
 * provided by the platform while initialising the driver.
 ******************************************************************************/
void arm_gicv3_distif_init(void)
{
	assert(driver_data);
	assert(driver_data->gicd_base);
	assert(driver_data->g1s_interrupt_array);
	assert(driver_data->g0_interrupt_array);

	/*
	 * Clear the "enable" bits for G0/G1S/G1NS interrupts before configuring
	 * the ARE_S bit. The Distributor might generate a system error
	 * otherwise.
	 */
	gicd_clr_ctlr(driver_data->gicd_base,
		      CTLR_ENABLE_G0_BIT |
		      CTLR_ENABLE_G1S_BIT |
		      CTLR_ENABLE_G1NS_BIT,
		      RWP_TRUE);

	/* Set the ARE_S bit now that interrupts have been disabled */
	if (gicv2_compat)
		gicd_set_ctlr(driver_data->gicd_base, CTLR_ARE_S_BIT, RWP_TRUE);

	/* Set the default attribute of all SPIs */
	arm_gicv3_spis_configure_defaults(driver_data->gicd_base);

	/* Configure the G1S SPIs */
	arm_gicv3_secure_spis_configure(driver_data->gicd_base,
					driver_data->g1s_interrupt_num,
					driver_data->g1s_interrupt_array,
					INT_TYPE_G1S);

	/* Configure the G0 SPIs */
	arm_gicv3_secure_spis_configure(driver_data->gicd_base,
					driver_data->g0_interrupt_num,
					driver_data->g0_interrupt_array,
					INT_TYPE_G0);

	/* Enable the secure SPIs now that they have been configured */
	gicd_set_ctlr(driver_data->gicd_base,
		      CTLR_ENABLE_G1S_BIT | CTLR_ENABLE_G0_BIT,
		      RWP_TRUE);
}

/*******************************************************************************
 * This function initialises the GIC Redistributor interface of the calling CPU
 * (identified by the 'proc_num' parameter) based upon the data provided by the
 * platform while initialising the driver.
 ******************************************************************************/
void arm_gicv3_rdistif_init(unsigned int proc_num)
{
	uintptr_t gicr_base;

	assert(proc_num < driver_data->rdistif_num);
	assert(driver_data->rdistif_base_addrs);
	assert(driver_data->gicd_base);
	assert(gicd_read_ctlr(driver_data->gicd_base) & CTLR_ARE_S_BIT);
	assert(driver_data->g1s_interrupt_array);
	assert(driver_data->g0_interrupt_array);

	gicr_base = driver_data->rdistif_base_addrs[proc_num];

	/* Set the default attribute of all SGIs and PPIs */
	arm_gicv3_ppi_sgi_configure_defaults(gicr_base);

	/* Configure the G1S SGIs/PPIs */
	arm_gicv3_secure_ppi_sgi_configure(gicr_base,
					   driver_data->g1s_interrupt_num,
					   driver_data->g1s_interrupt_array,
					   INT_TYPE_G1S);

	/* Configure the G0 SGIs/PPIs */
	arm_gicv3_secure_ppi_sgi_configure(gicr_base,
					   driver_data->g0_interrupt_num,
					   driver_data->g0_interrupt_array,
					   INT_TYPE_G0);
}

/*******************************************************************************
 * This function initialises the GIC CPU interface of the calling CPU using only
 * system register accesses.
 ******************************************************************************/
void arm_gicv3_cpuif_init(unsigned int proc_num)
{
	uintptr_t gicr_base;

	/* Disable the legacy interrupt bypass */
	unsigned int icc_sre_el3 = ICC_SRE_DIB_BIT | ICC_SRE_DFB_BIT;

	assert(proc_num < driver_data->rdistif_num);
	assert(driver_data->rdistif_base_addrs);

	/* Mark the connected core as awake */
	gicr_base = driver_data->rdistif_base_addrs[proc_num];
	gicr_write_waker(gicr_base, gicr_read_waker(gicr_base) & ~WAKER_PS_BIT);
	while (gicr_read_waker(gicr_base) & WAKER_CA_BIT);

	/*
	 * Enable system register access for EL3 and allow lower exception
	 * levels to configure the same for themselves. If the legacy mode is
	 * not supported, the SRE bit is RAO/WI
	 */
	icc_sre_el3 |= (gicv2_compat) ? (ICC_SRE_EN_BIT | ICC_SRE_SRE_BIT) : 0;
	write_icc_sre_el3(read_icc_sre_el3() | icc_sre_el3);
	isb();

	if (gicv2_compat) {
		unsigned int scr_el3 = read_scr_el3();

		/* Assert that we are in the secure state */
		assert((scr_el3 & SCR_NS_BIT) == 0);

		/*
		 * Switch to NS state to write Non secure ICC_SRE_EL1 and
		 * ICC_SRE_EL2 registers.
		 */
		write_scr_el3(scr_el3 | SCR_NS_BIT);
		isb();

		write_icc_sre_el2(read_icc_sre_el2() | icc_sre_el3);
		write_icc_sre_el1(ICC_SRE_SRE_BIT);
		isb();

		/*
		 * Switch to secure state to write secure ICC_SRE_EL1 register
		 */
		write_scr_el3(scr_el3 & (~SCR_NS_BIT));
		isb();

		write_icc_sre_el1(ICC_SRE_SRE_BIT);
		isb();
	}

	/* Program the idle priority in the PMR */
	write_icc_pmr_el1(GIC_PRI_MASK);

	/* Enable Group0 interrupts */
	write_icc_igrpen0_el1(IGRPEN1_EL1_ENABLE_G0_BIT);

	/* Enable Group1 Secure interrupts */
	write_icc_igrpen1_el3(read_icc_igrpen1_el3() | IGRPEN1_EL3_ENABLE_G1S_BIT);

	/* Synchronise accesses to group enable registers */
	isb();
}

/*******************************************************************************
 * This function deinitialises the GIC CPU interface of the calling CPU using
 * only system register accesses.
 ******************************************************************************/
void arm_gicv3_cpuif_deinit(unsigned int proc_num)
{
	uintptr_t gicr_base;

	assert(proc_num < driver_data->rdistif_num);
	assert(driver_data->rdistif_base_addrs);

	/* Disable legacy interrupt bypass */
	write_icc_sre_el3(read_icc_sre_el3() |
			  (ICC_SRE_DIB_BIT | ICC_SRE_DFB_BIT));

	/* Disable Group0 interrupts */
	write_icc_igrpen0_el1(read_icc_igrpen0_el1() &
			      ~IGRPEN1_EL1_ENABLE_G0_BIT);

	/* Disable Group1 Secure interrupts */
	write_icc_igrpen1_el3(read_icc_igrpen1_el3() &
			      ~IGRPEN1_EL3_ENABLE_G1S_BIT);

	/* Synchronise accesses to group enable registers */
	isb();

	/* Mark the connected core as asleep */
	gicr_base = driver_data->rdistif_base_addrs[proc_num];
	gicr_write_waker(gicr_base, gicr_read_waker(gicr_base) | WAKER_PS_BIT);
	while (!(gicr_read_waker(gicr_base) & WAKER_CA_BIT));
}

/*******************************************************************************
 * An ARM processor signals interrupt exceptions through the IRQ and FIQ pins.
 * The interrupt controller knows which pin/line it uses to signal a type of
 * interrupt. It lets the interrupt management framework determine for a type of
 * interrupt and security state, which line should be used in the SCR_EL3 to
 * control its routing to EL3. The interrupt line is represented as the bit
 * position of the IRQ or FIQ bit in the SCR_EL3.
 ******************************************************************************/
unsigned int arm_gicv3_interrupt_type_to_line(unsigned int type,
					      unsigned int security_state)
{
	assert(type == INTR_TYPE_S_EL1 ||
	       type == INTR_TYPE_EL3 ||
	       type == INTR_TYPE_NS);

	assert(sec_state_is_valid(security_state));

	switch (type) {
	case INTR_TYPE_S_EL1:
		if (security_state == SECURE)
			return __builtin_ctz(SCR_IRQ_BIT);
		else
			return __builtin_ctz(SCR_FIQ_BIT);
	case INTR_TYPE_NS:
		if (security_state == SECURE)
			return __builtin_ctz(SCR_FIQ_BIT);
		else
			return __builtin_ctz(SCR_IRQ_BIT);
	case INTR_TYPE_EL3:
	default:
		return __builtin_ctz(SCR_FIQ_BIT);
	}
}

/*******************************************************************************
 * This function returns the type of the highest priority pending interrupt at
 * the GIC cpu interface. INTR_TYPE_INVAL is returned when there is no
 * interrupt pending.
 ******************************************************************************/
unsigned int arm_gicv3_get_pending_interrupt_type(void)
{
	unsigned int id;

	id = read_icc_hppir0_el1() & HPPIR0_EL1_INTID_MASK;
	switch (id) {
	case PENDING_G1S_INTID:
		return INTR_TYPE_S_EL1;
	case PENDING_G1NS_INTID:
		return INTR_TYPE_NS;
	case GIC_SPURIOUS_INTERRUPT:
		return INTR_TYPE_INVAL;
	default:
		return INTR_TYPE_EL3;
	}
}


/*******************************************************************************
 * This function returns the id of the highest priority pending interrupt at
 * the GIC cpu interface. INTR_ID_UNAVAILABLE is returned when there is no
 * interrupt pending.
 ******************************************************************************/
unsigned int arm_gicv3_get_pending_interrupt_id(void)
{
	unsigned int id;

	if (IS_IN_EL3()) {
		id = read_icc_hppir0_el1() & HPPIR0_EL1_INTID_MASK;

		/* If the ID is less than 1020 then it is a valid G0 interrupt */
		if (id < PENDING_G1S_INTID)
			return id;

		if (id == GIC_SPURIOUS_INTERRUPT)
			return INTR_ID_UNAVAILABLE;

		/* Return the ID of the Group1 interrupt */
		return arm_gicv3_get_pending_grp1_interrupt_id(id);
	} else {
		id = read_icc_hppir1_el1() & HPPIR1_EL1_INTID_MASK;

		/* If the ID is less than 1020 then it is a valid G1 interrupt */
		if (id < PENDING_G1S_INTID)
			return id;

		return INTR_ID_UNAVAILABLE;
	}
}

/*******************************************************************************
 * This functions reads the GIC cpu interface Interrupt Acknowledge register
 * to start handling the pending interrupt. It returns the contents of the IAR.
 ******************************************************************************/
unsigned int arm_gicv3_acknowledge_interrupt(void)
{
	if (IS_IN_EL3())
		return read_icc_iar0_el1() & IAR0_EL1_INTID_MASK;
	return read_icc_iar1_el1() & IAR1_EL1_INTID_MASK;
}

/*******************************************************************************
 * This functions writes the GIC cpu interface End Of Interrupt register with
 * the passed value to finish handling the active interrupt
 ******************************************************************************/
void arm_gicv3_end_of_interrupt(unsigned int id)
{
	if (IS_IN_EL3())
		write_icc_eoir0_el1(id);
	else
		write_icc_eoir1_el1(id);
}

/*******************************************************************************
 * This function returns the type of the interrupt id depending upon the group
 * this interrupt has been configured under by the interrupt controller i.e.
 * group0 or group1.
 ******************************************************************************/
unsigned int arm_gicv3_get_interrupt_type(unsigned int id,
					  unsigned int proc_num)
{
	unsigned int group, grpmodr;
	uintptr_t gicr_base;

	if (id < MIN_SPI_ID) {
		assert(driver_data->rdistif_base_addrs);
		gicr_base = driver_data->rdistif_base_addrs[proc_num];
		group = gicr_get_igroupr0(gicr_base, id);
		grpmodr = gicr_get_igrpmodr0(gicr_base, id);
	} else {
		assert(driver_data->gicd_base);
		group = gicd_get_igroupr(driver_data->gicd_base, id);
		grpmodr = gicd_get_igrpmodr(driver_data->gicd_base, id);
	}

	if (group)
		return INTR_TYPE_NS;

	if (grpmodr)
		return INTR_TYPE_S_EL1;
	return INTR_TYPE_EL3;
}
