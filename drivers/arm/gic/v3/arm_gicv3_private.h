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

#ifndef __ARM_GICV3_PRIVATE_H__
#define __ARM_GICV3_PRIVATE_H__

#include <arm_gicv3.h>
#include <mmio.h>
#include <stdint.h>

/*******************************************************************************
 * GICv3 miscellaneous definitions
 ******************************************************************************/
/* Constants to indicate the status of the RWP bit */
#define RWP_TRUE		1
#define RWP_FALSE		0

/* Interrupt group definitions */
#define INT_TYPE_G0		0
#define INT_TYPE_G1S		1
#define INT_TYPE_G1NS		2

/* Interrupt IDs reported by the HPPIR and IAR registers */
#define PENDING_G1S_INTID	1020
#define PENDING_G1NS_INTID	1021

/*******************************************************************************
 * GICv3 specific Distributor interface register offsets and constants.
 ******************************************************************************/
#define GICD_STATUSR		0x10
#define GICD_SETSPI_NSR		0x40
#define GICD_CLRSPI_NSR		0x48
#define GICD_SETSPI_SR		0x50
#define GICD_CLRSPI_SR		0x50
#define GICD_IGRPMODR		0xd00
#define GICD_IROUTER		0x6100

#define IGRPMODR_SHIFT		5

/* GICv3 revision as reported by the PIDR2 register */
#define ARCH_REV_GICV3		0x3

/* GICD_CTLR bit definitions */
#define CTLR_ENABLE_G0_SHIFT		0
#define CTLR_ENABLE_G1NS_SHIFT		1
#define CTLR_ENABLE_G1S_SHIFT		2
#define CTLR_ARE_S_SHIFT		4
#define CTLR_ARE_NS_SHIFT		5
#define CTLR_DS_SHIFT			6
#define CTLR_E1NWF_SHIFT		7
#define GICD_CTLR_RWP_SHIFT		31

#define CTLR_ENABLE_G0_MASK		0x1
#define CTLR_ENABLE_G1NS_MASK		0x1
#define CTLR_ENABLE_G1S_MASK		0x1
#define CTLR_ARE_S_MASK			0x1
#define CTLR_ARE_NS_MASK		0x1
#define CTLR_DS_MASK			0x1
#define CTLR_E1NWF_MASK			0x1
#define GICD_CTLR_RWP_MASK		0x1

#define CTLR_ENABLE_G0_BIT		(1U << CTLR_ENABLE_G0_SHIFT)
#define CTLR_ENABLE_G1NS_BIT		(1U << CTLR_ENABLE_G1NS_SHIFT)
#define CTLR_ENABLE_G1S_BIT		(1U << CTLR_ENABLE_G1S_SHIFT)
#define CTLR_ARE_S_BIT			(1U << CTLR_ARE_S_SHIFT)
#define CTLR_ARE_NS_BIT			(1U << CTLR_ARE_NS_SHIFT)
#define CTLR_DS_BIT			(1U << CTLR_DS_SHIFT)
#define CTLR_E1NWF_BIT			(1U << CTLR_E1NWF_SHIFT)
#define GICD_CTLR_RWP_BIT		(1U << GICD_CTLR_RWP_SHIFT)

/*
 * Macro to wait for updates to :
 * GICD_CTLR[2:0] - the Group Enables
 * GICD_CTLR[5:4] - the ARE bits
 * GICD_ICENABLERn - the clearing of enable state for SPIs
 */
#define gicd_wait_for_pending_write(gicd_base)			\
	while (gicd_read_ctlr(gicd_base) & GICD_CTLR_RWP_BIT);

/* GICD_IROUTER shifts and masks */
#define IROUTER_IRM_SHIFT	31
#define IROUTER_IRM_MASK	0x1

/*
 * Macro to convert an mpidr to a value suitable for programming into a
 * GICD_IROUTER. Bits[31:24] in the MPIDR are cleared as they are not relevant
 * to GICv3.
 */
#define gicd_irouter_val_from_mpidr(mpidr, irm)		\
	((mpidr & ~(0xff << 24)) | 			\
	 (irm & IROUTER_IRM_MASK) << IROUTER_IRM_SHIFT)

/*******************************************************************************
 * GICv3 Re-distributor interface registers & constants
 ******************************************************************************/
#define GICR_PCPUBASE_SHIFT	0x11
#define GICR_SGIBASE_OFFSET	(1 << 0x10)	/* 64 KB */
#define GICR_CTLR		0x0
#define GICR_TYPER		0x08
#define GICR_WAKER		0x14
#define GICR_IGROUPR0		(GICR_SGIBASE_OFFSET + 0x80)
#define GICR_ISENABLER0		(GICR_SGIBASE_OFFSET + 0x100)
#define GICR_ICENABLER0		(GICR_SGIBASE_OFFSET + 0x180)
#define GICR_IPRIORITYR		(GICR_SGIBASE_OFFSET + 0x400)
#define GICR_ICFGR0		(GICR_SGIBASE_OFFSET + 0xc00)
#define GICR_ICFGR1		(GICR_SGIBASE_OFFSET + 0xc04)
#define GICR_IGRPMODR0		(GICR_SGIBASE_OFFSET + 0xd00)

/* GICR_CTLR bit definitions */
#define GICR_CTLR_RWP_SHIFT	3
#define GICR_CTLR_RWP_MASK	0x1
#define GICR_CTLR_RWP_BIT	(1U << GICR_CTLR_RWP_SHIFT)

/*
 * Macro to wait for updates to :
 * GICR_ICENABLER0
 * GICR_CTLR.DPG1S
 * GICR_CTLR.DPG1NS
 * GICR_CTLR.DPG0
 */
#define gicr_wait_for_pending_write(gicr_base)			\
	while (gicr_read_ctlr(gicr_base) & GICR_CTLR_RWP_BIT);

/* GICR_WAKER bit definitions */
#define WAKER_CA_SHIFT		2
#define WAKER_PS_SHIFT		1

#define WAKER_CA_MASK		0x1
#define WAKER_PS_MASK		0x1

#define WAKER_CA_BIT		(1U << WAKER_CA_SHIFT)
#define WAKER_PS_BIT		(1U << WAKER_PS_SHIFT)

/* GICR_TYPER bit definitions */
#define TYPER_AFF_VAL_SHIFT	32
#define TYPER_PROC_NUM_SHIFT	8
#define TYPER_LAST_SHIFT	4

#define TYPER_AFF_VAL_MASK	0xffffffff
#define TYPER_PROC_NUM_MASK	0xffff
#define TYPER_LAST_MASK		0x1

#define TYPER_LAST_BIT		(1UL << TYPER_LAST_SHIFT)

/*
 * Macro to convert a GICR_TYPER affinity value into a MPIDR value. Bits[31:24]
 * are zeroes.
 */
#define mpidr_from_gicr_typer(typer_val)				 \
	((((typer_val >> 56) & MPIDR_AFFLVL_MASK) << MPIDR_AFF3_SHIFT) | \
	 ((typer_val >> 32) & 0xffffff))

/*******************************************************************************
 * GICv3 CPU interface registers & constants
 ******************************************************************************/
/* ICC_SRE bit definitions*/
#define ICC_SRE_EN_BIT		(1U << 3)
#define ICC_SRE_DIB_BIT		(1U << 2)
#define ICC_SRE_DFB_BIT		(1U << 1)
#define ICC_SRE_SRE_BIT		(1U << 0)

/* ICC_IGRPEN1_EL3 bit definitions */
#define IGRPEN1_EL3_ENABLE_G1NS_SHIFT	0
#define IGRPEN1_EL3_ENABLE_G1S_SHIFT	1

#define IGRPEN1_EL3_ENABLE_G1NS_BIT	(1U << IGRPEN1_EL3_ENABLE_G1NS_SHIFT)
#define IGRPEN1_EL3_ENABLE_G1S_BIT	(1U << IGRPEN1_EL3_ENABLE_G1S_SHIFT)

/* ICC_IGRPEN0_EL1 bit definitions */
#define IGRPEN1_EL1_ENABLE_G0_SHIFT	0
#define IGRPEN1_EL1_ENABLE_G0_BIT	(1U << IGRPEN1_EL1_ENABLE_G0_SHIFT)

/* ICC_HPPIR0_EL1 bit definitions */
#define HPPIR0_EL1_INTID_SHIFT		0
#define HPPIR0_EL1_INTID_MASK		0xffffff

/* ICC_HPPIR1_EL1 bit definitions */
#define HPPIR1_EL1_INTID_SHIFT		0
#define HPPIR1_EL1_INTID_MASK		0xffffff

/* ICC_IAR0_EL1 bit definitions */
#define IAR0_EL1_INTID_SHIFT		0
#define IAR0_EL1_INTID_MASK		0xffffff

/* ICC_IAR1_EL1 bit definitions */
#define IAR1_EL1_INTID_SHIFT		0
#define IAR1_EL1_INTID_MASK		0xffffff

/*******************************************************************************
 * Private function prototypes
 ******************************************************************************/
unsigned int gicd_read_igrpmodr(uintptr_t base, unsigned int id);
unsigned int gicr_read_ipriorityr(uintptr_t base, unsigned int id);
unsigned int gicd_get_igrpmodr(uintptr_t base, unsigned int id);
unsigned int gicr_get_igrpmodr0(uintptr_t base, unsigned int id);
unsigned int gicr_get_igroupr0(uintptr_t base, unsigned int id);
unsigned int arm_gicv3_get_pending_grp1_interrupt_id(unsigned int pending_grp);
void gicd_write_igrpmodr(uintptr_t base, unsigned int id, unsigned int val);
void gicr_write_ipriorityr(uintptr_t base, unsigned int id, unsigned int val);
void gicd_set_igrpmodr(uintptr_t base, unsigned int id);
void gicr_set_igrpmodr0(uintptr_t base, unsigned int id);
void gicr_set_isenabler(uintptr_t base, unsigned int id);
void gicr_set_igroupr0(uintptr_t base, unsigned int id);
void gicd_clr_igrpmodr(uintptr_t base, unsigned int id);
void gicr_clr_igrpmodr0(uintptr_t base, unsigned int id);
void gicr_clr_igroupr0(uintptr_t base, unsigned int id);
void arm_gicv3_spis_configure_defaults(uintptr_t gicd_base);
void arm_gicv3_ppi_sgi_configure_defaults(uintptr_t gicr_base);
void arm_gicv3_secure_spis_configure(uintptr_t gicd_base,
				     unsigned int num_ints,
				     const unsigned int *sec_intr_list,
				     unsigned int int_grp);
void arm_gicv3_secure_ppi_sgi_configure(uintptr_t gicr_base,
					unsigned int num_ints,
					const unsigned int *sec_intr_list,
					unsigned int int_grp);
void arm_gicv3_rdistif_base_addrs_probe(uintptr_t *rdistif_base_addrs,
					unsigned int rdistif_num,
					uintptr_t gicr_base,
					mpidr_hash_fn mpidr_to_core_pos);

/*******************************************************************************
 * GIC Distributor interface accessors
 ******************************************************************************/
static inline uint64_t gicd_read_irouter(uintptr_t base, unsigned int id)
{
	return mmio_read_64(base + GICD_IROUTER + (id << 3));
}

static inline void gicd_write_irouter(uintptr_t base,
				      unsigned int id,
				      uint64_t affinity)
{
	mmio_write_64(base + GICD_IROUTER + (id << 3), affinity);
}

static inline void gicd_clr_ctlr(uintptr_t base,
				 unsigned int bitmap,
				 unsigned int rwp)
{
	gicd_write_ctlr(base, gicd_read_ctlr(base) & ~bitmap);
	if (rwp)
		gicd_wait_for_pending_write(base);
}

static inline void gicd_set_ctlr(uintptr_t base,
				 unsigned int bitmap,
				 unsigned int rwp)
{
	gicd_write_ctlr(base, gicd_read_ctlr(base) | bitmap);
	if (rwp)
		gicd_wait_for_pending_write(base);
}

/*******************************************************************************
 * GIC Redistributor interface accessors
 ******************************************************************************/
static inline uint64_t gicr_read_ctlr(uintptr_t base)
{
	return mmio_read_64(base + GICR_CTLR);
}

static inline uint64_t gicr_read_typer(uintptr_t base)
{
	return mmio_read_64(base + GICR_TYPER);
}

static inline uint32_t gicr_read_waker(uintptr_t base)
{
	return mmio_read_32(base + GICR_WAKER);
}

static inline void gicr_write_waker(uintptr_t base, uint32_t val)
{
	mmio_write_32(base + GICR_WAKER, val);
}

static inline uint32_t gicr_read_icenabler0(uintptr_t base)
{
	return mmio_read_32(base + GICR_ICENABLER0);
}

static inline void gicr_write_icenabler0(uintptr_t base, uint32_t val)
{
	mmio_write_32(base + GICR_ICENABLER0, val);
}

static inline uint32_t gicr_read_isenabler0(uintptr_t base)
{
	return mmio_read_32(base + GICR_ISENABLER0);
}

static inline void gicr_write_isenabler0(uintptr_t base, uint32_t val)
{
	mmio_write_32(base + GICR_ISENABLER0, val);
}

static inline uint32_t gicr_read_igroupr0(uintptr_t base)
{
	return mmio_read_32(base + GICR_IGROUPR0);
}

static inline void gicr_write_igroupr0(uintptr_t base, uint32_t val)
{
	mmio_write_32(base + GICR_IGROUPR0, val);
}

static inline uint32_t gicr_read_igrpmodr0(uintptr_t base)
{
	return mmio_read_32(base + GICR_IGRPMODR0);
}

static inline void gicr_write_igrpmodr0(uintptr_t base, uint32_t val)
{
	mmio_write_32(base + GICR_IGRPMODR0, val);
}

static inline uint32_t gicr_read_icfgr1(uintptr_t base)
{
	return mmio_read_32(base + GICR_ICFGR1);
}

static inline void gicr_write_icfgr1(uintptr_t base, uint32_t val)
{
	mmio_write_32(base + GICR_ICFGR1, val);
}

#endif /* __ARM_GICV3_PRIVATE_H__ */
