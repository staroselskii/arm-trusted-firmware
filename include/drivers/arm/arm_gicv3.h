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

#ifndef __ARM_GICV3_H__
#define __ARM_GICV3_H__

#include <stdint.h>

/*******************************************************************************
 * This structure describes some of the implementation defined attributes of the
 * GICv3 IP. It is used by the platform port to specify these attributes in order
 * to initialise the GICV3 driver. The attributes are described below.
 *
 * 1. The 'gicd_base' field contains the base address of the Distributor
 *    interface programmer's view.
 *
 * 2. The 'gicr_base' field contains the base address of the Re-distributor
 *    interface programmer's view.
 *
 * 3. The 'g0_interrupt_array' field is a ponter to an array in which each
 *    entry corresponds to an ID of a Group 0 interrupt.
 *
 * 4. The 'g0_interrupt_num' field contains the number of entries in the
 *    'g0_interrupt_array'.
 *
 * 5. The 'g1s_interrupt_array' field is a ponter to an array in which each
 *    entry corresponds to an ID of a Group 1 interrupt.
 *
 * 6. The 'g1s_interrupt_num' field contains the number of entries in the
 *    'g1s_interrupt_array'.
 *
 * 7. The 'rdistif_num' field contains the number of Redistributor interfaces
 *    the GIC implements. This is equal to the number of CPUs or CPU interfaces
 *    instantiated in the GIC.
 *
 * 8. The 'rdistif_base_addrs' field is a pointer to an array that has an entry
 *    for storing the base address of the Redistributor interface frame of each
 *    CPU in the system. The size of the array = 'rdistif_num'. The base
 *    addresses are detected during driver initialisation.
 *
 * 9. The 'mpidr_to_core_pos' field is a pointer to a hash function which the
 *    driver will use to convert an MPIDR value to a linear core index. This
 *    index will be used for accessing the 'rdistif_base_addrs' array. This is
 *    an optional field. A GICv3 implementation maps each MPIDR to a linear core
 *    index as well. This mapping can be found by reading the "Affinity Value"
 *    and "Processor Number" fields in the GICR_TYPER. It is IMP. DEF. if the
 *    "Processor Numbers" are suitable to index into an array to access core
 *    specific information. If this not the case, the platform port must provide
 *    a hash function. Otherwise, the "Processor Number" field will be used to
 *    access the array elements.
 ******************************************************************************/
typedef unsigned int (*mpidr_hash_fn)(unsigned long mpidr);
typedef struct arm_gicv3_driver_data {
	uintptr_t gicd_base;
	uintptr_t gicr_base;
	unsigned int g0_interrupt_num;
	unsigned int g1s_interrupt_num;
	const unsigned int *g0_interrupt_array;
	const unsigned int *g1s_interrupt_array;
	unsigned int rdistif_num;
	uintptr_t *rdistif_base_addrs;
	mpidr_hash_fn mpidr_to_core_pos;
} arm_gicv3_driver_data_t;

/*******************************************************************************
 * Function prototypes
 ******************************************************************************/
void arm_gicv3_driver_init(arm_gicv3_driver_data_t *plat_driver_data);
void arm_gicv3_distif_init(void);
void arm_gicv3_rdistif_init(unsigned int proc_num);
void arm_gicv3_cpuif_init(unsigned int proc_num);
void arm_gicv3_cpuif_deinit(unsigned int proc_num);
void arm_gicv3_end_of_interrupt(unsigned int id);
unsigned int arm_gicv3_get_pending_interrupt_type(void);
unsigned int arm_gicv3_get_pending_interrupt_id(void);
unsigned int arm_gicv3_acknowledge_interrupt(void);
unsigned int arm_gicv3_get_interrupt_type(unsigned int id,
					  unsigned int proc_num);
unsigned int arm_gicv3_interrupt_type_to_line(unsigned int type,
					      unsigned int security_state);


#endif /* __ARM_GICV3_H__ */
