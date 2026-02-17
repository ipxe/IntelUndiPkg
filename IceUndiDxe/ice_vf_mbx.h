/**************************************************************************

Copyright (c) 2016 - 2021, Intel Corporation

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of Intel Corporation nor the names of its contributors
      may be used to endorse or promote products derived from this software
      without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

***************************************************************************/

#ifdef SRIOV_SUPPORT
#ifndef _ICE_VF_MBX_H_
#define _ICE_VF_MBX_H_

#include "ice_type.h"
#include "ice_controlq.h"

/* Defining the mailbox message threshold as 63 asynchronous
 * pending messages. Normal VF functionality does not require
 * sending more than 63 asynchronous pending message.
 */

#ifdef E830_SUPPORT
 /* Threshold value should be used to initialize
  * MBX_VF_IN_FLIGHT_MSGS_AT_PF_CNT register.
  */
#endif /* E830_SUPPORT */
#define ICE_ASYNC_VF_MSG_THRESHOLD	63

#ifndef LINUX_SUPPORT
int
ice_aq_send_msg_to_pf(struct ice_hw *hw, enum virtchnl_ops v_opcode,
		      int v_retval, u8 *msg, u16 msglen,
		      struct ice_sq_cd *cd);
#endif /* !LINUX_SUPPORT */
#ifdef LINUX_SUPPORT
/* #ifdef CONFIG_PCI_IOV */
#endif /* LINUX_SUPPORT */
#ifdef AE_DRIVER
int
ice_aq_send_msg_to_pf(struct ice_hw *hw, u32 v_opcode, u32 v_retval, u8 *msg,
		      u16 msglen, struct ice_sq_cd *cd);

#endif /* AE_DRIVER */
int
ice_aq_send_msg_to_vf(struct ice_hw *hw, u16 vfid, u32 v_opcode, u32 v_retval,
		      u8 *msg, u16 msglen, struct ice_sq_cd *cd);

u32 ice_conv_link_speed_to_virtchnl(bool adv_link_support, u16 link_speed);

#ifdef E830_SUPPORT
void ice_e830_mbx_vf_dec_trig(struct ice_hw *hw,
			      struct ice_rq_event_info *event);
void ice_mbx_vf_clear_cnt_e830(struct ice_hw *hw, u16 vf_id);
#endif /* E830_SUPPORT */
int
ice_mbx_vf_state_handler(struct ice_hw *hw, struct ice_mbx_data *mbx_data,
			 struct ice_mbx_vf_info *vf_info, bool *report_malvf);
void ice_mbx_clear_malvf(struct ice_mbx_vf_info *vf_info);
void ice_mbx_init_vf_info(struct ice_hw *hw, struct ice_mbx_vf_info *vf_info);
void ice_mbx_init_snapshot(struct ice_hw *hw);
#ifdef LINUX_SUPPORT
/* #else CONFIG_PCI_IOV */
static inline int
ice_aq_send_msg_to_vf(struct ice_hw __always_unused *hw,
		      u16 __always_unused vfid, u32 __always_unused v_opcode,
		      u32 __always_unused v_retval, u8 __always_unused *msg,
		      u16 __always_unused msglen,
		      struct ice_sq_cd __always_unused *cd)
{
	return 0;
}

static inline u32
ice_conv_link_speed_to_virtchnl(bool __always_unused adv_link_support,
				u16 __always_unused link_speed)
{
	return 0;
}

static inline void ice_mbx_init_snapshot(struct ice_hw *hw)
{
}

/* #endif CONFIG_PCI_IOV */
#endif /* LINUX_SUPPORT */
#endif /* _ICE_VF_MBX_H_ */
#endif /* SRIOV_SUPPORT */
