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

#ifndef _ICE_COMMON_H_
#define _ICE_COMMON_H_

#include "ice_type.h"
#include "ice_nvm.h"
#ifndef NO_FLEXP_SUPPORT
#include "ice_flex_pipe.h"
#if defined(DPDK_SUPPORT) || defined(ADV_AVF_SUPPORT)
#include "ice_parser.h"
#endif /* DPDK_SUPPORT || ADV_AVF_SUPPORT */
#endif
#ifdef SRIOV_SUPPORT
#include "virtchnl.h"
#endif /* SRIOV_SUPPORT */
#if defined(PF_DRIVER) || defined(INTEGRATED_VF)
#include "ice_switch.h"
#ifdef FDIR_SUPPORT
#include "ice_fdir.h"
#endif /* FDIR_SUPPORT */
#endif /* defined(PF_DRIVER) || defined(INTEGRATED_VF) */


#define ICE_SQ_SEND_DELAY_TIME_MS	10
#define ICE_SQ_SEND_MAX_EXECUTE		3

#define LOOPBACK_MODE_NO       0
#define LOOPBACK_MODE_HIGH     2

#ifndef NO_RECOVERY_MODE_SUPPORT
enum ice_fw_modes {
	ICE_FW_MODE_NORMAL,
	ICE_FW_MODE_DBG,
	ICE_FW_MODE_REC,
	ICE_FW_MODE_ROLLBACK
};
#endif /* !NO_RECOVERY_MODE_SUPPORT */

enum ice_cgd_per_port {
	ICE_4_CGD_PER_PORT = 4,
	ICE_8_CGD_PER_PORT = 8
};

#ifdef GLOBAL_CFG_LOCK_SUPPORT
#ifndef ICE_DEFINE_LOCK
#error "You must provide ICE_DEFINE_LOCK in ice_osdep.h if you want global configuration lock support"
#endif /* !ICE_DEFINE_LOCK */
#endif /* GLOBAL_CFG_LOCK_SUPPORT */

#ifndef ICE_TDD
#if !defined(LINUX_SUPPORT) && !defined(DPDK_SUPPORT)
#if !defined(FREEBSD_SUPPORT)
/* prototype for functions used for SW locks */
void ice_free_list(struct LIST_HEAD_TYPE *list);
void ice_init_lock(struct ice_lock *lock);
void ice_acquire_lock(struct ice_lock *lock);
void ice_release_lock(struct ice_lock *lock);
void ice_destroy_lock(struct ice_lock *lock);
#ifdef LEGACY_PREBOOT_SUPPORT
void *ice_alloc_dma_mem(struct ice_hw *hw, struct ice_dma_mem *m, u16 size);
#else /* !LEGACY_PREBOOT_SUPPORT */
void *ice_alloc_dma_mem(struct ice_hw *hw, struct ice_dma_mem *m, u64 size);
#endif /* LEGACY_PREBOOT_SUPPORT */
void ice_free_dma_mem(struct ice_hw *hw, struct ice_dma_mem *m);
#endif /* !FREEBSD_SUPPORT */

void ice_idle_aq(struct ice_hw *hw, struct ice_ctl_q_info *cq);
bool ice_sq_done(struct ice_hw *hw, struct ice_ctl_q_info *cq);
#endif /* !LINUX_SUPPORT && !DPDK_SUPPORT */

int ice_init_fltr_mgmt_struct(struct ice_hw *hw);
void ice_cleanup_fltr_mgmt_struct(struct ice_hw *hw);
void ice_set_umac_shared(struct ice_hw *hw);
int ice_init_hw(struct ice_hw *hw);
void ice_deinit_hw(struct ice_hw *hw);
int ice_check_reset(struct ice_hw *hw);
int ice_reset(struct ice_hw *hw, enum ice_reset_req req);
#ifdef INTEGRATED_VF
bool ice_is_vf(struct ice_hw *hw);
#endif
int ice_create_all_ctrlq(struct ice_hw *hw);
int ice_init_all_ctrlq(struct ice_hw *hw);
void ice_shutdown_all_ctrlq(struct ice_hw *hw, bool unloading);
void ice_destroy_all_ctrlq(struct ice_hw *hw);
#ifdef LEGACY_PREBOOT_SUPPORT
int ice_init_check_adminq(struct ice_hw *hw);
#endif /* LEGACY_PREBOOT_SUPPORT */
int
ice_clean_rq_elem(struct ice_hw *hw, struct ice_ctl_q_info *cq,
		  struct ice_rq_event_info *e, u16 *pending);
int
ice_get_link_status(struct ice_port_info *pi, bool *link_up);
int ice_update_link_info(struct ice_port_info *pi);
int
ice_acquire_res(struct ice_hw *hw, enum ice_aq_res_ids res,
		enum ice_aq_res_access_type access, u32 timeout);
void ice_release_res(struct ice_hw *hw, enum ice_aq_res_ids res);
#ifndef NO_FLEXP_SUPPORT
int
ice_alloc_hw_res(struct ice_hw *hw, u16 type, u16 num, bool btm, u16 *res);
int
ice_free_hw_res(struct ice_hw *hw, u16 type, u16 num, u16 *res);
#endif /* !NO_FLEXP_SUPPORT */
int
ice_aq_alloc_free_res(struct ice_hw *hw, u16 num_entries,
		      struct ice_aqc_alloc_free_res_elem *buf, u16 buf_size,
		      enum ice_adminq_opc opc, struct ice_sq_cd *cd);
#ifndef AE_DRIVER
#ifndef NO_SBQ_SUPPORT
int
ice_sq_send_cmd_nolock(struct ice_hw *hw, struct ice_ctl_q_info *cq,
		       struct ice_aq_desc *desc, void *buf, u16 buf_size,
		       struct ice_sq_cd *cd);
#endif /* !NO_SBQ_SUPPORT */
#endif /* !AE_DRIVER */
int
ice_sq_send_cmd(struct ice_hw *hw, struct ice_ctl_q_info *cq,
		struct ice_aq_desc *desc, void *buf, u16 buf_size,
		struct ice_sq_cd *cd);
#ifndef PREBOOT_SUPPORT
void ice_clear_pxe_mode(struct ice_hw *hw);
#endif /* !PREBOOT_SUPPORT */
int ice_get_caps(struct ice_hw *hw);

#if defined(DDP_SUPPORT) || !defined(NO_FLEXP_SUPPORT)
void ice_set_safe_mode_caps(struct ice_hw *hw);
#endif /* DDP_SUPPORT || !NO_FLEXP_SUPPORT */

int
ice_aq_get_internal_data(struct ice_hw *hw, u16 cluster_id, u16 table_id,
			 u32 start, void *buf, u16 buf_size, u16 *ret_buf_size,
			 u16 *ret_next_cluster, u16 *ret_next_table,
			 u32 *ret_next_index, struct ice_sq_cd *cd);

#if !defined(LINUX_SUPPORT) && !defined(DPDK_SUPPORT) || defined(AE_DRIVER)
int ice_set_mac_type(struct ice_hw *hw);
#endif

#if defined(FPGA_SUPPORT) || defined(E822S_SUPPORT) || !defined(NO_SBQ_SUPPORT) 
void ice_dev_onetime_setup(struct ice_hw *hw);
#endif /* FPGA_SUPPORT || E822S_SUPPORT || (!NO_SBQ_SUPPORT && SWITCH_MODE) */

#ifndef LINUX_SUPPORT
/* Define a macro that will align a pointer to point to the next memory address
 * that falls on the given power of 2 (i.e., 2, 4, 8, 16, 32, 64...). For
 * example, given the variable pointer = 0x1006, then after the following call:
 *
 *      pointer = ICE_ALIGN(pointer, 4)
 *
 * ... the value of pointer would equal 0x1008, since 0x1008 is the next
 * address after 0x1006 which is divisible by 4.
 */
#define ICE_ALIGN(ptr, align)	(((ptr) + ((align) - 1)) & ~((align) - 1))

/* Define a macro for initializing array using indexes. Due to limitation
 * of MSVC compiler it is necessary to allow other projects to replace
 * that macro and strip the index from initialization.
 * Linux driver is using coccinelle to maintain source sync with upstream
 * and is not requiring this macro.
 */
#ifndef ice_arr_elem_idx
#define ice_arr_elem_idx(idx, val)	[(idx)] = (val)
#endif /* ice_arr_elem_idx */
#endif

int
ice_write_rxq_ctx(struct ice_hw *hw, struct ice_rlan_ctx *rlan_ctx,
		  u32 rxq_index);
int
ice_read_rxq_ctx(struct ice_hw *hw, struct ice_rlan_ctx *rlan_ctx,
		 u32 rxq_index);
#ifdef LINUX_SUPPORT
int
ice_read_txq_ctx(struct ice_hw *hw, struct ice_tlan_ctx *tlan_ctx,
		 u32 txq_index);
#endif /* LINUX_SUPPORT */
#if !defined(NO_UNUSED_CTX_CODE) || defined(AE_DRIVER)
int ice_clear_rxq_ctx(struct ice_hw *hw, u32 rxq_index);
int
ice_clear_tx_cmpltnq_ctx(struct ice_hw *hw, u32 tx_cmpltnq_index);
int
ice_write_tx_cmpltnq_ctx(struct ice_hw *hw,
			 struct ice_tx_cmpltnq_ctx *tx_cmpltnq_ctx,
			 u32 tx_cmpltnq_index);
int
ice_clear_tx_drbell_q_ctx(struct ice_hw *hw, u32 tx_drbell_q_index);
int
ice_write_tx_drbell_q_ctx(struct ice_hw *hw,
			  struct ice_tx_drbell_q_ctx *tx_drbell_q_ctx,
			  u32 tx_drbell_q_index);
#endif /* !NO_UNUSED_CTX_CODE || AE_DRIVER */

#ifdef AE_DRIVER
int
ice_copy_tx_cmpltnq_ctx_to_hw(struct ice_hw *hw, u8 *ice_tx_cmpltnq_ctx,
			      u32 tx_cmpltnq_index);
int
ice_copy_tx_drbell_q_ctx_to_hw(struct ice_hw *hw, u8 *ice_tx_drbell_q_ctx,
			       u32 tx_drbell_q_index);
int
ice_copy_rxq_ctx_to_hw(struct ice_hw *hw, u8 *ice_rxq_ctx, u32 rxq_index);
#endif /* AE_DRIVER */
int
ice_aq_get_rss_lut(struct ice_hw *hw, struct ice_aq_get_set_rss_lut_params *get_params);
int
ice_aq_set_rss_lut(struct ice_hw *hw, struct ice_aq_get_set_rss_lut_params *set_params);
int
ice_aq_get_rss_key(struct ice_hw *hw, u16 vsi_handle,
		   struct ice_aqc_get_set_rss_keys *keys);
int
ice_aq_set_rss_key(struct ice_hw *hw, u16 vsi_handle,
		   struct ice_aqc_get_set_rss_keys *keys);
#if !defined(LINUX_SUPPORT) || defined(AE_DRIVER)
int
ice_aq_add_lan_txq(struct ice_hw *hw, u8 count,
		   struct ice_aqc_add_tx_qgrp *qg_list, u16 buf_size,
		   struct ice_sq_cd *cd);
#endif /* !LINUX_SUPPORT || AE_DRIVER */
#if defined(PREBOOT_SUPPORT) || defined(AE_DRIVER)
int
ice_aq_dis_lan_txq(struct ice_hw *hw, u8 num_qgrps,
		   struct ice_aqc_dis_txq_item *qg_list, u16 buf_size,
		   enum ice_disq_rst_src rst_src, u16 vmvf_num,
		   struct ice_sq_cd *cd);
#endif /* PREBOOT_SUPPORT || AE_DRIVER */
#if !defined(NO_UNUSED_SCHED_CODE) || defined(AE_DRIVER)
int
ice_aq_move_recfg_lan_txq(struct ice_hw *hw, u8 num_qs, bool is_move,
			  bool is_tc_change, bool subseq_call, bool flush_pipe,
			  u8 timeout, u32 *blocked_cgds,
			  struct ice_aqc_move_txqs_data *buf, u16 buf_size,
			  u8 *txqs_moved, struct ice_sq_cd *cd);
#endif /* !NO_UNUSED_SCHED_CODE || AE_DRIVER */

#if defined(RDMA_SUPPORT) && !defined(LINUX_SUPPORT)
int
ice_aq_add_rdma_qsets(struct ice_hw *hw, u8 num_qset_grps,
		      struct ice_aqc_add_rdma_qset_data *qset_list,
		      u16 buf_size, struct ice_sq_cd *cd);

#endif /* RDMA_SUPPORT && !LINUX_SUPPORT */
#ifdef TXPP_SUPPORT
int
ice_aq_set_txtimeq(struct ice_hw *hw, u16 txtimeq, u8 q_count,
		   struct ice_aqc_set_txtime_qgrp *txtime_qg,
		   u16 buf_size, struct ice_sq_cd *cd);
int
ice_aq_ena_dis_txtimeq(struct ice_hw *hw, u16 txtimeq, u16 q_count, bool q_ena,
		       struct ice_aqc_ena_dis_txtime_qgrp *txtime_qg,
		       struct ice_sq_cd *cd);
extern const struct ice_ctx_ele ice_txtime_ctx_info[];
#endif /* TXPP_SUPPORT */
bool ice_check_sq_alive(struct ice_hw *hw, struct ice_ctl_q_info *cq);
#ifdef QV_SUPPORT
int ice_aq_q_shutdown(struct ice_hw *hw, struct ice_ctl_q_info *cq, bool unloading);
#else /* !QV_SUPPORT */
int ice_aq_q_shutdown(struct ice_hw *hw, bool unloading);
#endif /* QV_SUPPORT */
void ice_fill_dflt_direct_cmd_desc(struct ice_aq_desc *desc, u16 opcode);
extern const struct ice_ctx_ele ice_tlan_ctx_info[];
#ifdef AE_DRIVER
extern const struct ice_ctx_ele ice_rlan_ctx_info[];
extern const struct ice_ctx_ele ice_tx_cmpltnq_ctx_info[];
extern const struct ice_ctx_ele ice_tx_drbell_q_ctx_info[];
#endif /* AE_DRIVER */
int
ice_set_ctx(struct ice_hw *hw, u8 *src_ctx, u8 *dest_ctx,
	    const struct ice_ctx_ele *ce_info);
int
ice_get_ctx(u8 *src_ctx, u8 *dest_ctx, const struct ice_ctx_ele *ce_info);

#if !defined(NO_FLEXP_SUPPORT) && defined(GLOBAL_CFG_LOCK_SUPPORT)
extern struct ice_lock ice_global_cfg_lock_sw;
#endif /* !NO_FLEXP_SUPPORT && GLOBAL_CFG_LOCK_SUPPORT */

#ifdef QV_SUPPORT
int
ice_aq_send_cmd_sc(struct ice_hw *hw, struct ice_aq_desc *desc,
		   void *buf, u16 buf_size, struct ice_sq_cd *cd);
#endif /* QV_SUPPORT */
int
ice_aq_send_cmd(struct ice_hw *hw, struct ice_aq_desc *desc,
		void *buf, u16 buf_size, struct ice_sq_cd *cd);
int ice_aq_get_fw_ver(struct ice_hw *hw, struct ice_sq_cd *cd);

int
ice_aq_send_driver_ver(struct ice_hw *hw, struct ice_driver_ver *dv,
		       struct ice_sq_cd *cd);
int
ice_aq_set_port_params(struct ice_port_info *pi, u16 bad_frame_vsi,
		       bool save_bad_pac, bool pad_short_pac, bool double_vlan,
		       struct ice_sq_cd *cd);
int
ice_aq_get_phy_caps(struct ice_port_info *pi, bool qual_mods, u8 report_mode,
		    struct ice_aqc_get_phy_caps_data *caps,
		    struct ice_sq_cd *cd);
#if !defined(NO_PTP_SUPPORT)  || defined(QV_SUPPORT)
int
ice_aq_get_netlist_node_pin(struct ice_hw *hw,
			    struct ice_aqc_get_link_topo_pin *cmd,
			    u16 *node_handle);
#endif /* (!NO_PTP_SUPPORT && !SWITCH_MODE) || QV_SUPPORT */
int
ice_aq_get_netlist_node(struct ice_hw *hw, struct ice_aqc_get_link_topo *cmd,
			u8 *node_part_number, u16 *node_handle);
int
ice_find_netlist_node(struct ice_hw *hw, u8 node_type_ctx, u8 node_part_number,
		      u16 *node_handle);
#ifdef SYNCE_SUPPORT
int ice_get_pf_c827_idx(struct ice_hw *hw, u8 *idx);
bool ice_is_pf_c827(struct ice_hw *hw);
bool ice_is_phy_rclk_in_netlist(struct ice_hw *hw);
bool ice_is_clock_mux_in_netlist(struct ice_hw *hw);
#endif /* SYNCE_SUPPORT */
#ifdef GNSS_SUPPORT
bool ice_is_gps_in_netlist(struct ice_hw *hw);
#endif /* GNSS_SUPPORT */
#if defined(LINUX_SUPPORT) || defined(QV_SUPPORT)
int
ice_aq_list_caps(struct ice_hw *hw, void *buf, u16 buf_size, u32 *cap_count,
		 enum ice_adminq_opc opc, struct ice_sq_cd *cd);
#endif /* LINUX_SUPPORT || QV_SUPPORT */
#if defined(LINUX_SUPPORT)
int
ice_discover_dev_caps(struct ice_hw *hw, struct ice_hw_dev_caps *dev_caps);
#endif /* LINUX_SUPPORT */
#if defined(E830_SUPPORT) && defined(QV_SUPPORT)
int
ice_aq_update_oem_rot_revocation_struct(struct ice_hw *hw, void *buf,
					u16 buf_size, struct ice_sq_cd *cd);
int
ice_aq_key_revocation(struct ice_hw *hw, u8 cmd_flags,
		      void *buf, u8 buf_size, struct ice_sq_cd *cd);
#endif /* E830_SUPPORT && QV_SUPPORT */
void
ice_update_phy_type(u64 *phy_type_low, u64 *phy_type_high,
		    u16 link_speeds_bitmap);
#if !defined(LINUX_SUPPORT) && !defined(DPDK_SUPPORT)
int
ice_aq_manage_mac_read(struct ice_hw *hw, void *buf, u16 buf_size,
		       struct ice_sq_cd *cd);
#endif /* !LINUX_SUPPORT && !DPDK_SUPPORT */
int
ice_aq_manage_mac_write(struct ice_hw *hw, const u8 *mac_addr, u8 flags,
			struct ice_sq_cd *cd);

#if !defined(PREBOOT_SUPPORT) && !defined(QV_SUPPORT)
int ice_clear_pf_cfg(struct ice_hw *hw);
#endif /* !PREBOOT_SUPPORT && !QV_SUPPORT */
int
ice_aq_set_phy_cfg(struct ice_hw *hw, struct ice_port_info *pi,
		   struct ice_aqc_set_phy_cfg_data *cfg, struct ice_sq_cd *cd);
bool ice_fw_supports_link_override(struct ice_hw *hw);
bool ice_fw_supports_fec_dis_auto(struct ice_hw *hw);
int
ice_get_link_default_override(struct ice_link_default_override_tlv *ldo,
			      struct ice_port_info *pi);
bool ice_is_phy_caps_an_enabled(struct ice_aqc_get_phy_caps_data *caps);
#ifndef NO_DNL_SUPPORT
int ice_aq_get_phy_equalization(struct ice_hw *hw, u16 data_in, u16 op_code,
						u8 serdes_num, int *output);
#endif /* NO_DNL_SUPPORT */
#ifndef NO_UNUSED_LINK_PHY_CODE
#endif /* !NO_UNUSED_LINK_PHY_CODE*/
#if !defined(FPGA_SUPPORT) 
enum ice_fc_mode ice_caps_to_fc_mode(u8 caps);
enum ice_fec_mode ice_caps_to_fec_mode(u8 caps, u8 fec_options);
#endif /* !FPGA_SUPPORT && !SWITCH_MODE */
int
ice_set_fc(struct ice_port_info *pi, u8 *aq_failures,
	   bool ena_auto_link_update);
#if defined(WINDOWS_SUPPORT) || defined(LINUX_SUPPORT)
int
ice_cfg_phy_fc(struct ice_port_info *pi, struct ice_aqc_set_phy_cfg_data *cfg,
	       enum ice_fc_mode req_mode);
#endif /* WINDOWS_SUPPORT || LINUX_SUPPORT */
#if !defined(FPGA_SUPPORT) 
bool
ice_phy_caps_equals_cfg(struct ice_aqc_get_phy_caps_data *caps,
			struct ice_aqc_set_phy_cfg_data *cfg);
#endif /* !FPGA_SUPPORT && !SWITCH_MODE */
void
ice_copy_phy_caps_to_cfg(struct ice_port_info *pi,
			 struct ice_aqc_get_phy_caps_data *caps,
			 struct ice_aqc_set_phy_cfg_data *cfg);
#if !defined(FPGA_SUPPORT)
int
ice_cfg_phy_fec(struct ice_port_info *pi, struct ice_aqc_set_phy_cfg_data *cfg,
		enum ice_fec_mode fec);
#endif /* !SWITCH_MODE && !FPGA_SUPPORT */
int
ice_aq_set_link_restart_an(struct ice_port_info *pi, bool ena_link,
			   struct ice_sq_cd *cd);
int
ice_aq_set_mac_cfg(struct ice_hw *hw, u16 max_frame_size, bool auto_drop,
		   struct ice_sq_cd *cd);
int
ice_aq_set_mac_pfc_cfg(struct ice_hw *hw, u16 max_frame_size, u8 tc_bitmap, u16 xoff_quanta,
	u16 xoff_thresh, bool auto_drop, struct ice_sq_cd *cd);
#if defined(WINDOWS_SUPPORT)
int ice_enable_lse_interrupt(struct ice_port_info *pi);
#endif /* WINDOWS_SUPPORT */
#if !defined(FPGA_SUPPORT) || defined(QV_SUPPORT)
int
ice_aq_get_link_info(struct ice_port_info *pi, bool ena_lse,
		     struct ice_link_status *link, struct ice_sq_cd *cd);
int
ice_aq_set_event_mask(struct ice_hw *hw, u8 port_num, u16 mask,
		      struct ice_sq_cd *cd);
#endif /* !FPGA_SUPPORT || QV_SUPPORT */
int
ice_aq_set_mac_loopback(struct ice_hw *hw, bool ena_lpbk, struct ice_sq_cd *cd);

#if defined(QV_SUPPORT)
int
ice_aq_set_phy_debug(struct ice_hw *hw, u8 port_num, u8 cmd_flags, u8 phy_index,
		     struct ice_sq_cd *cd);
#endif /* INTERNAL_ONLY || QV_SUPPORT */

int
ice_aq_set_port_id_led(struct ice_port_info *pi, bool is_orig_mode,
		       struct ice_sq_cd *cd);
int
ice_aq_sff_eeprom(struct ice_hw *hw, u16 lport, u8 bus_addr,
		  u16 mem_addr, u8 page, u8 set_page, u8 *data, u8 length,
		  bool write, struct ice_sq_cd *cd);
u32 ice_get_link_speed(u16 index);

int
ice_aq_prog_topo_dev_nvm(struct ice_hw *hw,
			 struct ice_aqc_link_topo_params *topo_params,
			 struct ice_sq_cd *cd);
int
ice_aq_read_topo_dev_nvm(struct ice_hw *hw,
			 struct ice_aqc_link_topo_params *topo_params,
			 u32 start_address, u8 *buf, u8 buf_size,
			 struct ice_sq_cd *cd);


#ifndef NO_UNUSED_DEBUG_CODE
void ice_dump_port_info(struct ice_port_info *pi);
void ice_dump_caps(struct ice_hw *hw);
#ifndef NO_PTP_SUPPORT
void ice_dump_ptp_dev_caps(struct ice_hw *hw);
void ice_dump_ptp_func_caps(struct ice_hw *hw);
#endif /* !NO_PTP_SUPPORT */
int ice_dump_port_dflt_topo(struct ice_port_info *pi);
void ice_dump_port_topo(struct ice_port_info *pi);
#endif /* NO_UNUSED_DEBUG_CODE */

#ifndef NO_UNUSED_LINK_PHY_CODE
#endif /* !NO_UNUSED_LINK_PHY_CODE*/
int
ice_aq_get_port_options(struct ice_hw *hw,
			struct ice_aqc_get_port_options_elem *options,
			u8 *option_count, u8 lport, bool lport_valid,
			u8 *active_option_idx, bool *active_option_valid,
			u8 *pending_option_idx, bool *pending_option_valid);
int
ice_aq_set_port_option(struct ice_hw *hw, u8 lport, u8 lport_valid,
		       u8 new_option);
#if !defined(LINUX_SUPPORT) && !defined(DPDK_SUPPORT)
int
__ice_write_sr_word(struct ice_hw *hw, u32 offset, const u16 *data);
int
__ice_write_sr_buf(struct ice_hw *hw, u32 offset, u16 words, const u16 *data);
#endif /* !LINUX_SUPPORT && !DPDK_SUPPORT */
#ifdef RDMA_SUPPORT
int
ice_cfg_vsi_rdma(struct ice_port_info *pi, u16 vsi_handle, u16 tc_bitmap,
		 u16 *max_rdmaqs);
int
ice_ena_vsi_rdma_qset(struct ice_port_info *pi, u16 vsi_handle, u8 tc,
		      u16 *rdma_qset, u16 num_qsets, u32 *qset_teid);
int
ice_dis_vsi_rdma_qset(struct ice_port_info *pi, u16 count, u32 *qset_teid,
		      u16 *q_id);
#endif /* RDMA_SUPPORT */
int
ice_dis_vsi_txq(struct ice_port_info *pi, u16 vsi_handle, u8 tc, u8 num_queues,
		u16 *q_handle, u16 *q_ids, u32 *q_teids,
		enum ice_disq_rst_src rst_src, u16 vmvf_num,
		struct ice_sq_cd *cd);
int
ice_cfg_vsi_lan(struct ice_port_info *pi, u16 vsi_handle, u16 tc_bitmap,
		u16 *max_lanqs);
int
ice_ena_vsi_txq(struct ice_port_info *pi, u16 vsi_handle, u8 tc, u16 q_handle,
		u8 num_qgrps, struct ice_aqc_add_tx_qgrp *buf, u16 buf_size,
		struct ice_sq_cd *cd);
int
ice_replay_pre_init(struct ice_hw *hw, struct ice_switch_info *sw);
#ifdef BMSM_MODE
int ice_replay_vsi(struct ice_hw *hw, u16 vsi_handle, u8 lport);
#else /* !BMSM_MODE */
int ice_replay_vsi(struct ice_hw *hw, u16 vsi_handle);
#endif /* BMSM_MODE */
void ice_replay_post(struct ice_hw *hw);
struct ice_q_ctx *
ice_get_lan_q_ctx(struct ice_hw *hw, u16 vsi_handle, u8 tc, u16 q_handle);
#ifndef NO_SBQ_SUPPORT
int ice_sbq_rw_reg_lp(struct ice_hw *hw, struct ice_sbq_msg_input *in,
		      u16 flag, bool lock);
void ice_sbq_lock(struct ice_hw *hw);
void ice_sbq_unlock(struct ice_hw *hw);
int ice_sbq_rw_reg(struct ice_hw *hw, struct ice_sbq_msg_input *in, u16 flag);
#endif /* !NO_SBQ_SUPPORT */
#if defined(CGU_SUPPORT) || defined(SYNCE_SUPPORT)
int
ice_aq_cfg_cgu_err(struct ice_hw *hw, bool ena_event_report, bool ena_err_report,
		   struct ice_sq_cd *cd);
#endif /* CGU_SUPPORT || SYNCE_SUPPORT */
#ifdef SYNCE_SUPPORT
int
ice_aq_get_cgu_abilities(struct ice_hw *hw,
			 struct ice_aqc_get_cgu_abilities *abilities);
int
ice_aq_set_input_pin_cfg(struct ice_hw *hw, u8 input_idx, u8 flags1, u8 flags2,
			 u32 freq, s32 phase_delay);
int
ice_aq_get_input_pin_cfg(struct ice_hw *hw,
			 struct ice_aqc_get_cgu_input_config *cfg,
			 u8 input_idx);
int
ice_aq_set_output_pin_cfg(struct ice_hw *hw, u8 output_idx, u8 flags,
			  u8 src_sel, u32 freq, s32 phase_delay);
int
ice_aq_get_output_pin_cfg(struct ice_hw *hw, u8 output_idx, u8 *flags,
			  u8 *src_sel, u32 *freq, u32 *src_freq);
int
ice_aq_get_cgu_dpll_status(struct ice_hw *hw, u8 dpll_num, u8 *ref_state,
			   u16 *dpll_state, s64 *phase_offset, u8 *eec_mode);
int
ice_aq_set_cgu_dpll_config(struct ice_hw *hw, u8 dpll_num, u8 ref_state,
			   u8 config, u8 eec_mode);
int
ice_aq_set_cgu_ref_prio(struct ice_hw *hw, u8 dpll_num, u8 ref_idx,
			u8 ref_priority);
int
ice_aq_get_cgu_ref_prio(struct ice_hw *hw, u8 dpll_num, u8 ref_idx,
			u8 *ref_prio);
int
ice_aq_get_cgu_info(struct ice_hw *hw, u32 *cgu_id, u32 *cgu_cfg_ver,
		    u32 *cgu_fw_ver);
int
ice_aq_read_cgu_reg(struct ice_hw *hw, u16 offset, u8 data_len, u8 *data);
int
ice_aq_write_cgu_reg(struct ice_hw *hw, u16 offset, u8 data_len, u8 *data);
int
ice_aq_set_phy_rec_clk_out(struct ice_hw *hw, u8 phy_output, bool enable,
			   u32 *freq);
int
ice_aq_get_phy_rec_clk_out(struct ice_hw *hw, u8 phy_output, u8 *port_num,
			   u8 *flags, u32 *freq);
#endif /* SYNCE_SUPPORT */
int
ice_aq_get_sensor_reading(struct ice_hw *hw, u8 sensor, u8 format,
			  struct ice_aqc_get_sensor_reading_resp *data,
			  struct ice_sq_cd *cd);
void
ice_stat_update40(struct ice_hw *hw, u32 reg, bool prev_stat_loaded,
		  u64 *prev_stat, u64 *cur_stat);
void
ice_stat_update32(struct ice_hw *hw, u32 reg, bool prev_stat_loaded,
		  u64 *prev_stat, u64 *cur_stat);
#ifndef LINUX_SUPPORT
void
ice_stat_update_repc(struct ice_hw *hw, u16 vsi_handle, bool prev_stat_loaded,
		     struct ice_eth_stats *cur_stats);
#endif /* !LINUX_SUPPORT */
#ifndef NO_RECOVERY_MODE_SUPPORT
enum ice_fw_modes ice_get_fw_mode(struct ice_hw *hw);
#ifndef LEGACY_PREBOOT_SUPPORT
void ice_print_rollback_msg(struct ice_hw *hw);
#endif /* !LEGACY_PREBOOT_SUPPORT */
#endif /* !NO_RECOVERY_MODE_SUPPORT */
#if !defined(NO_PTP_SUPPORT) || (!defined(NO_SBQ_SUPPORT) && defined(PF_DRIVER) && !defined(PREBOOT_SUPPORT))
bool ice_is_generic_mac(struct ice_hw *hw);
#endif /* !NO_PTP_SUPPORT || (!NO_SBQ_SUPPORT && PF_DRIVER && !PREBOOT_SUPPORT) */
#endif /* !ICE_TDD */
#if defined(E810C_SUPPORT) || defined(E810_XXV_SUPPORT)
bool ice_is_e810(struct ice_hw *hw);
bool ice_is_e810t(struct ice_hw *hw);
#endif /* E810C_SUPPORT || E810_XVV_SUPPORT */
#ifdef E830_SUPPORT
bool ice_is_e830(struct ice_hw *hw);
#endif /* E830_SUPPORT */
#ifdef E825C_SUPPORT
bool ice_is_e825c(struct ice_hw *hw);
#endif /* E825C_SUPPORT */
#if defined(E823_SUPPORT) || defined(E823C_SUPPORT)
bool ice_is_e823(struct ice_hw *hw);
#endif /* E823_SUPPORT || E823C_SUPPORT */
#if (!defined(LINUX_SUPPORT) && !defined(DPDK_SUPPORT)) || defined(QV_SUPPORT)
int
ice_aq_alternate_write(struct ice_hw *hw, u32 reg_addr0, u32 reg_val0,
		       u32 reg_addr1, u32 reg_val1);
int
ice_aq_alternate_read(struct ice_hw *hw, u32 reg_addr0, u32 *reg_val0,
		      u32 reg_addr1, u32 *reg_val1);
int
ice_aq_alternate_write_done(struct ice_hw *hw, u8 bios_mode,
			    bool *reset_needed);
int ice_aq_alternate_clear(struct ice_hw *hw);
#endif /* (!LINUX_SUPPORT && !DPDK_SUPPORT) || defined(QV_SUPPORT) */
int
ice_sched_query_elem(struct ice_hw *hw, u32 node_teid,
		     struct ice_aqc_txsched_elem_data *buf);
#if !defined(LINUX_SUPPORT) && !defined(DPDK_SUPPORT)
int
ice_get_cur_lldp_persist_status(struct ice_hw *hw, u32 *lldp_status);
int
ice_get_dflt_lldp_persist_status(struct ice_hw *hw, u32 *lldp_status);
#endif /* !LINUX_SUPPORT && !DPDK_SUPPORT */
#ifdef QV_SUPPORT
int
ice_aq_set_pf_context(struct ice_hw *hw, u8 pf_id, struct ice_sq_cd *cd);
#endif /* QV_SUPPORT */
#ifndef NO_PTP_SUPPORT
int
ice_aq_set_driver_param(struct ice_hw *hw, enum ice_aqc_driver_params idx,
			u32 value, struct ice_sq_cd *cd);
int
ice_aq_get_driver_param(struct ice_hw *hw, enum ice_aqc_driver_params idx,
			u32 *value, struct ice_sq_cd *cd);
#endif /* !NO_PTP_SUPPORT */
#if defined(E830_SUPPORT) && defined(QV_SUPPORT)
int
ice_aq_control_srev(struct ice_hw *hw, u8 cmd_flags, u8 *min_srev_location,
		    u32 *min_srev_value, u32 *curr_nvm_srev_value,
		    struct ice_sq_cd *cd);
int
ice_aq_program_fuse_bits(struct ice_hw *hw, u32 fuse_id,
			 bool eom, u8 *fuse_value, u16 fuse_data_len,
			 struct ice_sq_cd *cd);
int
ice_aq_read_fuse_bits(struct ice_hw *hw, u32 fuse_id,
		      bool *eom, bool *lvid, u16 *fuse_bit_width,
		      u8 *fuse_value, u16 fuse_data_len,
		      struct ice_sq_cd *cd);
#endif /* E830_SUPPORT && QV_SUPPORT */
int
ice_aq_set_gpio(struct ice_hw *hw, u16 gpio_ctrl_handle, u8 pin_idx, bool value,
		struct ice_sq_cd *cd);
int
ice_aq_get_gpio(struct ice_hw *hw, u16 gpio_ctrl_handle, u8 pin_idx,
		bool *value, struct ice_sq_cd *cd);
bool ice_is_100m_speed_supported(struct ice_hw *hw);
#if defined(PREBOOT_SUPPORT) || defined(WINDOWS_SUPPORT)
u16
ice_get_link_speed_based_on_phy_type(u64 phy_type_low, u64 phy_type_high);
#endif /* PREBOOT_SUPPORT || WINDOWS_SUPPORT */
#if !defined(LINUX_SUPPORT) && (!defined(DPDK_SUPPORT) || defined(AE_DRIVER))
int ice_get_netlist_ver_info(struct ice_hw *hw, struct ice_netlist_info *netlist);
#endif /* !LINUX_SUPPORT && (!DPDK_SUPPORT || AE_DRIVER) */
#if !defined(NO_DCB_SUPPORT) || defined(AE_DRIVER)
int
ice_aq_set_lldp_mib(struct ice_hw *hw, u8 mib_type, void *buf, u16 buf_size,
		    struct ice_sq_cd *cd);
#endif /* !NO_DCB_SUPPORT || AE_DRIVER */
#ifndef NO_DCB_SUPPORT
bool ice_fw_supports_lldp_fltr_ctrl(struct ice_hw *hw);
int
ice_lldp_fltr_add_remove(struct ice_hw *hw, u16 vsi_num, bool add);
int ice_lldp_execute_pending_mib(struct ice_hw *hw);
#endif /* !NO_DCB_SUPPORT */
int
ice_aq_read_i2c(struct ice_hw *hw, struct ice_aqc_link_topo_addr topo_addr,
		u16 bus_addr, __le16 addr, u8 params, u8 *data,
		struct ice_sq_cd *cd);
int
ice_aq_write_i2c(struct ice_hw *hw, struct ice_aqc_link_topo_addr topo_addr,
		 u16 bus_addr, __le16 addr, u8 params, const u8 *data,
		 struct ice_sq_cd *cd);
#ifdef HEALTH_STATUS_SUPPORT
int
ice_aq_set_health_status_config(struct ice_hw *hw, u8 event_source,
				struct ice_sq_cd *cd);
bool ice_is_fw_health_report_supported(struct ice_hw *hw);
int
ice_aq_get_supported_health_status_codes(struct ice_hw *hw,
					 u8 *status_codes_buf,
					 u16 buf_size,
					 u16 *status_code_count,
					 struct ice_sq_cd *cd);
int
ice_aq_get_health_status(struct ice_hw *hw, u8 *health_status_buf,
			 u16 buf_size, u16 *health_status_count,
			 struct ice_sq_cd *cd);
int
ice_aq_clear_health_status(struct ice_hw *hw, struct ice_sq_cd *cd);
#endif /* HEALTH_STATUS_SUPPORT */
bool ice_fw_supports_report_dflt_cfg(struct ice_hw *hw);
/* AQ API version for FW auto drop reports */
bool ice_is_fw_auto_drop_supported(struct ice_hw *hw);
#if defined(E830_SUPPORT) &&  defined(QV_SUPPORT) || defined(PLATF_SUPPORT)
enum ice_platform_type ice_get_platf_type(struct ice_hw *hw);
#endif /* E830_SUPPORT && (INTERNAL_ONLY || QV_SUPPORT) || PLATF_SUPPORT */
#if defined(FW_SUPPORT) || defined(QV_SUPPORT)
int
ice_aq_get_mgblty_error_code_resp(struct ice_hw *hw, bool clean_err,
				  struct ice_aqc_mng_get_errors_buffer *first_error_code,
				  struct ice_aqc_mng_get_errors_buffer *last_error_code);
#endif /* FW_SUPPORT || QV_SUPPORT */
enum ice_cgd_per_port ice_get_port_max_cgd(struct ice_hw *hw);
#ifndef LEGACY_PREBOOT_SUPPORT
u8 ice_get_tc_by_priority(struct ice_hw *hw, u8 prio);
#endif /* !LEGACY_PREBOOT_SUPPORT */
int ice_get_cgd_idx(struct ice_hw *hw, u8 tc);
#endif /* _ICE_COMMON_H_ */
