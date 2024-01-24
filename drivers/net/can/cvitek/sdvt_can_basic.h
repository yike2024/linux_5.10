#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include "cvitek_can.h"

#ifndef SDVT_CAN_BASIC_H
#define SDVT_CAN_BASIC_H

//                           SmartDV Technologies Proprietary 
//            Copyright 2007-2023 SmartDV Technologies India Private Limited
// 
//                               CONFIDENTIAL INFORMATION
// 
//                                  All rights reserved
// 
//             The use, modification, or duplication of this product is protected 
//             according to SmartDV Technologies's licensing agreement.
// 
//             This Intellectual property contains confidential and proprietary 
//             information which are the properties of SmartDV Technologies. 
// 
//             Unauthorized use, disclosure, duplication, or reproduction are prohibited. 

// Date Created   : 02-06-2023
// Description    : This is C header file for sdvt_can IP core 
// Generator      : IIP Compiler Version 1.1

struct sdvt_can_command {
	u8 command_8b          ; // Command type
	uint32_t ident_32b     ; // Identifier
	u8 dlc_4b              ; // DLC field value
	u8 data_2d_8b [64]     ; // Data bytes
	u8 tb_tx_2d_8b [64]    ; // Register model
	u8 rx_data_2d_8b [64]  ; // Received data
	u8 rx_len_2d_8b [64]   ; // Received length
	u8 dut_rsp_2d_8b [64]  ; // Response Data bytes
	u8 irq_status0_8b      ; // IRQ status 0
	u8 irq_status1_8b      ; // IRQ status 1
	u8 irq_status2_8b      ; // IRQ status 2
	u8 irq_status3_8b      ; // IRQ status 3
	u8 data_len_code_4b    ; // data length
	u8 remote_resp_en_b    ; // Remote enable mode
};

struct sdvt_can_config {
	u8 cfg_can_mode_2b              ; // Can mode
	u8 cfg_ext_frame_mode_b         ; // extended_mode variable
	u8 cfg_remote_resp_en_b         ; // Remove enable mode
	u8 cfg_ext_ctrl_listen_b        ; // DUT Extended Control - Listen only mode
	u8 cfg_ext_ctrl_self_test_b     ; // DUT Extended Control - Self test mode
	u8 cfg_ext_ctrl_acc_filter_b    ; // DUT Extended Control - Acceptance filter mode - if '0' means dual filter and '1' single filter
	u8 cfg_ext_ctrl_rec_incr_b      ; // DUT Extended Control - Increment receiver error counter
	u8 cfg_ext_ctrl_remote_en_b     ; // DUT Extended Control - Enable receiving and processing remote frames
	u8 cfg_ext_ctrl_id_rx_b         ; // Extended Identifier  - Receiver side
	u8 cfg_ext_mode_en_b            ; // DUT Extended Control - Enable receiving and processing extebded mode
	u8 cfg_acceptance_code0_8b      ; // Acceptance code0 
	u8 cfg_acceptance_code1_8b      ; // Acceptance code1 
	u8 cfg_acceptance_code2_8b      ; // Acceptance code2 
	u8 cfg_acceptance_code3_8b      ; // Acceptance code3 
	u8 cfg_acceptance_mask0_8b      ; // Acceptance mask0 
	u8 cfg_acceptance_mask1_8b      ; // Acceptance mask1 
	u8 cfg_acceptance_mask2_8b      ; // Acceptance mask2 
	u8 cfg_acceptance_mask3_8b      ; // Acceptance mask3
	u8 cfg_nor_clk_divider_8b       ; // The controls the normal mode clock divider
	u8 cfg_nor_baud_prescaler_6b    ; // The controls the normal mode baud prescaler
	u8 cfg_nor_sync_jump_width_2b   ; // The controls the normal mode Sync jump width
	u8 cfg_nor_time_segment1_3b     ; // The controls the normal mode timing segement 1
	u8 cfg_nor_time_segment2_4b     ; // The controls the normal mode timing segement 2
	u8 cfg_nor_time_triple_sample_b ; // The controls the normal mode timing triple sample
	u8 cfg_fd_clk_divider_8b        ; // The controls the fd mode clock divider
	u8 cfg_fd_baud_prescaler_6b     ; // The controls the fd mode baud prescaler
	u8 cfg_fd_sync_jump_width_2b    ; // The controls the fd mode Sync jump width
	u8 cfg_fd_time_segment1_4b      ; // The controls the fd mode timing segement 1
	u8 cfg_fd_time_segment2_3b      ; // The controls the fd mode timing segement 2
	u8 cfg_fd_time_triple_sample_b  ; // The controls the fd mode timing triple sample
	u8 irq_enable0_8b               ; // IRQ enable 0
	u8 irq_enable1_8b               ; // IRQ enable 1
	u8 irq_enable2_8b               ; // IRQ enable 2
	u8 irq_enable3_8b               ; // IRQ enable 3
	u8 tx_b                         ; // Tx operation 
	u8 cfg_fd_brs_b                 ; // CAN FD - Enable BRS
	u8 mode_reset_8b                ; // mode reset
	u8 irq_enable_8b                ; // Irq enable
	u8 can_mode_8b                  ; // can mode register
	u8 std_arb_id0_8b               ; // 0]
	u8 std_arb_id1_8b               ; // 3] and normal frame and 28:21 for extended frame
	u8 std_arb_id2_8b               ; // Arbitration identifier bits 20:13 for extended frame
	u8 std_arb_id3_8b               ; // Arbitration identifier bits 12:5 for extended frame
	u8 std_arb_id4_8b               ; // Arbitration identifier bits 12:5 for extended frame
	u8 rsp_arb_id0_8b               ; // This register holds remote frame response Identifier bits 10:3 for normal frame and 28:21 for extended frame
	u8 rsp_arb_id1_8b               ; // This register holds remote frame response Identifier bits 2:0 for normal frame and 20:13 for extended frame
	u8 rsp_arb_id2_8b               ; // This register holds remote frame response Identifier bits 12:5 for extended frame
	u8 rsp_arb_id3_8b               ; // This register holds remote frame response Identifier bits 4:0 for extended frame
	u8 acceptance_code_8b           ; // Acceptance code
	u8 acceptance_mask_8b           ; // Acceptance mask 
	u8 bus_timing0_8b               ; // Bus timing register 0
	u8 bus_timing1_8b               ; // Bus timing register 1
};

#define STANDARD_DATA_FRAME                                        0         // Standard data frame
#define EXTENDED_DATA_FRAME                                        1         // Extended data frame
#define STANDARD_REMOTE_FRAME                                      2         // Standard remote frame
#define EXTENDED_REMOTE_FRAME                                      3         // Extended remote frame

// write_byte_reg :#This is for register byte write 
//            p_addr_32b :#Register address to write 
//             p_data_8b :#Register data to write 
extern void write_byte_reg (uint32_t p_addr_32b,u8 p_data_8b);

// write_reg :#This is for register write 
//            p_addr_32b :#Register address to write 
//             p_data_8b :#Register data to write 
extern void write_reg (uint32_t p_addr_32b,u8 p_data_8b);



// read_reg :#This is for register read 
//               returns :#Return the read data 
//            p_addr_32b :#Register address to read 

extern u8  read_reg (uint32_t p_addr_32b);

// sdvt_init_chip :#This method is used to initilize the chip 
//           p_config_st :#Pointer to config object 
void sdvt_init_chip (struct sdvt_can_classdev *cdev, struct sdvt_can_config* p_config_st);

// send_command :#This method is used for sending command 
//              p_cmd_st :# Pointer to command object 
//           p_config_st :# Pointer to config object 
void send_command ( struct sdvt_can_classdev *cdev,struct sdvt_can_command* p_cmd_st,struct sdvt_can_config* p_config_st);

// receive_frame :#This method is used for receving frame, this method is called by IRQ handler 
//                when frame is received 
//              p_cmd_st :# Pointer to command object 

extern void receive_frame (struct sdvt_can_classdev *cdev,struct sdvt_can_command *p_cmd_st,struct sdvt_can_config *p_config_st);

extern void receive_remote_frame (struct sdvt_can_classdev *cdev,struct sdvt_can_command *p_cmd_st,struct sdvt_can_config *p_config_st); 

// detect_irq_status :# This method is used to handle the interrupts 
//              p_cmd_st :# Pointer to command object 

extern void detect_irq_status (struct sdvt_can_classdev *cdev ,struct sdvt_can_command *p_cmd_st);



// mask_irq :# This method is used to mask the IRQ 
//           p_config_st :# Pointer to config object 
//             p_mask_8b :# Interrupts to mask 

void sdvt_mask_irq (struct sdvt_can_classdev *cdev,struct sdvt_can_config *p_config_st,uint32_t p_addr,u8 p_mask_8b);



// unmask_irq :# This method is used to unmask the IRQ 
//           p_config_st :# Pointer to config object 
//             p_mask_8b :# Interrupts to mask 
void sdvt_unmask_irq (struct sdvt_can_classdev *cdev,struct sdvt_can_config *p_config_st,uint32_t p_addr,u8 p_mask_8b);



// wait_tx_done :#This method is used for waiting for completion of request 
//             parameter :No parameter 

extern int32_t wait_tx_done (struct sdvt_can_classdev *cdev);


// wait_rx_valid :#This method is used for waiting for completion of request 
//             parameter :No parameter 

extern int32_t wait_rx_valid (struct sdvt_can_classdev *cdev);


// print_config :# This method is used to print all registers 
//           parameter :No parameter 

extern void print_register (struct sdvt_can_classdev *cdev);



// print_config :# This method is used to print all config registers 
//           p_config_st :# Pointer to config object 

extern void print_config (struct sdvt_can_classdev *cdev,struct sdvt_can_config *p_config_st);



// print_command :# This method is used to print command being sent 

//              p_cmd_st :#Pointer to command object 

extern void print_command (struct sdvt_can_classdev *cdev,struct sdvt_can_command *p_cmd_st);

#define CAN_TXRX_TIMEOUT 5000
//                           SmartDV Technologies Proprietary 
//            Copyright 2007-2023 SmartDV Technologies India Private Limited
// 
//                               CONFIDENTIAL INFORMATION
// 
//                                  All rights reserved
// 
//             The use, modification, or duplication of this product is protected 
//             according to SmartDV Technologies's licensing agreement.
// 
//             This Intellectual property contains confidential and proprietary 
//             information which are the properties of SmartDV Technologies. 
// 
//             Unauthorized use, disclosure, duplication, or reproduction are prohibited. 

#endif
