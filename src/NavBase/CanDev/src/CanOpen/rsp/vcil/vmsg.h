#ifndef _CFG_H
#define _CFG_H

#include "common.h"

int vmsg_baudrate_j1708 ( unsigned char baudrate_flag, unsigned char format );
//int vmsg_firmware_version ( unsigned char *ver, unsigned *ver_len );
int vmsg_firmware_version ( );
int vmsg_baudrate_module ( unsigned char baudrate );
int vmsg_mode_receive ( unsigned char mode );
int vmsg_module_control ( unsigned char can_channel1_protocol_select, unsigned char can_channel2_protocol_select, unsigned char j1708_channel_protocol_select );
int vmsg_reset_module ( void );
int vmsg_enter_bootloader ( void );
int vmsg_platform_name ( unsigned char *platform_name );
int vmsg_get_bootloader_checksum_method ( unsigned char *checksum );

int vmsg_tx_can ( unsigned char can_port, unsigned long can_id, unsigned char data_length, unsigned char *data );
int vmsg_tx_can_old ( unsigned char can_port, unsigned char rtr, unsigned long can_id, unsigned char data_length, unsigned char *data );
int vmsg_tx_can_ex ( unsigned char can_port, unsigned char rtr, unsigned long can_id, unsigned char data_length, unsigned char *data );
int vmsg_rx_can ( RPACKET *can_packet );
int vmsg_poll_can ( RPACKET *can_packet );
int vmsg_bitrate_can ( unsigned char can_port, unsigned char bitrate_flag, int mode );
int vmsg_bitrate_can_btr ( unsigned char can_port, unsigned char sjw, unsigned char bs1, unsigned char bs2, unsigned short prescaler, int mode );
int vmsg_read_can_esr ( unsigned char can_port );
int vmsg_read_can_bitrate ( unsigned char can_port );
int vmsg_clear_filter_list_can ( void );
int vmsg_add_filter_list_can ( unsigned char can_port, unsigned long can_id );
int vmsg_remove_filter_list_can ( unsigned char can_port, unsigned long can_id );
int vmsg_clear_filter_mask_can ( unsigned char can_port );
int vmsg_add_filter_mask_can ( unsigned char can_port, unsigned char bank, unsigned char RTR, unsigned char mode, unsigned long filter_id, unsigned long mask );
int vmsg_remove_filter_mask_can ( unsigned char can_port, unsigned char bank );
int vmsg_read_filter_mask_can ( unsigned char can_port, unsigned char bank );

int vmsg_tx_j1708 ( unsigned char mid, unsigned short pid, unsigned char priority, unsigned char data_length, unsigned char *data );
int vmsg_add_filter_list_j1708 ( unsigned char mid );
int vmsg_remove_filter_list_j1708 ( unsigned char mid );
int vmsg_remove_all_filter_j1708 ( void );
int vmsg_read_filter_list_j1708 ( void );

int vmsg_tx_j1939 ( unsigned char channel_number, unsigned int pgn, unsigned char dst, unsigned char src, unsigned pri, unsigned int data_length, unsigned char *data );
int vmsg_add_filter_list_j1939 ( unsigned char channel_number, unsigned int pgn );
int vmsg_remove_filter_list_j1939 ( unsigned char channel_number, unsigned int pgn );
int vmsg_read_filter_list_j1939 ( unsigned char channel );
int vmsg_remove_all_filter_j1939( unsigned char channel );
int vmsg_set_address_and_name_j1939( unsigned char channel , unsigned char address_enabled, unsigned char source_address, unsigned char name_enalbed, unsigned char* source_name);
int vmsg_get_address_and_name_j1939( unsigned char channel);

int vmsg_tx_j1587 ( unsigned char mid, unsigned short pid, unsigned char priority, unsigned char data_length, unsigned char *data );
int vmsg_add_filter_list_j1587 ( unsigned int pid );
int vmsg_remove_filter_list_j1587 ( unsigned int pid );
int vmsg_remove_all_filter_j1587 ( void );
int vmsg_read_filter_list_j1587 ( void );

int vmsg_tx_obd2 ( unsigned char channel_number, unsigned char dst, unsigned char src, unsigned char pri, unsigned char tat, unsigned char data_length, unsigned char *data );
int vmsg_add_filter_list_obd2 ( unsigned char channel_number, unsigned int pid );
int vmsg_remove_filter_list_obd2 ( unsigned char channel_number, unsigned char pid );
int vmsg_read_filter_list_obd2 ( unsigned char channel );
int vmsg_remove_all_filter_obd2( unsigned char channel );
#endif
