/*
    mrd.h

    define all header and structures for the miniramdump.

    mini ramdump is a very small memory collect that define by app core driver,
modem, rpm or other process, such as the dmesg log,android logcat log,f3 trace
buffer,ocimem, rpm msg ram, etc. the ramdump info will be saved in a log file.
    one system error reboot will collect once miniramdump info, and the cycle is
called one transaction. In each transaction, there are lots of subinfo item,
each item is called one commit.

    History:
        Jun 19,2013: KerryXi, init this file

    Copyright (c) 2013, Lenovo
    All rights reserved.
*/

#ifndef _MRD_H
#define _MRD_H

#define MRD_SMEM_TRIGGER_MAGIC  0x2144524d    // "MRD!"
#define MRD_FORCE_TRIGGER_MAGIC 0x2144524e    // "NRD!"
#define MRD_SMEM_DONE_MAGIC_MASK 0xFFFFFF00
#define MRD_SMEM_DONE_MAGIC_FLAG 0x21445200
#define MRD_SMEM_DONE_MAGIC_OK             0x2144524f    // "ORD!"
#define MRD_SMEM_DONE_MAGIC_FAIL_CHECK     0x21445250    // "PRD!"
#define MRD_SMEM_DONE_MAGIC_FAIL_TRANIF    0x21445251    // "QRD!"
#define MRD_SMEM_DONE_MAGIC_IGNORE         0x21445252    // "RRD!"

#define MRD_CRASH_MAGIC  0xBE    //mrd crash flag

#define FIX_IMEM_MRD_DUMP_SIGNATURE_OFFSET 0xC
#define FIX_IMEM_MRD_DUMP_TABLE_ADDR 0x34
#define FIX_IMEM_MRD_DUMP_DONE_FLAG_ADDR 0x30

#define MAX_MRD_SHAREINDEX_IMEM_PA_ADDRES 0xFE900000
#define MAX_MRD_SHAREINDEX_PA 0xF8000000
#define MIN_MRD_SHAREINDEX_PA 0x200000

#define MRD_MAJOR_VERSION(ver) ( ver >> 8)
#define MRD_MINOR_VERSION(ver) ( ver & 0xFF)
#define MRD_MAKE_VERSION(major, minor) (major << 8 | minor)

/*
   mrd shareindex type
*/
#define MRD_SHAREINDEX_SIZE 4096
#define MRD_SHAREINDEX_HEAD_SIZE 64

#define MRD_SHAREINDEX_HEAD_MAGIC 0x2444524d // "MRD$"
#define MRD_SHAREINDEX_VERSION MRD_MAKE_VERSION(0,1)

#define MRD_MODE_IGNORE    0
#define MRD_MODE_DLOADMODE 1
#define MRD_MODE_REBOOTMODE 2
#define MRD_MODE_MODE_MAX MRD_MODE_REBOOTMODE

typedef struct {
	unsigned int signature;         //  "MRD$"
	unsigned int version;
	unsigned int total_size;        //whole journal space size , should be equal the partition size
	unsigned int head_size;         //this header size, index item follow this head
	unsigned int total_count;       //
	unsigned int index_count;       //current index item count
	unsigned int lock;              //lock flag for other cpu core
	unsigned int mrdmode;           // 0: dloadmode    1: mrd mode, after mrd collection, reboot

	unsigned int boot_rtc;          //subsystem: krait, modem, rpm or else
	unsigned int reserve1;
	unsigned int reserve2;
	unsigned int reserve3;
} mrd_shareindex_header_t;

#define MRD_SHAREINDEX_ITEM_NAME_MAX_LENGTH (16)
#define MRD_SHAREINDEX_SPECIAL_TAG (0xFFFFFFFF)     //this is used for special usage
#define MRD_SHAREINDEX_VA_TAG (0xFFFFFFFE)          //this is used for va address

#define MRD_SUBSYS_KRAIT 1
#define MRD_SUBSYS_MODEM 2
#define MRD_SUBSYS_RPM   3
#define MRD_SUBSYS_ADSP  4
#define MRD_SUBSYS_WCNSS 5
#define MRD_SUBSYS_VENUS 6
#define MRD_SUBSYS_SBL	 7

typedef struct {
	char name[MRD_SHAREINDEX_ITEM_NAME_MAX_LENGTH];    // the name can identify each commit content
	unsigned int subsys;              //subsystem: krait, modem, rpm or else
	unsigned int address;             //the physical memory address

	unsigned int size;                //the physical memory length
	unsigned int param1;              //the commit tag parameter, unkown use  0xFFFFFFFF are stands for special fields.
	unsigned int param2;              //the commit tag parameter, unkown use
} mrd_shareindex_item_t;    //size should be 32


/*
      mrd journal format type
*/
#define MRD_JOURNAL_HEAD_MAGIC 0x4044524d    // "MRD@"
#define MRD_JOURNAL_VERSION  MRD_MAKE_VERSION(0, 1)

//each mrd blocktype
#define MRD_JOURNAL_SUPERBLOCK_DESCRIPTOR   1
#define MRD_JOURNAL_TRANSACTION_DESCRIPTOR  2
#define MRD_JOURNAL_COMMIT_DESCRIPTOR       3

typedef struct {
	unsigned int magic;          //all descriptor are MRD_JOURNAL_HEAD_MAGIC magic nunmber
	unsigned int block_type;     //mrd JOURNAL sb , transaction or commit block type
	unsigned int sequence;       //mrd current transaction, should cycle 1~MRD_JOURNAL_MAX_TRANSACTION_SEQUENCE
} mrd_journal_header_t;

/*
   the mini ramdump log superblock. all fields are in little-endian byte order.
*/
#define MRD_JOURNAL_BLOCK_SIZE                512
#define MRD_JOURNAL_SB_BLOCKS                 1
#define MRD_JOURNAL_TRANSACTION_BLOCKS        (16*2048)  //16MB
#define MRD_JOURNAL_MIN_TRANSACTION_COUNT     2          //min tranactions should be filled
#define MRD_JOURNAL_MAX_TRANSACTION_COUNT     8          //the max transactions can be filled
#define MRD_JOURNAL_MAX_TRANSACTION_SEQUENCE  8192       //the max transactions sequence
#define MRD_JOURNAL_TIME_LENGTH               28
typedef struct {
	mrd_journal_header_t header;

	// the following fields should be constructed when the sb first time formated
	unsigned int version;
	unsigned int block_size;
	unsigned int block_count;                 //whole mrd blocks count include sb
	/*the mrd log file can contain transaction's count, calculated by actual partition size,
	  should be smaller than MRD_JOURNAL_MAX_TRANSACTION_COUNT
	*/
	unsigned int trans_count;
	unsigned int blocks_per_trans;            //each transaction blocks

	/*    the first block for each new transaction, used transaction should be 1~block_count,
	      0 stands for unused. the first transaction block address is 1
	*/
	unsigned int trans_block_map[MRD_JOURNAL_MAX_TRANSACTION_COUNT];
	unsigned int create_time;                 //create the mrd journal sb time stamp

	// the following fields should be update every transaction written
	unsigned int write_time;                  //last write transaction time stamp
	char   write_time_string[MRD_JOURNAL_TIME_LENGTH];	//time string: 2013-09-18 21:33:39
} mrd_journal_superblock_descriptor;

/*
    the mini ramdump transaction structure.
*/
#define MRD_TRANSACTION_DESCRIPTOR_BLOCKS           2
#define MRD_TRANSACTION_SW_VERION_MAX_LENGTH        128
#define MRD_TRANSACTION_DIGEST_MAX_LENGTH           128
#define MRD_TRANSACTION_LINUXBANNER_MAX_LENGTH      192
#define MRD_TRANSACTION_LINUXCOMMANDLINE_MAX_LENGTH 384

#define MRD_TRANSACTION_ERROR_NOERR                 0
#define MRD_TRANSACTION_ERROR_COMMIT_TOO_LONG       1

typedef struct {
	mrd_journal_header_t header;

	unsigned int commit_count;                         //the commit count in current transaction
	unsigned int commit_total_blocks;                  //total commit block count include transaction descriptor size
	unsigned int commit_first_block;                   //the first block of commit

	//first avail block nr in current transaction, every commit finish, will update this area
	//this is temp variable, it starts with commit_first_block and ends with commit_count
	unsigned int commit_first_avail_blocks;
	unsigned int error_status;                                   //0: there is no error    other value: error

	unsigned int write_time;                                     //last write transaction time stamp
	char   write_time_string[MRD_JOURNAL_TIME_LENGTH];           //time string: 2013-09-18 21:33:39
	char sw_ver[MRD_TRANSACTION_SW_VERION_MAX_LENGTH];           //such as S696_1_S_3_1018_0209_121105
	char l_banner[MRD_TRANSACTION_LINUXBANNER_MAX_LENGTH];       //linux_banner
	char cmd_line[MRD_TRANSACTION_LINUXCOMMANDLINE_MAX_LENGTH];  //such as app panic, modem dump
	char digest[MRD_TRANSACTION_DIGEST_MAX_LENGTH];              //such as app panic, modem dump
} mrd_journal_transaction_descriptor;

/*
    the mini ramdump journal commit structure.
    every commit size will align with sector
*/
typedef struct {
	mrd_journal_header_t header;
	mrd_shareindex_item_t shareindex;
	unsigned int  blocks;           //the occupy blocks from the descriptor to the buf data end
	/*char buf[0];*/                 //the data of the commit
} mrd_journal_commit_descriptor;

#define MRD_BLOCK_ALIGNUP(p)	((((unsigned long)(p) + 511) & ~(512 - 1)) >> 9)

#endif //_MRD_H
