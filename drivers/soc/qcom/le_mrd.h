/*
This is lenovo MRD (mini ramdump) header file.

Author:Kerry Xi
Date: Sep, 2013
Copy Right: Lenovo 2013
*/

#ifndef _LENOVO_MRD_H
#define _LENOVO_MRD_H

#include <linux/types.h>
#include "mrd.h"

typedef struct {
	unsigned int shareindex_table_phys;
	mrd_shareindex_header_t *shareindex_table_ptr;
} le_mrd_shareindex_pair ;

//register one shareindex
//return value: 0: success,  -1: fail
int mrd_register_shareindex(mrd_shareindex_item_t* index);

#endif  //_LENOVO_MRD_H
