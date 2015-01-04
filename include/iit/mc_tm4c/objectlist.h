#ifndef __objectlist__
#define __objectlist__

#define DTYPE_UNSIGNED8 0
#define DTYPE_UNSIGNED16 1
#define DTYPE_INTEGER16 2
#define DTYPE_UNSIGNED64 3
#define DTYPE_REAL32 4
#define DTYPE_VISIBLE_STRING 5

#define ATYPE_RO 17
#define ATYPE_RW 18
typedef struct
{
   int subindex;
   int datatype;
   int bitlength;
   int access;
   int value;
   const char *data;
} objd;


extern objd SDO6000[];
extern objd SDO7000[];
extern objd SDO8000[];
extern objd SDO8001[];

#endif
