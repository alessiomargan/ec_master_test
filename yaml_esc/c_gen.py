#! /usr/bin/env python

import os
import time
import pprint
import StringIO
import struct
import yaml


struct2c_map = {'b': 'char',
                'h': 'short',
                'i': 'int',
                'l': 'long',
                'q': 'long long',
                'f': 'float',
                'd': 'double',
                }

struct2ec_map = {'b': 'DTYPE_INTEGER8',
                 'h': 'DTYPE_INTEGER16',
                 'i': 'DTYPE_INTEGER32',
                 'l': 'DTYPE_INTEGER32',
                 'q': 'DTYPE_INTEGER64',
                 'f': 'DTYPE_REAL32',
                 'd': 'DTYPE_REAL64',
                 'B': 'DTYPE_UNSIGNED8',
                 'H': 'DTYPE_UNSIGNED16',
                 'I': 'DTYPE_UNSIGNED32',
                 'L': 'DTYPE_UNSIGNED32',
                 'Q': 'DTYPE_UNSIGNED64',
                 's': 'DTYPE_VISIBLE_STRING',
                 }

rwo_map = { 'RO' : 'ATYPE_RO', 'RW' : 'ATYPE_RW' }

def get_ecat_type(key) :
    
    try: return struct2ec_map[key]
    except KeyError :
        if key.startswith('s') :
            return struct2ec_map['s']
         
def get_ecat_bits_size(key) :
    
    try: 
        struct2ec_map[key]        
        return struct.calcsize(key)*8
    except KeyError :
        if key.startswith('s') :
            return key[1:]

def gen_header(out) :

    out.write('''/*

	Copyright (C) 2015 Italian Institute of Technology

	Developer:
        Alessio Margan (%s , alessio.margan@iit.it)

        Generated with %s :
        %s
*/

#include <string.h>

''' % (time.strftime("%Y", time.gmtime()),
       __file__,
       time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()))
    )


class Esc_c_gen :

    def __init__(self, name, sdos):

        self.name = name
        self.sdos = sdos


    def __repr__(self):

        out = StringIO.StringIO()
        out.write("""/*
        
        Copyright (C) 2015 Italian Institute of Technology
        
        Developer:
        Alessio Margan (%s , alessio.margan@iit.it)
        
        Generated with %s :
        %s
*/
        
#include <string.h>
        
""" % (time.strftime("%Y", time.gmtime()),
       __file__,
       time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())))
        
        out.write("static const iit::ecat::objd_t source_SDOs[] = {\n\n")
        for index,items in self.sdos.iteritems() :
            for subindex,v in zip(range(1,len(items)+1),items) :
                out.write('\t{ %s, %s, %s,\t\t%s, %s, "%s",\t\t%s},\n' %
                               (hex(index),subindex,
                                get_ecat_type(v[1]),
                                get_ecat_bits_size(v[1]),
                                rwo_map[v[2]],v[0],0))
            out.write("\n")
        out.write("\t{0, 0, 0, 0, 0, 0, 0 }\n};\n")
        
        out.write("""
void %s::init_SDOs(void) {                                         
                                                                      
    int objd_num, i = 0;                                              
                                                                      
    objd_num = sizeof(source_SDOs)/sizeof(objd_t);                    
    SDOs = new objd_t [objd_num];                                     
                                                                      
    memcpy((void*)SDOs, source_SDOs, sizeof(source_SDOs));\n\n""" % self.name) 
        
        for index,items in self.sdos.iteritems() :
            out.write("    //%s\n" % hex(index))
            for subindex,v in zip(range(1,len(items)+1),items) :
                out.write('    SDOs[i++].data = (void*)&%s::rx_pdo.%s;\n' % 
                          (self.name, v[0]))        
            out.write("\n")
        
        out.write("""
    // end marker
    SDOs[i++].data = 0;

    assert ( objd_num == i );
}\n""")
        
        return out.getvalue()


if __name__ == '__main__' :

    yaml_file = u'test.yaml'
    out_file = u'sti_Caz.cpp'

    with file(out_file,'w') as out :
        esc_gen = yaml.load(file(yaml_file,'r'))
        out.write(str(esc_gen))

