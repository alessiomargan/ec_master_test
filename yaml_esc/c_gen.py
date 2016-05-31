#! /usr/bin/env python

import os
import time
import pprint
import StringIO
import struct
import yaml
import string

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

def get_c_field(key,var):
    try :
        return struct2c_map[key.lower()] if key.islower() else 'unsigned '+ struct2c_map[key.lower()],var
    except KeyError :
        if key.startswith('s') :
            return 'char','%s[%s]' % (var,key[1:])

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
            return int(key[1:])*8

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


    def gen_cpp(self):

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

        out.write("#include <%s.h>\n\n" % self.name)

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

    def gen_struct(self):
        
        out = StringIO.StringIO()
        
        out.write('struct ${cls}PdoTypes {\n')
        out.write('\t// TX  slave_input -- master output\n''')
        out.write('\tstruct pdo_tx {\n')
        for sdo in self.sdos[0x6000] :
            out.write('\t\t\t%s %s;\n' % get_c_field(sdo[1],sdo[0]))
        out.write('\t} __attribute__ ( ( __packed__ ) );\n\n')
        out.write('\t// RX  slave_output -- master input\n')
        out.write('\tstruct pdo_rx {\n')
        for sdo in self.sdos[0x7000] :
            out.write('\t\t\t%s %s;\n' % get_c_field(sdo[1],sdo[0]))
        out.write('\t} __attribute__ ( ( __packed__ ) );\n')
        out.write('};\n\n')
                
        out.write('struct ${cls}SdoTypes {\n')
        for sdo in self.sdos[0x8000]+self.sdos[0x8001] :
            out.write('\t%s %s;\n' % get_c_field(sdo[1],sdo[0]))
        out.write('};\n')
                    
        return string.Template(out.getvalue()).substitute(cls=self.name)
        
    def gen_class(self):
        
        class_template =  '''
class ${cls} :
    public BasicEscWrapper<${cls}PdoTypes,${cls}SdoTypes>
{
public:
    typedef BasicEscWrapper<${cls}PdoTypes,${cls}SdoTypes> Base;

    ${cls} ( const ec_slavet& slave_descriptor ) :
        Base ( slave_descriptor ) {
    }

    virtual ~${cls} ( void ) {
        delete [] SDOs;
        DPRINTF ( "~%s %d\\n", typeid ( this ).name(), position );
    }

    virtual const objd_t * get_SDOs() {
        return SDOs;
    }

    virtual void init_SDOs ( void );

    //virtual void on_readPDO ( void ) { }
    //virtual void on_writePDO ( void ) { }
    
    //virtual void on_readSDO ( void ) { }
    //virtual void on_writeSDO ( void ) { }
    
private:

    objd_t * SDOs;

};
    
'''
        return string.Template(class_template).substitute(cls=self.name)
                        
    def gen_h(self):

        out = StringIO.StringIO()
        out.write("""/*

        Copyright (C) 2015 Italian Institute of Technology

        Developer:
        Alessio Margan (%s , alessio.margan@iit.it)

        Generated with %s :
        %s
*/

#ifndef __IIT_ECAT_ADVR_%s_H__
#define __IIT_ECAT_ADVR_%s_H__

#include <iit/ecat/slave_wrapper.h>

namespace iit {
namespace ecat {
namespace advr {

%s

%s

}
}
}

#endif
""" % (time.strftime("%Y", time.gmtime()),
       __file__,
       time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()),
       self.name.upper(),
       self.name.upper(),
       self.gen_struct(),
       self.gen_class()))

        return out.getvalue()


if __name__ == '__main__' :

    yaml_file = u'motor_xt.yaml'

    esc_gen = yaml.load(file(yaml_file,'r'))
    
    out_file = u'%s.cpp' % esc_gen.name
    with file(out_file,'w') as out :
        out.write(str(esc_gen.gen_cpp()))

    out_file = u'%s.h' % esc_gen.name
    with file(out_file,'w') as out :
        out.write(str(esc_gen.gen_h()))