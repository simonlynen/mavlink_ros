#!/usr/bin/env python
'''
parse a MAVLink protocol XML file and generate a C implementation

Copyright Simon Lynen 2012 / adapted from Andrew Tridgell 2011
Released under GNU GPL version 3 or later
'''

import sys, textwrap, os, time
import mavparse, mavtemplate

t = mavtemplate.MAVTemplate()

def generate_version_h(directory, xml):
    '''generate version.h'''
    f = open(os.path.join(directory, "version.h"), mode='w')
    t.write(f,'''
/** @file
 *	@brief MAVLink comm protocol built from ${basename}.xml
 *	@see http://pixhawk.ethz.ch/software/mavlink
 */
#ifndef MAVLINK_VERSION_H
#define MAVLINK_VERSION_H

#define MAVLINK_BUILD_DATE "${parse_time}"
#define MAVLINK_WIRE_PROTOCOL_VERSION "${wire_protocol_version}"
#define MAVLINK_MAX_DIALECT_PAYLOAD_SIZE ${largest_payload}
 
#endif // MAVLINK_VERSION_H
''', xml)
    f.close()


def generate_mavlink_ros_bridge_h(directory, xml):
    '''generate main header per XML file'''
    f = open(os.path.join(directory, "mavlink_ros_bridge.h"), mode='w')
    t.write(f, '''
/** @file
 *	@brief MAVLink ROS bridge
 *	@see http://qgroundcontrol.org/mavlink/
 */
#include "version.h"
#include "ros/ros.h"
#include "mavlink.h"

#ifndef MAVLINK_ROS_${basename}_BRIDGE_H
#define MAVLINK_ROS_${basename}_BRIDGE_H

#include <common/mavlink_ros_bridge.h>  //include the common mavlink_ros_bridge file

#ifndef NODEHANDLESINGLETON
#define NODEHANDLESINGLETON
//holds the only nodeHandle
class NodeHandleSingleton{
    public:
        static ros::NodeHandle& Instance()
        {
            if(!_pInstance)
            {
                Create();
            }
            return *_pInstance;
        }
    private:
        static void Create()
        {
            static ros::NodeHandle theInstance;
            _pInstance = &theInstance;
        }
        static ros::NodeHandle* _pInstance;
        NodeHandleSingleton();
        virtual ~NodeHandleSingleton();
        NodeHandleSingleton(const NodeHandleSingleton&);
        NodeHandleSingleton& operator=(const NodeHandleSingleton&);
};    
#endif //NODEHANDLESINGLETON

bool publisher_${basename}_advertised = false;

${{message:ros::Publisher mavlink_ros_msg_${name_lower}_publisher;
}}

// MESSAGE PUBLISHER DEFINITIONS
${{message:#include "mavlink_ros_msg_${name_lower}_publisher.h"
}}


/**
 * @brief Calls the appropriate parsing function for every message type
 *
 * @param msg The message to decode
 */
static inline void mavlink_ros_msg_decode_and_send_${basename}(const mavlink_message_t* msg)
{
    if(!publisher_${basename}_advertised){
     // Register all publishers   
    ${{message:\tstd::cout<<"Registering publisher for ${name_lower}"<<std::endl;
\tmavlink_ros_msg_${name_lower}_publisher = NodeHandleSingleton::Instance().advertise<mavlink_ros::mavlink_ros_msg_${name_lower}> ("/${basename}/${name_lower}", 1000);
\tstd::cout<<"Registered publisher for ${name_lower}"<<std::endl;
    }}
    \tpublisher_${basename}_advertised = true;
    }

    // Cases for all message types
    switch(msg->msgid)
    {
    ${{message:\tcase MAVLINK_MSG_ID_${name}:
    \t\tmavlink_ros_msg_${name_lower}_decode(msg);
    \t\tbreak;
}}
    default:
    //what a dirty hack
    #ifndef ROS_MSG_INCL //because the specialized version gets included first it will get the messages to parse first.
    #define ROS_MSG_INCL //this will then be defined for common.h
    mavlink_ros_msg_decode_and_send_common(msg); //which now get the message to parse
    break;
    #else
    ROS_DEBUG("unknown message type: %i", msg->msgid); //if this is also not defined in common, we make some noise
    break;
    #endif
    }
}

#endif // MAVLINK_ROS_${basename}_BRIDGE_H
''', xml)

    f.close()
             

def generate_message_rosmsg_h(directory, m):
    '''generate per-message header for a XML file'''
    f = open(os.path.join(directory, 'mavlink_ros_msg_%s_publisher.h' % m.name_lower), mode='w')
    t.write(f, '''
// MESSAGE ${name} PACKING TO ROS MESSAGE

#ifndef MAVLINK_ROS_MSG_ID_${name}_PUBLISHER_H
#define MAVLINK_ROS_MSG_ID_${name}_PUBLISHER_H
#include "mavlink.h"
#include <mavlink_ros_protocol.h>
#include <boost/array.hpp>

//include the ros generated message definition header that belongs to this message 
#include "mavlink_ros/mavlink_ros_msg_${name_lower}.h"

${{fields:
/**
 * @brief Get field ${name} from ${name_lower} message
 *
 * @return ${description}
 */
static inline ${return_type} mavlink_ros_msg_${name_lower}_get_${name}(const mavlink_message_t* msg${get_arg})
{
	return _MAV_RETURN_${type_tag}${array_tag}(msg, ${array_return_arg} ${wire_offset});
}
}}

/**
 * @brief Decode a ${name_lower} message into a ros message
 *
 * @param msg The message to decode
 */
static inline void mavlink_ros_msg_${name_lower}_decode(const mavlink_message_t* msg)
{
\t//instantiate ros message and copy data
\tmavlink_ros::mavlink_ros_msg_${name_lower} ${name_lower};
${{ordered_fields:	${decode_left}mavlink_ros_msg_${name_lower}_get_${name}(msg${decode_right});
}}
\t//publish the ros message
\tmavlink_ros_msg_${name_lower}_publisher.publish(${name_lower});
}

#endif //MAVLINK_ROS_MSG_ID_${name}_PUBLISHER_H
''', m)
    f.close()

def generate_message_rosmsg(directory, m):
    '''generate per-message rosmsg for a XML file'''
    f = open(os.path.join(directory, 'mavlink_ros_msg_%s.msg' % m.name_lower), mode='w')
    t.write(f, '''
#MAVLink comm protocol ros message generated from ${name}.xml
#see http://qgroundcontrol.org/mavlink/
#see http://ros.org/

#mavlink message name: ${name} id: ${id}

Header header
${{ordered_fields:${type}${array_suffix} ${name}\t\t#< ${description}
}}
''', m, True, True)
    f.close()

class mav_include(object):
    def __init__(self, base):
        self.base = base

def generate_one(basename, xml):
    '''generate headers for one XML file'''

    incl_directory = os.path.join(basename, "include", xml.basename)
    msg_directory = os.path.join(basename, "msg")
    mavparse.mkdir_p(msg_directory)
    
    print("Generating ROS messages in directory %s" % msg_directory)
    print("Generating ROS headers in directory %s" % incl_directory)
    mavparse.mkdir_p(incl_directory)

    if xml.little_endian:
        xml.mavlink_endian = "MAVLINK_LITTLE_ENDIAN"
    else:
        xml.mavlink_endian = "MAVLINK_BIG_ENDIAN"

    if xml.crc_extra:
        xml.crc_extra_define = "1"
    else:
        xml.crc_extra_define = "0"

    if xml.sort_fields:
        xml.aligned_fields_define = "1"
    else:
        xml.aligned_fields_define = "0"

    # work out the included headers
    xml.include_list = []
    for i in xml.include:
        base = i[:-4]
        xml.include_list.append(mav_include(base))

    # form message lengths array
    xml.message_lengths_array = ''
    for mlen in xml.message_lengths:
        xml.message_lengths_array += '%u, ' % mlen
    xml.message_lengths_array = xml.message_lengths_array[:-2]

    # and message CRCs array
    xml.message_crcs_array = ''
    for crc in xml.message_crcs:
        xml.message_crcs_array += '%u, ' % crc
    xml.message_crcs_array = xml.message_crcs_array[:-2]

    # form message info array
    xml.message_info_array = ''
    for name in xml.message_names:
        if name is not None:
            xml.message_info_array += 'MAVLINK_MESSAGE_INFO_%s, ' % name
        else:
            # Several C compilers don't accept {NULL} for
            # multi-dimensional arrays and structs
            # feed the compiler a "filled" empty message
            xml.message_info_array += '{"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, '
    xml.message_info_array = xml.message_info_array[:-2]

    # add some extra field attributes for convenience with arrays
    for m in xml.message:
        m.msg_name = m.name
        if xml.crc_extra:
            m.crc_extra_arg = ", %s" % m.crc_extra
        else:
            m.crc_extra_arg = ""
        for f in m.fields:
            if f.print_format is None:
                f.c_print_format = 'NULL'
            else:
                f.c_print_format = '"%s"' % f.print_format
            if f.array_length != 0: 
                if f.type == 'char':
                    f.ros_type = 'unsigned char'
                else:
                    f.ros_type = f.type
                f.type_tag = ''
                f.array_suffix = '[%u]' % f.array_length
                f.array_prefix = '*'
                f.array_tag = 'boost_array<%s, %u>' % (f.ros_type, f.array_length)
                f.array_arg = ', %u' % f.array_length
                f.array_return_arg = '%s, ' % (f.name)
                f.array_const = 'const '
                f.decode_left = ''
                f.decode_right = ', %s.%s' % (m.name_lower, f.name)
                f.return_type = 'uint16_t'
                f.get_arg = ', boost::array<%s,%u>& %s' % (f.ros_type, f.array_length, f.name)
                if f.type == 'char':
                    f.c_test_value = '"%s"' % f.test_value
                else:
                    test_strings = []
                    for v in f.test_value:
                        test_strings.append(str(v))
                    f.c_test_value = '{ %s }' % ', '.join(test_strings)
            else:
                f.type_tag = f.type
                f.array_suffix = ''
                f.array_prefix = ''
                f.array_tag = ''
                f.array_arg = ''
                f.array_return_arg = ''
                f.array_const = ''
                f.decode_left = "%s.%s = " % (m.name_lower, f.name)
                f.decode_right = ''
                f.get_arg = ''
                f.return_type = f.type
                if f.type == 'char':
                    f.c_test_value = "'%s'" % f.test_value
                elif f.type == 'uint64_t':
                    f.c_test_value = "%sULL" % f.test_value                    
                elif f.type == 'int64_t':
                    f.c_test_value = "%sLL" % f.test_value                    
                else:
                    f.c_test_value = f.test_value

    # cope with uint8_t_mavlink_version
    for m in xml.message:
        m.arg_fields = []
        m.array_fields = []
        m.scalar_fields = []
        for f in m.ordered_fields:
            if f.array_length != 0:
                m.array_fields.append(f)
            else:
                m.scalar_fields.append(f)
        for f in m.fields:
            if not f.omit_arg:
                m.arg_fields.append(f)
                f.putname = f.name
            else:
                f.putname = f.const_value

    generate_version_h(incl_directory, xml)
    generate_mavlink_ros_bridge_h(incl_directory, xml)
    for m in xml.message:
        generate_message_rosmsg_h(incl_directory, m)
        generate_message_rosmsg(msg_directory, m)


def generate(basename, xml_list):
    '''generate complete MAVLink ROS messages and respective C implemenation'''

    for xml in xml_list:
        generate_one(basename, xml)
