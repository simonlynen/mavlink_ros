#ifndef  _MAVLINK_ROS_PROTOCOL_H_
#define  _MAVLINK_ROS_PROTOCOL_H_

#include "protocol.h"
#include <boost/array.hpp>

template<typename T, std::size_t N> 
static inline uint16_t _MAV_RETURN_boost_array(const mavlink_message_t *msg, boost::array<T, N>& value, uint8_t wire_offset)
{
	memcpy(value.c_array(), &_MAV_PAYLOAD(msg)[wire_offset], N);
	return N;
}


#endif // _MAVLINK_ROS_PROTOCOL_H_
