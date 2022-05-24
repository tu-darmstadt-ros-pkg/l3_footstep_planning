#!/usr/bin/env python

import struct
import rospy

def pack_bool(value):
    return struct.pack('<?', value)

def pack_char(value):
    return struct.pack('<c', value)

def pack_int8(value):
    return struct.pack('<b', value)

def pack_int16(value):
    return struct.pack('<h', value)

def pack_int32(value):
    return struct.pack('<i', value)

def pack_int64(value):
    return struct.pack('<q', value)

def pack_uint8(value):
    return struct.pack('<B', value)

def pack_uint16(value):
    return struct.pack('<H', value)

def pack_uint32(value):
    return struct.pack('<I', value)

def pack_uint64(value):
    return struct.pack('<Q', value)

def pack_float(value):
    return struct.pack('<f', value)

def pack_double(value):
    return struct.pack('<d', value)

def pack_string(value):
    return pack_uint64(len(value)) + value



def unpack(fmt, size, bytestream, offset = 0):
    return struct.unpack_from(fmt, bytestream, offset)[0], offset + size

def unpack_bool(bytestream, offset = 0):
    return unpack('<?', 4, bytestream, offset)

def unpack_char(bytestream, offset = 0):
    return unpack('<c', 1, bytestream, offset)

def unpack_int8(bytestream, offset = 0):
    return unpack('<b', 1, bytestream, offset)

def unpack_int16(bytestream, offset = 0):
    return unpack('<h', 2, bytestream, offset)

def unpack_int32(bytestream, offset = 0):
    return unpack('<i', 4, bytestream, offset)

def unpack_int64(bytestream, offset = 0):
    return unpack('<q', 8, bytestream, offset)

def unpack_uint8(bytestream, offset = 0):
    return unpack('<B', 1, bytestream, offset)

def unpack_uint16(bytestream, offset = 0):
    return unpack('<H', 2, bytestream, offset)

def unpack_uint32(bytestream, offset = 0):
    return unpack('<I', 4, bytestream, offset)

def unpack_uint64(bytestream, offset = 0):
    return unpack('<Q', 8, bytestream, offset)

def unpack_float(bytestream, offset = 0):
    return unpack('<f', 4, bytestream, offset)

def unpack_double(bytestream, offset = 0):
    return unpack('<d', 8, bytestream, offset)

def unpack_string(bytestream, offset = 0):
    size, offset = unpack_uint64(bytestream, offset)
    value = "".join(struct.unpack_from('<' + str(size) + 'c', bytestream, offset))
    offset += size
    return value, offset

