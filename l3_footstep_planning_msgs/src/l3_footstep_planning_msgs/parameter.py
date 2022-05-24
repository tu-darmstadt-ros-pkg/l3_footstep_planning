#!/usr/bin/env python

import struct
import rospy
import l3_footstep_planning_msgs.msg

from vigir_generic_params.msg import ParameterMsg
from l3_footstep_planning_msgs.serialization import *

class Parameter():

    TYPE_INVALID    = 0
    TYPE_BOOLEAN    = 1
    TYPE_INT        = 2
    TYPE_DOUBLE     = 3
    TYPE_STRING     = 4
    TYPE_DATE_TIME  = 5
    TYPE_BASE64     = 6
    TYPE_ARRAY      = 7
    TYPE_STRUCT     = 8

    _type = int()
    _name = str()
    _value = None

    def __init__(self, _type = TYPE_INVALID, name = str(), value = None, msg = None):
        if msg is not None:
            self.from_msg(msg)
        elif (_type != self.TYPE_INVALID):
            self.set_type(_type)
            self.set_name(name)
            self.set_value(value)

    def __str__(self):
        return self._name + " (" + str(self.get_type_as_text()) + "): " + str(self._value)

    def __repr__(self):
        return self.__str__()

    def set_name(self, name):
        self._name = name

    def get_name(self):
        return self._name

    def set_type(self, t):
        self._type = t

    def get_type(self):
        return self._type

    def get_type_as_text(self):
        if self.get_type() == self.TYPE_BOOLEAN:
            return "bool"
        elif self.get_type() == self.TYPE_INT:
            return "int"
        elif (self.get_type() == self.TYPE_DOUBLE):
            return "double"
        elif (self.get_type() == self.TYPE_STRING):
            return "string"
        elif (self.get_type() == self.TYPE_DATE_TIME):
            return "date_time"
        elif (self.get_type() == self.TYPE_BASE64):
            return "base64"
        elif (self.get_type() == self.TYPE_ARRAY):
            return "array"
        elif (self.get_type() == self.TYPE_STRUCT):
            return "struct"
        else:
            return "unknown"

    def set_value(self, value):
        try:
            if self.get_type() == self.TYPE_BOOLEAN:
                if (value.lower() == "true"):
                    self._value = True
                if (value.lower() == "false"):
                    self._value = False
            elif self.get_type() == self.TYPE_INT:
                self._value = int(value)
            elif (self.get_type() == self.TYPE_DOUBLE):
                self._value = float(value)
            elif (self.get_type() == self.TYPE_STRING):
                self._value = str(value)
            elif (self.get_type() == self.TYPE_ARRAY):
                if (len(self._value) == 0):
                    return False
                _type = self._value[0].get_type()
                _name = self._value[0].get_name()
                _value = []
                for val in value.split():
                    val = val.strip('[],')
                    p = Parameter(_type, _name, val)
                    if p.get_value() is not None:
                        _value.append(p)
                    else:
                        return False
                self._value = _value
            else:
                rospy.logerr("set_value called on parameter with incompatible type!")
                return False
        except ValueError:
            return False
        return True

    def get_value(self):
        if (self.get_type() == self.TYPE_ARRAY):
            val = []
            for p in self._value:
                val.append(p.get_value())
            return val
        else:
            return self._value

    def to_msg(self):
        msg = l3_footstep_planning_msgs.msg.ParameterMsg()
        msg.key.data = self.get_name()

        # dispatch type
        msg.data += pack_uint8(self.get_type())

        if self.get_type() == self.TYPE_BOOLEAN:
            msg.data += pack_bool(self._value)
        elif self.get_type() == self.TYPE_INT:
            msg.data += pack_int32(self._value)
        elif (self.get_type() == self.TYPE_DOUBLE):
            msg.data += pack_double(self._value)
        elif (self.get_type() == self.TYPE_STRING):
            msg.data += pack_string(self._value)
        elif (self.get_type() == self.TYPE_ARRAY):
            msg.data += pack_uint32(len(self._value))
            for p in self._value:
                msg.data += p.to_msg().data
        elif (self.get_type() == self.TYPE_STRUCT):
            msg.data += pack_uint32(len(self._value))
            for p in self._value:
                msg.data += pack_string(p.get_name()) + p.to_msg().data
        else:
            print("ERROR: Unsupported type (" + str(self.get_type()) + ")!")

        return msg

    def from_msg(self, msg, offset = 0):

        self._name = msg.key.data

        # dispatch type
        self._type, offset = unpack_uint8(msg.data, offset)

        if self.get_type() == self.TYPE_BOOLEAN:
            self._value, offset = unpack_bool(msg.data, offset)
        elif self.get_type() == self.TYPE_INT:
            self._value, offset = unpack_int32(msg.data, offset)
        elif (self.get_type() == self.TYPE_DOUBLE):
            self._value, offset = unpack_double(msg.data, offset)
        elif (self.get_type() == self.TYPE_STRING):
            self._value, offset = unpack_string(msg.data, offset)
        elif (self.get_type() == self.TYPE_ARRAY):
            size, offset = unpack_uint32(msg.data, offset)
            self._value = []
            for i in range(size):
                p = Parameter()
                offset = p.from_msg(msg, offset)
                self._value.append(p)
        elif (self.get_type() == self.TYPE_STRUCT):
            size, offset = unpack_uint32(msg.data, offset)
            self._value = []
            for i in range(size):
                # read name
                name, offset = unpack_string(msg.data, offset)
                # read param
                p = Parameter()
                offset = p.from_msg(msg, offset)
                p.set_name(name)
                self._value.append(p)
        else:
            print("ERROR: Unsupported type (" + str(self.get_type()) + ")!")

        return offset

