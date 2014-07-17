'''
Created on Jul 16, 2014

@author: posilva
'''

import re

import os
from utils import get_timestamp, generate_pkg_name

class MAVInclude(object):
    """
    """
    
    def __init__(self, filename):
        """
        """
        
        self.filename = filename
        self.name , _ = os.path.splitext( os.path.basename( self.filename ) )
        self.package_name =generate_pkg_name(self.name)  
        self.messages=[]
        self.enums=[]

class MAVMessage(object):
    """
    """
    
    def __init__(self, name, msgid, package_name,desc=None):
        """
        """
        self.name = name
        self.package_name = package_name
        self.id = msgid
        self.description = desc
        self.msg_fields = []

    def add_field(self, field):
        """
        """
        self.msg_fields.append(field)
    
    def to_msg(self):
        """
        """
        
        msg = "# Automatically Generated in " + str(get_timestamp()) + "\n" 
        msg += "# MESSAGE: " + self.name + "\n"   
        if not self.description is None:
            msg += "# Description:" + self.description.replace("\n", "\n#") + "\n"

        msg += "uint8 ID = " + self.id + "\n"        
        msg += "uint8 sysid" + "\n"
        msg += "uint8 compid" + "\n"

        for field in self.msg_fields:
            msg += field.to_string()
        return msg
    
    def filename(self):
        """
        """
        return self.name + ".msg"



class MAVEnum(object):
    """
    """
    
    def __init__(self, name, desc=None):
        """
        """
        self.name = "E_" + name
        self.description = desc
        self.entries = []


    def add_entry(self, entry):
        """
        """
        self.entries.append(entry)
    
    def to_msg(self):
        """
        """
        
        msg = "# Automatically Generated in " + str(get_timestamp) + "\n" 
        msg += "# ENUM: " + self.name + "\n"   
        if not self.description is None:
            msg += "# Description:" + self.description.replace("\n", "\n#") + "\n"
        for entry in self.entries:
            msg += entry.to_string()
        return msg
    
    def filename(self):
        """
        """
        return self.name + ".msg"

            
class MAVEntry(object):
    """
    """
    
    def __init__(self, name, value, desc=None):
        """
        """
        self.name = name
        self.description = desc
        self.value = value
        self.field_type = "uint16" if value > 255   else "uint8"
        
    def to_string(self):
        """                                                                    
        """
        msg = ""
        if not self.description is None: 
            msg += "# " + self.description + "\n"
        msg += self.field_type + " " + self.name + " = " + str(self.value) + "\n"
        return msg


data_types2ros = {
                    'char':'char',
                    'uint8_t':'uint8',
                    'uint8_t_mavlink_version':'uint8',
                    'int8_t':'int8',
                    'uint16_t':'uint16',
                    'int16_t':'int16',
                    'uint32_t':'uint32',
                    'int32_t':'int32',
                    'uint64_t':'uint64',
                    'int64_t':'int64',
                    'float':'float32',
                    'double':'float64',
                    }
data_types2mav = {
                    'char':'char',
                    'uint8':'uint8_t',
                    'int8':'int8_t',
                    'uint16':'uint16_t',
                    'int16':'int16_t',
                    'uint32':'uint32_t',
                    'int32':'int32_t',
                    'uint64':'uint64_t',
                    'int64':'int64_t',
                    'float32':'float',
                    'float64':'double',
                    }
class MAVField(object):
    """
    """
   
                        
    def is_array(self, data_type):
        """
        """
        m = re.search(r'(.*)\[(.*)\].*', data_type, re.I)
        if m:
            t = m.group(1)
            s = m.group(2)
            return (t, s)
        else: 
            return (None, None)
        if (data_type):
            pass
        
    def get_mav_type(self, data_type):
        """
        """
        t, s = self.is_array(data_type)
        if t is None:
            return data_types2mav[data_type]
        else:
            self.is_array_type = True
            self.array_size = int(s)
            return data_types2mav[t]
    
    def get_ros_type(self, data_type):
        """
        """
        t, s = self.is_array(data_type)
        if t is None:
            return data_types2ros[data_type]
        else:
            self.is_array_type = True
            self.array_size = int(s)
            return data_types2ros[t]
        
    
    def __init__(self, name, field_type, desc=None):
        """
        """
        self.name = name
        self.description = desc
        self.is_array_type = False
        self.array_size = 0
        self.field_type = self.get_ros_type(field_type)

    def to_string(self):
        """
        """
        msg = ""                                                                                     
        if not self.description is None:
            msg += "# " + self.description 
            
        data_type = self.field_type if not self.is_array_type else self.field_type + "[" + str(self.array_size) + "]"
        msg += data_type + " " + self.name + "\n"
        return msg                 

