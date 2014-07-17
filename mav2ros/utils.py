'''
Created on Jul 17, 2014

@author: posilva
'''
import os
import errno
import datetime
def get_timestamp():
    return datetime.datetime.utcnow()

def generate_pkg_name(name):
    return "mavlink_" + name + "_msgs"

def generate_file(filename,content):
    #===========================================================================
    # write a file to disk
    #===========================================================================
    f = open(filename, 'w')
    f.write(content)
    f.flush()
    f.close()

def mk_dirs(dirname):
    #===========================================================================
    # Force that a named directory exists; if it does not, attempt to create it.
    #===========================================================================
    try:
        os.makedirs(dirname)
    except OSError, e:
        if e.errno != errno.EEXIST:
            raise