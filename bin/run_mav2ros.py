#!/usr/local/bin/python2.7
# encoding: utf-8
'''
bin.run_mav2ros -- Execute Mavlink to ROS package generator

bin.run_mav2ros is a Tool to execute Mavlink to ROS package generator

It defines classes_and_methods

@author:     Pedro Marques da Silva

@copyright:  2014 Pedro Marques da Silva. All rights reserved.

@license:    LGPL

@contact:    posilva@gmail.com
@deffield    updated: Updated
'''

import sys
import os

from argparse import ArgumentParser
from argparse import RawDescriptionHelpFormatter
from mav2ros.tools import MAVGenerator 
__all__ = []
__version__ = 0.1
__date__ = '2014-07-24'
__updated__ = '2014-07-24'

DEBUG = 0
TESTRUN = 0
PROFILE = 0

class CLIError(Exception):
    '''Generic exception to raise and log different fatal errors.'''
    def __init__(self, msg):
        super(CLIError).__init__(type(self))
        self.msg = "E: %s" % msg
    def __str__(self):
        return self.msg
    def __unicode__(self):
        return self.msg

def main(argv=None): # IGNORE:C0111
    '''Command line options.'''

    if argv is None:
        argv = sys.argv
    else:
        sys.argv.extend(argv)

    program_name = os.path.basename(sys.argv[0])
    program_version = "v%s" % __version__
    program_build_date = str(__updated__)
    program_version_message = '%%(prog)s %s (%s)' % (program_version, program_build_date)
    program_shortdesc = __import__('__main__').__doc__.split("\n")[1]
    program_license = '''%s

  Created by Pedro Marques da Silva on %s.
  Copyright 2014 Pedro Marques da Silva. All rights reserved.

  Licensed under the LGPL 2.0
  http://www.apache.org/licenses/LICENSE-2.0

  Distributed on an "AS IS" basis without warranties
  or conditions of any kind, either express or implied.

USAGE
''' % (program_shortdesc, str(__date__))

    try:
        # Setup argument parser
        parser = ArgumentParser(description=program_license, formatter_class=RawDescriptionHelpFormatter)
        parser.add_argument("-v", "--verbose", dest="verbose", action="count", help="set verbosity level [default: %(default)s]")
        parser.add_argument("-m", "--messages", dest="messages", help="Mavlink messages definition file . [default: %(default)s]", metavar="MESSAGE_DEFINITION_XML")
        parser.add_argument("-o", "--output_dir", dest="output", help="Generation output base dir. ie. 'src' folder of a ROS workspace . [default: %(default)s]", metavar="OUTPUT_DIR" )
        parser.add_argument("--mavgen", dest="with_mavlink",action='store_true', help="Enable  mavlink generator 'mavgen.py'")
        parser.add_argument("--no-mavgen", dest="with_mavlink",action='store_false', help="Disable  mavlink generator 'mavgen.py'  [default: %(default)s]")
        parser.set_defaults(with_mavlink=True)
        parser.add_argument('-V', '--version', action='version', version=program_version_message)

        # Process arguments
        args = parser.parse_args()

        verbose = args.verbose
        messages = args.messages
        output_dir = args.output
        with_mavlink = args.with_mavlink
        
        if verbose > 0:
            print("Verbose mode on")
            print ("Definition File: " + messages)
            print ("Output dir: " + output_dir)
                
        if (not os.path.exists(messages)):    
            CLIError("Definition file missing ")
            
        generator = MAVGenerator
        generator.generate(True, with_mavlink)
        return 0
    except KeyboardInterrupt:
        ### handle keyboard interrupt ###
        return 0
    except Exception, e:
        if DEBUG or TESTRUN:
            raise(e)
        indent = len(program_name) * " "
        sys.stderr.write(program_name + ": " + repr(e) + "\n")
        sys.stderr.write(indent + "  for help use --help")
        return 2

if __name__ == "__main__":
    if DEBUG:
        sys.argv.append("-h")
        sys.argv.append("-v")
        
    if TESTRUN:
        import doctest
        doctest.testmod()
    if PROFILE:
        import cProfile
        import pstats
        profile_filename = 'bin.run_mav2ros_profile.txt'
        cProfile.run('main()', profile_filename)
        statsfile = open("profile_stats.txt", "wb")
        p = pstats.Stats(profile_filename, stream=statsfile)
        stats = p.strip_dirs().sort_stats('cumulative')
        stats.print_stats()
        statsfile.close()
        sys.exit(0)
    sys.exit(main())