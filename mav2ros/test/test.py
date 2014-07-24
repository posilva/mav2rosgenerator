'''
Created on Jul 17, 2014

@author: posilva
'''
import unittest

from mav2ros.tools import *
class TestMAV2ROSGenerator(unittest.TestCase):
    def test_mavgenerator(self):
        definitions_file = '/home/posilva/Sistema/Develop/libs/mavlink-1.0.11/message_definitions/v1.0/ardupilotmega.xml'
        output_dir = '/home/posilva/Sistema/Develop/projects/github/mav2rosgenerator/ardupilotmega-ros-pkg-generated'

        generator = MAVGenerator(definitions_file, output_dir)
        generator.generate(True, True);

        self.assertEqual(len(generator.parser.enums), 6, "Enums Parsed: " + str(len(generator.parser.enums)) + " Expected: 8")
        self.assertEqual(len(generator.parser.messages), 22, "Messages Parsed: " + str(len(generator.parser.messages)) + " Expected: 22")
        self.assertEqual(len(generator.parser.includes), 1, "Includes Parsed: " + str(len(generator.parser.includes)) + " Expected: 1")


if __name__ == "__main__":
    suite = unittest.TestLoader().loadTestsFromTestCase(TestMAV2ROSGenerator)
    unittest.TextTestRunner(verbosity=2).run(suite)
# unittest.main()
