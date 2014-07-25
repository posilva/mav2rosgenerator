'''
Created on Jul 17, 2014

@author: posilva
'''

import os

from model import MAVEntry, MAVEnum, MAVMessage, MAVField, MAVInclude, data_types2mav
from templates import *
from utils import generate_file, get_timestamp, generate_pkg_name, mk_dirs
import xml.etree.ElementTree as ET


class MavlinkGenerator(object):
    """
    """
    def __init__(self, definitions, include_dir, version="1.0.11"):
        self.definitions = definitions
        self.include_dir = include_dir
        self.version = version
        pass
    def generate(self):
        url = "https://github.com/mavlink/mavlink/archive/" + self.version + ".tar.gz"
        mavlink_file = os.path.basename(url);

        import tempfile
        temp_dir = tempfile.mkdtemp()
        mavlink_download_file = temp_dir + os.sep + mavlink_file

        try:
            import urllib2
            mavlink_targz = urllib2.urlopen(url)
            output = open(mavlink_download_file, 'wb')
            output.write(mavlink_targz.read())
            output.close()

            if not os.path.exists(mavlink_download_file):
                raise "Failed download file: '" + mavlink_download_file + "'"
        except :
            raise "Failed download file: '" + mavlink_download_file + "' from '" + url + "'"

        import tarfile
        tar = tarfile.open(mavlink_download_file, 'r|gz')
        tar.extractall(temp_dir)
        root_dir = temp_dir + os.sep + tar.members[0].name

        mav_generator = root_dir + os.sep + "generator" + os.sep + "mavgen.py"
        if (not os.path.exists(mav_generator)):
            raise "Generator not found: '" + mav_generator + "'"

        gen_dir = self.include_dir
        cmd = "" + mav_generator + "  --lang=C --wire-protocol=1.0 --output=" + gen_dir + " " + self.definitions

        import subprocess
        proc = subprocess.Popen(args=cmd, stdout=subprocess.PIPE, shell=True)
        (out, _) = proc.communicate()
        return out

class ROSMessagesGenerator(object):
    """
    """
    def __init__(self, messages, pkg_dir):
        self.messages = messages
        self.pkg_dir = pkg_dir
        pass

    def generate(self):
        msg_dir = self.pkg_dir + os.sep + "msg"
        mk_dirs(msg_dir)
        for msg in self.messages:
            filename = msg_dir + os.sep + msg.name + ".msg"
            generate_file(filename, msg.to_msg())

class ROSPackageGenerator(object):
    """
    """
    def __init__(self, mav_generator):
        self.mav_generator = mav_generator

        pass


    def __gen_cmake_add_messages(self):
        add_messages = ""
        for message in self.mav_generator.parser.messages + self.mav_generator.parser.enums:
            add_messages += "\t" + message.name + ".msg" + os.linesep
        return add_messages


    def __gen_cmake_node(self, include_node=False):
        cmake_node = ""
        if (include_node):
            cmake_node = cmake_node_template
        return cmake_node

    def generate(self, include_node=False):

        messages_deps = ""
        run_deps = ""
        build_deps = ""
        for include in self.mav_generator.parser.includes:
            messages_deps += include.package_name + os.linesep
            build_deps += "\t" + "<build_depend>" + include.package_name + "</build_depend>" + os.linesep
            run_deps += "\t" + "<run_depend>" + include.package_name + "</run_depend>" + os.linesep

        raw_message=""
        if (include_node):
            raw_message=MAV_RAW_DATA_MSG+".msg"    
        cmakelists = cmakelists_template.replace(CMAKE_PKG_ADD_MESSAGE_PLACEHOLDER, self.__gen_cmake_add_messages()).\
                                  replace(CMAKE_PKG_ADD_RAW_MESSAGE_PLACEHOLDER, raw_message).\
                                  replace(CMAKE_PKG_ADD_NODE_PLACEHOLDER, self.__gen_cmake_node(include_node)).\
                                  replace(CMAKE_PKG_GENERATE_MESSAGES_PLACEHOLDER, messages_deps).\
                                  replace(CMAKE_PKG_DEPS_PLACEHOLDER, messages_deps).\
                                  replace(PKG_NAME_PLACEHOLDER, self.mav_generator.package_name)

        generate_file(self.mav_generator.package_dir + os.sep + "CMakeLists.txt", cmakelists)

        package_xml = package_xml_template.replace(PKG_XML_RUN_DEPS_PLACEHOLDER, run_deps).\
                                  replace(PKG_XML_BUILD_DEPS_PLACEHOLDER, build_deps).\
                                  replace(PKG_NAME_PLACEHOLDER, self.mav_generator.package_name)

        generate_file(self.mav_generator.package_dir + os.sep + "package.xml", package_xml)


class ROSAPIGenerator(object):
    """
    """

    def __init__(self, mav_generator):
        self.mav_generator = mav_generator

    def __gen_header_file(self, header_name):
        """
        """

        header_file = "// Automatically Generated in " + str(get_timestamp()) + os.linesep
        include_guard = self.mav_generator.package_name.upper() + "_" + header_name.upper() + "_H"
        header_file += "#ifndef " + include_guard + os.linesep
        header_file += "#define " + include_guard + os.linesep

        for include in self.mav_generator.parser.includes:
            for message in (include.messages + include.enums):
                header_file += "#include <" + include.package_name + os.sep + message.name + ".h>" + os.linesep

        for message in (self.mav_generator.parser.messages + self.mav_generator.parser.enums):
            header_file += "#include <" + self.mav_generator.package_name + os.sep + message.name + ".h>" + os.linesep

        header_file += "#include <" + self.mav_generator.package_name + os.sep + MAV_RAW_DATA_MSG+".h>" + os.linesep
        header_file += "#endif // " + include_guard + os.linesep
        return header_file
    
    def __gen_publishers_decl(self, message_list):
        publishers = ""
        for message in message_list:
            publishers += publisher_decl_template.replace(NODE_MESSAGE_NAME_LOWER_PLACE_HOLDER, message.name.lower()).\
                                                replace(NODE_MESSAGE_NAME_PLACE_HOLDER, message.name).\
                                                replace(PKG_NAME_PLACEHOLDER, message.package_name)

        return publishers
        
    def __gen_publishers_init(self, message_list):
        publishers = ""
        for message in message_list:
            publishers += publisher_init_template.replace(NODE_MESSAGE_NAME_LOWER_PLACE_HOLDER, message.name.lower()).\
                                                replace(NODE_MESSAGE_NAME_PLACE_HOLDER, message.name).\
                                                replace(PKG_NAME_PLACEHOLDER, message.package_name)

        return publishers
        
    def __gen_subscribers_decl(self, message_list):
        subscribers = ""
        for message in message_list:
            subscribers += subscriber_decl_template.replace(NODE_MESSAGE_NAME_LOWER_PLACE_HOLDER, message.name.lower()).\
                                                replace(NODE_MESSAGE_NAME_PLACE_HOLDER, message.name).\
                                                replace(PKG_NAME_PLACEHOLDER, message.package_name)
        return subscribers
        
        
     
    def __gen_source_file(self):
        message_list = []
        
        for include in self.mav_generator.parser.includes:
            message_list += include.messages
        message_list += self.mav_generator.parser.messages
        
        source_file = ros_node_template.\
                        replace(NODE_MESSAGES_CALLBACKS_PLACE_HOLDER, self.__gen_callbacks(message_list)).\
                        replace(NODE_PUBLISHERS_DECL_PLACE_HOLDER, self.__gen_publishers_decl(message_list)).\
                        replace(NODE_PUBLISHERS_INIT_PLACE_HOLDER, self.__gen_publishers_init(message_list)).\
                        replace(NODE_SUBSCRIBERS_DECL_PLACE_HOLDER, self.__gen_subscribers_decl(message_list)).\
                        replace(NODE_MAV_SWITCH_CASES_PLACE_HOLDER, self.__gen_switch_cases(message_list)).\
                        replace(PKG_NAME_PLACEHOLDER, self.mav_generator.package_name).\
                        replace(PKG_DEFINITIONS_NAME_PLACEHOLDER, self.mav_generator.parser.definition_name)
        return source_file

    def __gen_switch_case_fields(self, message):
        switch_case_fields = ""
        for field in message.msg_fields:
            if (not field.is_array_type):
                switch_case_fields += "\tm." + field.name + " = $NODE_MESSAGE_NAME_LOWER$_in." + field.name + ";\n"
            else:
                switch_case_fields += "\tmemcpy(&(m." + field.name + "), &($NODE_MESSAGE_NAME_LOWER$_in." + field.name + "), sizeof(" + data_types2mav[field.field_type] + ")*" + str(field.array_size) + ");\n"

        return switch_case_fields

    def __gen_switch_cases(self, message_list):
        switch_cases = ""
        for message in message_list:
            case = switch_case_template.\
            replace(NODE_MAV_SWITCH_CASE_FIELDS_PLACE_HOLDER, self.__gen_switch_case_fields(message)).\
            replace(NODE_MESSAGE_NAME_LOWER_PLACE_HOLDER, message.name.lower()).\
            replace(NODE_MESSAGE_NAME_PLACE_HOLDER, message.name).\
            replace(PKG_NAME_PLACEHOLDER, message  .package_name)
            switch_cases += case

        return switch_cases

    def __gen_callback_message_fields(self, message):
        message_fields = ""
        for field in message.msg_fields:
            if (not field.is_array_type):
                message_fields += "\t$NODE_MESSAGE_NAME_LOWER$_out." + field.name + " = msg->" + field.name + ";\n"
            else:
                message_fields += "\tmemcpy(&($NODE_MESSAGE_NAME_LOWER$_out." + field.name + "), &(msg->" + field.name + "[0]), sizeof(" + data_types2mav[field.field_type] + ")* (int)(msg->" + field.name + ".size()));\n"
        
        return message_fields

    def __gen_callbacks(self, message_list):
        messages_callback = ""
        for message in message_list:
            callback = messages_callback_template.replace(NODE_MESSAGES_CALLBACK_FIELDS_PLACE_HOLDER, self.__gen_callback_message_fields(message)).\
                                                  replace(NODE_MESSAGE_NAME_LOWER_PLACE_HOLDER, message.name.lower()).\
                                                  replace(NODE_MESSAGE_NAME_PLACE_HOLDER, message.name).\
                                                  replace(PKG_NAME_PLACEHOLDER, message.package_name)
            messages_callback += callback

        return messages_callback
    
    
    def generate(self):
        
        filename = self.mav_generator.msg_dir + os.sep +MAV_RAW_DATA_MSG+".msg"
        generate_file(filename, mav_raw_message_template)
        
        header_name = "mavlink2ros"
        generate_file(self.mav_generator.include_dir + os.sep + header_name + ".h" , self.__gen_header_file(header_name))
        
        generate_file(self.mav_generator.source_dir + os.sep + self.mav_generator.package_name + "_node.cpp" , self.__gen_source_file())

        pass

class MAVParser(object):
    """
    """
    def __init__(self, package_name, definitions=None):
        """
        """
        self.definitions = None
        self.package_name = package_name
        self.set_definitions(definitions)
        self.messages = [];
        self.enums = [];
        self.includes = [];
        # get definitions root node

    def set_definitions(self, definitions):
        """
        """
        if (not definitions is None):
            self.definitions = definitions
            self.definition_name, _ = os.path.splitext(os.path.basename(self.definitions))
            self.definitions_dir = os.path.dirname(os.path.abspath(self.definitions))
            tree = ET.parse(self.definitions)
            self.mavlink_node = tree.getroot()
            if (self.mavlink_node.tag != "mavlink"):
                raise "Invalid root tag for a mavlink definitions file: Expected 'mavlink'"



    def __internal_parse(self):
        for include in self.mavlink_node.iter('include'):
            include_file = self.definitions_dir + os.sep + include.text;
            mav_include = MAVInclude(include_file)
            self.includes.append(mav_include)
        for node in self.mavlink_node.iter('enum'):
            self.__parse_message(node)
        for node in self.mavlink_node.iter('message'):
            self.__parse_message(node)

        pass

    def parse(self, definitions=None):
        """
        """

        if (definitions is None and self.definitions is None):
            raise "No valid file given to parse"
        else:
            if (self.definitions is None):
                self.set_definitions(definitions)
            else:
                self.set_definitions(self.definitions)

            self.__internal_parse()
        return True


    def __parse_message(self, node):
        """
        """
        is_message = (node.tag == 'message')
        is_enum = (node.tag == 'enum')

        if not (is_enum or is_message) :
            raise "Invalid node must have 'message' or 'enum' tag"

        if 'name' not in node.attrib:
            raise "Missing required attribute: 'name'"

        desc = node.find('description')
        if not desc is None:
            desc = desc.text

        if is_message:
            mav_message = MAVMessage(node.attrib['name'] , node.attrib['id'], self.package_name, desc)
            fields = node.findall("./field")
            if len(fields) > 0 :
                for field in fields:
                    desc = field.find('description')
                    if not desc is None:
                        desc = desc.text
                    mav_field = MAVField(field.attrib['name'], field.attrib['type'], desc)
                    mav_message.add_field(mav_field)
            self.messages.append(mav_message)
        elif is_enum:
            mav_enum = MAVEnum(node.attrib['name'], desc)
            entries = node.findall("./entry")
            if len(entries) > 0 :
                default_entry_value = 0
                for entry in entries:
                    desc = entry.find('description')
                    if not desc is None:
                        desc = desc.text
                    if 'value' not in entry.attrib :
                        default_entry_value += 1
                        entry_value = default_entry_value;
                    else:
                        entry_value = int(entry.attrib['value'])
                        default_entry_value = int(entry_value)

                    mav_entry = MAVEntry(node.tag.upper()[0] + "_" + entry.attrib['name'], entry_value, desc)
                    mav_enum.add_entry(mav_entry)
            self.enums.append(mav_enum)

class MAVGenerator(object):
    """
    """

    def __init__(self, definitions , output_dir):
        """
        """
        if (definitions is None):
            raise "Invalid definitions file: cannot be NoneType"

        if (output_dir is None):
            raise "Invalid Output dir: cannot be NoneType"

        self.output_dir = output_dir

        filename, _ = os.path.splitext(os.path.basename(definitions))
        # setup ROS packege name
        self.package_name = generate_pkg_name(filename)
        # setup ROS package paths
        self.package_dir = output_dir + os.sep + self.package_name
        self.include_dir = self.package_dir + os.sep + "include" + os.sep + self.package_name
        self.source_dir = self.package_dir + os.sep + "src"
        self.msg_dir = self.package_dir + os.sep + "msg"
        # force path creation
        mk_dirs(self.package_dir)
        mk_dirs(self.include_dir)
        mk_dirs(self.source_dir)
        mk_dirs(self.msg_dir)
        # create parser instance
        self.parser = MAVParser(self.package_name, definitions)

    def generate(self, with_node=False , with_mavlink=False):
        """
        """
        if not self.parser.parse():
            raise "File parsing failed, reason: " + self.parser.definitions
        
        # generate code for each included dependency
        for include in self.parser.includes:
            include_generator = MAVGenerator(include.filename, self.output_dir)
            include_generator.generate();

            include.messages = include_generator.parser.messages
            include.enums = include_generator.parser.enums

        msg_generator = ROSMessagesGenerator(self.parser.messages + self.parser.enums , self.package_dir);
        msg_generator.generate();

        package_generator = ROSPackageGenerator(self)
        package_generator.generate(with_node)

        if (with_node):
            
            api_generator = ROSAPIGenerator(self)
            api_generator.generate()
            
        if (with_mavlink):
            mavlink_generator = MavlinkGenerator(self.parser.definitions, self.include_dir)
            mavlink_generator.generate()
