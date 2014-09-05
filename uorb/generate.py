import xml.etree.ElementTree as ET
import string
import time
import datetime

uorb_file_header = '''# Do not edit!
# automatically generated using uorb/generate.py
# edit uorb/uorb.xml

generation_timestamp = '{timestamp:s}'
'''

topic_class_template = '''

class Topic_{name:s}(object):
    """
    {doc_string:s}

    Parameters
    ----------
    {attrib_doc_string:s}
    """

    def __init__(self, {arg_list_comma:s}):
        {init_list:s}

    @property
    def _fields(self):
        return ({str_field_names:s})

    @property
    def _values(self):
        return ({self_field_names:s})

    def __str__(self):
        return str(self._fields) + ' ' + str(self._values)

    __repr__ = __str__
'''
topic_attribute_init = "self.{field_name:s} = {field_name:s}"
attrib_doc = "{field_name:s} :\n        {field_descr:s}"
indent12 = "\n            "
indent8 = "\n        "
indent4 = "\n    "
timestamp = datetime.datetime.fromtimestamp(time.time()).strftime('%Y-%m-%d %H:%M:%S')

with open('_generated.py', 'w') as f:
    f.write(uorb_file_header.format(**locals()))
    tree = ET.parse('uorb.xml')
    for topic in tree.getroot().findall('topic'):
        name = topic.attrib['name']
        doc_string = topic.find('description').text

        # create arg list
        field_names = []
        field_names_init = []
        self_field_names = []
        str_field_names = []
        attrib_doc_strings = []
        for field in topic.findall('field'):
            field_names.append(field.attrib['name'])
            field_names_init.append(field.attrib['name'] + '=None')
            field_descr = field.text
            field_name = field.attrib['name']
            self_field_names.append('self.'+field_name)
            str_field_names.append("'{:s}'".format(field_name))
            attrib_doc_strings.append(attrib_doc.format(**locals()))
        str_field_names = string.join(str_field_names, ', ')
        self_field_names = string.join(self_field_names, ', ')
        arg_list = string.join(field_names, ','+indent12)
        arg_list_space = string.join(field_names, ' ')
        arg_list_comma = string.join(field_names_init, ',' +indent12)
        attrib_doc_string = string.join(attrib_doc_strings, indent4)

        # create init list
        init_list = []
        for field_name in field_names:
            init_list.append(topic_attribute_init.format(**locals()))
        init_list = string.join(init_list, indent8)

        f.write(topic_class_template.format(**locals()))
