import unittest
import pprint

from _base import *
from _generated import *


class Test(unittest.TestCase):

    def setUp(self):
        pass

    def test_pub_sub(self):

        manager = Manager()
        topic_actuator_armed = Topic_actuator_armed(
            0, 0, 0, 0, 0)
        actuator_armed_pub = Publication(
            manager, 'actuator_armed', topic_actuator_armed)
        actuator_armed_sub = Subscription(
            manager, 'actuator_armed')

        print '\ninitial pub data'
        print actuator_armed_pub.data

        actuator_armed_pub.data.armed = 1
        print '\npub data set armed', actuator_armed_pub.data

        print '\nsub updated:', actuator_armed_sub.updated()
        assert(actuator_armed_sub.updated() is False)

        print '\nsub data:', actuator_armed_sub.data
        assert(actuator_armed_sub.data.armed == 0)

        print '\npublish new actuator_armed topic'
        actuator_armed_pub.publish()

        print'\nsub updated:',  actuator_armed_sub.updated()
        assert(actuator_armed_sub.updated() is True)

        print'\nupdate sub data'
        actuator_armed_sub.update()

        print '\nsub data:', actuator_armed_sub.data
        assert(actuator_armed_sub.data.armed == 1)

        print'\nsub updated:',  actuator_armed_sub.updated()
        assert(actuator_armed_sub.updated() is False)

        pprint.pprint(manager._publisher_handle)
        pprint.pprint(manager._subscriber_handle)
        pprint.pprint(manager._topics)
