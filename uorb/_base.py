import uuid
import copy

__all__ = ['Manager', 'Subscription', 'Publication']


class Manager(object):

    def __init__(self):
        self._topics = {}
        self._publisher_handle = {}
        self._subscriber_handle = {}

    def advertise(self, name):
        if name in self._topics.keys():
            raise ValueError('{:s} already advertised'.format(name))
        handle = uuid.uuid4().get_hex()
        self._publisher_handle[handle] = name
        self._topics[name] = {
            'handle': handle,
            'data': None,
            'subscribers': {},
        }
        return handle

    def unadvertise(self, handle):
        name = self._publisher_handle[handle]
        self._topics.pop(name, None)
        self._publisher_handle.pop(handle, None)

    def publish(self, handle, data):
        name = self._publisher_handle[handle]
        self._topics[name]['data'] = copy.copy(data)
        for sub_handle in self._topics[name]['subscribers'].keys():
            self._topics[name]['subscribers'][sub_handle]['updated'] = True

    def subscribe(self, name):
        handle = uuid.uuid4().get_hex()
        self._topics[name]['subscribers'][handle] = {'updated': True}
        self._subscriber_handle[handle] = name
        return handle

    def unsubscribe(self, handle):
        name = self._subscriber_handle[handle]
        try:
            self._topics[name]['subscribers'].pop(handle, None)
        except KeyError:
            # okay if publisher already deleted topic
            pass
        self._subscriber_handle.pop(handle, None)
        return handle

    def updated(self, handle):
        name = self._subscriber_handle[handle]
        return self._topics[name]['subscribers'][handle]['updated']

    def copy(self, handle):
        name = self._subscriber_handle[handle]
        self._topics[name]['subscribers'][handle]['updated'] = False
        return copy.copy(self._topics[name]['data'])


class Subscription(object):

    def __init__(self, manager, topic_name):
        handle = manager.subscribe(topic_name)
        self.data = manager.copy(handle)
        self._manager = manager
        self._handle = handle

    def updated(self):
        return self._manager.updated(self._handle)

    def update(self):
        if self.updated():
            self.data = self._manager.copy(self._handle)

    def __del__(self):
        self._manager.unsubscribe(self._handle)


class Publication(object):

    def __init__(self, manager, topic_name, data):
        self._handle = manager.advertise(topic_name)
        self._manager = manager
        self.data = data
        self.publish()

    def publish(self):
        self._manager.publish(self._handle, self.data)

    def __del__(self):
        self._manager.unadvertise(self._handle)
