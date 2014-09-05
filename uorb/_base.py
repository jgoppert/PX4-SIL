import uuid
import copy

__all__ = ['Manager', 'Subscription', 'Publication']


class Manager(object):

    # TODO, should have manager initialize all topics to break the loop
    # of subscriber/publisher initialization

    def __init__(self, copy_type='pointer'):
        """
        Parameters
        ----------

        copy_type : {'pointer', 'shallow', 'deep'}
            (Pointer) copying will be fast and safe if all modules are in the
            same thread.
            (Shallow) copying will be safe in muultithreading
            as long as object are not compound objects but will be slower.
            (Deep) copying will
            be safe no matter what but will be very slow.
        """
        self._topics = {}
        self._publisher_handle = {}
        self._subscriber_handle = {}
        self.copy_type = copy_type

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

    def _copy_type(self, data):
        if self.copy_type == 'pointer':
            return data
        elif self.copy_type == 'shallow':
            return copy.copy(data)
        elif self.copy_type == 'deep':
            return copy.deepcopy(data)
        else:
            raise ValueError('unknown copy type')

    def unadvertise(self, handle):
        name = self._publisher_handle[handle]
        self._topics.pop(name, None)
        self._publisher_handle.pop(handle, None)

    def publish(self, handle, data):
        name = self._publisher_handle[handle]
        self._topics[name]['data'] = self._copy_type(data)
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
        return self._copy_type(self._topics[name]['data'])


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
