import scipy.integrate
import numpy as np
import collections
import time
import uorb
import copy


def dict_to_namedtuple(name, d):
    return collections.namedtuple(name, d.keys())(**d)


def nested_dict_to_namedtuple(d):
    d = copy.copy(d)
    for key in d.keys():
        d[key] = dict_to_namedtuple(key, d[key])
    d = dict_to_namedtuple('d', d)
    return d


class PeriodicProcess(object):

    def __init__(self, period):
        """
        Initialize data structure.
        """
        self.period = period

    def initialize(self, t):
        """
        Reset process.
        """
        self.time_stamp = t

    def update(self, t):
        """
        Run the process if enough time has elapsed
        """
        if t >= self.time_stamp + self.period:
            self.time_stamp = t
            self.run(t)

    def run(self, t):
        pass

    def finalize(self):
        pass


class PeriodicScheduler(object):

    def __init__(self, real_time, t0, tf, period):
        self.process_list = []
        self.real_time = real_time
        self.t0 = t0
        self.tf = tf
        self.period = period

    def run(self):
        real_time = self.real_time
        t0 = self.t0
        tf = self.tf
        period = self.period
        t = t0
        for process in self.process_list:
            if process.period < period:
                raise RuntimeWarning('process period < scheduler period')
        for process in self.process_list:
            process.initialize(t)

        while t + period < tf:
            if real_time:
                time_start = time.time()
            for process in self.process_list:
                process.update(t)
            if real_time:
                sleep_time = period - (time.time() - time_start)
                if sleep_time > 0:
                    time.sleep(sleep_time)
                else:
                    raise RuntimeWarning('failing to meet real time schedule')
            t = t + period
        for process in self.process_list:
            process.finalize()


class Logger(PeriodicProcess):

    def __init__(self, period, tf, topics, uorb_manager):
        super(Logger, self).__init__(period)
        self.uorb_manager = uorb_manager
        self.topics = topics
        self.subs = {}
        self.log = {}
        self.tf = tf
        self.n_t = int(self.tf/self.period) - 1
        self.t = np.zeros(self.n_t)

    def initialize(self, t):
        super(Logger, self).initialize(t)
        for topic in self.topics:
            self.subs[topic] = uorb.Subscription(self.uorb_manager, topic)
            d = self.subs[topic].data.__dict__
            self.log[topic] = {}
            for key in d:
                self.log[topic][key] = [None]*self.n_t
                self.log[topic][key][0] = d[key]
        self.count = 0
        self.t[0] = t

    def run(self, t):
        if self.count >= self.n_t - 1:
            return
        else:
            self.count = self.count + 1
        self.t[self.count] = t
        for topic in self.subs.keys():
            self.subs[topic].update()
            d = self.subs[topic].data.__dict__
            for field in d.keys():
                self.log[topic][field][self.count] = d[field]

    def finalize(self):
        self.log['log'] = {'t': self.t}
        for topic in self.subs.keys():
            d = self.subs[topic].data.__dict__
            for field in d.keys():
                # truncate unused data
                self.log[topic][field] = self.log[topic][field][:self.count+1]
                # try to convert to data type flow, won't work for structs
                try:
                    self.log[topic][field] = np.array(
                        self.log[topic][field]).astype(float)
                except:
                    pass
