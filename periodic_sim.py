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


class Estimator(PeriodicProcess):

    def __init__(self, period, uorb_manager):
        super(Estimator, self).__init__(period)
        self.uorb_manager = uorb_manager

    def initialize(self, t):
        super(Estimator, self).initialize(t)
        data = uorb.Topic_vehicle_global_position(
            timestamp=1e6*t, time_gps_usec=0, lat=0, lon=0, alt=0,
            vel_n=0, vel_e=0, vel_d=0, yaw=0, eph=0, epv=0)
        self.pos = uorb.Publication(
            self.uorb_manager,
            'vehicle_global_position', data)

    def run(self, t):
        self.pos.data.timestamp = 1e6*t
        self.pos.data.alt = np.sin(t)
        self.pos.publish()


class Logger(PeriodicProcess):

    def __init__(self, period, tf, uorb_manager):
        super(Logger, self).__init__(period)
        self.uorb_manager = uorb_manager
        self.topics = ['vehicle_global_position']
        self.subs = {}
        self.log = {}
        self.tf = tf
        self.n_t = int(self.tf/self.period) - 1

    def initialize(self, t):
        super(Logger, self).initialize(t)
        for topic in self.topics:
            self.subs[topic] = uorb.Subscription(self.uorb_manager, topic)
            d = self.subs[topic].data.__dict__
            self.log[topic] = {}
            for key in d:
                self.log[topic][key] = np.array([None]*self.n_t)
                self.log[topic][key][0] = d[key]
        self.count = 0

    def run(self, t):
        if self.count >= self.n_t - 1:
            return
        else:
            self.count = self.count + 1
        for topic in self.subs.keys():
            self.subs[topic].update()
            d = self.subs[topic].data.__dict__
            for field in d.keys():
                self.log[topic][field][self.count] = d[field]

    def finalize(self):
        for topic in self.subs.keys():
            d = self.subs[topic].data.__dict__
            for field in d.keys():
                # truncate unused data
                self.log[topic][field] = self.log[topic][field][:self.count]
                # try to convert to data type flow, won't work for structs
                try:
                    self.log[topic][field] = (
                        self.log[topic][field]).astype(float)
                except:
                    pass


class StateFeedbackController(PeriodicProcess):
    def __init__(self, period, K, data, xh_topic, u_topic):
        super(StateFeedbackController, self).__init__(period)
        self.data = data
        self.K = K
        self.xh_topic = xh_topic
        self.u_topic = u_topic

    def initialize(self, t):
        super(StateFeedbackController, self).initialize(t)
        self.data[self.u_topic] = 0
        self.data[self.u_topic + '_time_stamp'] = t

    def run(self, t):
        # print 'run controller'
        xh = self.data[self.xh_topic]
        u = -self.K.dot(xh)
        self.data[self.u_topic] = u
        self.data[self.u_topic + '_time_stamp'] = t


class ContinuousDynamics(PeriodicProcess):

    def __init__(self, period, x0, f_xdot, data):
        super(ContinuousDynamics, self).__init__(period)
        self.x0 = x0
        self.f_xdot = f_xdot
        self.ode = scipy.integrate.ode(self.f_xdot)
        self.data = data

    def initialize(self, t):
        super(ContinuousDynamics, self).initialize(t)
        self.ode.set_initial_value(self.x0, t)
        self.data['x'] = self.x0
        self.data['x_time_stamp'] = t

    def run(self, t):
        # print 'run dynamics'
        ode = self.ode
        if t == ode.t:
            return
        ode.set_f_params(self.data['u'])
        ode.integrate(t)
        if not ode.successful():
            raise ValueError('ode integration failed')

        # publish
        self.data['x'] = ode.y
        self.data['x_time_stamp'] = t

    @property
    def x(self):
        return self.ode.y


class Sensor(PeriodicProcess):

    def __init__(self, period, data, x_topic, y_topic):
        super(Sensor, self).__init__(period)
        self.data = data
        self.x_topic = x_topic
        self.y_topic = y_topic

    def initialize(self, t):
        super(Sensor, self).initialize(t)
        self.data[self.y_topic] = 0
        self.data['y_time_stamp'] = t

    def run(self, t):
        # print 'run sensor'
        self.time_stamp = t
        x = self.data['x']
        y = x

        # publish
        self.data['y'] = y
        self.data['y_time_stamp'] = t


class DiscreteKalmanFilter(PeriodicProcess):

    def __init__(self, period, xh0, data):
        super(DiscreteKalmanFilter, self).__init__(period)
        self.xh0 = xh0
        self.K = np.eye(1)
        self.H = np.eye(1)
        self.data = data

    def initialize(self, t):
        super(DiscreteKalmanFilter, self).initialize(t)
        self.xh = self.xh0
        self.y_time_stamp = None
        self.data['xh'] = self.xh
        self.data['xh_time_stamp'] = t

    def run(self, t):
        # print 'run estimator'

        # predict
        self.xh = self.xh

        # correct
        if self.data['y_time_stamp'] != self.y_time_stamp:
            xh = self.xh
            K = self.K
            H = self.H
            y = self.data['y']
            self.xh = xh + K.dot(y - H.dot(xh))

        # publish
        self.data['xh'] = self.xh
        self.data['xh_time_stamp'] = t
