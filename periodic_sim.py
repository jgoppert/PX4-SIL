import scipy.integrate
import numpy as np
import collections
import time


GPS_topic = collections.namedtuple(
    'GPS_topic',
    ['time_stamp', 'lat', 'lon', 'alt'])


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


class Logger(PeriodicProcess):

    def __init__(self, period, tf, data):
        super(Logger, self).__init__(period)
        self.data = data
        self.log = {}
        self.tf = tf
        self.n_t = np.floor(tf/period)

    def initialize(self, t):
        n_t = self.n_t
        super(Logger, self).initialize(t)
        for key in self.data.keys():
            self.log[key] = np.zeros((n_t, len(np.array([self.data[key]]))))
        self.log['t'] = np.zeros((n_t, 1))
        self.count = -1
        self.run(t)

    def run(self, t):
        count = self.count + 1
        if count >= self.n_t:
            return
        log = self.log
        log['t'][count] = t
        for key in self.data.keys():
            log[key][count] = self.data[key]
        self.count = count

    def finalize(self):
        for key in self.log.keys():
            self.log[key] = self.log[key][:self.count, :]

    def get_log_as_namedtuple(self):
        return collections.namedtuple(
            'log', self.log.keys())(**(self.log))
