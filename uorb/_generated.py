# Do not edit!
# automatically generated using uorb/generate.py
# edit uorb/uorb.xml

generation_timestamp = '2014-09-03 23:24:41'

import numpy as np



class Topic_actuator_armed(object):
    """
    Controls if actuator output is live.

    Input:
        timestamp : Microseconds since system boot.
        armed : Set to true if system is armed.
        ready_to_arm : Set to true if system is ready to be armed.
        lockdown : Set to true if actuators are forcibly disabled (due to emergency or HIL).
        force_failsafe : Set to true if actuators are forced to the failsafe position.
    """

    def __init__(self, timestamp, armed, ready_to_arm, lockdown, force_failsafe):
        self.timestamp = timestamp
        self.armed = armed
        self.ready_to_arm = ready_to_arm
        self.lockdown = lockdown
        self.force_failsafe = force_failsafe

    @property
    def _fields(self):
        return ('timestamp', 'armed', 'ready_to_arm', 'lockdown', 'force_failsafe')

    @property
    def _values(self):
        return (self.timestamp, self.armed, self.ready_to_arm, self.lockdown, self.force_failsafe)

    def __str__(self):
        return str(self._fields) + ' ' + str(self._values)

    __repr__ = __str__


class Topic_actuator_controls(object):
    """
    Values publicshed to these topics are
    the outputs of the vehicle control system, and are expected to be mixed
    and used to drive the actuators (serovs, speed controllers, etc.) that
    operate the vehicle.

    Each topic can be published by a single controller.

    Input:
        timestamp : Microseconds since system boot.
        control : The control values in natural units.
    """

    def __init__(self, timestamp, control):
        self.timestamp = timestamp
        self.control = control

    @property
    def _fields(self):
        return ('timestamp', 'control')

    @property
    def _values(self):
        return (self.timestamp, self.control)

    def __str__(self):
        return str(self._fields) + ' ' + str(self._values)

    __repr__ = __str__


class Topic_actuator_outputs(object):
    """
    Actuator output values.
    Values published to these topics are the outputs of the control mixing system as 
    sent to the actuators (servos, motors, etc.) that operate the vehicle.

    Each topic can be published by a single output driver.

    Input:
        timestamp : Microseconds since system boot.
        output : Output data in natural output units.
        noutputs : Number of valid outputs.
    """

    def __init__(self, timestamp, output, noutputs):
        self.timestamp = timestamp
        self.output = output
        self.noutputs = noutputs

    @property
    def _fields(self):
        return ('timestamp', 'output', 'noutputs')

    @property
    def _values(self):
        return (self.timestamp, self.output, self.noutputs)

    def __str__(self):
        return str(self._fields) + ' ' + str(self._values)

    __repr__ = __str__


class Topic_airspeed(object):
    """
    Definition of airspeed topic.

    Input:
        timestamp : Microseconds since system boot.
        indicated_airspeed_m_s : Indicated airspeed in meters per second, -1 if unknown.
        air_temperature_celsius : Air temperature in degrees celsius, -1000 if unknown.
    """

    def __init__(self, timestamp, indicated_airspeed_m_s, air_temperature_celsius):
        self.timestamp = timestamp
        self.indicated_airspeed_m_s = indicated_airspeed_m_s
        self.air_temperature_celsius = air_temperature_celsius

    @property
    def _fields(self):
        return ('timestamp', 'indicated_airspeed_m_s', 'air_temperature_celsius')

    @property
    def _values(self):
        return (self.timestamp, self.indicated_airspeed_m_s, self.air_temperature_celsius)

    def __str__(self):
        return str(self._fields) + ' ' + str(self._values)

    __repr__ = __str__


class Topic_battery_status(object):
    """
    Battery voltages and status.

    Input:
        timestamp : Microseconds since system boot.
        voltage_v : Battery voltage in volts, 0 if unknown.
        voltage_filtered_v : Battery voltage filtered in volts, 0 if unknown.
        current_a : Battery current in amperes, -1 if unknown.
        discharged_mah : Discharged amount in mAh.
    """

    def __init__(self, timestamp, voltage_v, voltage_filtered_v, current_a, discharged_mah):
        self.timestamp = timestamp
        self.voltage_v = voltage_v
        self.voltage_filtered_v = voltage_filtered_v
        self.current_a = current_a
        self.discharged_mah = discharged_mah

    @property
    def _fields(self):
        return ('timestamp', 'voltage_v', 'voltage_filtered_v', 'current_a', 'discharged_mah')

    @property
    def _values(self):
        return (self.timestamp, self.voltage_v, self.voltage_filtered_v, self.current_a, self.discharged_mah)

    def __str__(self):
        return str(self._fields) + ' ' + str(self._values)

    __repr__ = __str__


class Topic_debug_key_value(object):
    """
    Actual data, this is specific to the type of data which is stored in
    this struct line containing L0GME will be added by the Python logging
    code generator to the logged dataset.

    Input:
        timestamp_ms : Milliseconds since system boot.
        key :  max 10 charasters as key.
        value : The value to send as debug output.
    """

    def __init__(self, timestamp_ms, key, value):
        self.timestamp_ms = timestamp_ms
        self.key = key
        self.value = value

    @property
    def _fields(self):
        return ('timestamp_ms', 'key', 'value')

    @property
    def _values(self):
        return (self.timestamp_ms, self.key, self.value)

    def __str__(self):
        return str(self._fields) + ' ' + str(self._values)

    __repr__ = __str__


class Topic_differential_pressure(object):
    """
    Differential pressure packet.

    Input:
        timestamp : Microseconds since system boot, needed to integrate.
        error_count : Number of errors detected by driver.
        differential_pressure_raw_pa :  Raw differential pressure reading (may be negative).
        differential_pressure_filtered_pa :  Low pass filtered differential pressure reading (may be negative).
        max_differential_pressure_pa : The value to send as debug output.
        temperature : Temperature provided by sensor, celsius, -1000.0f if unknown.
    """

    def __init__(self, timestamp, error_count, differential_pressure_raw_pa, differential_pressure_filtered_pa, max_differential_pressure_pa, temperature):
        self.timestamp = timestamp
        self.error_count = error_count
        self.differential_pressure_raw_pa = differential_pressure_raw_pa
        self.differential_pressure_filtered_pa = differential_pressure_filtered_pa
        self.max_differential_pressure_pa = max_differential_pressure_pa
        self.temperature = temperature

    @property
    def _fields(self):
        return ('timestamp', 'error_count', 'differential_pressure_raw_pa', 'differential_pressure_filtered_pa', 'max_differential_pressure_pa', 'temperature')

    @property
    def _values(self):
        return (self.timestamp, self.error_count, self.differential_pressure_raw_pa, self.differential_pressure_filtered_pa, self.max_differential_pressure_pa, self.temperature)

    def __str__(self):
        return str(self._fields) + ' ' + str(self._values)

    __repr__ = __str__


class Topic_encoders(object):
    """
    Encoders topic.

    Input:
        timestamp : Microseconds since system boot.
        counts : Counts of encoder.
        velocity : Counts of encoder/second.
    """

    def __init__(self, timestamp, counts, velocity):
        self.timestamp = timestamp
        self.counts = counts
        self.velocity = velocity

    @property
    def _fields(self):
        return ('timestamp', 'counts', 'velocity')

    @property
    def _values(self):
        return (self.timestamp, self.counts, self.velocity)

    def __str__(self):
        return str(self._fields) + ' ' + str(self._values)

    __repr__ = __str__


class Topic_esc_status(object):
    """
    Electronic speed controller status.

    Input:
        counter : Incremented by writing thread everytime new data stored.
        timestamp : Microseconds since system boot.
        esc_count : Number of connected ESCs.
        esc : ESC data structure.
    """

    def __init__(self, counter, timestamp, esc_count, esc):
        self.counter = counter
        self.timestamp = timestamp
        self.esc_count = esc_count
        self.esc = esc

    @property
    def _fields(self):
        return ('counter', 'timestamp', 'esc_count', 'esc')

    @property
    def _values(self):
        return (self.counter, self.timestamp, self.esc_count, self.esc)

    def __str__(self):
        return str(self._fields) + ' ' + str(self._values)

    __repr__ = __str__


class Topic_estimator_status(object):
    """
    Estimator status report. This is a generic report 
    struct which allows any of the onboard estimators to wrie the interal
    state to the system log.

    Input:
        timestamp : Microseconds since system boot.
        states : Internal filter states.
        n_states : Number of filter states used.
        nan_flags : Bitmask to indicate NaN states.
        health_flags : Bitmask to indicate sensor health (vel, pos, hgt).
        timeout_flags : Bitmask to indicate timeout (vel, pos, hgt).
    """

    def __init__(self, timestamp, states, n_states, nan_flags, health_flags, timeout_flags):
        self.timestamp = timestamp
        self.states = states
        self.n_states = n_states
        self.nan_flags = nan_flags
        self.health_flags = health_flags
        self.timeout_flags = timeout_flags

    @property
    def _fields(self):
        return ('timestamp', 'states', 'n_states', 'nan_flags', 'health_flags', 'timeout_flags')

    @property
    def _values(self):
        return (self.timestamp, self.states, self.n_states, self.nan_flags, self.health_flags, self.timeout_flags)

    def __str__(self):
        return str(self._fields) + ' ' + str(self._values)

    __repr__ = __str__


class Topic_fence(object):
    """
    List of fence vertices.

    Input:
        count : Number of actual vertices.
        vertices : Fence vertices.
    """

    def __init__(self, count, vertices):
        self.count = count
        self.vertices = vertices

    @property
    def _fields(self):
        return ('count', 'vertices')

    @property
    def _values(self):
        return (self.count, self.vertices)

    def __str__(self):
        return str(self._fields) + ' ' + str(self._values)

    __repr__ = __str__


class Topic_filtered_bottom_flow(object):
    """
    Filtered bottom optical flow in bodyframe.

    Input:
        timestamp : Microseconds since system boot.
        sumx : Integrated bodyframe x flow in meteres.
        sumy : Integrated bodyframe x flow in meteres.
        vx : Flow bodyframe x speed m/s.
        vy : Flow bodyframe y speed m/s.
    """

    def __init__(self, timestamp, sumx, sumy, vx, vy):
        self.timestamp = timestamp
        self.sumx = sumx
        self.sumy = sumy
        self.vx = vx
        self.vy = vy

    @property
    def _fields(self):
        return ('timestamp', 'sumx', 'sumy', 'vx', 'vy')

    @property
    def _values(self):
        return (self.timestamp, self.sumx, self.sumy, self.vx, self.vy)

    def __str__(self):
        return str(self._fields) + ' ' + str(self._values)

    __repr__ = __str__


class Topic_home_position(object):
    """
    GPS home position in WGS84 coordinates.

    Input:
        timestamp : Microseconds since system boot.
        lat : Latitude in degrees.
        lon : Longitude in degrees.
        alt : Altitude in meteres.
        x : x coordinate in meters.
        y : y coordinate in meters.
        z : z coordinate in meters.
    """

    def __init__(self, timestamp, lat, lon, alt, x, y, z):
        self.timestamp = timestamp
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.x = x
        self.y = y
        self.z = z

    @property
    def _fields(self):
        return ('timestamp', 'lat', 'lon', 'alt', 'x', 'y', 'z')

    @property
    def _values(self):
        return (self.timestamp, self.lat, self.lon, self.alt, self.x, self.y, self.z)

    def __str__(self):
        return str(self._fields) + ' ' + str(self._values)

    __repr__ = __str__


class Topic_manual_control_setpoint(object):
    """
    None

    Input:
        timestamp : Microseconds since system boot.
    """

    def __init__(self, timestamp):
        self.timestamp = timestamp

    @property
    def _fields(self):
        return ('timestamp')

    @property
    def _values(self):
        return (self.timestamp)

    def __str__(self):
        return str(self._fields) + ' ' + str(self._values)

    __repr__ = __str__


class Topic_mission(object):
    """
    None

    Input:
        timestamp : Microseconds since system boot.
    """

    def __init__(self, timestamp):
        self.timestamp = timestamp

    @property
    def _fields(self):
        return ('timestamp')

    @property
    def _values(self):
        return (self.timestamp)

    def __str__(self):
        return str(self._fields) + ' ' + str(self._values)

    __repr__ = __str__


class Topic_mission_result(object):
    """
    None

    Input:
        timestamp : Microseconds since system boot.
    """

    def __init__(self, timestamp):
        self.timestamp = timestamp

    @property
    def _fields(self):
        return ('timestamp')

    @property
    def _values(self):
        return (self.timestamp)

    def __str__(self):
        return str(self._fields) + ' ' + str(self._values)

    __repr__ = __str__


class Topic_navigation_capabilities(object):
    """
    None

    Input:
        timestamp : Microseconds since system boot.
    """

    def __init__(self, timestamp):
        self.timestamp = timestamp

    @property
    def _fields(self):
        return ('timestamp')

    @property
    def _values(self):
        return (self.timestamp)

    def __str__(self):
        return str(self._fields) + ' ' + str(self._values)

    __repr__ = __str__


class Topic_offboard_control_setpoint(object):
    """
    None

    Input:
        timestamp : Microseconds since system boot.
    """

    def __init__(self, timestamp):
        self.timestamp = timestamp

    @property
    def _fields(self):
        return ('timestamp')

    @property
    def _values(self):
        return (self.timestamp)

    def __str__(self):
        return str(self._fields) + ' ' + str(self._values)

    __repr__ = __str__


class Topic_omnidirectional_flow(object):
    """
    None

    Input:
        timestamp : Microseconds since system boot.
    """

    def __init__(self, timestamp):
        self.timestamp = timestamp

    @property
    def _fields(self):
        return ('timestamp')

    @property
    def _values(self):
        return (self.timestamp)

    def __str__(self):
        return str(self._fields) + ' ' + str(self._values)

    __repr__ = __str__


class Topic_optical_flow(object):
    """
    None

    Input:
        timestamp : Microseconds since system boot.
    """

    def __init__(self, timestamp):
        self.timestamp = timestamp

    @property
    def _fields(self):
        return ('timestamp')

    @property
    def _values(self):
        return (self.timestamp)

    def __str__(self):
        return str(self._fields) + ' ' + str(self._values)

    __repr__ = __str__


class Topic_parameter_update(object):
    """
    None

    Input:
        timestamp : Microseconds since system boot.
    """

    def __init__(self, timestamp):
        self.timestamp = timestamp

    @property
    def _fields(self):
        return ('timestamp')

    @property
    def _values(self):
        return (self.timestamp)

    def __str__(self):
        return str(self._fields) + ' ' + str(self._values)

    __repr__ = __str__


class Topic_positional_setpoint_triplet(object):
    """
    None

    Input:
        timestamp : Microseconds since system boot.
    """

    def __init__(self, timestamp):
        self.timestamp = timestamp

    @property
    def _fields(self):
        return ('timestamp')

    @property
    def _values(self):
        return (self.timestamp)

    def __str__(self):
        return str(self._fields) + ' ' + str(self._values)

    __repr__ = __str__


class Topic_rc_channels(object):
    """
    None

    Input:
        timestamp : Microseconds since system boot.
    """

    def __init__(self, timestamp):
        self.timestamp = timestamp

    @property
    def _fields(self):
        return ('timestamp')

    @property
    def _values(self):
        return (self.timestamp)

    def __str__(self):
        return str(self._fields) + ' ' + str(self._values)

    __repr__ = __str__


class Topic_safety(object):
    """
    None

    Input:
        timestamp : Microseconds since system boot.
    """

    def __init__(self, timestamp):
        self.timestamp = timestamp

    @property
    def _fields(self):
        return ('timestamp')

    @property
    def _values(self):
        return (self.timestamp)

    def __str__(self):
        return str(self._fields) + ' ' + str(self._values)

    __repr__ = __str__


class Topic_satellite_info(object):
    """
    None

    Input:
        timestamp : Microseconds since system boot.
    """

    def __init__(self, timestamp):
        self.timestamp = timestamp

    @property
    def _fields(self):
        return ('timestamp')

    @property
    def _values(self):
        return (self.timestamp)

    def __str__(self):
        return str(self._fields) + ' ' + str(self._values)

    __repr__ = __str__


class Topic_sensor_combined(object):
    """
    None

    Input:
        timestamp : Microseconds since system boot.
    """

    def __init__(self, timestamp):
        self.timestamp = timestamp

    @property
    def _fields(self):
        return ('timestamp')

    @property
    def _values(self):
        return (self.timestamp)

    def __str__(self):
        return str(self._fields) + ' ' + str(self._values)

    __repr__ = __str__


class Topic_servorail_status(object):
    """
    None

    Input:
        timestamp : Microseconds since system boot.
    """

    def __init__(self, timestamp):
        self.timestamp = timestamp

    @property
    def _fields(self):
        return ('timestamp')

    @property
    def _values(self):
        return (self.timestamp)

    def __str__(self):
        return str(self._fields) + ' ' + str(self._values)

    __repr__ = __str__


class Topic_subsystem_info(object):
    """
    None

    Input:
        timestamp : Microseconds since system boot.
    """

    def __init__(self, timestamp):
        self.timestamp = timestamp

    @property
    def _fields(self):
        return ('timestamp')

    @property
    def _values(self):
        return (self.timestamp)

    def __str__(self):
        return str(self._fields) + ' ' + str(self._values)

    __repr__ = __str__


class Topic_system_power(object):
    """
    None

    Input:
        timestamp : Microseconds since system boot.
    """

    def __init__(self, timestamp):
        self.timestamp = timestamp

    @property
    def _fields(self):
        return ('timestamp')

    @property
    def _values(self):
        return (self.timestamp)

    def __str__(self):
        return str(self._fields) + ' ' + str(self._values)

    __repr__ = __str__


class Topic_tecs_status(object):
    """
    None

    Input:
        timestamp : Microseconds since system boot.
    """

    def __init__(self, timestamp):
        self.timestamp = timestamp

    @property
    def _fields(self):
        return ('timestamp')

    @property
    def _values(self):
        return (self.timestamp)

    def __str__(self):
        return str(self._fields) + ' ' + str(self._values)

    __repr__ = __str__


class Topic_telemetry_status(object):
    """
    None

    Input:
        timestamp : Microseconds since system boot.
    """

    def __init__(self, timestamp):
        self.timestamp = timestamp

    @property
    def _fields(self):
        return ('timestamp')

    @property
    def _values(self):
        return (self.timestamp)

    def __str__(self):
        return str(self._fields) + ' ' + str(self._values)

    __repr__ = __str__


class Topic_vehicle_attitude(object):
    """
    Attitude in NED body frame in SI units. This is similar to mavlink message ATTITUDE but for onboard use.
    @see http://en.wikipedia.org/wiki/International_System_of_Units

    Input:
        timestamp : Microseconds since system boot.
        roll : Roll angle (rad, Tait-Bryan, NED)
        pitch : Pitch angle (rad, Tait-Bryan, NED)
        yaw : Yaw angle (rad, Tait-Bryan, NED)
        rollspeed : Roll anglular speed (rad/s, Tait-Bryan, NED)
        pitchspeed : Pitch angular speed (rad/s, Tait-Bryan, NED)
        yawspeed : Yaw angular speed (rad/s, Tait-Bryan, NED)
        rollacc : Roll anglular acceleration (rad/s^2, Tait-Bryan, NED)
        pitchacc : Pitch angular acceleration (rad/s^2, Tait-Bryan, NED)
        yawacc : Yaw angular acceleration (rad/s^2, Tait-Bryan, NED)
        rate_offsets : Offsets of the body angular rates from zero
        R : Rotation matrix body to NED
        q : Quaternion body to NED
        g_comp : Compensated gravity vector
        R_valid : Rotation matrix valid
        q_valid : Quaternion valid
    """

    def __init__(self, timestamp, roll, pitch, yaw, rollspeed, pitchspeed, yawspeed, rollacc, pitchacc, yawacc, rate_offsets, R, q, g_comp, R_valid, q_valid):
        self.timestamp = timestamp
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.rollspeed = rollspeed
        self.pitchspeed = pitchspeed
        self.yawspeed = yawspeed
        self.rollacc = rollacc
        self.pitchacc = pitchacc
        self.yawacc = yawacc
        self.rate_offsets = rate_offsets
        self.R = R
        self.q = q
        self.g_comp = g_comp
        self.R_valid = R_valid
        self.q_valid = q_valid

    @property
    def _fields(self):
        return ('timestamp', 'roll', 'pitch', 'yaw', 'rollspeed', 'pitchspeed', 'yawspeed', 'rollacc', 'pitchacc', 'yawacc', 'rate_offsets', 'R', 'q', 'g_comp', 'R_valid', 'q_valid')

    @property
    def _values(self):
        return (self.timestamp, self.roll, self.pitch, self.yaw, self.rollspeed, self.pitchspeed, self.yawspeed, self.rollacc, self.pitchacc, self.yawacc, self.rate_offsets, self.R, self.q, self.g_comp, self.R_valid, self.q_valid)

    def __str__(self):
        return str(self._fields) + ' ' + str(self._values)

    __repr__ = __str__


class Topic_vehicle_attitude_setpoint(object):
    """
    Vehicle attitude setpoint.

    Input:
        timestamp : Microseconds since system boot.
        roll_body : Roll angle (rad, Tait-Bryan, NED)
        pitch_body : Pitch angle (rad, Tait-Bryan, NED)
        yaw_body : Yaw angle (rad, Tait-Bryan, NED)
        R_body : Rotation matrix of setpoint, body to NED
        R_valid : Set to true if R_body is valid
        q_d : Desired quaternion for quaternion control
        q_d_valid : Set to true if q_d is valid
        q_e : Attitude error quaternion
        q_e_valid : Set to true if q_e is valid
        thrust : Thrust in Newton the power system should generate
        roll_reset_integral : Reset roll integrator (navigation logic change)
        pitch_reset_integral : Reset pitch integrator (navigation logic change)
        yaw_reset_integral : Reset yaw integrator (navigation logic change)
    """

    def __init__(self, timestamp, roll_body, pitch_body, yaw_body, R_body, R_valid, q_d, q_d_valid, q_e, q_e_valid, thrust, roll_reset_integral, pitch_reset_integral, yaw_reset_integral):
        self.timestamp = timestamp
        self.roll_body = roll_body
        self.pitch_body = pitch_body
        self.yaw_body = yaw_body
        self.R_body = R_body
        self.R_valid = R_valid
        self.q_d = q_d
        self.q_d_valid = q_d_valid
        self.q_e = q_e
        self.q_e_valid = q_e_valid
        self.thrust = thrust
        self.roll_reset_integral = roll_reset_integral
        self.pitch_reset_integral = pitch_reset_integral
        self.yaw_reset_integral = yaw_reset_integral

    @property
    def _fields(self):
        return ('timestamp', 'roll_body', 'pitch_body', 'yaw_body', 'R_body', 'R_valid', 'q_d', 'q_d_valid', 'q_e', 'q_e_valid', 'thrust', 'roll_reset_integral', 'pitch_reset_integral', 'yaw_reset_integral')

    @property
    def _values(self):
        return (self.timestamp, self.roll_body, self.pitch_body, self.yaw_body, self.R_body, self.R_valid, self.q_d, self.q_d_valid, self.q_e, self.q_e_valid, self.thrust, self.roll_reset_integral, self.pitch_reset_integral, self.yaw_reset_integral)

    def __str__(self):
        return str(self._fields) + ' ' + str(self._values)

    __repr__ = __str__


class Topic_vehicle_bodyframe_speed_setpoint(object):
    """
    None

    Input:
        timestamp : Microseconds since system boot.
    """

    def __init__(self, timestamp):
        self.timestamp = timestamp

    @property
    def _fields(self):
        return ('timestamp')

    @property
    def _values(self):
        return (self.timestamp)

    def __str__(self):
        return str(self._fields) + ' ' + str(self._values)

    __repr__ = __str__


class Topic_vehicle_command(object):
    """
    None

    Input:
        timestamp : Microseconds since system boot.
    """

    def __init__(self, timestamp):
        self.timestamp = timestamp

    @property
    def _fields(self):
        return ('timestamp')

    @property
    def _values(self):
        return (self.timestamp)

    def __str__(self):
        return str(self._fields) + ' ' + str(self._values)

    __repr__ = __str__


class Topic_vehicle_control_mode(object):
    """
    None

    Input:
        timestamp : Microseconds since system boot.
    """

    def __init__(self, timestamp):
        self.timestamp = timestamp

    @property
    def _fields(self):
        return ('timestamp')

    @property
    def _values(self):
        return (self.timestamp)

    def __str__(self):
        return str(self._fields) + ' ' + str(self._values)

    __repr__ = __str__


class Topic_vehicle_force_setpoint(object):
    """
    None

    Input:
        timestamp : Microseconds since system boot.
    """

    def __init__(self, timestamp):
        self.timestamp = timestamp

    @property
    def _fields(self):
        return ('timestamp')

    @property
    def _values(self):
        return (self.timestamp)

    def __str__(self):
        return str(self._fields) + ' ' + str(self._values)

    __repr__ = __str__


class Topic_vehicle_global_position(object):
    """
    The estimated vehicle global position.

    Input:
        timestamp : Microseconds since system boot.
        time_gps_usec : GPS timestamp in microseconds.
        lat : Latitude in degrees.
        lon : Longitude in degrees.
        alt : Altitude AMSL in meters.
        vel_n : Ground north velocity m/s.
        vel_e : Ground east velocity m/s.
        vel_d : Ground down velocity m/s.
        yaw : Yaw in radians -PI..+PI.
        eph : Standard devation of position estimate horizontally.
        epv : Standard devation of position estimate vertically.
    """

    def __init__(self, timestamp, time_gps_usec, lat, lon, alt, vel_n, vel_e, vel_d, yaw, eph, epv):
        self.timestamp = timestamp
        self.time_gps_usec = time_gps_usec
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.vel_n = vel_n
        self.vel_e = vel_e
        self.vel_d = vel_d
        self.yaw = yaw
        self.eph = eph
        self.epv = epv

    @property
    def _fields(self):
        return ('timestamp', 'time_gps_usec', 'lat', 'lon', 'alt', 'vel_n', 'vel_e', 'vel_d', 'yaw', 'eph', 'epv')

    @property
    def _values(self):
        return (self.timestamp, self.time_gps_usec, self.lat, self.lon, self.alt, self.vel_n, self.vel_e, self.vel_d, self.yaw, self.eph, self.epv)

    def __str__(self):
        return str(self._fields) + ' ' + str(self._values)

    __repr__ = __str__


class Topic_vehicle_global_velocity_setpoint(object):
    """
    None

    Input:
        timestamp : Microseconds since system boot.
    """

    def __init__(self, timestamp):
        self.timestamp = timestamp

    @property
    def _fields(self):
        return ('timestamp')

    @property
    def _values(self):
        return (self.timestamp)

    def __str__(self):
        return str(self._fields) + ' ' + str(self._values)

    __repr__ = __str__


class Topic_vehicle_gps_position(object):
    """
    None

    Input:
        timestamp : Microseconds since system boot.
    """

    def __init__(self, timestamp):
        self.timestamp = timestamp

    @property
    def _fields(self):
        return ('timestamp')

    @property
    def _values(self):
        return (self.timestamp)

    def __str__(self):
        return str(self._fields) + ' ' + str(self._values)

    __repr__ = __str__


class Topic_vehicle_local_position(object):
    """
    None

    Input:
        timestamp : Microseconds since system boot.
    """

    def __init__(self, timestamp):
        self.timestamp = timestamp

    @property
    def _fields(self):
        return ('timestamp')

    @property
    def _values(self):
        return (self.timestamp)

    def __str__(self):
        return str(self._fields) + ' ' + str(self._values)

    __repr__ = __str__


class Topic_vehicle_local_position_setpoint(object):
    """
    Local position in NED frame.

    Input:
        timestamp : Microseconds since system boot.
        x : position in x direction, meters, NED.
        y : position in y direction, meters, NED.
        z : position in z direction, meters, NED.
        yaw : heading in radians -PI..+PI.
    """

    def __init__(self, timestamp, x, y, z, yaw):
        self.timestamp = timestamp
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw

    @property
    def _fields(self):
        return ('timestamp', 'x', 'y', 'z', 'yaw')

    @property
    def _values(self):
        return (self.timestamp, self.x, self.y, self.z, self.yaw)

    def __str__(self):
        return str(self._fields) + ' ' + str(self._values)

    __repr__ = __str__


class Topic_vehicle_rates_setpoint(object):
    """
    The vehilce rates setpoint.

    Input:
        timestamp : Microseconds since system boot.
        roll : Roll anglular speed (rad/s, Tait-Bryan, NED)
        pitch : Pitch angular speed (rad/s, Tait-Bryan, NED)
        yaw : Yaw angular speed (rad/s, Tait-Bryan, NED)
        thrust : Thrust normalized to 0..1
    """

    def __init__(self, timestamp, roll, pitch, yaw, thrust):
        self.timestamp = timestamp
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.thrust = thrust

    @property
    def _fields(self):
        return ('timestamp', 'roll', 'pitch', 'yaw', 'thrust')

    @property
    def _values(self):
        return (self.timestamp, self.roll, self.pitch, self.yaw, self.thrust)

    def __str__(self):
        return str(self._fields) + ' ' + str(self._values)

    __repr__ = __str__


class Topic_vehicle_status(object):
    """
    None

    Input:
        timestamp : Microseconds since system boot.
    """

    def __init__(self, timestamp):
        self.timestamp = timestamp

    @property
    def _fields(self):
        return ('timestamp')

    @property
    def _values(self):
        return (self.timestamp)

    def __str__(self):
        return str(self._fields) + ' ' + str(self._values)

    __repr__ = __str__


class Topic_vehicle_vicon_position(object):
    """
    None

    Input:
        timestamp : Microseconds since system boot.
    """

    def __init__(self, timestamp):
        self.timestamp = timestamp

    @property
    def _fields(self):
        return ('timestamp')

    @property
    def _values(self):
        return (self.timestamp)

    def __str__(self):
        return str(self._fields) + ' ' + str(self._values)

    __repr__ = __str__


class Topic_vehicle_vicon_estimate(object):
    """
    None

    Input:
        timestamp : Microseconds since system boot.
    """

    def __init__(self, timestamp):
        self.timestamp = timestamp

    @property
    def _fields(self):
        return ('timestamp')

    @property
    def _values(self):
        return (self.timestamp)

    def __str__(self):
        return str(self._fields) + ' ' + str(self._values)

    __repr__ = __str__


class Topic_wind_estimate(object):
    """
    Wind estimate topic.

    Input:
        timestamp : Microseconds since system boot.
        windspeed_north : Wind component in north/ X direction.
        windspeed_east : Wind component in east/ Y direction.
        covariance_north : Uncertainty in north/ X direction. Set to zero (no uncertainty) if not estimated.
        covariance_east : Uncertainty in east/ Y direction. Set to zero (no uncertainty) if not estimated.
    """

    def __init__(self, timestamp, windspeed_north, windspeed_east, covariance_north, covariance_east):
        self.timestamp = timestamp
        self.windspeed_north = windspeed_north
        self.windspeed_east = windspeed_east
        self.covariance_north = covariance_north
        self.covariance_east = covariance_east

    @property
    def _fields(self):
        return ('timestamp', 'windspeed_north', 'windspeed_east', 'covariance_north', 'covariance_east')

    @property
    def _values(self):
        return (self.timestamp, self.windspeed_north, self.windspeed_east, self.covariance_north, self.covariance_east)

    def __str__(self):
        return str(self._fields) + ' ' + str(self._values)

    __repr__ = __str__
