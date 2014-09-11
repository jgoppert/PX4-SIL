# Do not edit!
# automatically generated using uorb/generate.py
# edit uorb/uorb.xml

generation_timestamp = '2014-09-11 15:51:24'


class Topic_actuator_armed(object):
    """
    Controls if actuator output is live.

    Parameters
    ----------
    timestamp :
        Microseconds since system boot.
    armed :
        Set to true if system is armed.
    ready_to_arm :
        Set to true if system is ready to be armed.
    lockdown :
        Set to true if actuators are forcibly disabled (due to emergency or HIL).
    force_failsafe :
        Set to true if actuators are forced to the failsafe position.
    """

    def __init__(self, timestamp=None,
            armed=None,
            ready_to_arm=None,
            lockdown=None,
            force_failsafe=None):
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

    Parameters
    ----------
    timestamp :
        Microseconds since system boot.
    control :
        The control values in natural units.
    """

    def __init__(self, timestamp=None,
            control=None):
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

    Parameters
    ----------
    timestamp :
        Microseconds since system boot.
    output :
        Output data in natural output units.
    noutputs :
        Number of valid outputs.
    """

    def __init__(self, timestamp=None,
            output=None,
            noutputs=None):
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

    Parameters
    ----------
    timestamp :
        Microseconds since system boot.
    indicated_airspeed_m_s :
        Indicated airspeed in meters per second, -1 if unknown.
    air_temperature_celsius :
        Air temperature in degrees celsius, -1000 if unknown.
    """

    def __init__(self, timestamp=None,
            indicated_airspeed_m_s=None,
            air_temperature_celsius=None):
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

    Parameters
    ----------
    timestamp :
        Microseconds since system boot.
    voltage_v :
        Battery voltage in volts, 0 if unknown.
    voltage_filtered_v :
        Battery voltage filtered in volts, 0 if unknown.
    current_a :
        Battery current in amperes, -1 if unknown.
    discharged_mah :
        Discharged amount in mAh.
    """

    def __init__(self, timestamp=None,
            voltage_v=None,
            voltage_filtered_v=None,
            current_a=None,
            discharged_mah=None):
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

    Parameters
    ----------
    timestamp_ms :
        Milliseconds since system boot.
    key :
         max 10 charasters as key.
    value :
        The value to send as debug output.
    """

    def __init__(self, timestamp_ms=None,
            key=None,
            value=None):
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

    Parameters
    ----------
    timestamp :
        Microseconds since system boot, needed to integrate.
    error_count :
        Number of errors detected by driver.
    differential_pressure_raw_pa :
         Raw differential pressure reading (may be negative).
    differential_pressure_filtered_pa :
         Low pass filtered differential pressure reading (may be negative).
    max_differential_pressure_pa :
        The value to send as debug output.
    temperature :
        Temperature provided by sensor, celsius, -1000.0f if unknown.
    """

    def __init__(self, timestamp=None,
            error_count=None,
            differential_pressure_raw_pa=None,
            differential_pressure_filtered_pa=None,
            max_differential_pressure_pa=None,
            temperature=None):
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

    Parameters
    ----------
    timestamp :
        Microseconds since system boot.
    counts :
        Counts of encoder.
    velocity :
        Counts of encoder/second.
    """

    def __init__(self, timestamp=None,
            counts=None,
            velocity=None):
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

    Parameters
    ----------
    counter :
        Incremented by writing thread everytime new data stored.
    timestamp :
        Microseconds since system boot.
    esc_count :
        Number of connected ESCs.
    esc :
        ESC data structure.
    """

    def __init__(self, counter=None,
            timestamp=None,
            esc_count=None,
            esc=None):
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

    Parameters
    ----------
    timestamp :
        Microseconds since system boot.
    states :
        Internal filter states.
    n_states :
        Number of filter states used.
    nan_flags :
        Bitmask to indicate NaN states.
    health_flags :
        Bitmask to indicate sensor health (vel, pos, hgt).
    timeout_flags :
        Bitmask to indicate timeout (vel, pos, hgt).
    """

    def __init__(self, timestamp=None,
            states=None,
            n_states=None,
            nan_flags=None,
            health_flags=None,
            timeout_flags=None):
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

    Parameters
    ----------
    count :
        Number of actual vertices.
    vertices :
        Fence vertices.
    """

    def __init__(self, count=None,
            vertices=None):
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

    Parameters
    ----------
    timestamp :
        Microseconds since system boot.
    sumx :
        Integrated bodyframe x flow in meteres.
    sumy :
        Integrated bodyframe x flow in meteres.
    vx :
        Flow bodyframe x speed m/s.
    vy :
        Flow bodyframe y speed m/s.
    """

    def __init__(self, timestamp=None,
            sumx=None,
            sumy=None,
            vx=None,
            vy=None):
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

    Parameters
    ----------
    timestamp :
        Microseconds since system boot.
    lat :
        Latitude in degrees.
    lon :
        Longitude in degrees.
    alt :
        Altitude in meteres.
    x :
        x coordinate in meters.
    y :
        y coordinate in meters.
    z :
        z coordinate in meters.
    """

    def __init__(self, timestamp=None,
            lat=None,
            lon=None,
            alt=None,
            x=None,
            y=None,
            z=None):
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

    Parameters
    ----------
    timestamp :
        Microseconds since system boot.
    """

    def __init__(self, timestamp=None):
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

    Parameters
    ----------
    timestamp :
        Microseconds since system boot.
    """

    def __init__(self, timestamp=None):
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

    Parameters
    ----------
    timestamp :
        Microseconds since system boot.
    """

    def __init__(self, timestamp=None):
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

    Parameters
    ----------
    timestamp :
        Microseconds since system boot.
    """

    def __init__(self, timestamp=None):
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

    Parameters
    ----------
    timestamp :
        Microseconds since system boot.
    """

    def __init__(self, timestamp=None):
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

    Parameters
    ----------
    timestamp :
        Microseconds since system boot.
    """

    def __init__(self, timestamp=None):
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

    Parameters
    ----------
    timestamp :
        Microseconds since system boot.
    """

    def __init__(self, timestamp=None):
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

    Parameters
    ----------
    timestamp :
        Microseconds since system boot.
    """

    def __init__(self, timestamp=None):
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

    Parameters
    ----------
    timestamp :
        Microseconds since system boot.
    """

    def __init__(self, timestamp=None):
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

    Parameters
    ----------
    timestamp :
        Microseconds since system boot.
    """

    def __init__(self, timestamp=None):
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

    Parameters
    ----------
    timestamp :
        Microseconds since system boot.
    """

    def __init__(self, timestamp=None):
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

    Parameters
    ----------
    timestamp :
        Microseconds since system boot.
    """

    def __init__(self, timestamp=None):
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
    Sensor readings in raw and SI-unit form.
    These values are read from the sensors. Raw values are in sensor-specific units,
    the scaled values are in SI-units, as visible from the ending of the variable
    or the comments. The use of the SI fields is in general advised, as these fields
    are scaled and offset-compensated where possible and do not change with board
    revisions and sensor updates.
        

    Parameters
    ----------
    timestamp :
        Microseconds since system boot.
    gyro_raw :
        Raw sensor values of angular velocity.
    gyro_rad_s :
        Angular velocity in radian per seconds.
    accelerometer_raw :
         Raw acceleration in NED body frame.
    accelerometer_m_s2 :
        Acceleration in NED body frame, in m/s^2.
    accelerometer_mode :
        Accelerometer measurement mode.
    accelerometer_range_m_s2 :
        Accelerometer measurement range in m/s^2.
    accelerometer_timestamp :
        Accelerometer timestamp.
    magnetometer_raw :
        Raw magnetic field in NED body frame.
    magnetometer_ga :
        Magnetic field in NED body frame, in Gauss.
    magnetometer_mode :
        Magnetometer measurement mode.
    magnetometer_range_ga :
         +/- measurement range in Gauss.
    magnetometer_cuttoff_freq_hz :
        Internal analog low pass frequency of sensor.
    magnetometer_timestamp :
        Magnetometer timestamp.
    gyro1_raw :
        Raw sensor values of angular velocity.
    gyro1_rad_s :
        Angular velocity in radian per seconds.
    gyro1_timestamp :
        Gyro timestamp.
    accelerometer1_raw :
        Raw acceleration in NED body frame.
    accelerometer1_m_s2 :
        Acceleration in NED body frame, in m/s^2.
    accelerometer1_timestamp :
        Accelerometer timestamp.
    magnetometer1_raw :
        Raw magnetic field in NED body frame
    magnetometer1_ga :
        Magnetic field in NED body frame, in Gauss
    magnetometer1_timestamp :
        Magnetometer timestamp.
    gyro2_raw :
        Raw sensor values of angular velocity.
    gyro2_rad_s :
        Angular velocity in radian per seconds.
    gyro2_timestamp :
        Gyro timestamp.
    accelerometer2_raw :
        Raw acceleration in NED body frame.
    accelerometer2_m_s2 :
        Acceleration in NED body frame, in m/s^2.
    accelerometer2_timestamp :
        Accelerometer timestamp.
    magnetometer2_raw :
        Raw magnetic field in NED body frame.
    magnetometer2_ga :
        Magnetic field in NED body frame, in Gauss.
    magnetometer2_timestamp :
        Magnetometer timestamp.
    baro_pres_mbar :
        Barometric pressure, already temp. comp.
    baro_alt_meter :
        Altitude, already temp. comp.
    baro_temp_celcius :
        Temperature in degrees celsius.
    adc_voltage_v :
        ADC voltages of ADC Chan 10/11/12/13 or -1.
    adc_mapping :
        Channel indices of each of these values.
    mcu_temp_celcius :
        Internal temperature measurement of MCU.
    baro_timestamp :
        Barometer timestamp.
    differential_pressure_pa :
        Airspeed sensor differential pressure.
    differential_pressure_timestamp :
        Last measurement timestamp.
    differential_pressure_filtered_pa :
        Low pass filtered airspeed sensor differential pressure reading.
    """

    def __init__(self, timestamp=None,
            gyro_raw=None,
            gyro_rad_s=None,
            accelerometer_raw=None,
            accelerometer_m_s2=None,
            accelerometer_mode=None,
            accelerometer_range_m_s2=None,
            accelerometer_timestamp=None,
            magnetometer_raw=None,
            magnetometer_ga=None,
            magnetometer_mode=None,
            magnetometer_range_ga=None,
            magnetometer_cuttoff_freq_hz=None,
            magnetometer_timestamp=None,
            gyro1_raw=None,
            gyro1_rad_s=None,
            gyro1_timestamp=None,
            accelerometer1_raw=None,
            accelerometer1_m_s2=None,
            accelerometer1_timestamp=None,
            magnetometer1_raw=None,
            magnetometer1_ga=None,
            magnetometer1_timestamp=None,
            gyro2_raw=None,
            gyro2_rad_s=None,
            gyro2_timestamp=None,
            accelerometer2_raw=None,
            accelerometer2_m_s2=None,
            accelerometer2_timestamp=None,
            magnetometer2_raw=None,
            magnetometer2_ga=None,
            magnetometer2_timestamp=None,
            baro_pres_mbar=None,
            baro_alt_meter=None,
            baro_temp_celcius=None,
            adc_voltage_v=None,
            adc_mapping=None,
            mcu_temp_celcius=None,
            baro_timestamp=None,
            differential_pressure_pa=None,
            differential_pressure_timestamp=None,
            differential_pressure_filtered_pa=None):
        self.timestamp = timestamp
        self.gyro_raw = gyro_raw
        self.gyro_rad_s = gyro_rad_s
        self.accelerometer_raw = accelerometer_raw
        self.accelerometer_m_s2 = accelerometer_m_s2
        self.accelerometer_mode = accelerometer_mode
        self.accelerometer_range_m_s2 = accelerometer_range_m_s2
        self.accelerometer_timestamp = accelerometer_timestamp
        self.magnetometer_raw = magnetometer_raw
        self.magnetometer_ga = magnetometer_ga
        self.magnetometer_mode = magnetometer_mode
        self.magnetometer_range_ga = magnetometer_range_ga
        self.magnetometer_cuttoff_freq_hz = magnetometer_cuttoff_freq_hz
        self.magnetometer_timestamp = magnetometer_timestamp
        self.gyro1_raw = gyro1_raw
        self.gyro1_rad_s = gyro1_rad_s
        self.gyro1_timestamp = gyro1_timestamp
        self.accelerometer1_raw = accelerometer1_raw
        self.accelerometer1_m_s2 = accelerometer1_m_s2
        self.accelerometer1_timestamp = accelerometer1_timestamp
        self.magnetometer1_raw = magnetometer1_raw
        self.magnetometer1_ga = magnetometer1_ga
        self.magnetometer1_timestamp = magnetometer1_timestamp
        self.gyro2_raw = gyro2_raw
        self.gyro2_rad_s = gyro2_rad_s
        self.gyro2_timestamp = gyro2_timestamp
        self.accelerometer2_raw = accelerometer2_raw
        self.accelerometer2_m_s2 = accelerometer2_m_s2
        self.accelerometer2_timestamp = accelerometer2_timestamp
        self.magnetometer2_raw = magnetometer2_raw
        self.magnetometer2_ga = magnetometer2_ga
        self.magnetometer2_timestamp = magnetometer2_timestamp
        self.baro_pres_mbar = baro_pres_mbar
        self.baro_alt_meter = baro_alt_meter
        self.baro_temp_celcius = baro_temp_celcius
        self.adc_voltage_v = adc_voltage_v
        self.adc_mapping = adc_mapping
        self.mcu_temp_celcius = mcu_temp_celcius
        self.baro_timestamp = baro_timestamp
        self.differential_pressure_pa = differential_pressure_pa
        self.differential_pressure_timestamp = differential_pressure_timestamp
        self.differential_pressure_filtered_pa = differential_pressure_filtered_pa

    @property
    def _fields(self):
        return ('timestamp', 'gyro_raw', 'gyro_rad_s', 'accelerometer_raw', 'accelerometer_m_s2', 'accelerometer_mode', 'accelerometer_range_m_s2', 'accelerometer_timestamp', 'magnetometer_raw', 'magnetometer_ga', 'magnetometer_mode', 'magnetometer_range_ga', 'magnetometer_cuttoff_freq_hz', 'magnetometer_timestamp', 'gyro1_raw', 'gyro1_rad_s', 'gyro1_timestamp', 'accelerometer1_raw', 'accelerometer1_m_s2', 'accelerometer1_timestamp', 'magnetometer1_raw', 'magnetometer1_ga', 'magnetometer1_timestamp', 'gyro2_raw', 'gyro2_rad_s', 'gyro2_timestamp', 'accelerometer2_raw', 'accelerometer2_m_s2', 'accelerometer2_timestamp', 'magnetometer2_raw', 'magnetometer2_ga', 'magnetometer2_timestamp', 'baro_pres_mbar', 'baro_alt_meter', 'baro_temp_celcius', 'adc_voltage_v', 'adc_mapping', 'mcu_temp_celcius', 'baro_timestamp', 'differential_pressure_pa', 'differential_pressure_timestamp', 'differential_pressure_filtered_pa')

    @property
    def _values(self):
        return (self.timestamp, self.gyro_raw, self.gyro_rad_s, self.accelerometer_raw, self.accelerometer_m_s2, self.accelerometer_mode, self.accelerometer_range_m_s2, self.accelerometer_timestamp, self.magnetometer_raw, self.magnetometer_ga, self.magnetometer_mode, self.magnetometer_range_ga, self.magnetometer_cuttoff_freq_hz, self.magnetometer_timestamp, self.gyro1_raw, self.gyro1_rad_s, self.gyro1_timestamp, self.accelerometer1_raw, self.accelerometer1_m_s2, self.accelerometer1_timestamp, self.magnetometer1_raw, self.magnetometer1_ga, self.magnetometer1_timestamp, self.gyro2_raw, self.gyro2_rad_s, self.gyro2_timestamp, self.accelerometer2_raw, self.accelerometer2_m_s2, self.accelerometer2_timestamp, self.magnetometer2_raw, self.magnetometer2_ga, self.magnetometer2_timestamp, self.baro_pres_mbar, self.baro_alt_meter, self.baro_temp_celcius, self.adc_voltage_v, self.adc_mapping, self.mcu_temp_celcius, self.baro_timestamp, self.differential_pressure_pa, self.differential_pressure_timestamp, self.differential_pressure_filtered_pa)

    def __str__(self):
        return str(self._fields) + ' ' + str(self._values)

    __repr__ = __str__


class Topic_servorail_status(object):
    """
    None

    Parameters
    ----------
    timestamp :
        Microseconds since system boot.
    """

    def __init__(self, timestamp=None):
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

    Parameters
    ----------
    timestamp :
        Microseconds since system boot.
    """

    def __init__(self, timestamp=None):
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

    Parameters
    ----------
    timestamp :
        Microseconds since system boot.
    """

    def __init__(self, timestamp=None):
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

    Parameters
    ----------
    timestamp :
        Microseconds since system boot.
    """

    def __init__(self, timestamp=None):
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

    Parameters
    ----------
    timestamp :
        Microseconds since system boot.
    """

    def __init__(self, timestamp=None):
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

    Parameters
    ----------
    timestamp :
        Microseconds since system boot.
    roll :
        Roll angle (rad, Tait-Bryan, NED)
    pitch :
        Pitch angle (rad, Tait-Bryan, NED)
    yaw :
        Yaw angle (rad, Tait-Bryan, NED)
    rollspeed :
        Roll anglular speed (rad/s, Tait-Bryan, NED)
    pitchspeed :
        Pitch angular speed (rad/s, Tait-Bryan, NED)
    yawspeed :
        Yaw angular speed (rad/s, Tait-Bryan, NED)
    rollacc :
        Roll anglular acceleration (rad/s^2, Tait-Bryan, NED)
    pitchacc :
        Pitch angular acceleration (rad/s^2, Tait-Bryan, NED)
    yawacc :
        Yaw angular acceleration (rad/s^2, Tait-Bryan, NED)
    rate_offsets :
        Offsets of the body angular rates from zero
    R :
        Rotation matrix body to NED
    q :
        Quaternion body to NED
    g_comp :
        Compensated gravity vector
    R_valid :
        Rotation matrix valid
    q_valid :
        Quaternion valid
    """

    def __init__(self, timestamp=None,
            roll=None,
            pitch=None,
            yaw=None,
            rollspeed=None,
            pitchspeed=None,
            yawspeed=None,
            rollacc=None,
            pitchacc=None,
            yawacc=None,
            rate_offsets=None,
            R=None,
            q=None,
            g_comp=None,
            R_valid=None,
            q_valid=None):
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

    Parameters
    ----------
    timestamp :
        Microseconds since system boot.
    roll_body :
        Roll angle (rad, Tait-Bryan, NED)
    pitch_body :
        Pitch angle (rad, Tait-Bryan, NED)
    yaw_body :
        Yaw angle (rad, Tait-Bryan, NED)
    R_body :
        Rotation matrix of setpoint, body to NED
    R_valid :
        Set to true if R_body is valid
    q_d :
        Desired quaternion for quaternion control
    q_d_valid :
        Set to true if q_d is valid
    q_e :
        Attitude error quaternion
    q_e_valid :
        Set to true if q_e is valid
    thrust :
        Thrust in Newton the power system should generate
    roll_reset_integral :
        Reset roll integrator (navigation logic change)
    pitch_reset_integral :
        Reset pitch integrator (navigation logic change)
    yaw_reset_integral :
        Reset yaw integrator (navigation logic change)
    """

    def __init__(self, timestamp=None,
            roll_body=None,
            pitch_body=None,
            yaw_body=None,
            R_body=None,
            R_valid=None,
            q_d=None,
            q_d_valid=None,
            q_e=None,
            q_e_valid=None,
            thrust=None,
            roll_reset_integral=None,
            pitch_reset_integral=None,
            yaw_reset_integral=None):
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

    Parameters
    ----------
    timestamp :
        Microseconds since system boot.
    """

    def __init__(self, timestamp=None):
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

    Parameters
    ----------
    timestamp :
        Microseconds since system boot.
    """

    def __init__(self, timestamp=None):
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

    Parameters
    ----------
    timestamp :
        Microseconds since system boot.
    """

    def __init__(self, timestamp=None):
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

    Parameters
    ----------
    timestamp :
        Microseconds since system boot.
    """

    def __init__(self, timestamp=None):
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

    Parameters
    ----------
    timestamp :
        Microseconds since system boot.
    time_gps_usec :
        GPS timestamp in microseconds.
    lat :
        Latitude in degrees.
    lon :
        Longitude in degrees.
    alt :
        Altitude AMSL in meters.
    vel_n :
        Ground north velocity m/s.
    vel_e :
        Ground east velocity m/s.
    vel_d :
        Ground down velocity m/s.
    yaw :
        Yaw in radians -PI..+PI.
    eph :
        Standard devation of position estimate horizontally.
    epv :
        Standard devation of position estimate vertically.
    """

    def __init__(self, timestamp=None,
            time_gps_usec=None,
            lat=None,
            lon=None,
            alt=None,
            vel_n=None,
            vel_e=None,
            vel_d=None,
            yaw=None,
            eph=None,
            epv=None):
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

    Parameters
    ----------
    timestamp :
        Microseconds since system boot.
    """

    def __init__(self, timestamp=None):
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
    GPS position in WGS84 coordinates.

    Parameters
    ----------
    timestamp_position :
        Timestamp for position information.
    lat :
        Latitude in 1E-7 degrees.
    lon :
        Longitude in 1E-7 degrees.
    alt :
        Altitude in 1E-3 meters (millimeters) above MSL.
    timestamp_variance :
        TODO.
    s_variance_m_s :
        speed accuracy estimate m/s.
    c_variance_rad :
        course accuracy estimate rad.
    fix_type :
        0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
    eph :
        GPS HDOP horizontal dilution of position in m.
    epv :
        GPS VDOP horizontal dilution of position in m.
    noise_per_ms :
        TODO.
    jamming_indicator :
        Indicated GPS is jammed.
    timestamp_velocity :
        Timestamp for velocity informations.
    vel_m_s :
        GPS ground speed (m/s).
    vel_n_m_s :
        GPS ground speed in m/s.
    vel_e_m_s :
        GPS ground speed in m/s.
    vel_d_m_s :
        GPS ground speed in m/s.
    cog_rad :
        Course over ground (NOT heading, but direction of movement) in rad, -PI..PI.
    vel_ned_valid :
        Flag to indicate if NED speed is valid.
    timestamp_time :
        Timestamp for time information.
    time_gps_usec :
        Timestamp (microseconds in GPS format), this is the timestamp which comes from the gps module.
    satellites_used :
        Number of satellites used.
    """

    def __init__(self, timestamp_position=None,
            lat=None,
            lon=None,
            alt=None,
            timestamp_variance=None,
            s_variance_m_s=None,
            c_variance_rad=None,
            fix_type=None,
            eph=None,
            epv=None,
            noise_per_ms=None,
            jamming_indicator=None,
            timestamp_velocity=None,
            vel_m_s=None,
            vel_n_m_s=None,
            vel_e_m_s=None,
            vel_d_m_s=None,
            cog_rad=None,
            vel_ned_valid=None,
            timestamp_time=None,
            time_gps_usec=None,
            satellites_used=None):
        self.timestamp_position = timestamp_position
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.timestamp_variance = timestamp_variance
        self.s_variance_m_s = s_variance_m_s
        self.c_variance_rad = c_variance_rad
        self.fix_type = fix_type
        self.eph = eph
        self.epv = epv
        self.noise_per_ms = noise_per_ms
        self.jamming_indicator = jamming_indicator
        self.timestamp_velocity = timestamp_velocity
        self.vel_m_s = vel_m_s
        self.vel_n_m_s = vel_n_m_s
        self.vel_e_m_s = vel_e_m_s
        self.vel_d_m_s = vel_d_m_s
        self.cog_rad = cog_rad
        self.vel_ned_valid = vel_ned_valid
        self.timestamp_time = timestamp_time
        self.time_gps_usec = time_gps_usec
        self.satellites_used = satellites_used

    @property
    def _fields(self):
        return ('timestamp_position', 'lat', 'lon', 'alt', 'timestamp_variance', 's_variance_m_s', 'c_variance_rad', 'fix_type', 'eph', 'epv', 'noise_per_ms', 'jamming_indicator', 'timestamp_velocity', 'vel_m_s', 'vel_n_m_s', 'vel_e_m_s', 'vel_d_m_s', 'cog_rad', 'vel_ned_valid', 'timestamp_time', 'time_gps_usec', 'satellites_used')

    @property
    def _values(self):
        return (self.timestamp_position, self.lat, self.lon, self.alt, self.timestamp_variance, self.s_variance_m_s, self.c_variance_rad, self.fix_type, self.eph, self.epv, self.noise_per_ms, self.jamming_indicator, self.timestamp_velocity, self.vel_m_s, self.vel_n_m_s, self.vel_e_m_s, self.vel_d_m_s, self.cog_rad, self.vel_ned_valid, self.timestamp_time, self.time_gps_usec, self.satellites_used)

    def __str__(self):
        return str(self._fields) + ' ' + str(self._values)

    __repr__ = __str__


class Topic_vehicle_local_position(object):
    """
    None

    Parameters
    ----------
    timestamp :
        Microseconds since system boot.
    """

    def __init__(self, timestamp=None):
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

    Parameters
    ----------
    timestamp :
        Microseconds since system boot.
    x :
        position in x direction, meters, NED.
    y :
        position in y direction, meters, NED.
    z :
        position in z direction, meters, NED.
    yaw :
        heading in radians -PI..+PI.
    """

    def __init__(self, timestamp=None,
            x=None,
            y=None,
            z=None,
            yaw=None):
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

    Parameters
    ----------
    timestamp :
        Microseconds since system boot.
    roll :
        Roll anglular speed (rad/s, Tait-Bryan, NED)
    pitch :
        Pitch angular speed (rad/s, Tait-Bryan, NED)
    yaw :
        Yaw angular speed (rad/s, Tait-Bryan, NED)
    thrust :
        Thrust normalized to 0..1
    """

    def __init__(self, timestamp=None,
            roll=None,
            pitch=None,
            yaw=None,
            thrust=None):
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

    Parameters
    ----------
    timestamp :
        Microseconds since system boot.
    """

    def __init__(self, timestamp=None):
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

    Parameters
    ----------
    timestamp :
        Microseconds since system boot.
    """

    def __init__(self, timestamp=None):
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

    Parameters
    ----------
    timestamp :
        Microseconds since system boot.
    """

    def __init__(self, timestamp=None):
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

    Parameters
    ----------
    timestamp :
        Microseconds since system boot.
    windspeed_north :
        Wind component in north/ X direction.
    windspeed_east :
        Wind component in east/ Y direction.
    covariance_north :
        Uncertainty in north/ X direction. Set to zero (no uncertainty) if not estimated.
    covariance_east :
        Uncertainty in east/ Y direction. Set to zero (no uncertainty) if not estimated.
    """

    def __init__(self, timestamp=None,
            windspeed_north=None,
            windspeed_east=None,
            covariance_north=None,
            covariance_east=None):
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


class Topic_sim_state(object):
    """
    Simulated aircraft state.

    Parameters
    ----------
    timestamp :
        Microseconds since system boot.
    roll :
        Roll angle (rad, Tait-Bryan, NED)
    pitch :
        Pitch angle (rad, Tait-Bryan, NED)
    yaw :
        Yaw angle (rad, Tait-Bryan, NED)
    rollspeed :
        Roll anglular speed (rad/s, Tait-Bryan, NED)
    pitchspeed :
        Pitch angular speed (rad/s, Tait-Bryan, NED)
    yawspeed :
        Yaw angular speed (rad/s, Tait-Bryan, NED)
    lat :
        Latitude in degrees.
    lon :
        Longitude in degrees.
    alt :
        Altitude in meteres.
    vx :
        Ground speed x(latitude) m/s.
    vy :
        Ground speed y(longitude) m/s.
    vz :
        Ground speed z(altitude) m.
    xacc :
        X acceleration m/s^2.
    yacc :
        X acceleration m/s^2.
    zacc :
        X acceleration m/s^2.
    """

    def __init__(self, timestamp=None,
            roll=None,
            pitch=None,
            yaw=None,
            rollspeed=None,
            pitchspeed=None,
            yawspeed=None,
            lat=None,
            lon=None,
            alt=None,
            vx=None,
            vy=None,
            vz=None,
            xacc=None,
            yacc=None,
            zacc=None):
        self.timestamp = timestamp
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.rollspeed = rollspeed
        self.pitchspeed = pitchspeed
        self.yawspeed = yawspeed
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.vx = vx
        self.vy = vy
        self.vz = vz
        self.xacc = xacc
        self.yacc = yacc
        self.zacc = zacc

    @property
    def _fields(self):
        return ('timestamp', 'roll', 'pitch', 'yaw', 'rollspeed', 'pitchspeed', 'yawspeed', 'lat', 'lon', 'alt', 'vx', 'vy', 'vz', 'xacc', 'yacc', 'zacc')

    @property
    def _values(self):
        return (self.timestamp, self.roll, self.pitch, self.yaw, self.rollspeed, self.pitchspeed, self.yawspeed, self.lat, self.lon, self.alt, self.vx, self.vy, self.vz, self.xacc, self.yacc, self.zacc)

    def __str__(self):
        return str(self._fields) + ' ' + str(self._values)

    __repr__ = __str__
