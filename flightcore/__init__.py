from dronekit import *
from pymavlink import *
import json
import time
import os

__SIMLY__ = '0.0.0.0:4006'
__DRONE__ = '0.0.0.0:14551'

MODE_MANUAL = 'manual'
MODE_ACRO = 'acro'
MODE_RATTITUDE = 'rattitude'
MODE_ALT = 'althold'
MODE_POS = 'poshold'
MODE_GUIDED = 'guided'
MODE_TAKEOFF = 'takeoff'
MODE_RTL = 'rtl'
MODE_AUTO = 'auto'

#
# Luci Flight Core API
# Experimental, hasn't been tested. Use at your own risk.
#

class Fmu(object):
    def __init__(self, drone):
        self.master = drone
        self.vehicle = connect(drone, wait_ready = True)
        self.rgbled = self.vehicle.rgbled
        self.home = None

    def __translate(value, leftMin, leftMax, rightMin, rightMax):
        # Figure out how 'wide' each range is
        leftSpan = leftMax - leftMin
        rightSpan = rightMax - rightMin

        # Convert the left range into a 0-1 range (float)
        valueScaled = float(value - leftMin) / float(leftSpan)

        # Convert the 0-1 range into a value in the right range.
        return rightMin + (valueScaled * rightSpan)

    #
    # Returns vehicle object. See Dronekit docs for more information.
    #
    def vehicle(self):
        return self.vehicle

    def up(self, rate = 1.0):
        channel = self.getProperty('RC_MAP_THROTTLE')

        channelMin = self.getProperty('RC'+str(channel)+'_MIN')
        channelMax = self.getProperty('RC'+str(channel)+'_MAX')
        channelMid = self.getProperty('RC'+str(channel)+'_TRIM')

        maxRange = channelMax - channelMid
        minRange = channelMin - channelMid

        if rate > 10:
            rate = 10

        if rate < 0:
            rate = 0

        # ensure the UAV does not move when rate is 0
        if rate == 0.0:
            self.vehicle.channels.overrides[channel] = channelMid
            return

        self.vehicle.channels.overrides[channel] = self.__translate(rate, 0, 10, channelMid, channelMax)

    def down(self, rate = 1.0):
        channel = self.getProperty('RC_MAP_THROTTLE')

        channelMin = self.getProperty('RC'+str(channel)+'_MIN')
        channelMax = self.getProperty('RC'+str(channel)+'_MAX')
        channelMid = self.getProperty('RC'+str(channel)+'_TRIM')

        maxRange = channelMax - channelMid
        minRange = channelMin - channelMid

        if rate > 10:
            rate = 10

        if rate < 0:
            rate = 0

        # ensure the UAV does not move when rate is 0
        if rate == 0.0:
            self.vehicle.channels.overrides[channel] = channelMid
            return

        rate = -rate

        self.vehicle.channels.overrides[channel] = self.__translate(rate, -10, 0, channelMin, channelMid)

    def left(self, rate = 1.0):
        channel = self.getProperty('RC_MAP_ROLL')

        channelMin = self.getProperty('RC'+str(channel)+'_MIN')
        channelMax = self.getProperty('RC'+str(channel)+'_MAX')
        channelMid = self.getProperty('RC'+str(channel)+'_TRIM')

        maxRange = channelMax - channelMid
        minRange = channelMin - channelMid

        if rate > 10:
            rate = 10

        if rate < 0:
            rate = 0

        # ensure the UAV does not move when rate is 0
        if rate == 0.0:
            self.vehicle.channels.overrides[channel] = channelMid
            return

        self.vehicle.channels.overrides[channel] = self.__translate(rate, 0, 10, channelMid, channelMax)

    def right(self, rate = 1.0, dist):
        channel = self.getProperty('RC_MAP_ROLL')

        channelMin = self.getProperty('RC'+str(channel)+'_MIN')
        channelMax = self.getProperty('RC'+str(channel)+'_MAX')
        channelMid = self.getProperty('RC'+str(channel)+'_TRIM')

        maxRange = channelMax - channelMid
        minRange = channelMin - channelMid

        if rate > 10:
            rate = 10

        if rate < 0:
            rate = 0

        # ensure the UAV does not move when rate is 0
        if rate == 0.0:
            self.vehicle.channels.overrides[channel] = channelMid
            return

        rate = -rate

        self.vehicle.channels.overrides[channel] = self.__translate(rate, -10, 0, channelMin, channelMid)

    def forward(self, rate = 1.0):
        channel = self.getProperty('RC_MAP_PITCH')

        channelMin = self.getProperty('RC'+str(channel)+'_MIN')
        channelMax = self.getProperty('RC'+str(channel)+'_MAX')
        channelMid = self.getProperty('RC'+str(channel)+'_TRIM')

        maxRange = channelMax - channelMid
        minRange = channelMin - channelMid

        if rate > 10:
            rate = 10

        if rate < 0:
            rate = 0

        # ensure the UAV does not move when rate is 0
        if rate == 0.0:
            self.vehicle.channels.overrides[channel] = channelMid
            return

        self.vehicle.channels.overrides[channel] = self.__translate(rate, 0, 10, channelMid, channelMax)

    def backward(self, rate = 1.0):
        channel = self.getProperty('RC_MAP_PITCH')

        channelMin = self.getProperty('RC'+str(channel)+'_MIN')
        channelMax = self.getProperty('RC'+str(channel)+'_MAX')
        channelMid = self.getProperty('RC'+str(channel)+'_TRIM')

        maxRange = channelMax - channelMid
        minRange = channelMin - channelMid

        if rate > 10:
            rate = 10

        if rate < 0:
            rate = 0

        # ensure the UAV does not move when rate is 0
        if rate == 0.0:
            self.vehicle.channels.overrides[channel] = channelMid
            return

        rate = -rate

        self.vehicle.channels.overrides[channel] = self.__translate(rate, -10, 0, channelMin, channelMid)

    def rotate(self, angle = 0.0, timeout = 5000):
        channel = self.getProperty('RC_MAP_YAW')

        channelMin = self.getProperty('RC'+str(channel)+'_MIN')
        channelMax = self.getProperty('RC'+str(channel)+'_MAX')
        channelMid = self.getProperty('RC'+str(channel)+'_TRIM')

        maxRange = channelMax - channelMid
        minRange = channelMin - channelMid

        startTime = time.time()

        while int(self.vehicle.attitude.yaw) != int(angle):
            self.vehicle.channels.overrides[channel] = channelMid + 10
            if (time.time() - startTime) > timeout:
                break


    def takeoff(self, alt = 1):
        if self.home is None:
            self.home = [self.vehicle.location.global_frame.lat,
                            self.vehicle.location.global_frame.lon]
        self.change_mode('AUTO')
        self.arm()
        self.vehicle.simple_takeoff(alt)

    def land(self):
        self.vehicle.mode = VehicleMode("RTL")
        self.home = None

    def addBroadcast(self, ip):
        os.system('mavproxy.py --daemon --master=' + self.master + '--out=' + ip)

    def arm(self):
        if self.vehicle.is_armable:
            self.vehicle.armed = True
            while not self.vehicle.armed:
                time.sleep(.1)

    def disarm(self):
        self.vehicle.armed = False
        self.home = None
        while self.vehicle.armed:
            time.sleep(.1)

    def setMode(self, mode):
        self.vehicle.mode = VehicleMode(mode)
        while self.vehicle.mode.name != mode:
            time.sleep(1)

    def goto(self, location, relative=None):
        if relative:
            self.vehicle.simple_goto(
                LocationGlobalRelative(
                    float(location[0]), float(location[1]),
                    float(self.altitude)
                )
            )
        else:
            self.vehicle.simple_goto(
                LocationGlobal(
                    float(location[0]), float(location[1]),
                    float(self.altitude)
                )
            )
        self.vehicle.flush()

    def telem(self):
        attribs = {}
        if self.vehicle.attitude:
            attribs['roll'] = self.vehicle.attitude.roll
            attribs['yaw'] = self.vehicle.attitude.yaw
            attribs['pitch'] = self.vehicle.attitude.pitch
        if self.vehicle.heading:
            attribs['heading'] = self.vehicle.heading
        if self.vehicle.battery:
            attribs['power'] = self.vehicle.battery.level
        if self.vehicle.gps_0:
            attribs['satellites'] = self.vehicle.gps_info.satellites_visible
        if self.vehicle.gimbal:
            attribs['gimbalRoll'] = self.vehicle.gimbal.roll
            attribs['gimbalPitch'] = self.vehicle.gimbal.pitch
            attribs['gimbalYaw'] = self.vehicle.gimbal.yaw
        if self.vehicle.location and self.vehicle.location.global_frame:
            attribs['latitude'] = self.vehicle.location.global_frame.lat
            attribs['longitude'] = self.vehicle.location.global_frame.lon
            attribs['altitude'] = self.vehicle.location.global_frame.alt
        if self.vehicle.location and self.vehicle.location.local_frame:
            attribs['east'] = self.vehicle.location.local_frame.east
            attribs['north'] = self.vehicle.location.local_frame.north
            attribs['down'] = self.vehicle.location.local_frame.down
        if self.vehicle.airspeed:
            attribs['airspeed'] = self.vehicle.airspeed
        if self.vehicle.groundspeed:
            attribs['groundspeed'] = self.vehicle.groundspeed
        if self.vehicle.channels:
            for i in len(self.vehicle.channels):
                attribs['rcChannel'+str(i+1)] = self.vehicle.channels[i]
        if self.vehicle.mode:
            attribs['flightMode'] = self.vehicle.mode
        if self.vehicle.rangefinder:
            attribs['range'] = self.vehicle.range_finder.distance
        if self.vehicle.velocity:
            attribs['velocity'] = self.vehicle.velocity
        if self.vehicle.system_status:
            attribs['state'] = self.vehicle.system_status.status

        attribs['heartbeat'] = self.vehicle.last_heartbeat
        attribs['armable'] = self.vehicle.is_armable

        return attribs


    def hold(self):
        for i in len(self.vehicle.channels.overrides):
            self.vehicle.channels.overrides[i] = None

    def runPlaylist(self, playlist):
        data = json.loads(playlist)
        for item in data:
            if item.cmd == 'up':
                self.up(item.rate)
            elif item.cmd == 'down':
                self.down(item.rate)
            elif item.cmd == 'left':
                self.left(item.rate)
            elif item.cmd == 'right':
                self.right(item.rate)
            elif item.cmd == 'forward':
                self.forward(item.rate)
            elif item.cmd == 'backward':
                self.backward(item.rate)
            elif item.cmd == 'rotate':
                self.rotate(angle)
            elif item.cmd == 'takeoff':
                self.takeoff(item.alt)
            elif self.cmd == 'hold':
                self.hold()
            elif item.cmd == 'land':
                self.land()
            else:
                self.land()
            if item.wait:
                time.sleep(item.wait)
            else:
                time.sleep(1)

    def setProperty(self, param, value):
        self.vehicle.parameters[param] = value

    def getProperty(self, param):
        return self.vehicle.parameters[param]

    def rgb(self):
        return self.rgbled
