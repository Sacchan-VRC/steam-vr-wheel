import openvr
import quaternionic
import math

from steam_vr_wheel.pyvjoy.vjoydevice import HID_USAGE_X, HID_USAGE_Y, HID_USAGE_Z, HID_USAGE_RX, HID_USAGE_RY, HID_USAGE_RZ
from steam_vr_wheel._virtualpad import VirtualPad, RightTrackpadAxisDisablerMixin, LeftTrackpadAxisDisablerMixin

class Throttle():

    def __init__(self, size=0.3, inverted = False, starting = 0.0, centeronrelease = False):
        self.throttle_z = starting
        self.throttle_real_world_z = 0
        self.throttle_at_real_world_z = 0
        self.throttle_relative_zeroed = False
        self.throttle_grabbed = False
        self.invertion = -1 if inverted else 1
        self.size = size
        self.centeronrelease = centeronrelease

    @property
    def x(self):
        return self.throttle_z/self.size

    def grabbed(self):
        self.throttle_grabbed = True

    def ungrabbed(self):
        self.throttle_grabbed = False
        if self.centeronrelease:
            self.throttle_z = 0.5

    def update(self, value,):
        if self.throttle_grabbed:
            if not self.throttle_relative_zeroed:
                self.throttle_real_world_z = value * self.invertion
                if self.centeronrelease:
                    self.throttle_at_real_world_z = 0.5
                else:
                    self.throttle_at_real_world_z = self.throttle_z
                self.throttle_relative_zeroed = True
            else:
                self.throttle_z = self.invertion * value - self.throttle_real_world_z + self.throttle_at_real_world_z
                if self.throttle_z<0:
                    self.throttle_z = 0
                elif self.throttle_z>self.size:
                    self.throttle_z = self.size

        else:
            self.throttle_relative_zeroed = False

class Joystick(RightTrackpadAxisDisablerMixin, LeftTrackpadAxisDisablerMixin, VirtualPad):
    def __init__(self):
        self.amplification = 10
        super().__init__()
        self.x = 0
        self.y = 0
        self.throttle_z = Throttle(size=1)
        self.throttle_y = Throttle(size=1)
        self.throttle_x = Throttle(size=1)
        resetondrop = self.config.joystick_resets_to_center_when_ungrabbed
        self.grabbable_x = Throttle(size=1, starting=0.5, centeronrelease=resetondrop)
        self.grabbable_y = Throttle(size=1, starting=0.5, centeronrelease=resetondrop)
        self.grabbable_z = Throttle(size=1, starting=0.5, centeronrelease=resetondrop)

        self.joystick_grabbed = False

    @property
    def joystick_grabbed(self):
        try:
            return self._joystick_grabbed
        except:
            return False

    @joystick_grabbed.setter
    def joystick_grabbed(self, x):
        self._joystick_grabbed = bool(x)
        if x:
            self.grabbable_x.grabbed()
            self.grabbable_y.grabbed()
            self.grabbable_z.grabbed()
        else:
            self.grabbable_x.ungrabbed()
            self.grabbable_y.ungrabbed()
            self.grabbable_z.ungrabbed()

    def set_button_press(self, button, hand):
        super().set_button_press(button, hand)
        if button == openvr.k_EButton_Grip and hand == 'left':
            self.throttle_z.grabbed()
            self.throttle_x.grabbed()
            self.throttle_y.grabbed()
        if button == openvr.k_EButton_Grip and hand == 'right':
            if self.config.joystick_grabbing_switch:
                self.joystick_grabbed = not self.joystick_grabbed
            else:
                self.joystick_grabbed = True

    def set_button_unpress(self, button, hand):
        super().set_button_unpress(button, hand)
        if button == openvr.k_EButton_Grip and hand == 'left':
            self.throttle_z.ungrabbed()
            self.throttle_x.ungrabbed()
            self.throttle_y.ungrabbed()
        if button == openvr.k_EButton_Grip and hand == 'right' and (not self.config.joystick_grabbing_switch):
            self.joystick_grabbed = False

    def _update_joystick_normal(self, axisX, axisY, axisZ):

        self.device.set_axis(HID_USAGE_X, int(axisX-1 * 0x8000))
        self.device.set_axis(HID_USAGE_Y, int(axisY-1 * 0x8000))
        self.device.set_axis(HID_USAGE_Z, int(axisZ-1 * 0x8000))

    def _update_grabbable_joystick(self, axisX, axisY, axisZ):

        self.grabbable_x.update(axisX)
        self.grabbable_y.update(axisY)
        self.grabbable_z.update(axisZ)

        self.device.set_axis(HID_USAGE_X, int(self.grabbable_x.x * 0x8000))
        self.device.set_axis(HID_USAGE_Y, int(self.grabbable_y.x * 0x8000))
        self.device.set_axis(HID_USAGE_Z, int(self.grabbable_z.x * 0x8000))

    def update(self, left_ctr, right_ctr):
        super().update(left_ctr, right_ctr)

        #make the 45 configurable
        axisX = (math.acos(right_ctr.rotquat[3]) * (math.pi * .5) * math.degrees(1)) / 45
        axisY = -(math.acos(right_ctr.rotquat[1]) * (math.pi * .5) * math.degrees(1)) / 45
        axisZ = (math.acos(right_ctr.rotquat[2]) * (math.pi * .5) * math.degrees(1)) / 45

        if self.config.joystick_updates_only_when_grabbed:
            self._update_grabbable_joystick(axisX, axisY, axisZ)
        else:
            self._update_joystick_normal(axisX, axisY, axisZ)

        self.throttle_z.update(left_ctr.z)
        self.throttle_y.update(left_ctr.y)
        self.throttle_x.update(left_ctr.yaw)

        self.device.set_axis(HID_USAGE_RZ, int(self.throttle_z.x * 0x8000))
        self.device.set_axis(HID_USAGE_RY, int(self.throttle_y.x * 0x8000))
        self.device.set_axis(HID_USAGE_RX, int(self.throttle_x.x * 0x8000))
