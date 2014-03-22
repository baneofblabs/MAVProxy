#!/usr/bin/env python
'''
camera view module
Malcolm Gill
Feb 2014
'''

import math
from MAVProxy.modules.mavproxy_map import mp_slipmap
from MAVProxy.modules.mavproxy_map import mp_elevation
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_settings
from cuav.lib import cuav_util
from cuav.camera.cam_params import CameraParams

mpstate = None

def scale_rc(servo, min, max, param):
    '''scale a PWM value'''
    # default to servo range of 1000 to 2000
    min_pwm = mpstate.mav_param.get('%s_MIN' % param, 0)
    max_pwm = mpstate.mav_param.get('%s_MAX' % param, 0)
    if min_pwm == 0 or max_pwm == 0:
        return 0
    if max_pwm == min_pwm:
        p = 0.0
    else:
        p = (servo-min_pwm) / float(max_pwm-min_pwm)
    v = min + p*(max-min)
    if v < min:
        v = min
    if v > max:
        v = max
    return v

class module_state(object):
    def __init__(self):
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.mount_roll = 0
        self.mount_tilt = 0
        self.mount_pan = 0
        self.height = 0
        self.lat = 0
        self.lon = 0
        self.home_height = 0
        self.hdg = 0
        self.elevation_model = mp_elevation.ElevationModel()
        self.camera_params = CameraParams() # TODO how to get actual camera params
        self.settings = mp_settings.MPSettings(
            [ ('r', float, 0.5),
              ('g', float, 0.5),
              ('b', float, 1.0),
            ])
        self.update_col()

        # these values are for ardupilot, from RC_Channel_aux.h
        # but i feel there should be a more generally correct way of obtaining these constants
        self.k_mount = {6:'PAN', 7:'TILT', 8:'ROLL'}
        self.mnt_stab = {self.k_mount[n] : mpstate.mav_param.get('MNT_STAB_{0}'.format(self.k_mount[n]), 0) for n in self.k_mount}

        # also not sure how to determine which channels are available
        self.check_channels = [5, 6, 7, 8]
        self.rc_function = {chan : mpstate.mav_param.get('RC{0}_FUNCTION'.format(chan), 0) for chan in self.check_channels}

        self.update_mount_servo_channels()
    def update_col(self):
        self.col = tuple(int(255*c) for c in (self.settings.r, self.settings.g, self.settings.b))
    def update_mount_servo_channels(self):
        ''' Sets e.g. to {5:'PAN', 7:'ROLL'} where keys are servo channels. '''
        self.channels = {ch : self.k_mount[self.rc_function[ch]]
                              for ch in self.rc_function
                                  if self.rc_function[ch] in self.k_mount and
                                     self.mnt_stab[self.k_mount[self.rc_function[ch]]]}

def name():
    '''return module name'''
    return "cameraview"

def description():
    '''return module description'''
    return "camera view module"

def cmd_cameraview(args):
    '''camera view commands'''
    state = mpstate.cameraview_state
    if args and args[0] == 'set':
        if len(args) < 3:
            state.settings.show_all()
        else:
            state.settings.set(args[1], args[2])
            state.update_col()
    else:
        print 'usage: cameraview set'

def init(_mpstate):
    '''initialise module'''
    global mpstate
    mpstate = _mpstate
    mpstate.cameraview_state = module_state()
    mpstate.command_map['cameraview'] = (cmd_cameraview, "camera view")

def unload():
    '''unload module'''
    pass

# documented in common.xml, can't find these constants in code
scale_latlon = 1e-7
scale_hdg = 1e-2
scale_relative_alt = 1e-3
def mavlink_packet(m):
    '''handle an incoming mavlink packet'''
    state = mpstate.cameraview_state
    if m.get_type() == 'GLOBAL_POSITION_INT':
        state.lat, state.lon = m.lat*scale_latlon, m.lon*scale_latlon
        state.hdg = m.hdg*scale_hdg
        state.height = m.relative_alt*scale_relative_alt + state.home_height - state.elevation_model.GetElevation(state.lat, state.lon)
    elif m.get_type() == 'ATTITUDE':
        state.roll, state.pitch, state.yaw = math.degrees(m.roll), math.degrees(m.pitch), math.degrees(m.yaw)
    elif m.get_type() in ['GPS_RAW', 'GPS_RAW_INT']:
        if mpstate.wp_state.wploader.count() > 0:
            home = mpstate.wp_state.wploader.wp(0).x, mpstate.wp_state.wploader.wp(0).y
        else:
            home = [mpstate.master().field('HOME', c)*scale_latlon for c in ['lat', 'lon']]
        old = state.home_height # TODO TMP
        state.home_height = state.elevation_model.GetElevation(*home)

        # TODO TMP
        if state.home_height != old:
            # tridge said to get home pos from wploader,
            # but this is not the same as from master() below...!!
            # using master() gives the right coordinates
            # (i.e. matches GLOBAL_POSITION_INT coords, and $IMHOME in sim_arduplane.sh)
            # and wploader is a bit off
            print 'home height changed from',old,'to',state.home_height
    elif m.get_type() == 'PARAM_VALUE':
        param_id = m.param_id[:m.param_id.find('\0')] # strip trailing nulls
        if param_id in ['RC{0}_FUNCTION'.format(channel) for channel in state.check_channels]:
            channel = int(param_id[2:param_id.find('_')])
            state.rc_function[channel] = int(m.param_value) # is this dangerous?
        elif param_id in ['MNT_STAB_{0}'.format(state.k_mount[n]) for n in state.k_mount]:
            axis = param_id[param_id.rfind('_')+1:]
            state.mnt_stab[axis] = m.param_value
        state.update_mount_servo_channels()
    elif m.get_type() == 'SERVO_OUTPUT_RAW':
        for (channel, axis) in state.channels.iteritems():
            servo = 'servo{0}_raw'.format(channel)
            centidegrees = scale_rc(getattr(m, servo),
                                    mpstate.mav_param.get('MNT_ANGMIN_{0}'.format(axis[:3])),
                                    mpstate.mav_param.get('MNT_ANGMAX_{0}'.format(axis[:3])),
                                    param='RC{0}'.format(channel))
            setattr(state, 'mount_{0}'.format(axis.lower()), centidegrees*0.01)
    else:
        return
    if mpstate.map: # if the map module is loaded, redraw polygon
        # get rid of the old polygon
        mpstate.map.add_object(mp_slipmap.SlipClearLayer('CameraView'))

        # camera view polygon determined by projecting corner pixels of the image onto the ground
        pixel_positions = [cuav_util.pixel_position(px[0],px[1], state.height, state.pitch+state.mount_tilt, state.roll+state.mount_roll, state.yaw+state.mount_pan, state.camera_params) for px in [(0,0), (state.camera_params.xresolution,0), (state.camera_params.xresolution,state.camera_params.yresolution), (0,state.camera_params.yresolution)]]
        if any(pixel_position is None for pixel_position in pixel_positions):
            # at least one of the pixels is not on the ground
            # so it doesn't make sense to try to draw the polygon
            return
        gps_positions = [mp_util.gps_newpos(state.lat, state.lon, math.degrees(math.atan2(*pixel_position)), math.hypot(*pixel_position)) for pixel_position in pixel_positions]

        # draw new polygon
        mpstate.map.add_object(mp_slipmap.SlipPolygon('cameraview', gps_positions+[gps_positions[0]], # append first element to close polygon
                                                      layer='CameraView', linewidth=2, colour=state.col))
