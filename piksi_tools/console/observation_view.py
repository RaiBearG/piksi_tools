#!/usr/bin/env python
# Copyright (C) 2011-2014, 2016 Swift Navigation Inc.
# Contact: Fergus Noble <fergus@swift-nav.com>
#          Pasi Miettinen <pasi.miettinen@exafore.com>
#
# This source is subject to the license found in the file 'LICENSE' which must
# be be distributed together with this source. All other rights reserved.
#
# THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
# EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.

from traits.api import Dict, HasTraits, List, Bool, Str, Float, Int
from traitsui.api import Item, View, HGroup, VGroup, TabularEditor, Spring
from traitsui.tabular_adapter import TabularAdapter
from enable.savage.trait_defs.ui.svg_button import SVGButton
from piksi_tools.console.utils import determine_path,sopen
from piksi_tools.console.utils import code_to_str, code_is_gps
from piksi_tools.console.utils import L1CA
from piksi_tools.console.utils import EMPTY_STR

import os
import datetime
import copy

from sbp.observation import *

class SimpleAdapter(TabularAdapter):
    columns = [('PRN', 0),
               ('Pseudorange (m)', 1),
               ('Carrier Phase (cycles)', 2),
               ('C/N0 (dB-Hz)', 3),
               ('Meas. Doppler (Hz)', 4),
               ('Comp. Doppler (Hz)', 5),
               ('Lock', 6),
               ('Flags', 7)]
    font='courier'
    alignment='center'

class ObservationView(HasTraits):
  python_console_cmds = Dict()

  _obs_table_list = List()
  obs = Dict()
  old_cp = Dict()
  new_cp = Dict()
  old_tow = Float()
  gps_tow = Float()
  obs_count = Int()
  gps_week = Int()
  l1_count = Int()
  l2_count = Int()
  name = Str('Local')

  def trait_view(self, view):
    return View(
      VGroup(
        HGroup(
          Spring(width=4, springy=False),
          Item('', label='GPS Week:', emphasized = True, tooltip='GPS Week Number (since 1980'),
          Item('gps_week', style='readonly', show_label=False),
          Item('', label='GPS TOW:', emphasized = True, tooltip='GPS milliseconds in week'),
          Item('gps_tow', style='readonly', show_label=False),
          Item('', label='Total obs:', emphasized = True, tooltip='GPS milliseconds in week'),
          Item('obs_count', style='readonly', show_label=False),
          Item('', label='L1 obs:', emphasized = True, tooltip='GPS milliseconds in week'),
          Item('l1_count', style='readonly', show_label=False),
          Item('', label='L2 obs:', emphasized = True, tooltip='GPS milliseconds in week'),
          Item('l2_count', style='readonly', show_label=False),
          ),
        HGroup(
          Item('_obs_table_list', style='readonly',
               editor=TabularEditor(adapter=SimpleAdapter()), show_label=False),
        ),
        label=self.name,
        show_border=True
      )
    )

  def update_obs(self):
    self._obs_table_list =\
      [(prn,) + obs for prn, obs in sorted(self.obs.items(),
                                           key=lambda x: x[0])]
    self.obs_count = len(self.obs)
    self.l1_count = sum(['L1' in obs for obs in self.obs])
    self.l2_count = sum(['L2' in obs for obs in self.obs])

  def obs_packed_callback(self, sbp_msg, **metadata):
    if (sbp_msg.sender is not None and
        (self.relay ^ (sbp_msg.sender == 0))):
      return
    tow = sbp_msg.header.t.tow
    wn = sbp_msg.header.t.wn
    seq = sbp_msg.header.n_obs

    tow = float(tow) / 1000.0

    total = seq >> 4
    count = seq & ((1 << 4) - 1)
    # Confirm this packet is good.
    # Assumes no out-of-order packets
    if count == 0:
      self.old_tow = self.gps_tow
      self.gps_tow = tow
      self.gps_week = wn
      self.prev_obs_total = total
      self.prev_obs_count = 0
      self.old_cp = self.new_cp
      self.new_cp = {}
      self.obs = {}
    elif self.gps_tow != tow or\
         self.gps_week != wn or\
         self.prev_obs_count + 1 != count or\
         self.prev_obs_total != total:
      print "We dropped a packet. Skipping this observation sequence"
      self.prev_obs_count = -1
      return
    else:
      self.prev_obs_count = count

    # Save this packet
    # See sbp_piksi.h for format
    for o in sbp_msg.obs:
      # Handle all the message specific stuff
      prn = o.sid.sat
      flags = 0
      msdopp = 0

      # Old PRN values had to add one for GPS
      if (code_is_gps(o.sid.code)
            and sbp_msg.msg_type in [SBP_MSG_OBS_DEP_A, SBP_MSG_OBS_DEP_B, SBP_MSG_OBS_DEP_C]):
        prn += 1

      prn = '{} ({})'.format(prn, code_to_str(o.sid.code))

      # DEP_B and DEP_A obs had different pseudorange scaling
      if sbp_msg.msg_type in [SBP_MSG_OBS_DEP_A, SBP_MSG_OBS_DEP_B] :
        divisor = 1e2
      else:
        divisor = 5e1

      if sbp_msg.msg_type not in [SBP_MSG_OBS_DEP_A, SBP_MSG_OBS_DEP_B, SBP_MSG_OBS_DEP_C]:
        flags = o.flags
        msdopp = float(o.D.i) + float(o.D.f) / (1 << 8)
        self.gps_tow += sbp_msg.header.t.ns_residual * 1e-9

      try:
        ocp = self.old_cp[prn]
      except:
        ocp = 0

      cp = float(o.L.i) + float(o.L.f) / (1 << 8)

      # Compute time difference of carrier phase for display, but only if carrier phase is valid
      if ocp != 0 and ((sbp_msg.msg_type in [SBP_MSG_OBS_DEP_A, SBP_MSG_OBS_DEP_B, SBP_MSG_OBS_DEP_C]) or (flags & 0x3) == 0x3):
        # Doppler per RINEX has opposite sign direction to carrier phase
        if self.gps_tow != self.old_tow:
          cpdopp = (ocp - cp) / float(self.gps_tow - self.old_tow)
        else:
          print "Received two complete observation sets with identical TOW"
          cpdopp = 0

        # Older messages had flipped sign carrier phase values
        if sbp_msg.msg_type in [SBP_MSG_OBS_DEP_A, SBP_MSG_OBS_DEP_B]:
          cpdopp = -cpdopp
      else:
        cpdopp = 0

      # Save carrier phase value, but only if value is valid
      if (sbp_msg.msg_type in [SBP_MSG_OBS_DEP_A, SBP_MSG_OBS_DEP_B, SBP_MSG_OBS_DEP_C]) or (flags & 0x3) == 0x3:
        self.new_cp[prn] = cp

      flags_str = "0x{:04X} = ".format(flags)

      # Add some indicators to help understand the flags values
      # Bit 0 is Pseudorange valid
      if (flags & 0x01):
        flags_str += "PR "
      # Bit 1 is Carrier phase valid
      if (flags & 0x02):
        flags_str += "CP "
      # Bit 2 is Half-cycle ambiguity
      if (flags & 0x04):
        flags_str += "1/2C "
      # Bit 3 is Measured Doppler Valid
      if (flags & 0x08):
        flags_str += "MD "

      pr_str = "{:11.2f}".format(float(o.P) / divisor)
      cp_str = "{:13.2f}".format(cp)
      cn0_str = "{:2.1f}".format(float(o.cn0) / 4)
      msdopp_str = "{:9.2f}".format(msdopp)
      cpdopp_str = "{:9.2f}".format(cpdopp)
      lock_str = "{:5d}".format(o.lock)

      # Sets invalid values to zero
      if sbp_msg.msg_type not in [SBP_MSG_OBS_DEP_A, SBP_MSG_OBS_DEP_B, SBP_MSG_OBS_DEP_C]:
        if not (flags & 0x01):
          pr_str = EMPTY_STR
        if not (flags & 0x02):
          cp_str = EMPTY_STR
        if not (flags & 0x08):
          msdopp_str = EMPTY_STR
      if (cpdopp == 0):
        cpdopp_str = EMPTY_STR

      self.obs[prn] = (pr_str, cp_str, cn0_str, msdopp_str, cpdopp_str, lock_str, flags_str)

    if (count == total - 1):
      self.t = datetime.datetime(1980, 1, 6) + \
               datetime.timedelta(weeks=self.gps_week) + \
               datetime.timedelta(seconds=self.gps_tow)

      self.update_obs()

    return

  def __init__(self, link, name='Local', relay=False, dirname=None):
    super(ObservationView, self).__init__()
    self.dirname = dirname
    self.obs_count = 0
    self.gps_tow = 0.0
    self.gps_week = 0
    self.relay = relay
    self.name = name
    self.rinex_file = None
    self.eph_file = None
    self.link = link
    self.link.add_callback(self.obs_packed_callback,
                           [SBP_MSG_OBS,
                            SBP_MSG_OBS_DEP_A,
                            SBP_MSG_OBS_DEP_B,
                            SBP_MSG_OBS_DEP_C])
    self.python_console_cmds = {'obs': self}
