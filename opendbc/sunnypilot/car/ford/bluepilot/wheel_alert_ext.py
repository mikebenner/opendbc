"""Ford steering wheel vibration and cluster chime for openpilot's alerts. Not in BluePilot 7.

openpilot replaces the camera's Lane_Assist_Data1 (0x3CA) with its own frame and leaves the lane departure
warning fields at zero, so the wheel never vibrates while openpilot runs. This fills in LdwActvStats_D_Req and
LdwActvIntns_D_Req (byte 0, bits 4-0) on that frame:

  lane departure   openpilot's own detection (not steering, above 31 mph)            medium, on that side
  take control     a steerRequired alert, e.g. "Turn Exceeds Steering Limit"          medium
  critical         a steerRequired alert with a warning sound (a forced disengage)    high, and the cluster's
                                                                                      hands-on-wheel chime

The vibration comes in bursts with a rest in between, so a long alert doesn't buzz without a break.

The lane keeping aid field (LkaActvStats_D2_Req, byte 0 bits 7-5) is never written: it stays zero, which is all
the panda lets through. BluePilot lateral's own bits in this frame (bytes 4-6) are kept as they are.

Between bursts the frame says "idle, intensity high" instead of upstream's "idle, intensity none". On a 2023
Expedition a burst sent from upstream's baseline did nothing: the PSCM reported the lane departure warning as
suppressed (Lane_Assist_Data3_FD1 LaActAvail_D_Actl 0 or 2) the whole time, and intensity none may be why. This
is a probe: whether the PSCM then reports the warning as available is read from the drive log.

Off unless the host publishes send_wheel_vibration_alerts in CarControlSP.params.
"""
from opendbc.car import DT_CTRL, structs
from opendbc.car.can_definitions import CanData
from opendbc.sunnypilot.car.ford.bluepilot.published import published_bool

VisualAlert = structs.CarControl.HUDControl.VisualAlert
AudibleAlert = structs.CarControl.HUDControl.AudibleAlert

# setting: off when the host doesn't publish it
WHEEL_ALERT_PARAMS = {
  "send_wheel_vibration_alerts": False,
}

LANE_ASSIST_DATA1 = 0x3CA
LKA_ACTION_MASK = 0xE0  # LkaActvStats_D2_Req, left untouched
# LdwActvStats_D_Req
LDW_IDLE, LDW_WARNING_LEFT, LDW_WARNING_RIGHT = 0, 2, 4
# LdwActvIntns_D_Req
INTENSITY_NONE, INTENSITY_MEDIUM, INTENSITY_HIGH = 0, 2, 3
# LaHandsOff_D_Dsply: hands-on-wheel message with its chime
HANDS_OFF_CHIME = 2

NONE, DEPARTURE, TAKE_CONTROL, CRITICAL = 0, 1, 2, 3
BURST_S = {DEPARTURE: 1.0, TAKE_CONTROL: 1.0, CRITICAL: 2.0}
REST_S = {DEPARTURE: 2.0, TAKE_CONTROL: 3.0, CRITICAL: 1.0}
INTENSITY = {DEPARTURE: INTENSITY_MEDIUM, TAKE_CONTROL: INTENSITY_MEDIUM, CRITICAL: INTENSITY_HIGH}
BASELINE_INTENSITY = INTENSITY_HIGH  # sent with LDW_IDLE between bursts while the setting is on


def wanted(hud_control) -> tuple[int, int]:
  """(level, LdwActvStats_D_Req) for this frame's alerts."""
  if hud_control.visualAlert == VisualAlert.steerRequired:
    critical = hud_control.audibleAlert in (AudibleAlert.warningSoft, AudibleAlert.warningImmediate)
    return (CRITICAL if critical else TAKE_CONTROL), LDW_WARNING_LEFT
  if hud_control.leftLaneDepart:
    return DEPARTURE, LDW_WARNING_LEFT
  if hud_control.rightLaneDepart:
    return DEPARTURE, LDW_WARNING_RIGHT
  return NONE, LDW_IDLE


def set_ldw_request(msg: CanData, status: int, intensity: int) -> CanData:
  address, dat, bus = msg
  dat = bytearray(dat)
  dat[0] = (dat[0] & LKA_ACTION_MASK) | ((status & 0x7) << 2) | (intensity & 0x3)
  return CanData(address, bytes(dat), bus)


class WheelAlertExt:
  def __init__(self):
    self.level = NONE
    self.status = LDW_IDLE
    self.burst_frames = 0
    self.rest_frames = 0
    self.hands_off_level: int | None = None  # for the cluster message (hud_ext); None leaves it as it was

  @property
  def vibrating(self) -> bool:
    return self.burst_frames > 0

  def update(self, CC, CC_SP, can_sends: list[CanData]) -> list[CanData]:
    """Call every frame, after the Lane_Assist_Data1 frame (if any this frame) is in can_sends."""
    if not published_bool(CC_SP, "send_wheel_vibration_alerts", WHEEL_ALERT_PARAMS):
      self.__init__()
      return can_sends

    level, status = wanted(CC.hudControl)
    self.hands_off_level = HANDS_OFF_CHIME if level == CRITICAL else None

    idle = self.burst_frames == 0 and self.rest_frames == 0
    if level != NONE and (idle or level > self.level):
      # a new burst; a more serious alert doesn't wait for the current burst or rest to finish
      self.level, self.status = level, status
      self.burst_frames = round(BURST_S[level] / DT_CTRL)
      self.rest_frames = round(REST_S[level] / DT_CTRL)
    elif self.burst_frames > 0:
      self.burst_frames -= 1
    elif self.rest_frames > 0:
      self.rest_frames -= 1
      if self.rest_frames == 0:
        self.level = NONE

    status, intensity = (self.status, INTENSITY[self.level]) if self.vibrating else (LDW_IDLE, BASELINE_INTENSITY)
    return [set_ldw_request(m, status, intensity) if m[0] == LANE_ASSIST_DATA1 else m for m in can_sends]
