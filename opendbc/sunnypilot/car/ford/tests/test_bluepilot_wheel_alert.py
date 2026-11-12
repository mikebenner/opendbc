"""Ford wheel vibration and cluster chime for openpilot's alerts (sunnypilot/car/ford/bluepilot/wheel_alert_ext.py)."""
import unittest
from types import SimpleNamespace as NS

from opendbc.can import CANPacker, CANParser
from opendbc.car import structs
from opendbc.car.can_definitions import CanData
from opendbc.car.ford.interface import CarInterface
from opendbc.car.ford.values import CAR
from opendbc.sunnypilot.car.ford.bluepilot import fordcan_ext, wheel_alert_ext as wa
from opendbc.sunnypilot.car.ford.bluepilot.hud_ext import HudExt
from opendbc.sunnypilot.car.ford.bluepilot.wheel_alert_ext import WheelAlertExt

VisualAlert = structs.CarControl.HUDControl.VisualAlert
AudibleAlert = structs.CarControl.HUDControl.AudibleAlert
DBC = "ford_lincoln_base_pt"
CAN = NS(main=0, camera=2)
LKA, LKAS_UI = 0x3CA, 0x3D8
ON = [NS(key="send_wheel_vibration_alerts", value=b"1")]


def hud(**kw):
  base = dict(leftLaneDepart=False, rightLaneDepart=False, leftLaneVisible=False, rightLaneVisible=False,
              leadVisible=False, leadDistanceBars=3, visualAlert=VisualAlert.none, audibleAlert=AudibleAlert.none)
  return NS(**{**base, **kw})


TAKE_CONTROL = dict(visualAlert=VisualAlert.steerRequired, audibleAlert=AudibleAlert.promptRepeat)
CRITICAL = dict(visualAlert=VisualAlert.steerRequired, audibleAlert=AudibleAlert.warningSoft)


def decode(msg, name="Lane_Assist_Data1"):
  address, dat, bus = msg
  parser = CANParser(DBC, [(name, 10)], bus)
  parser.update([(0, [(address, dat, bus)])])
  return parser.vl[name]


def lka_frame(**values):
  return CanData(*CANPacker(DBC).make_can_msg("Lane_Assist_Data1", CAN.main, values))


class TestWanted(unittest.TestCase):
  def test_nothing(self):
    self.assertEqual(wa.wanted(hud()), (wa.NONE, wa.LDW_IDLE))

  def test_take_control(self):
    self.assertEqual(wa.wanted(hud(**TAKE_CONTROL))[0], wa.TAKE_CONTROL)
    self.assertEqual(wa.wanted(hud(visualAlert=VisualAlert.steerRequired))[0], wa.TAKE_CONTROL)

  def test_critical_needs_the_take_control_visual_and_a_warning_sound(self):
    self.assertEqual(wa.wanted(hud(**CRITICAL))[0], wa.CRITICAL)
    self.assertEqual(wa.wanted(hud(visualAlert=VisualAlert.steerRequired, audibleAlert=AudibleAlert.warningImmediate))[0], wa.CRITICAL)
    # a forward collision warning has the sound but is not a take-control alert
    self.assertEqual(wa.wanted(hud(visualAlert=VisualAlert.fcw, audibleAlert=AudibleAlert.warningSoft))[0], wa.NONE)

  def test_lane_departure_names_the_side(self):
    self.assertEqual(wa.wanted(hud(leftLaneDepart=True)), (wa.DEPARTURE, wa.LDW_WARNING_LEFT))
    self.assertEqual(wa.wanted(hud(rightLaneDepart=True)), (wa.DEPARTURE, wa.LDW_WARNING_RIGHT))

  def test_other_visual_alerts_do_nothing(self):
    for v in (VisualAlert.fcw, VisualAlert.brakePressed, VisualAlert.wrongGear, VisualAlert.seatbeltUnbuckled, VisualAlert.speedTooHigh):
      self.assertEqual(wa.wanted(hud(visualAlert=v))[0], wa.NONE)


class TestFrame(unittest.TestCase):
  def test_sets_only_the_warning_fields(self):
    msg = wa.set_ldw_request(lka_frame(), wa.LDW_WARNING_RIGHT, wa.INTENSITY_HIGH)
    vl = decode(msg)
    self.assertEqual((vl["LdwActvStats_D_Req"], vl["LdwActvIntns_D_Req"]), (4, 3))
    for sig in ("LkaActvStats_D2_Req", "LaRefAng_No_Req", "LaCurvature_No_Calc", "LaRampType_B_Req", "LkaDrvOvrrd_D_Rq"):
      self.assertEqual(vl[sig], decode(lka_frame())[sig], sig)

  def test_never_writes_the_lane_keeping_aid_action(self):
    for status in range(8):
      for intensity in range(4):
        self.assertEqual(wa.set_ldw_request(lka_frame(), status, intensity)[1][0] >> 5, 0)

  def test_keeps_bluepilot_laterals_bits(self):
    address, dat, bus = lka_frame()
    dat = bytearray(dat)
    dat[4] |= 0x01
    dat[5], dat[6] = 0x12, 0x34  # angle mode engaged + shadow curvature
    out = wa.set_ldw_request(CanData(address, bytes(dat), bus), wa.LDW_WARNING_LEFT, wa.INTENSITY_MEDIUM)
    self.assertEqual(out[1][1:], bytes(dat)[1:])
    self.assertEqual(out[1][0], (wa.LDW_WARNING_LEFT << 2) | wa.INTENSITY_MEDIUM)


class TestWheelAlertExt(unittest.TestCase):
  def run_frames(self, ext, n, params=ON, **hud_kw):
    """byte 0 of the Lane_Assist_Data1 frame on each of n frames (one frame sent per call)."""
    out = []
    for _ in range(n):
      sends = ext.update(NS(hudControl=hud(**hud_kw)), NS(params=params), [lka_frame()])
      out.append(sends[0][1][0])
    return out

  def test_off_unless_published(self):
    ext = WheelAlertExt()
    self.assertEqual(set(self.run_frames(ext, 300, params=[], **CRITICAL)), {0})
    self.assertIsNone(ext.hands_off_level)
    self.assertEqual(set(self.run_frames(ext, 300, params=[NS(key="send_wheel_vibration_alerts", value=b"0")], **CRITICAL)), {0})

  def test_no_alert_no_vibration(self):
    self.assertEqual(set(self.run_frames(WheelAlertExt(), 500)), {0})

  def test_take_control_is_one_second_on_three_off(self):
    b = self.run_frames(WheelAlertExt(), 900, **TAKE_CONTROL)
    on = (wa.LDW_WARNING_LEFT << 2) | wa.INTENSITY_MEDIUM
    self.assertEqual(set(b), {0, on})
    self.assertEqual([i for i in range(1, 900) if b[i] != b[i - 1]], [100, 401, 501, 802])
    self.assertEqual(b[0], on)

  def test_critical_is_two_seconds_on_one_off_at_high(self):
    ext = WheelAlertExt()
    b = self.run_frames(ext, 700, **CRITICAL)
    on = (wa.LDW_WARNING_LEFT << 2) | wa.INTENSITY_HIGH
    self.assertEqual([i for i in range(1, 700) if b[i] != b[i - 1]], [200, 301, 501, 602])
    self.assertEqual(b[0], on)
    self.assertEqual(ext.hands_off_level, 2)

  def test_a_short_alert_still_gets_its_full_burst_then_stops(self):
    ext = WheelAlertExt()
    b = self.run_frames(ext, 20, **TAKE_CONTROL) + self.run_frames(ext, 600)
    self.assertEqual(sum(x != 0 for x in b), 100)
    self.assertEqual(set(b[100:]), {0})

  def test_critical_interrupts_a_take_control_rest(self):
    ext = WheelAlertExt()
    self.run_frames(ext, 150, **TAKE_CONTROL)  # burst over, resting
    self.assertFalse(ext.vibrating)
    b = self.run_frames(ext, 5, **CRITICAL)
    self.assertEqual(b[0], (wa.LDW_WARNING_LEFT << 2) | wa.INTENSITY_HIGH)

  def test_lane_departure_does_not_interrupt_a_rest(self):
    ext = WheelAlertExt()
    self.run_frames(ext, 150, **TAKE_CONTROL)
    self.assertEqual(set(self.run_frames(ext, 50, rightLaneDepart=True)), {0})

  def test_lane_departure_vibrates_on_its_side(self):
    b = self.run_frames(WheelAlertExt(), 10, rightLaneDepart=True)
    self.assertEqual(b[0], (wa.LDW_WARNING_RIGHT << 2) | wa.INTENSITY_MEDIUM)

  def test_other_frames_pass_through(self):
    ext = WheelAlertExt()
    other = CanData(0x3D8, bytes(8), 0)
    sends = ext.update(NS(hudControl=hud(**CRITICAL)), NS(params=ON), [other, lka_frame()])
    self.assertEqual(sends[0], other)
    self.assertNotEqual(sends[1][1][0], 0)

  def test_switching_off_stops_at_once(self):
    ext = WheelAlertExt()
    self.run_frames(ext, 10, **CRITICAL)
    self.assertEqual(self.run_frames(ext, 3, params=[], **CRITICAL), [0, 0, 0])


class TestClusterChime(unittest.TestCase):
  def stock(self):
    return dict(CANParser(DBC, [("IPMA_Data", 10)], 0).vl["IPMA_Data"])

  def test_level_override(self):
    packer = CANPacker(DBC)
    for steer_alert, level, want in ((False, None, 0), (True, None, 1), (True, 2, 2), (False, 2, 2)):
      msg = fordcan_ext.create_lkas_ui_msg(packer, CAN, steer_alert, hud(), self.stock(), level)
      self.assertEqual(decode(msg, "IPMA_Data")["LaHandsOff_D_Dsply"], want)

  def test_hud_sends_at_once_when_the_level_changes(self):
    CP = structs.CarParams()
    h = HudExt(CP)
    CS = NS(lkas_status_stock_values=self.stock(), acc_tja_status_stock_values=dict(CANParser(DBC, [("ACCDATA_3", 10)], 0).vl["ACCDATA_3"]),
            out=NS(cruiseState=NS(standstill=False)))
    CC = NS(hudControl=hud(), latActive=False)
    packer = CANPacker(DBC)
    h.update(CC, NS(params=[]), CS, True, True, False, packer, CAN, 1)
    quiet = [m for m in h.update(CC, NS(params=[]), CS, True, True, False, packer, CAN, 2) if m[0] == LKAS_UI]
    self.assertEqual(quiet, [])
    chime = [m for m in h.update(CC, NS(params=[]), CS, True, True, False, packer, CAN, 3, 2) if m[0] == LKAS_UI]
    self.assertEqual(decode(chime[0], "IPMA_Data")["LaHandsOff_D_Dsply"], 2)


class TestCarController(unittest.TestCase):
  """The real Ford car controller: the request goes out on Lane_Assist_Data1 and nowhere else."""
  def frames(self, params, **hud_kw):
    CP = CarInterface.get_non_essential_params(CAR.FORD_EXPEDITION_MK4)
    CP_SP = CarInterface.get_non_essential_params_sp(CP, CAR.FORD_EXPEDITION_MK4)
    CI = CarInterface(CP, CP_SP)
    CI.update([])
    CC = structs.CarControl.new_message()
    for k, v in hud_kw.items():
      setattr(CC.hudControl, k, v)
    CC_SP = structs.CarControlSP()
    CC_SP.params = [structs.CarControlSP.Param(key=p.key, value=p.value) for p in params]
    lka = []
    for _ in range(120):
      _, sends = CI.apply(CC.as_reader(), CC_SP, 0)
      lka += [m for m in sends if m[0] == LKA]
    return lka

  def test_take_control_alert_vibrates(self):
    lka = self.frames(ON, visualAlert=VisualAlert.steerRequired)
    self.assertEqual(len(lka), 40)  # 33 Hz
    self.assertTrue(all(m[2] == CAN.main for m in lka))
    self.assertEqual({m[1][0] >> 5 for m in lka}, {0})
    self.assertEqual({m[1][0] for m in lka[:33]}, {(wa.LDW_WARNING_LEFT << 2) | wa.INTENSITY_MEDIUM})
    self.assertEqual({m[1][0] for m in lka[35:]}, {0})

  def test_setting_off_sends_upstreams_frame(self):
    self.assertEqual({m[1][0] for m in self.frames([], visualAlert=VisualAlert.steerRequired)}, {0})


if __name__ == '__main__':
  unittest.main()
