"""BluePilot (bp-full): Ford highway following (sunnypilot/car/ford/bluepilot/longitudinal_ext.py)."""
import unittest
from types import SimpleNamespace as NS

from opendbc.car.ford.values import CarControllerParams
from opendbc.sunnypilot.car.ford.bluepilot.longitudinal_ext import LongitudinalExt, preview_signals

MPH = 0.44704


def cc_sp(lead=None, **params):
  lead_ns = NS(**{"status": lead is not None, "dRel": 0.0, "vRel": 0.0, "vLead": 0.0, **(lead or {})})
  return NS(leadOne=lead_ns, params=[NS(key=k, value=v) for k, v in params.items()])


def cs(mph, gas=False, brake=False):
  return NS(out=NS(vEgo=mph * MPH, gasPressed=gas, brakePressed=brake))


ACTIVE = NS(longActive=True)


class TestLongitudinalExt(unittest.TestCase):
  def setUp(self):
    self.ext = LongitudinalExt()

  def run_at(self, mph, op_accel, op_gas, lead=None, pitch=0.0, frames=1, **params):
    for _ in range(frames):
      out = self.ext.update(ACTIVE, cc_sp(lead, **params), cs(mph), op_accel, op_gas, pitch)
    return out

  def test_below_50_mph_is_upstream_accel(self):
    accel, gas, brake, precharge = self.run_at(40, -1.0, CarControllerParams.INACTIVE_GAS)
    self.assertEqual(accel, -1.0)
    self.assertTrue(brake and precharge)

  def test_speed_deadband(self):
    self.run_at(51, 0.5, 0.5)
    self.assertTrue(self.ext.bp_speed_allow)
    self.run_at(47, 0.5, 0.5)
    self.assertTrue(self.ext.bp_speed_allow)  # stays on until below 45
    self.run_at(44, 0.5, 0.5)
    self.assertFalse(self.ext.bp_speed_allow)

  def test_gaining_close_cuts_gas(self):
    lead = dict(dRel=30.0, vRel=-2.0, vLead=28.0)  # 1.2 s at 55 mph, closing
    accel, gas, _, _ = self.run_at(55, 0.3, 0.3, lead)
    self.assertEqual(gas, 0.0)

  def test_pacing_caps_gas(self):
    lead = dict(dRel=50.0, vRel=0.0, vLead=25.0)
    _, gas, _, _ = self.run_at(55, 1.0, 1.0, lead)
    self.assertAlmostEqual(gas, 0.2)

  def test_slow_lead_is_upstream(self):
    lead = dict(dRel=50.0, vRel=0.0, vLead=15.0)  # lead under 40 mph
    _, gas, _, _ = self.run_at(55, 1.0, 1.0, lead)
    self.assertEqual(gas, 1.0)

  def test_braking_ramps_in(self):
    lead = dict(dRel=60.0, vRel=-1.0, vLead=24.0)  # far, slow closing: rate limit applies
    accel, _, _, _ = self.run_at(55, -1.0, CarControllerParams.INACTIVE_GAS, lead)
    self.assertAlmostEqual(accel, -0.002)

  def test_no_lead_holds_accel(self):
    # BP7 behavior, ported as-is (#16 follow-up)
    accel, _, brake, _ = self.run_at(55, -1.0, CarControllerParams.INACTIVE_GAS, frames=3)
    self.assertEqual(accel, 0.0)
    self.assertFalse(brake)

  def test_disabled_is_upstream_accel(self):
    lead = dict(dRel=30.0, vRel=-2.0, vLead=28.0)
    accel, gas, _, _ = self.run_at(55, 0.3, 0.3, lead, disable_BP_long_UI=b"1")
    self.assertEqual((accel, gas), (0.3, 0.3))

  def test_downhill_compensation_off(self):
    # -0.1 accel on a downhill: with compensation the pitch pushes it into braking; without, it doesn't
    _, _, brake, _ = self.run_at(40, -0.1, CarControllerParams.INACTIVE_GAS, pitch=-0.5)
    self.assertTrue(brake)
    self.ext = LongitudinalExt()
    _, _, brake, _ = self.run_at(40, -0.1, CarControllerParams.INACTIVE_GAS, pitch=-0.5, disable_downhill_comp_UI=b"1")
    self.assertFalse(brake)

  def test_no_gas_while_braking(self):
    _, gas, brake, _ = self.run_at(40, -1.0, 0.4)
    self.assertTrue(brake)
    self.assertEqual(gas, CarControllerParams.INACTIVE_GAS)


class TestPreviewSignals(unittest.TestCase):
  """ACCDATA's predicted-acceleration and target-speed fields (preview_signals)."""
  INACTIVE = CarControllerParams.INACTIVE_GAS
  VMAX = 145.0

  def sig(self, gas, accel, mph, long_active=True, **params):
    return preview_signals(cc_sp(**params), long_active, gas, accel, mph * MPH, self.VMAX)

  def test_off_is_upstream_constants(self):
    for gas, accel, mph in ((1.5, 1.5, 5), (self.INACTIVE, -2.0, 40), (0.2, 0.2, 70)):
      self.assertEqual(self.sig(gas, accel, mph), (self.INACTIVE, self.VMAX))

  def test_not_active_is_upstream_constants(self):
    self.assertEqual(self.sig(1.0, 1.0, 20, long_active=False, FordAccSendPrediction=b"1", FordAccSendTargetSpeed=b"1"),
                     (self.INACTIVE, self.VMAX))

  def test_prediction_is_the_gas_request(self):
    pred, v_trg = self.sig(1.5, 1.5, 5, FordAccSendPrediction=b"1")
    self.assertEqual(pred, 1.5)
    self.assertEqual(v_trg, self.VMAX)  # the other field is untouched

  def test_prediction_inactive_while_braking_or_coasting(self):
    pred, _ = self.sig(self.INACTIVE, -2.0, 40, FordAccSendPrediction=b"1")
    self.assertEqual(pred, self.INACTIVE)

  def test_prediction_stays_inside_the_panda_gas_limits(self):
    for gas in (CarControllerParams.MIN_GAS, 0.0, 1.0, CarControllerParams.ACCEL_MAX):
      pred, _ = self.sig(gas, gas, 10, FordAccSendPrediction=b"1")
      self.assertTrue(-0.5 <= pred <= 2.0)

  def test_target_speed_leads_the_car_when_accelerating(self):
    pred, v_trg = self.sig(1.5, 1.5, 5, FordAccSendTargetSpeed=b"1")
    self.assertEqual(pred, self.INACTIVE)
    self.assertAlmostEqual(v_trg, (5 * MPH + 1.5 * 2.0) * 3.6, places=4)

  def test_target_speed_is_the_car_speed_when_steady(self):
    _, v_trg = self.sig(0.0, 0.0, 60, FordAccSendTargetSpeed=b"1")
    self.assertAlmostEqual(v_trg, 60 * MPH * 3.6, places=4)

  def test_target_speed_trails_the_car_when_braking_and_stops_at_zero(self):
    _, v_trg = self.sig(self.INACTIVE, -2.0, 40, FordAccSendTargetSpeed=b"1")
    self.assertAlmostEqual(v_trg, (40 * MPH - 2.0 * 2.0) * 3.6, places=4)
    _, v_trg = self.sig(self.INACTIVE, -2.0, 2, FordAccSendTargetSpeed=b"1")
    self.assertEqual(v_trg, 0.0)

  def test_target_speed_ignores_a_positive_brake_path_accel_while_gas_is_inactive(self):
    _, v_trg = self.sig(self.INACTIVE, 0.2, 30, FordAccSendTargetSpeed=b"1")
    self.assertAlmostEqual(v_trg, 30 * MPH * 3.6, places=4)

  def test_target_speed_never_exceeds_the_maximum(self):
    _, v_trg = self.sig(2.0, 2.0, 95, FordAccSendTargetSpeed=b"1")
    self.assertEqual(v_trg, self.VMAX)


if __name__ == '__main__':
  unittest.main()
