"""BluePilot (bp-full): Ford ICBM (Intelligent Cruise Button Management), from BluePilot 7.

While openpilot asks for a set-speed change (CC_SP.intelligentCruiseButtonManagement.sendButton),
press Set+ or Set- on Steering_Data_FD1 on the camera and main buses, the way cancel/resume are sent.
Upstream turns ICBM off with openpilot longitudinal; the host may keep it on there for Ford, where the PCM
still owns the cruise setpoint and takes these presses. The panda passes them only while controls are allowed.

Called from CarController.update() in opendbc/car/ford/carcontroller.py.
"""
from opendbc.car import structs
from opendbc.car.can_definitions import CanData
from opendbc.car.ford import fordcan

SendButtonState = structs.IntelligentCruiseButtonManagement.SendButtonState

# the Steering_Data_FD1 signal each request presses
BUTTON_SIGNALS = {
  SendButtonState.increase: "CcAslButtnSetIncPress",
  SendButtonState.decrease: "CcAslButtnSetDecPress",
}
ICBM_STEP = 6  # frames between presses (about 17 Hz): BP7's (frame - last press) * DT_CTRL > 0.05
STEERING_DATA_FD1 = 0x083
SET_BUTTONS_BYTE, SET_BUTTONS_MASK = 3, 0x18  # CcAslButtnSetIncPress (bit 27) and CcAslButtnSetDecPress (bit 28)


def drop_held_set_buttons(can_sends: list[CanData]) -> list[CanData]:
  """Clear Set+/Set- in openpilot's own cancel/resume/TJA frames.

  Those frames copy whatever the driver is pressing. The driver's own frame is already on the bus, and the
  panda rejects a frame carrying Set+/Set- while controls aren't allowed, which would block a cancel sent
  while the driver holds one of them.
  """
  out = []
  for msg in can_sends:
    address, dat, bus = msg
    if address == STEERING_DATA_FD1 and len(dat) > SET_BUTTONS_BYTE and dat[SET_BUTTONS_BYTE] & SET_BUTTONS_MASK:
      dat = bytearray(dat)
      dat[SET_BUTTONS_BYTE] &= ~SET_BUTTONS_MASK & 0xFF
      msg = CanData(address, bytes(dat), bus)
    out.append(msg)
  return out


class IcbmExt:
  def __init__(self):
    self.last_button_frame = 0

  def update(self, CC_SP, CS, packer, CAN, frame: int, allowed: bool = True) -> list[CanData]:
    # allowed: cruise engaged on both sides and no cancel/resume going out. The request comes from the host a
    # frame or more behind, so it can be stale across a disengage.
    button = CC_SP.intelligentCruiseButtonManagement.sendButton
    if not allowed or button == SendButtonState.none or frame - self.last_button_frame < ICBM_STEP:
      return []

    # upstream's builder passes the stock values through: hand it the stock values with the button pressed
    # only the requested press goes out, not a Set+/Set- the driver may be holding
    stock = {**CS.buttons_stock_values, **dict.fromkeys(BUTTON_SIGNALS.values(), 0), BUTTON_SIGNALS[button]: 1}
    self.last_button_frame = frame
    return [fordcan.create_button_msg(packer, CAN.camera, stock),
            fordcan.create_button_msg(packer, CAN.main, stock)]
