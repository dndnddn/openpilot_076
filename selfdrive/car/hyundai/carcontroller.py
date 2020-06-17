from cereal import car, log
from selfdrive.car import apply_std_steer_torque_limits
from selfdrive.car.hyundai.hyundaican import create_lkas11, create_clu11, create_lfa_mfa, create_mdps12
from selfdrive.car.hyundai.values import Buttons, SteerLimitParams, CAR
from opendbc.can.packer import CANPacker
from selfdrive.config import Conversions as CV
from selfdrive.car.hyundai.spdcontroller  import SpdController
import common.log as trace1
import common.CTime1000 as tm

VisualAlert = car.CarControl.HUDControl.VisualAlert
LaneChangeState = log.PathPlan.LaneChangeState




class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.apply_steer_last = 0
    self.car_fingerprint = CP.carFingerprint
    self.packer = CANPacker(dbc_name)
    self.steer_rate_limited = False
    self.resume_cnt = 0
    self.lkas11_cnt = 0
    self.last_resume_frame = 0
    self.last_lead_distance = 0

    self.longcontrol = False

    self.steer_torque_over_timer = 0
    self.steer_torque_ratio = 1
    self.steer_torque_ratio_dir = 1


    self.timer1 = tm.CTime1000("time")

    # hud
    self.hud_timer_left = 0
    self.hud_timer_right = 0

  def limit_ctrl(self, value, limit, offset ):
      p_limit = offset + limit
      m_limit = offset - limit
      if value > p_limit:
          value = p_limit
      elif  value < m_limit:
          value = m_limit
      return value


  def process_hud_alert(self, enabled, visual_alert, left_lane, right_lane):
    sys_warning = (visual_alert == VisualAlert.steerRequired)

    if left_lane:
      self.hud_timer_left = 100

    if right_lane:
      self.hud_timer_right = 100

    if self.hud_timer_left:
      self.hud_timer_left -= 1
 
    if self.hud_timer_right:
      self.hud_timer_right -= 1


    # initialize to no line visible
    sys_state = 1
    if self.hud_timer_left and self.hud_timer_right or sys_warning:  # HUD alert only display when LKAS status is active
      if (self.steer_torque_ratio > 0.5) and (enabled or sys_warning):
        sys_state = 3
      else:
        sys_state = 4
    elif self.hud_timer_left:
      sys_state = 5
    elif self.hud_timer_right:
      sys_state = 6

    return sys_warning, sys_state


  def update(self, CC, CS, frame,  sm ):
    enabled = CC.enabled
    actuators = CC.actuators
    pcm_cancel_cmd = CC.cruiseControl.cancel
    visual_alert = CC.hudControl.visualAlert
    left_lane = CC.hudControl.leftLaneVisible
    right_lane = CC.hudControl.rightLaneVisible
    path_plan = sm['pathPlan']


    abs_angle_steers =  abs(actuators.steerAngle)
    v_ego_kph = CS.out.vEgo * CV.MS_TO_KPH

    # Steering Torque
    param = SteerLimitParams()

    if abs_angle_steers < 1 or v_ego_kph < 5:
        param.STEER_DELTA_UP  = 2
        param.STEER_DELTA_DOWN = 3



    new_steer = actuators.steer * param.STEER_MAX
    apply_steer = apply_std_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, param)
    self.steer_rate_limited = new_steer != apply_steer



    # streer over check
    if v_ego_kph > 5 and abs( CS.out.steeringTorque ) > 180:  #사용자 핸들 토크
      self.steer_torque_over_timer = 200

    if path_plan.laneChangeState != LaneChangeState.off:
      self.steer_torque_over_timer = 0
    elif self.steer_torque_over_timer:
      self.steer_torque_over_timer -= 1
 
    if self.steer_torque_over_timer:  #or CS.out.steerWarning:
      self.steer_torque_ratio_dir = -1
    else:
      self.steer_torque_ratio_dir = 1

    # smoth torque enable or disable
    if self.steer_torque_ratio_dir >= 1:
      if self.steer_torque_ratio < 1:
        self.steer_torque_ratio += 0.001
    elif self.steer_torque_ratio_dir <= -1:
      if self.steer_torque_ratio > 0:
        self.steer_torque_ratio -= 0.001
    else:
      self.steer_torque_ratio = 1

    apply_steer_limit = param.STEER_MAX
    if not left_lane  and not right_lane:
      apply_steer *= 0.5
      if self.steer_torque_ratio > 0.5:
        self.steer_torque_ratio = 0.5
    elif self.steer_torque_ratio < 1:
      apply_steer_limit = int(self.steer_torque_ratio * param.STEER_MAX)
      apply_steer = self.limit_ctrl( apply_steer, apply_steer_limit, 0 )



    # disable if steer angle reach 90 deg, otherwise mdps fault in some models
    lkas_active = enabled and abs(CS.out.steeringAngle) < 90. #and self.lkas_button

    # fix for Genesis hard fault at low speed
    if CS.out.vEgo < 16.7 and self.car_fingerprint == CAR.HYUNDAI_GENESIS:
      lkas_active = 0

    if not lkas_active:
      apply_steer = 0

    steer_req = 1 if apply_steer else 0

    self.apply_steer_last = apply_steer

    sys_warning, sys_state = self.process_hud_alert( lkas_active, visual_alert, left_lane, right_lane )



    can_sends = []
    if frame == 0: # initialize counts from last received count signals
      self.lkas11_cnt = CS.lkas11["CF_Lkas_MsgCount"] + 1

    self.lkas11_cnt %= 0x10

    can_sends.append(create_lkas11(self.packer, self.lkas11_cnt, self.car_fingerprint, apply_steer, steer_req,
                                   CS.lkas11, sys_warning, sys_state, enabled,
                                   left_lane, right_lane  ))

    #if not CS.Mdps_ToiUnavail:
    can_sends.append(create_mdps12(self.packer, frame, CS.mdps12))


    dRel, yRel, vRel = SpdController.get_lead( sm )
    #dRel2, yRel2, vRel2 = SpdController.get_radarState( sm )
    

    str_log1 = 'torg:{:5.0f} C={:.1f}/{:.1f} V={:.1f}/{:.1f} '.format(  apply_steer, CS.lead_objspd, CS.lead_distance, dRel, vRel )
    str_log2 = 'limit={:.1f} LC={} tm={:.1f}'.format( apply_steer_limit, path_plan.laneChangeState, self.timer1.sampleTime()  )
    trace1.printf( '{} {}'.format( str_log1, str_log2 ) )

    str_log2 = 'U={:.0f}  LK={:.0f} steer={:5.0f} '.format( CS.Mdps_ToiUnavail, CS.lkas_button_on, CS.out.steeringTorque  )
    trace1.printf2( '{}'.format( str_log2 ) )

    if pcm_cancel_cmd:
      can_sends.append(create_clu11(self.packer, frame, CS.clu11, Buttons.CANCEL))

    elif CS.out.cruiseState.standstill:
      # run only first time when the car stopped
      if self.last_lead_distance == 0:
        # get the lead distance from the Radar
        self.last_lead_distance = CS.lead_distance
        self.resume_cnt = 0
      # when lead car starts moving, create 6 RES msgs
      elif CS.lead_distance != self.last_lead_distance and (frame - self.last_resume_frame) > 5:
        can_sends.append(create_clu11(self.packer, frame, CS.clu11, Buttons.RES_ACCEL))
        self.resume_cnt += 1
        # interval after 6 msgs
        if self.resume_cnt > 5:
          self.last_resume_frame = frame
    # reset lead distnce after the car starts moving
    elif self.last_lead_distance != 0:
      self.last_lead_distance = 0


    # 20 Hz LFA MFA message
    if frame % 5 == 0 and self.car_fingerprint in [CAR.SONATA, CAR.PALISADE]:
      can_sends.append(create_lfa_mfa(self.packer, frame, enabled))

    # counter inc
    self.lkas11_cnt += 1
    return can_sends

