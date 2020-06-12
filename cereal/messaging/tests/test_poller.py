import unittest
import time
import cereal.messaging as messaging

import concurrent.futures


def poller():
  context = messaging.Context()

  p = messaging.Poller()

  sub = messaging.SubSocket()
  sub.connect(context, 'controlsState')
  p.registerSocket(sub)

  socks = p.poll(10000)
  r = [s.receive(non_blocking=True) for s in socks]

  return r


class TestPoller(unittest.TestCase):
  def test_poll_once(self):
    context = messaging.Context()

    pub = messaging.PubSocket()
    pub.connect(context, 'controlsState')

    with concurrent.futures.ThreadPoolExecutor() as e:
      poll = e.submit(poller)

      time.sleep(0.1)  # Slow joiner syndrome

      # Send message
      pub.send("a")

      # Wait for poll result
      result = poll.result()

    del pub
    context.term()

    self.assertEqual(result, [b"a"])

  def test_poll_and_create_many_subscribers(self):
    context = messaging.Context()

    pub = messaging.PubSocket()
    pub.connect(context, 'controlsState')

    with concurrent.futures.ThreadPoolExecutor() as e:
      poll = e.submit(poller)

      time.sleep(0.1)  # Slow joiner syndrome
      c = messaging.Context()
      for _ in range(10):
        messaging.SubSocket().connect(c, 'controlsState')

      time.sleep(0.1)

      # Send message
      pub.send("a")

      # Wait for poll result
      result = poll.result()

    del pub
    context.term()

    self.assertEqual(result, [b"a"])

  def test_multiple_publishers_exception(self):
    context = messaging.Context()

    with self.assertRaises(messaging.MultiplePublishersError):
      pub1 = messaging.PubSocket()
      pub1.connect(context, 'controlsState')

      pub2 = messaging.PubSocket()
      pub2.connect(context, 'controlsState')

      pub1.send("a")

    del pub1
    del pub2
    context.term()

  def test_multiple_messages(self):
    context = messaging.Context()

    pub = messaging.PubSocket()
    pub.connect(context, 'controlsState')

    sub = messaging.SubSocket()
    sub.connect(context, 'controlsState')

    time.sleep(0.1)  # Slow joiner

    for i in range(100):
      pub.send(str(i))

    msg_seen = False
    i = 0
    while True:
      r = sub.receive(non_blocking=True)

      if r is not None:
        self.assertEqual(str(i), r.decode('utf8'))

        msg_seen = True
        i += 1

      if r is None and msg_seen:  # ZMQ sometimes receives nothing on the first receive
        break

    del pub
    del sub
    context.term()

  def test_msg(self):
    # controlsState
    dat = messaging.new_message('controlsState')
    dat.valid = 1
    controlsState = dat.controlsState
    controlsState.alertText1 = self.AM.alert_text_1
    controlsState.alertText2 = self.AM.alert_text_2
    controlsState.alertSize = self.AM.alert_size
    controlsState.alertStatus = self.AM.alert_status
    controlsState.alertBlinkingRate = self.AM.alert_rate
    controlsState.alertType = self.AM.alert_type
    controlsState.alertSound = self.AM.audible_alert
    controlsState.driverMonitoringOn = self.sm['dMonitoringState'].faceDetected
    #controlsState.canMonoTimes = list(CS.canMonoTimes)
    #controlsState.planMonoTime = self.sm.logMonoTime['plan']
    #controlsState.pathPlanMonoTime = self.sm.logMonoTime['pathPlan']
    controlsState.enabled = 1
    controlsState.active = 1
    controlsState.vEgo = 1
    controlsState.vEgoRaw = 1
    controlsState.angleSteers = 1
    controlsState.curvature = 1
    controlsState.steerOverride =1
    controlsState.state = 1
    controlsState.engageable = 1
    controlsState.longControlState = 1
    #controlsState.vPid = float(self.LoC.v_pid)
    #controlsState.vCruise = float(self.v_cruise_kph)
    #controlsState.upAccelCmd = float(self.LoC.pid.p)
    #controlsState.uiAccelCmd = float(self.LoC.pid.i)
    #controlsState.ufAccelCmd = float(self.LoC.pid.f)
    controlsState.angleSteersDes = 1
    controlsState.vTargetLead = 1
    controlsState.aTarget = 2
    #controlsState.jerkFactor = float(self.sm['plan'].jerkFactor)
    #controlsState.gpsPlannerActive = self.sm['plan'].gpsPlannerActive
    #controlsState.vCurvature = self.sm['plan'].vCurvature
    #controlsState.decelForModel = self.sm['plan'].longitudinalPlanSource == LongitudinalPlanSource.model
    #controlsState.cumLagMs = -self.rk.remaining * 1000.
    #controlsState.startMonoTime = int(start_time * 1e9)
    #controlsState.mapValid = self.sm['plan'].mapValid
    #controlsState.forceDecel = bool(force_decel)
    ##controlsState.canErrorCounter = self.can_error_counter
    #controlsState.alertTextMsg1 = str(trace1.global_alertTextMsg1)
    #controlsState.alertTextMsg2 = str(trace1.global_alertTextMsg2)

    self.pm.send('controlsState', dat)

  def test_conflate(self):
    context = messaging.Context()

    pub = messaging.PubSocket()
    pub.connect(context, 'controlsState')

    sub = messaging.SubSocket()
    sub.connect(context, 'controlsState', conflate=True)

    time.sleep(0.1)  # Slow joiner
    pub.send('a')
    pub.send('b')

    self.assertEqual(b'b', sub.receive())

    del pub
    del sub
    context.term()





if __name__ == "__main__":
  msg = TestPoller
  msg.test_msg()
  #unittest.main()
