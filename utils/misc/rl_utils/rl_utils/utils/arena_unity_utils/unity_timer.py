from threading import Event

from rosgraph_msgs.msg import Clock
import rospy


class UnityTimer:
    def __init__(
        self,
        update_duration: float,
        start_time: rospy.Time,
        clock_topic: str = "clock"
    ):
        """Initializes Arena Unity oriented timer.

        Args:
            update_duration (float): Update duration for the timer in seconds.
            start_time (rospy.Time): Starting time received from Arena Unity.
            clock_topic (str): Name of topic where Arena Unity clock is 
                published to.
        """
        self._current_time = start_time
        self._update_duration = update_duration
        self._update_offset = rospy.Duration(secs=self._update_duration, nsecs=0)
        self._next_update = self._current_time + self._update_offset
        self._clock_subscrition = rospy.Subscriber(clock_topic, Clock, self._clock_callback)

        self._event = Event()
        self._waiting = False

    def _clock_callback(self, clock_msg: Clock):
        self._current_time = rospy.Time(clock_msg.clock.secs, clock_msg.clock.nsecs)

        if self._waiting and self._current_time >= self._next_update:
            self._waiting = False

            # update next update time
            if self._current_time >= self._next_update + self._update_offset:
                rospy.logwarn(f"Training loop missed rate of {1.0 / self._update_duration} Hz. Missed update at {self._next_update} nsecs and rescheduling next update to {self._current_time + self._update_offset} nsecs")
                self._next_update = self._current_time + self._update_offset
            else:
                self._next_update = self._next_update + self._update_offset

            self._event.set()
            self._event.clear()

    def wait_for_next_update(self):
        """Waits until the unity clock has passed one update interval since the last one.
        """
        self._waiting = True
        success = self._event.wait(timeout=10)
        
        if not success:
            raise RuntimeError("Timeout of 10 seconds reached, when waiting for Unity to make a step!")
