import time, math

from geometry_msgs.msg import Point
from irobot_create_msgs.srv import ResetPose
from irobot_create_msgs.action import NavigateToPosition

from create3.utils.robot.constraints import MAX_SPEED, WHEEL_DISTANCE_APART

waypoints = [Point()]


class Positions():
    def __init__(self, positions: list[Point]):
        self._positions = positions.copy()
        self.index = 0
        self._reverse()
        self.count = len(self._positions)

    def _reverse(self):
        self._positions.reverse()
        
    def get_previous(self) -> Point:
        if self.index != 0:
            return self._positions[self.index - 1]
        return self._positions[self.index]

    def get(self) -> Point:
        return self._positions[self.index]
    
    def get_time(self, current_pos: Point) -> float:
        pos = self.get()
        x = current_pos.x - pos.x
        y = current_pos.y - pos.y
        linear_time = math.sqrt(x**2 + y**2) / (MAX_SPEED / 100)
        angular_time = math.pi / (MAX_SPEED / WHEEL_DISTANCE_APART)
        return linear_time
    
    def next(self):
        self.index += 1
    
    def not_finished(self) -> bool:
        return len(waypoints) > self.index
    
    def get_goal(self) -> NavigateToPosition.Goal:
        goal_msg = NavigateToPosition.Goal()
        goal_msg.achieve_goal_heading = False
        goal_msg.max_translation_speed = MAX_SPEED / 100
        goal_msg.max_rotation_speed = MAX_SPEED / WHEEL_DISTANCE_APART
        goal_msg.goal_pose.pose.position.z = 0.0
        goal_msg.goal_pose.pose.position.x = self._positions[self.index].x
        goal_msg.goal_pose.pose.position.y = self._positions[self.index].y
        return goal_msg

class Navigation():

    def _controls(self):
        global waypoints
        if self.joy.buttons[2]:  # Triangle PS4
            self.get_logger().info('Navigation: Navigating back to start!')

            self.rumble()
            
            nav = Positions(waypoints)
            error_dist = 0.40 # This is the radius of the bot in meters
            position = self.tf.transforms[1].transform.translation
            self.get_logger().info('Navigation: Recorded points {0}'.format(nav.count))

            while nav.not_finished():
                pos = nav.get()
                old_pos = nav.get_previous()
                old_x = old_pos.x
                old_y = old_pos.y
                
                if nav.index != 0 and is_between(old_x, pos.x, error_dist) and is_between(old_y, pos.y, error_dist):
                    continue
                elif nav.index == 0 and is_between(position.x, pos.x, error_dist) and is_between(position.y, pos.y, error_dist):
                    continue
                else:
                    self.get_logger().info('Navigating to X: {0} Y: {1}'.format(pos.x, pos.y))
                    msg = nav.get_goal()
                    future = self.navigate.send_goal_async(msg)
                    time.sleep(nav.get_time(position))
                    future.cancel()
                    
                    
                if pos.x == 0 and pos.y == 0:
                    self.get_logger().info('Navigation Finished')
                    waypoints = [Point()]
                    self.reset_pose.call_async(ResetPose.Request())
                    
                    self.rumble()
                    break
                    
                nav.next()

    def _save_nav(self):
        global waypoints
        if self.tf.transforms:
            for tf in self.tf.transforms:
                if tf.child_frame_id == 'base_footprint':
                    current_time = time.time()
                    if current_time - self.last_save > 0.5:
                        pos = Point()
                        pos.x = self.tf.transforms[1].transform.translation.x
                        pos.y = self.tf.transforms[1].transform.translation.y
                        waypoints.append(pos)
                        self.last_save = current_time
                    break

def check_sign(data):
    if data > 0:
        return 1
    else:
        return -1

def is_between(old_pos, pos, diff):
    return (old_pos - check_sign(old_pos) * diff) < pos > (old_pos + check_sign(old_pos) * diff)