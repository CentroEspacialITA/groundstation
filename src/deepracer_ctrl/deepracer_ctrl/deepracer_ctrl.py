import rclpy
import pygame
from pygame.locals import *

from deepracer_interfaces_pkg.msg import ServoCtrlMsg
from deepracer_ctrl import constants
import math

# Dimensões da tela
WIDTH = 800
HEIGHT = 600


class DeepracerCtrlNode(Node):
    def __init__(self):
        super().__init__('deepracer_ctrl_node')
        self.publisher_ = self.create_publisher(ServoCtrlMsg, '/ctrl_pkg/servo_msg', 1)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            print("Joystick name: ", self.joystick.get_name())
        
        screen = pygame.display.set_mode((WIDTH, HEIGHT))
        self.clock = pygame.time.Clock()
        font = pygame.font.Font(None, 24)

    def get_rescaled_manual_speed(categorized_throttle, max_speed_pct):
        """Return the non linearly rescaled speed value based on the max_speed_pct.

        Args:
            categorized_throttle (float): Float value ranging from -1.0 to 1.0.
            max_speed_pct (float): Float value ranging from 0.0 to 1.0 taken as input
                                from maximum speed input.
        Returns:
            float: Categorized value of the input speed.
        """
        """
        Example return values:
        categorized_throttle: -0.3 max_speed_pct: 0.1 mapped_speed: -0.1383742911153119
        categorized_throttle: -0.5 max_speed_pct: 0.1 mapped_speed: -0.224952741020794
        categorized_throttle: -0.8 max_speed_pct: 0.1 mapped_speed: -0.3463137996219282

        categorized_throttle: -0.3 max_speed_pct: 0.5 mapped_speed: -0.208
        categorized_throttle: -0.5 max_speed_pct: 0.5 mapped_speed: -0.33333333333333337
        categorized_throttle: -0.8 max_speed_pct: 0.5 mapped_speed: -0.5013333333333334

        categorized_throttle: -0.3 max_speed_pct: 1.0 mapped_speed: -0.552
        categorized_throttle: -0.5 max_speed_pct: 1.0 mapped_speed: -0.8
        categorized_throttle: -0.8 max_speed_pct: 1.0 mapped_speed: -0.992

        categorized_throttle: 0.3 max_speed_pct: 0.1 mapped_speed: 0.1383742911153119
        categorized_throttle: 0.5 max_speed_pct: 0.1 mapped_speed: 0.224952741020794
        categorized_throttle: 0.8 max_speed_pct: 0.1 mapped_speed: 0.3463137996219282

        categorized_throttle: 0.3 max_speed_pct: 0.5 mapped_speed: 0.208
        categorized_throttle: 0.5 max_speed_pct: 0.5 mapped_speed: 0.33333333333333337
        categorized_throttle: 0.8 max_speed_pct: 0.5 mapped_speed: 0.5013333333333334

        categorized_throttle: 0.3 max_speed_pct: 1.0 mapped_speed: 0.552
        categorized_throttle: 0.5 max_speed_pct: 1.0 mapped_speed: 0.8
        categorized_throttle: 0.8 max_speed_pct: 1.0 mapped_speed: 0.992
        """
        # return 0.0 if categorized_throttle or maximum speed pct is 0.0
        if categorized_throttle == 0.0 or max_speed_pct == 0.0:
            return 0.0

        # get the parameter value to calculate the coefficients a, b in the equation y=ax^2+bx
        # The lower the update_speed_scale_value parameter, higher the impact on the
        # final mapped_speed.
        # Hence the update_speed_scale_value parameter is inversely associated with max_speed_pct
        # and bounded by MANUAL_SPEED_SCALE_BOUNDS.
        # Ex: max_speed_pct = 0.5; update_speed_scale_value = 3
        #     max_speed_pct = 1.0; update_speed_scale_value = 1
        # Lower the update_speed_scale_value: categorized_throttle value gets mapped to
        # higher possible values.
        #   Example: update_speed_scale_value = 1.0;
        #            categorized_throttle = 0.8 ==> mapped_speed = 0.992
        # Higher the update_speed_scale_value: categorized_throttle value gets mapped to
        # lower possible values.
        #   Example: update_speed_scale_value = 3.0;
        #            categorized_throttle = 0.8 ==> mapped_speed = 0.501

        inverse_max_speed_pct = (1 - max_speed_pct)
        update_speed_scale_value = \
            constants.MANUAL_SPEED_SCALE_BOUNDS[0] + \
            inverse_max_speed_pct * \
            (constants.MANUAL_SPEED_SCALE_BOUNDS[1] - constants.MANUAL_SPEED_SCALE_BOUNDS[0])
        speed_mapping_coefficients = dict()

        # recreate the mapping coefficeints for the non-linear equation ax^2 + bx based on
        # the update_speed_scale_value.
        # These coefficents map the [update_speed_scale_value, update_speed_scale_value/2]
        # values to DEFAULT_SPEED_SCALE values [1.0, 0.8].
        speed_mapping_coefficients["a"] = \
            (1.0 / update_speed_scale_value**2) * \
            (2.0 * constants.DEFAULT_SPEED_SCALES[0] - 4.0 * constants.DEFAULT_SPEED_SCALES[1])
        speed_mapping_coefficients["b"] = \
            (1.0 / update_speed_scale_value) * \
            (4.0 * constants.DEFAULT_SPEED_SCALES[1] - constants.DEFAULT_SPEED_SCALES[0])
        return math.copysign(1.0, categorized_throttle) * \
            (speed_mapping_coefficients["a"] * abs(categorized_throttle)**2 +
            speed_mapping_coefficients["b"] * abs(categorized_throttle))


    def get_categorized_manual_throttle(throttle):
        """Return the value of the category in which the input throttle belongs to.

        Args:
            input (float): Float value ranging from -1.0 to 1.0 taken as input from joystick.

        Returns:
            float: Value of the category the input throttle belonged to.
        """
        if abs(throttle) >= 0.8:
            throttle = math.copysign(0.8, throttle)
        elif abs(throttle) >= 0.5:
            throttle = math.copysign(0.5, throttle)
        elif abs(throttle) > 0.0:
            throttle = math.copysign(0.3, throttle)
        return throttle


    def get_categorized_manual_angle(angle):
        """Return the value of the category in which the input angle belongs to.

        Args:
            input (float): Float value ranging from -1.0 to 1.0 taken as input from joystick.

        Returns:
            float: Value of the category the input angle belonged to.
        """
        if abs(angle) >= 0.8:
            angle = math.copysign(0.8, angle)
        elif abs(angle) >= 0.5:
            angle = math.copysign(0.5, angle)
        elif abs(angle) > 0:
            angle = math.copysign(0.3, angle)
        return angle


    def timer_callback(self):
        pygame.event.pump()
        # Send current joystick status
        print("Axis X:", self.joystick.get_axis(0))
        print("Axis Y:", self.joystick.get_axis(1))
        print("Axis Z:", self.joystick.get_axis(2))
                
        # Atualiza a tela
        pygame.display.flip()
        self.clock.tick(60)  # Limita a taxa de atualização da tela a 60 FPS


def main(args=None):
        rclpy.init(args=args)
        deepracer_ctrl_node = DeepracerCtrlNode()
        rclpy.spin(deepracer_ctrl_node)

        deepracer_ctrl_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()