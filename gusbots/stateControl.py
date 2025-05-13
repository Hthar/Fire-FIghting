import math
from gusbots import SpeedEstimator
from gusbots.controller import GoToGo


class Inputs:
    """State machine inputs"""

    autonomous = 0  # change state to autonomous mode (GoToGoal behavior)
    joy_left = 0  # value of the left joystick
    joy_right = 0  # value of the right joystick
    x = 0  # x-position of the robot
    y = 0  # y-position of the robot
    theta = 0  # robot orientation
    dt = 0  # time delta between the last execution (used by the PID)
    el = 0  # left encoder instance
    er = 0  # right endoer instance
    L = 0  # distance between the wheels


class Outputs:
    """State machine outputs"""

    left_motor = 0  # left motor output (0..1)
    right_motor = 0  # right motor ouput (0..1)


class State:
    """When creating a new state, extend this class"""

    def run(self, input, output):
        pass

    def entry(self, input, output):
        pass

    def exit(self, input, output):
        pass


######################################
# States
######################################


class InitSt(State):
    name = "Init"

    def __init__(self):
        pass

    def run(self, input, output):
        return ManualSt.name


class ManualSt(State):
    name = "Manual"

    def enter(self, input, output):
        print("Manual mode state")

    def run(self, input, output):
        next_state = ManualSt.name
        output.left_motor = 0
        output.right_motor = 0
        if input.autonomous == 1:
            next_state = GoToGoalSt.name
        return next_state


class GoToGoalSt(State):
    name = "GoToGoal"
    next_goal = 0
    array_of_goals = [[1, 0], [0, 0]]  # Square path

    def entry(self, input, output):
        self.goal = GoToGoalSt.array_of_goals[GoToGoalSt.next_goal]
        GoToGoalSt.next_goal = (GoToGoalSt.next_goal + 1) % len(
            GoToGoalSt.array_of_goals
        )
        print("GoToGoal state", self.goal, GoToGoalSt.next_goal)
        self.leftPrevCmd = 0
        self.rightPrevCmd = 0
        self.controller = GoToGo.GoToGoal()

    def limit(self, value, downLimit, upLimit):
        return max(min(value, upLimit), downLimit)

    def rateLimit(self, value, ctrlVar, upLimit, downLimit):
        ctrlVar = self.limit(value, (downLimit + ctrlVar), (upLimit + ctrlVar))
        return ctrlVar

    def run(self, input, output):
        next_state = GoToGoalSt.name

        # Run controller
        w = self.controller.step(
            self.goal[1], self.goal[0], input.x, input.y, input.theta, input.dt
        )

        # Estimate motor outputs
        left, right = SpeedEstimator.uni_to_diff(0.03, w, input.el, input.er, input.L)

        # Normalize and rate limit
        max_val = max(abs(left), abs(right))
        if max_val > 0:
            left_norm = left / max_val
            right_norm = right / max_val
        else:
            left_norm = left
            right_norm = right

        left_limited = self.rateLimit(left_norm, self.leftPrevCmd, 0.1, -0.1)
        right_limited = self.rateLimit(right_norm, self.rightPrevCmd, 0.1, -0.1)

        # Apply minimum speed and direction
        left_final = max(0.5, abs(left_limited)) * (1 if left_limited >= 0 else -1)
        right_final = max(0.5, abs(right_limited)) * (1 if right_limited >= 0 else -1)

        self.leftPrevCmd = left_final
        self.rightPrevCmd = right_final

        output.left_motor = left_final
        output.right_motor = right_final

        # Check if goal reached
        if abs(input.x - self.goal[0]) < 0.08 and abs(input.y - self.goal[1]) < 0.08:
            next_state = AtTheGoalSt.name

        return next_state


class AtTheGoalSt(State):
    name = "AtTheGoal"

    def entry(self, input, output):
        print("AtTheGoal state")
        output.left_motor = 0
        output.right_motor = 0

    def run(self, input, output):
        next_state = AtTheGoalSt.name
        output.left_motor = 0
        output.right_motor = 0
        return next_state


######################################
# Main control
######################################


class stateControl:
    def __init__(self):
        self.states = {
            InitSt.name: InitSt(),
            ManualSt.name: ManualSt(),
            GoToGoalSt.name: GoToGoalSt(),
            AtTheGoalSt.name: AtTheGoalSt(),
        }
        self.input = Inputs()
        self.output = Outputs()
        self.currentState = InitSt.name
        self.states[self.currentState].entry(self.input, self.output)

    def step(self):
        next_state = self.states[self.currentState].run(self.input, self.output)
        if next_state != self.currentState:
            self.states[self.currentState].exit(self.input, self.output)
            self.currentState = next_state
            self.states[self.currentState].entry(self.input, self.output)
