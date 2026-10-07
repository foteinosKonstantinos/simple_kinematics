from controller import Robot

class IRB460040_2F140:

    def __init__(self,tolerance:float=1e-3):
        self.tolerance = tolerance
        self.robot = Robot()
        self.timestep =  int(self.robot.getBasicTimeStep())
        self.devices = dict()
        self.current_goal:dict|None = None
        for code in ['A','B','C','E','F']:
            self.devices[code] = {
                "motor": self.robot.getDevice(f"{code} motor"),
                "sensor": self.robot.getDevice(f"{code} sensor")
            }
            self.devices[code]["sensor"].enable(self.timestep)
        self.devices["finger"] = {
            "motor": self.robot.getDevice("ROBOTIQ 2F-140 Gripper::left finger joint"),
            "sensor": self.robot.getDevice("ROBOTIQ 2F-140 Gripper left finger joint sensor")
        }
        self.devices["finger"]["sensor"].enable(self.timestep)
        self.tmp_finger_position = None

    def get_device_names(self) -> list[str]:
        return list(self.devices.keys())

    def goal_set_general(self, thetas:dict, velocities:dict|None=None) -> None:
        if self.current_goal is not None:
            print(f"Warning: overriding current goal ({self.current_goal})")
        for code in thetas.keys():
            self.devices[code]["motor"].setPosition(thetas[code])
            if velocities is not None and velocities[code] is not None:
                self.devices[code]["motor"].setVelocity(velocities[code])
        self.current_goal = thetas

    def goal_set_pose(self, a:float, b:float, c:float, e:float ,f:float, velocity:float|None=None) -> None:
        self.goal_set_general({'A':a,'B':b,'C':c,'E':e,'F':f}, {'A':velocity,'B':velocity,'C':velocity,'E':velocity,'F':velocity} if velocity is not None else None)

    def goal_open_gripper(self) -> None:
        self.goal_set_general({"finger":0})

    def goal_close_gripper(self) -> None:
        self.goal_set_general({"finger":0.7})

    def goal_is_achieved(self) -> bool:
        if self.current_goal is None:
            return True
        for code in self.current_goal.keys():
            if code == "finger":
                # object was grasped <=> fingers don't move
                current = self.devices[code]["sensor"].getValue()
                if self.tmp_finger_position is not None and abs(current-self.tmp_finger_position) <= self.tolerance:
                    continue
                else:
                    self.tmp_finger_position = current
            if abs(self.devices[code]["sensor"].getValue() - self.current_goal[code]) > self.tolerance:
                return False
        return True

    def clear_state(self):
        self.current_goal = None
        self.tmp_finger_position = None

    def execute(self, subgoals:list):
        idx = -1
        while self.robot.step(self.timestep) != -1:
            if self.goal_is_achieved() and idx+1 < len(subgoals):
                self.clear_state()
                idx += 1
                print(f"Goal: '{subgoals[idx][0]}'")
                if subgoals[idx][1] == "pose":
                    self.goal_set_pose(*subgoals[idx][2])
                elif subgoals[idx][2] == "close":
                    self.goal_close_gripper()
                else:
                    self.goal_open_gripper()

robot_wrapper = IRB460040_2F140()
print("TODO: embed kinematics into the class or connect with TCP")
robot_wrapper.execute([
    ("Open gripper","gripper", "open"),
    ("Approach object", "pose", (-0.16903359511849014, 0.31858235148213515, 0.006589765122541549, 1.245623675113349, -0.954431606310316, 0.2)),
    ("Grasp object (1)", "pose", (-0.16903359488955466, 0.36909373797845635, 0.1109240093950825, 1.090778323358614, -0.9544315991262101, 0.2)),
    ("Grasp object (2)", "gripper", "close"),
    ("Throw object (1)", "pose", (0.4252050140314512, 0.2744462234275038, -0.2516296482853479, 1.547978602669879, 0.4252050077824021, 0.2)),
    ("Throw object (2)", "gripper", "open"),
    ("Return to zero configuration", "pose", (0, 0, 0, 0, 0, 0.2)),
])