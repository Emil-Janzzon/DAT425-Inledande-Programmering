from dataclasses import dataclass
import json
from typing import List, Optional, Dict
from model.operation import Operation, Transition
from predicates.state import State
import predicates.guards
import predicates.actions
from predicates.guards import AlwaysTrue, Guard, And
from predicates.guards import AlwaysFalse

@dataclass
class Model(object):
    initial_state: State
    operations: Dict[str, Operation]
    transitions: List[Transition]

    def __post_init__(self):
        ops = {o: "i" for o in self.operations}
        self.initial_state = self.initial_state.next(**ops)

g = predicates.guards.from_str
a = predicates.actions.from_str


def the_model() -> Model:

    initial_state = State(
        # control variables
        robot_run = False,   # trigger action when true. Change to false and then to true to trigger again
        robot_command = 'move_j',   # move_j, move_l, pick, place
        robot_velocity = 0.5,
        robot_acceleration = 0.3,
        robot_goal_frame = 'unknown',   # where to go with the tool tcp
        robot_tcp_frame = 'suction_cup_1', # the tool tcp to use
        gesture = 'unknown',

        bool_to_plc_1 = False,
        bool_to_plc_2 = False,
        bool_to_plc_3 = False,
        bool_to_plc_4 = False,
        bool_to_plc_5 = False,
        int_to_plc_1 = 0,
        int_to_plc_2 = 0,
        int_to_plc_3 = 0,
        int_to_plc_4 = 0,
        int_to_plc_5 = 0,

        goal_as_string = "cyl_at_pose_2",
        replan = False,
        lock_run = False,

        aruco_run = False,
        lock_done = False,

        # measured variables
        robot_state = "initial",  # "exec", "done", "failed" 
        robot_pose = 'unknown',

        replanned = False,

        bool_from_plc_1 = False,
        bool_from_plc_2 = False,
        bool_from_plc_3 = False,
        bool_from_plc_4 = False,
        bool_from_plc_5 = False,
        int_from_plc_1 = 0,
        int_from_plc_2 = 0,
        int_from_plc_3 = 0,
        int_from_plc_4 = 0,
        int_from_plc_5 = 0,

        aruco_done = False,

        #estimated
        suction_cup_1_occ = False, # If a suction cup is occupied or not
        suction_cup_2_occ = False,
        cyl_at_pose_1 = True,
        cyl_at_pose_2 = False,
        cyl_at_pose_3 = False,

        arucos_locked = False,
        trigger_goal_pos1 = False,
        trigger_goal_pos2 = False
    )

    ops = {}



    ## operations to be completed
    # move to pose_1
    ops[f"op_move_to_pose_1_N"] = Operation(

        name = f"op_move_to_pose_1_N",

        precondition = Transition("pre",
            g(f"!robot_run && robot_state == initial && robot_pose == above_pose_1_N"),
            a(f"robot_command = move_j, robot_run, robot_goal_frame = pose_1_N")),

        postcondition = Transition("post",
            g(f"robot_state == done"),
            a(f"!robot_run, robot_pose = pose_1_N")),

        effects = (),
        to_run = Transition.default()
    )

    ops[f"op_move_to_pose_2_N"] = Operation(

        name = f"op_move_to_pose_2_N",

        precondition = Transition("pre",
            g(f"!robot_run && robot_state == initial && robot_pose == above_pose_2_N"),
            a(f"robot_command = move_j, robot_run, robot_goal_frame = pose_2_N")),

        postcondition = Transition("post",
            g(f"robot_state == done"),
            a(f"!robot_run, robot_pose = pose_2_N")),

        effects = (),
        to_run = Transition.default()
    )

    ops[f"op_move_to_pose_3_N"] = Operation(

        name = f"op_move_to_pose_3_N",

        precondition = Transition("pre",
            g(f"!robot_run && robot_state == initial && robot_pose == above_pose_3_N"),
            a(f"robot_command = move_j, robot_run, robot_goal_frame = pose_3_N")),

        postcondition = Transition("post",
            g(f"robot_state == done"),
            a(f"!robot_run, robot_pose = pose_3_N")),

        effects = (),
        to_run = Transition.default()
    )

    ops[f"op_move_to_above_pose_1_N"] = Operation(

        name = f"op_move_to_above_pose_1_N",

        precondition = Transition("pre",
            g(f"!robot_run && robot_state == initial"),# && robot_pose == above_table_N
            a(f"robot_command = move_j, robot_run, robot_goal_frame = above_pose_1_N")),

        postcondition = Transition("post",
            g(f"robot_state == done"),
            a(f"!robot_run, robot_pose = above_pose_1_N")),

        effects = (),
        to_run = Transition.default()
    )
    ops[f"op_move_to_above_pose_2_N"] = Operation(

        name = f"op_move_to_above_pose_2_N",

        precondition = Transition("pre",
            g(f"!robot_run && robot_state == initial"),# && robot_pose == above_table_N
            a(f"robot_command = move_j, robot_run, robot_goal_frame = above_pose_2_N")),

        postcondition = Transition("post",
            g(f"robot_state == done"),
            a(f"!robot_run, robot_pose = above_pose_2_N")),

        effects = (),
        to_run = Transition.default()
    )

    ops[f"op_move_to_above_pose_3_N"] = Operation(

        name = f"op_move_to_above_pose_3_N",

        precondition = Transition("pre",
            g(f"!robot_run && robot_state == initial"),# && robot_pose == above_table_N
            a(f"robot_command = move_j, robot_run, robot_goal_frame = above_pose_3_N")),

        postcondition = Transition("post",
            g(f"robot_state == done"),
            a(f"!robot_run, robot_pose = above_pose_3_N")),

        effects = (),
        to_run = Transition.default()
    )
    ops[f"op_pick_suc_at_pose_1_N"] = Operation(

            name = f"op_pick_suc_at_pose_1_N",

            precondition = Transition("pre",
                g(f"!robot_run && robot_state == initial && robot_pose == pose_1_N && cyl_at_pose_1 == True && suction_cup_1_occ == False "),
                a(f"robot_command = pick, robot_run, robot_goal_frame = pose_1_N")),

            postcondition = Transition("post",
                g(f"robot_state == done"),
                a(f"!robot_run, cyl_at_pose_1 = False, suction_cup_1_occ = True")),

            effects = (),
            to_run = Transition.default()
        )
    ops[f"op_pick_suc_at_pose_2_N"] = Operation(

            name = f"op_pick_suc_at_pose_2_N",

            precondition = Transition("pre",
                g(f"!robot_run && robot_state == initial && robot_pose == pose_2_N && cyl_at_pose_2 == True && suction_cup_1_occ == False "),
                a(f"robot_command = pick, robot_run, robot_goal_frame = pose_2_N")),

            postcondition = Transition("post",
                g(f"robot_state == done"),
                a(f"!robot_run, cyl_at_pose_2 = False, suction_cup_1_occ = True")),

            effects = (),
            to_run = Transition.default()
        )
    ops[f"op_pick_suc_at_pose_3_N"] = Operation(

            name = f"op_pick_suc_at_pose_3_N",

            precondition = Transition("pre",
                g(f"!robot_run && robot_state == initial && robot_pose == pose_3_N && cyl_at_pose_3 == True && suction_cup_1_occ == False "),
                a(f"robot_command = pick, robot_run, robot_goal_frame = pose_3_N")),

            postcondition = Transition("post",
                g(f"robot_state == done"),
                a(f"!robot_run, cyl_at_pose_3 = False, suction_cup_1_occ = True")),

            effects = (),
            to_run = Transition.default()
        )
    ops[f"op_place_suc_at_pose_1_N"] = Operation(

            name = f"op_place_suc_at_pose_1_N",

            precondition = Transition("pre",
                g(f"!robot_run && robot_state == initial && robot_pose == pose_1_N && cyl_at_pose_1 == False && suction_cup_1_occ == True "),
                a(f"robot_command = place, robot_run, robot_goal_frame = pose_1_N")),

            postcondition = Transition("post",
                g(f"robot_state == done"),
                a(f"!robot_run, cyl_at_pose_1 = True, suction_cup_1_occ = False")),

            effects = (),
            to_run = Transition.default()
        )
    ops[f"op_place_suc_at_pose_2_N"] = Operation(

            name = f"op_place_suc_at_pose_2_N",

            precondition = Transition("pre",
                g(f"!robot_run && robot_state == initial && robot_pose == pose_2_N && cyl_at_pose_2 == False && suction_cup_1_occ == True "),
                a(f"robot_command = place, robot_run, robot_goal_frame = pose_2_N")),

            postcondition = Transition("post",
                g(f"robot_state == done"),
                a(f"!robot_run, cyl_at_pose_2 = True, suction_cup_1_occ = False")),

            effects = (),
            to_run = Transition.default()
        )
    ops[f"op_place_suc_at_pose_3_N"] = Operation(

            name = f"op_place_suc_at_pose_3_N",

            precondition = Transition("pre",
                g(f"!robot_run && robot_state == initial && robot_pose == pose_3_N && cyl_at_pose_3 == False && suction_cup_1_occ == True "),
                a(f"robot_command = place, robot_run, robot_goal_frame = pose_3_N")),

            postcondition = Transition("post",
                g(f"robot_state == done"),
                a(f"!robot_run, cyl_at_pose_3 = True, suction_cup_1_occ = False")),

            effects = (),
            to_run = Transition.default()
        )
    """ops[f"op_place_cyl_at_1_N"] = Operation(
                name=f"op_place_cyl_at_1",
                precondition=Transition(
                    "pre",
                    g(f"robot_pose == pose_1_N && suction_cup_1_occ == True"),
                    a(f"robot_run, robot_command = place_1_N")
                ),
                postcondition=Transition(
                    "post",
                    g(f"robot_command == done"),
                    a(f"!robot_run, cyl_at_pose_1 = ")
                ),
                effects=(),
                to_run=Transition.default()
            )"""
    # move to pose_2
    """ops[f"op_move_to_pose_1_N"] = Operation(

        name = f"op_move_to_pose_1_N",

        precondition = Transition("pre",
            g(f"!robot_run && robot_state == initial && robot_pose == above_table_N"),
            a(f"robot_command = move_j. robot_run, robot_goal_frame = pose_1_N")),

        postcondition = Transition("post",
            g(f"robot_state == done"),
            a(f"!robot_run, robot_pose <- pose_1_N")),

        effects = (),
        to_run = Transition.default()
    )"""
    # move to pose_3

    # move to above_table

    # pick at pose_1 with suction_cup_1 or suction_cup_2

    # pick at pose_2 with suction_cup_1 or suction_cup_2 ...

    # place at pose_1 with suction_cup_1 or suction_cup_2

    # place at pose_2 with suction_cup_1 or suction_cup_2 ...

    # lock aruco tags

    # ... you're encouraged to add other operations if necessary and you need to decide it!

    # To be used to run "free" transitions. 
    # Example of setting a goal
    transitions: List[Transition] = [
        Transition("trigger_goal_pos2_pre", g("trigger_goal_pos2"), a("!replan")),
        Transition("trigger_goal_pos2_post", g("trigger_goal_pos2 && !replanned"), a("!trigger_goal_pos2, replan, goal_as_string <= cyl_at_pose_2 == True")),
        
        Transition("trigger_goal_pos1_pre", g("trigger_goal_pos1"), a("!replan")),
        Transition("trigger_goal_pos1_post", g("trigger_goal_pos1 && !replanned"), a("!trigger_goal_pos1, replan, goal_as_string <= cyl_at_pose_1 == True")),
    ]



    return Model(
        initial_state,
        ops,
        transitions
    )

def from_goal_to_goal(state: State) -> Guard:
    """
    Create a goal predicate 
    """
    goal: str = state.get("goal_as_string")
    if goal != "":
        return g(goal)
    
    return AlwaysFalse()