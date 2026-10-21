"""Writes a DBC file (SavvyCAN, BUSMASTER, CANdb++, cantools) for the ESC's ODrive-compatible CAN protocol.

    python can/make_dbc.py                 -> can/esc_odrive_node0.dbc
    python can/make_dbc.py 0 1 2 3         -> can/esc_odrive_nodes_0_1_2_3.dbc (message names get a _N<node> suffix)

Standard frames: ODrive CANSimple, ID = node_id << 5 | cmd. Extra frames of this ESC: 29-bit, 0x10000000 | node_id << 8 | index.
Keep in sync with Core/Src/esc_can_odrive.c.
"""
import os
import sys

EXT_BASE = 0x10000000

# (cmd, name, dlc, sender, [(signal, start, length, signed, factor, unit, is_float)])
F, I, U = "f", "i", "u"
STD = [
    (0x00, "Get_Version", 8, "ESC", [("Protocol_Version", 0, 8, U, 1, ""), ("Hw_Version_Major", 8, 8, U, 1, ""),
                                     ("Hw_Version_Minor", 16, 8, U, 1, ""), ("Hw_Version_Variant", 24, 8, U, 1, ""),
                                     ("Fw_Version_Major", 32, 8, U, 1, ""), ("Fw_Version_Minor", 40, 8, U, 1, ""),
                                     ("Fw_Version_Revision", 48, 8, U, 1, ""), ("Fw_Version_Unreleased", 56, 8, U, 1, "")]),
    (0x01, "Heartbeat", 8, "ESC", [("Axis_Error", 0, 32, U, 1, ""), ("Axis_State", 32, 8, U, 1, ""),
                                   ("Procedure_Result", 40, 8, U, 1, ""), ("Trajectory_Done_Flag", 48, 1, U, 1, "")]),
    (0x02, "Estop", 0, "Master", []),
    (0x03, "Get_Error", 8, "ESC", [("Active_Errors", 0, 32, U, 1, ""), ("Disarm_Reason", 32, 32, U, 1, "")]),
    (0x06, "Set_Axis_Node_ID", 4, "Master", [("Axis_Node_ID", 0, 32, U, 1, "")]),
    (0x07, "Set_Axis_State", 4, "Master", [("Axis_Requested_State", 0, 32, U, 1, "")]),
    (0x09, "Get_Encoder_Estimates", 8, "ESC", [("Pos_Estimate", 0, 32, F, 1, "rev"), ("Vel_Estimate", 32, 32, F, 1, "rev/s")]),
    (0x0B, "Set_Controller_Mode", 8, "Master", [("Control_Mode", 0, 32, U, 1, ""), ("Input_Mode", 32, 32, U, 1, "")]),
    (0x0C, "Set_Input_Pos", 8, "Master", [("Input_Pos", 0, 32, F, 1, "rev"), ("Vel_FF", 32, 16, I, 0.001, "rev/s"),
                                         ("Torque_FF", 48, 16, I, 0.001, "Nm")]),
    (0x0D, "Set_Input_Vel", 8, "Master", [("Input_Vel", 0, 32, F, 1, "rev/s"), ("Input_Torque_FF", 32, 32, F, 1, "Nm")]),
    (0x0E, "Set_Input_Torque", 4, "Master", [("Input_Torque", 0, 32, F, 1, "Nm")]),
    (0x0F, "Set_Limits", 8, "Master", [("Velocity_Limit", 0, 32, F, 1, "rev/s"), ("Current_Limit", 32, 32, F, 1, "A")]),
    (0x11, "Set_Traj_Vel_Limit", 4, "Master", [("Traj_Vel_Limit", 0, 32, F, 1, "rev/s")]),
    (0x12, "Set_Traj_Accel_Limits", 8, "Master", [("Traj_Accel_Limit", 0, 32, F, 1, "rev/s^2"),
                                                  ("Traj_Decel_Limit", 32, 32, F, 1, "rev/s^2")]),
    (0x13, "Set_Traj_Inertia", 4, "Master", [("Traj_Inertia", 0, 32, F, 1, "Nm/(rev/s^2)")]),
    (0x14, "Get_Iq", 8, "ESC", [("Iq_Setpoint", 0, 32, F, 1, "A"), ("Iq_Measured", 32, 32, F, 1, "A")]),
    (0x15, "Get_Temperature", 8, "ESC", [("FET_Temperature", 0, 32, F, 1, "degC"), ("Motor_Temperature", 32, 32, F, 1, "degC")]),
    (0x16, "Reboot", 1, "Master", [("Action", 0, 8, U, 1, "")]),
    (0x17, "Get_Bus_Voltage_Current", 8, "ESC", [("Bus_Voltage", 0, 32, F, 1, "V"), ("Bus_Current", 32, 32, F, 1, "A")]),
    (0x18, "Clear_Errors", 1, "Master", [("Identify", 0, 8, U, 1, "")]),
    (0x19, "Set_Absolute_Position", 4, "Master", [("Position", 0, 32, F, 1, "rev")]),
    (0x1A, "Set_Pos_Gain", 4, "Master", [("Pos_Gain", 0, 32, F, 1, "(rev/s)/rev")]),
    (0x1B, "Set_Vel_Gains", 8, "Master", [("Vel_Gain", 0, 32, F, 1, "Nm/(rev/s)"), ("Vel_Integrator_Gain", 32, 32, F, 1, "Nm/rev")]),
    (0x1C, "Get_Torques", 8, "ESC", [("Torque_Target", 0, 32, F, 1, "Nm"), ("Torque_Estimate", 32, 32, F, 1, "Nm")]),
    (0x1D, "Get_Powers", 8, "ESC", [("Electrical_Power", 0, 32, F, 1, "W"), ("Mechanical_Power", 32, 32, F, 1, "W")]),
]
EXT = [
    (0, "ESC_Phase", 8, "ESC", [("Ia", 0, 16, I, 0.01, "A"), ("Ib", 16, 16, I, 0.01, "A"), ("Ic", 32, 16, I, 0.01, "A"),
                                ("ESC_State", 48, 8, U, 1, ""), ("ESC_Fault", 56, 8, U, 1, "")]),
    (1, "ESC_DQ", 8, "ESC", [("Id", 0, 16, I, 0.01, "A"), ("Iq", 16, 16, I, 0.01, "A"), ("Vd", 32, 16, I, 0.01, "V"),
                             ("Vq", 48, 16, I, 0.01, "V")]),
    (2, "ESC_Motion", 8, "ESC", [("Speed", 0, 32, F, 1, "rpm"), ("Speed_Cmd", 32, 32, F, 1, "rpm")]),
    (3, "ESC_Supply", 8, "ESC", [("Vbus", 0, 16, U, 0.001, "V"), ("Ibus_Est", 16, 16, I, 0.01, "A"),
                                 ("T_Driver", 32, 16, I, 0.1, "degC"), ("T_Motor", 48, 16, I, 0.1, "degC")]),
    (4, "ESC_Diag", 8, "ESC", [("ISR_Max", 0, 16, U, 0.1, "us"), ("Rejected_Samples", 16, 16, U, 1, ""),
                               ("Late_Samples", 32, 16, U, 1, ""), ("Owner", 48, 8, U, 1, ""), ("Mode", 56, 8, U, 1, "")]),
]
VALS = {
    "Axis_State": "1 \"IDLE\" 3 \"FULL_CALIBRATION_SEQUENCE\" 8 \"CLOSED_LOOP_CONTROL\" 9 \"LOCKIN_SPIN\"",
    "Axis_Requested_State": "1 \"IDLE\" 3 \"FULL_CALIBRATION_SEQUENCE\" 4 \"MOTOR_CALIBRATION\" 7 \"ENCODER_OFFSET_CALIBRATION\" "
                            "8 \"CLOSED_LOOP_CONTROL\" 9 \"LOCKIN_SPIN\"",
    "Control_Mode": "0 \"VOLTAGE_CONTROL\" 1 \"TORQUE_CONTROL\" 2 \"VELOCITY_CONTROL\" 3 \"POSITION_CONTROL\"",
    "Input_Mode": "0 \"INACTIVE\" 1 \"PASSTHROUGH\" 2 \"VEL_RAMP\" 3 \"POS_FILTER\" 5 \"TRAP_TRAJ\" 6 \"TORQUE_RAMP\"",
    "Procedure_Result": "0 \"SUCCESS\" 1 \"BUSY\" 2 \"CANCELLED\" 3 \"DISARMED\" 5 \"POLE_PAIR_CPR_MISMATCH\" "
                        "6 \"PHASE_RESISTANCE_OUT_OF_RANGE\" 7 \"PHASE_INDUCTANCE_OUT_OF_RANGE\" 15 \"NOT_CONVERGING\"",
    "Action": "0 \"REBOOT\" 1 \"SAVE_CONFIGURATION\"",
    "ESC_State": "0 \"IDLE\" 1 \"PRECHARGE\" 2 \"ALIGN\" 3 \"RUN\" 4 \"FAULT\" 5 \"CALIB\"",
    "ESC_Fault": "0 \"none\" 1 \"overcurrent\" 2 \"overvoltage\" 3 \"undervoltage\" 4 \"overtemp\" 5 \"encoder\" 6 \"overspeed\" "
                 "7 \"estop\" 8 \"CAN_timeout\"",
    "Owner": "0 \"none\" 1 \"USB\" 2 \"CAN\"",
    "Mode": "0 \"open_loop\" 1 \"torque\" 2 \"speed\" 3 \"position\"",
}


def dbc(nodes):
    sfx = (lambda n: "_N%d" % n) if len(nodes) > 1 else (lambda n: "")
    out = ['VERSION ""', "", "NS_ :", "    SIG_VALTYPE_", "    VAL_", "", "BS_:", "", "BU_: ESC Master", ""]
    floats, vals = [], []
    for n in nodes:
        msgs = [((n << 5) | c, nm, dlc, snd, sg) for c, nm, dlc, snd, sg in STD]
        msgs += [((EXT_BASE | (n << 8) | i) | 0x80000000, nm, dlc, snd, sg) for i, nm, dlc, snd, sg in EXT]
        for mid, nm, dlc, snd, sigs in msgs:
            rcv = "Master" if snd == "ESC" else "ESC"
            out.append("BO_ %d %s%s: %d %s" % (mid, nm, sfx(n), dlc, snd))
            for s, start, ln, kind, fac, unit in sigs:
                sign = "-" if kind in (F, I) else "+"
                out.append(' SG_ %s : %d|%d@1%s (%g,0) [0|0] "%s" %s' % (s, start, ln, sign, fac, unit, rcv))
                if kind == F:
                    floats.append("SIG_VALTYPE_ %d %s : 1;" % (mid, s))
                if s in VALS:
                    vals.append("VAL_ %d %s %s ;" % (mid, s, VALS[s]))
            out.append("")
    out.append('CM_ "ESC with ODrive CANSimple protocol (v0.6 layout). Extra 29-bit frames 0x10000000 | node << 8 | index. '
               'Generated by can/make_dbc.py";')
    out += floats + vals
    return "\n".join(out) + "\n"


def main():
    nodes = [int(a) for a in sys.argv[1:]] or [0]
    here = os.path.dirname(os.path.abspath(__file__))
    name = "esc_odrive_node%d.dbc" % nodes[0] if len(nodes) == 1 else "esc_odrive_nodes_%s.dbc" % "_".join(map(str, nodes))
    path = os.path.join(here, name)
    with open(path, "w", newline="\n") as f:
        f.write(dbc(nodes))
    print("written", path)


if __name__ == "__main__":
    main()
