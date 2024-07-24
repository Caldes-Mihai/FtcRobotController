package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

@Config
public class ConstantValues {
    public static boolean INVERT_LEFT_FRONT = true;
    public static boolean INVERT_RIGHT_FRONT = false;
    public static boolean INVERT_LEFT_BACK = true;
    public static boolean INVERT_RIGHT_BACK = false;
    public static boolean INVERT_SLIDER1 = false;
    public static boolean INVERT_SLIDER2 = true;
    public static boolean INVERT_SLIDER1_SERVO = false;
    public static boolean INVERT_SLIDER2_SERVO = true;
    public static boolean INVERT_CLAW1 = true;
    public static boolean INVERT_CLAW2 = false;
    public static boolean INVERT_CLAW_WRIST_HORIZONTAL = false;
    public static boolean INVERT_CLAW_WRIST_VERTICAL = false;
    public static double CLAW1_HOLD_POS = 0.12;
    public static double CLAW2_HOLD_POS = 0.65;
    public static double CLAW1_RELEASE_POS = 0.4;
    public static double CLAW2_RELEASE_POS = 0.9;
    public static double CLAW_WRIST_HORIZONTAL = 0.47;
    public static double CLAW_WRIST_LEFT_DIAGONAL = 0.65;
    public static double CLAW_WRIST_VERTICAL = 0.12;
    public static double CLAW_WRIST_RIGHT_DIAGONAL = 0.295;
    public static double RETRACTED_SLIDERS_POS = 50;
    public static double RETRACTED_SLIDER_SERVO_POS = 0.5;

    public static double RETRACTED_CLAW_WRIST_VERTICAL_SERVO_POS = 0.1;

    public static double PICKUP_SLIDER_SERVO_POS = 0.45;
    public static double EXTENDED_SLIDERS_POS = 3200;
    public static double EXTENDED_SLIDER_SERVO_POS = 0;
    public static double EXTENDED_CLAW_WRIST_VERTICAL_SERVO_POS = 0.75;

    public static double SLIDERS_THRESHOLD = 200;
    public static double INTAKE_MAX_SPEED = 1;
    public static double INTAKE_MAX_SLOW_SPEED = 0.3;
    public static double INTAKE_SERVO_RETRACT_POS = 0.7;
    public static double INTAKE_SERVO_EXTEND_POS = 0.7;
    public static double DRONE_START = 0.82;
    public static double DRONE_END = 0.5;
    public static double NORMAL_SPEED = 1;
    public static double PRECISION_MODE_SPEED = 0.3;
    public static double CAMERA_OFFSET_Y = 12;
    public static double CAMERA_OFFSET_X = 6;
    public static int PIXEL_DISTANCE_THRESHOLD = 3;
    public static GamepadKeys.Button INTAKE = GamepadKeys.Button.X;
    public static GamepadKeys.Button REVERSE_INTAKE = GamepadKeys.Button.B;
    public static GamepadKeys.Button EXTEND_INTAKE = GamepadKeys.Button.DPAD_UP;
    public static GamepadKeys.Button CLAW_1 = GamepadKeys.Button.LEFT_BUMPER;
    public static GamepadKeys.Button CLAW_2 = GamepadKeys.Button.RIGHT_BUMPER;
    public static GamepadKeys.Trigger EXTEND_OUTTAKE = GamepadKeys.Trigger.RIGHT_TRIGGER;
    public static GamepadKeys.Trigger RETRACT_OUTTAKE = GamepadKeys.Trigger.LEFT_TRIGGER;
    public static GamepadKeys.Button DRONE = GamepadKeys.Button.Y;
    public static GamepadKeys.Button RESET_IMU = GamepadKeys.Button.START;

    public static GamepadKeys.Button PRECISION_MODE = GamepadKeys.Button.RIGHT_BUMPER;

    public static boolean withinRange(double input1, double input2, double deviation) {
        return Math.abs(input1 - input2) <= deviation;
    }

}
