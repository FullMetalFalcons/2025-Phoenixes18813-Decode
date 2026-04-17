package org.firstinspires.ftc.teamcode.roadrunnertuning.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class DriveConstants {
    public static final double TICKS_PER_REV = 537.7;
    public static final double MAX_RPM = 435;

    public static final boolean RUN_USING_ENCODER = false;
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0,
            getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV));

    public static double WHEEL_RADIUS = 1.88976;
    public static double GEAR_RATIO = 1.0;
    public static double TRACK_WIDTH = 19;

    public static double kV = 0.01538;
    public static double kA = 0.00017;
    public static double kStatic = 0.006;

    public static double MAX_VEL = 75;
    public static double MAX_ACCEL = 80;
    public static double MAX_ANG_VEL = 6.0;
    public static double MAX_ANG_ACCEL = 6.0;

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMotorVelocityF(double ticksPerSecond) {
        return 32767 / ticksPerSecond;
    }
}