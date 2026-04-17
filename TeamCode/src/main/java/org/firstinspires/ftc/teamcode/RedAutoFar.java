// Copyright Alben Beena 2026 - Version 3.0 - MIRRORED FROM BLUE

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.roadrunnertuning.drive.SampleMecanumDrive;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.roadrunnertuning.trajectorysequence.TrajectorySequence;
import com.qualcomm.robotcore.hardware.*;

import java.util.List;

@Autonomous(name = "Red Auto Far", group = "!auto")
public class RedAutoFar extends LinearOpMode {

    DcMotorEx lowerShooter, upperShooter, intake;
    Servo front, mid, back;
    Limelight3A limelight;

    static final double SERVO_DOWN = 1.0;
    static final double SERVO_UP   = 0.0;

    // ========== BATTERY COMPENSATION SETTINGS ==========
    static final double TARGET_VOLTAGE        = 12.31;
    static final double MIN_VOLTAGE           = 11.5;
    static final double MAX_VOLTAGE           = 13.59;
    static final double BASE_SHOOTER_POWER    = 0.83;

    static final double INTAKE_POWER = -1.00;

    // ========== DISTANCE-BASED SHOOTER SETTINGS ==========
    static final double CAMERA_HEIGHT_INCHES   = 17.42;
    static final double TARGET_HEIGHT_INCHES   = 29.5;
    static final double CAMERA_ANGLE_DEGREES   = 30.0;

    static final double CLOSE_DISTANCE_INCHES  = 36.0;
    static final double FAR_DISTANCE_INCHES    = 72.0;
    static final double CLOSE_POWER_MULTIPLIER = 0.95;
    static final double FAR_POWER_MULTIPLIER   = 1.05;

    static final double MID_CHAMBER_YAW_OFFSET   = Math.toRadians(0);
    static final double FRONT_CHAMBER_YAW_OFFSET = Math.toRadians(-5);
    static final double BACK_CHAMBER_YAW_OFFSET  = Math.toRadians(2);

    // ========== DECODE APRILTAG IDS ==========
    // NOTE: Update these three tag IDs to whatever red-side decode tags your game uses
    static final int DECODE_TAG_21 = 21;
    static final int DECODE_TAG_22 = 22;
    static final int DECODE_TAG_23 = 23;

    // ========== DECODE ORDER ==========
    enum BallColor { PURPLE, GREEN, UNKNOWN }
    BallColor[] requiredOrder = new BallColor[3];
    boolean orderDetected = false;

    // ========== KEY POSES (mirrored from Blue — Y values flipped) ==========
    static final Pose2d   START_POSE = new Pose2d(60, 15, Math.toRadians(-90));
    static final Vector2d SHOOT_POS  = new Vector2d(57, 15);
    static final Vector2d INTAKE_TOP    = new Vector2d(29, 23);
    static final Vector2d INTAKE_BOTTOM = new Vector2d(29, 69);

    // ========== SHOOTING HEADINGS (mirrored from Blue) ==========
    static final double CYCLE_0_SHOOT_TURN    = -16;  // Mirrored: negative of Blue's +20
    static final double CYCLE_1_SHOOT_HEADING = -125; // Mirrored: 180 - (-55) flipped across Y

    private double currentShooterPower    = BASE_SHOOTER_POWER;
    private double lastCalculatedDistance = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {

        lowerShooter = hardwareMap.get(DcMotorEx.class, "lowerShooter");
        upperShooter = hardwareMap.get(DcMotorEx.class, "upperShooter");
        intake       = hardwareMap.get(DcMotorEx.class, "intake");

        front = hardwareMap.get(Servo.class, "front");
        mid   = hardwareMap.get(Servo.class, "mid");
        back  = hardwareMap.get(Servo.class, "back");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        front.setPosition(SERVO_DOWN);
        mid.setPosition(SERVO_DOWN);
        back.setPosition(SERVO_DOWN);

        lowerShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        upperShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        double batteryVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        currentShooterPower = calculateShooterPower(batteryVoltage);

        // ========== SCAN FOR DECODE ORDER DURING INIT ==========
        while (!isStarted() && !isStopRequested()) {
            detectDecodeOrder();
            telemetry.addData("Status", "Ready - Red Far");
            telemetry.addData("Battery Voltage", "%.2f V", batteryVoltage);
            telemetry.addData("Base Shooter Power", "%.2f", currentShooterPower);
            telemetry.addData("Limelight", limelight.isConnected() ? "Connected" : "Disconnected");
            telemetry.addData("DECODE Order", orderDetected ? getOrderString() : "Searching... (point at tag 21/22/23)");
            telemetry.update();
            sleep(200);
        }

        waitForStart();
        drive.setPoseEstimate(START_POSE);

        // ========== THE SIX BALL FAR AUTO ==========

        // ── CYCLE 0: Drive to shoot position and take first 3 shots ──
        TrajectorySequence toShootPos = drive.trajectorySequenceBuilder(START_POSE)
                .addTemporalMarker(0, () -> setShooterPower(currentShooterPower))
                .lineToConstantHeading(SHOOT_POS)
                .build();
        drive.followTrajectorySequence(toShootPos);

        drive.turn(Math.toRadians(CYCLE_0_SHOOT_TURN));  // Turn to first shooting heading
        sleep(200);
        shootBalls(drive); // Shots 1, 2, 3

        // ── CYCLE 1: Intake and return for shots 4, 5, 6 ──
        TrajectorySequence cycle1intake = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(INTAKE_TOP.getX(), INTAKE_TOP.getY(), Math.toRadians(90)))
                .addTemporalMarker(0.1, () -> intake.setPower(INTAKE_POWER))
                .lineToConstantHeading(INTAKE_BOTTOM)
                .waitSeconds(0.5)
                .lineToConstantHeading(INTAKE_TOP)
                .build();
        drive.followTrajectorySequence(cycle1intake);

        TrajectorySequence cycle1shoot = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(SHOOT_POS.getX(), SHOOT_POS.getY(), Math.toRadians(CYCLE_1_SHOOT_HEADING)))
                .addTemporalMarker(() -> intake.setPower(0))
                .addTemporalMarker(() -> setShooterPower(currentShooterPower))
                .build();
        drive.followTrajectorySequence(cycle1shoot);

        sleep(150);
        shootBalls(drive); // Shots 4, 5, 6

        TrajectorySequence endPark = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .waitSeconds(1.2)
                .lineToLinearHeading(new Pose2d(27, 23, Math.toRadians(-90)))
                .build();
        drive.followTrajectorySequence(endPark);

        limelight.stop();

        telemetry.addData("Status", "Complete! - Red Far");
        telemetry.update();
    }

    // ========== DECODE ORDER DETECTION ==========

    void detectDecodeOrder() {
        LLResultTypes.FiducialResult tag21 = getTagById(DECODE_TAG_21);
        LLResultTypes.FiducialResult tag22 = getTagById(DECODE_TAG_22);
        LLResultTypes.FiducialResult tag23 = getTagById(DECODE_TAG_23);

        if (tag21 != null) {
            requiredOrder[0] = BallColor.GREEN;
            requiredOrder[1] = BallColor.PURPLE;
            requiredOrder[2] = BallColor.PURPLE;
            orderDetected = true;
        } else if (tag22 != null) {
            requiredOrder[0] = BallColor.PURPLE;
            requiredOrder[1] = BallColor.GREEN;
            requiredOrder[2] = BallColor.PURPLE;
            orderDetected = true;
        } else if (tag23 != null) {
            requiredOrder[0] = BallColor.PURPLE;
            requiredOrder[1] = BallColor.PURPLE;
            requiredOrder[2] = BallColor.GREEN;
            orderDetected = true;
        } else {
            orderDetected = false;
        }
    }

    String getOrderString() {
        if (!orderDetected) return "UNKNOWN";
        return requiredOrder[0] + " -> " + requiredOrder[1] + " -> " + requiredOrder[2];
    }

    // ========== SHOOTING ==========

    void shootBalls(SampleMecanumDrive drive) {
        intake.setPower(INTAKE_POWER);

        telemetry.addData("Base Shooter Power", "%.2f", currentShooterPower);
        telemetry.addData("Shooting Order", orderDetected ? getOrderString() : "Default (mid->front->back)");
        telemetry.update();

        if (orderDetected) {
            for (int i = 0; i < 3; i++) {
                int servoNum = mapColorToServo(requiredOrder[i], i);
                aimAtGoalAndGetHeading(drive, getYawOffsetForServo(servoNum));
                shootBall(servoNum);
            }
        } else {
            // SHOT 1: MID
            aimAtGoalAndGetHeading(drive, MID_CHAMBER_YAW_OFFSET);
            shootBall(2);

            // SHOT 2: FRONT
            aimAtGoalAndGetHeading(drive, FRONT_CHAMBER_YAW_OFFSET);
            shootBall(1);

            // SHOT 3: BACK
            aimAtGoalAndGetHeading(drive, BACK_CHAMBER_YAW_OFFSET);
            shootBall(3);
        }

        intake.setPower(0);
    }

    int mapColorToServo(BallColor color, int positionIndex) {
        if (color == BallColor.GREEN) return 1;
        if (positionIndex == 0 || (positionIndex == 1 && requiredOrder[0] != BallColor.PURPLE)) return 2;
        return 3;
    }

    double getYawOffsetForServo(int servoNum) {
        if (servoNum == 1) return FRONT_CHAMBER_YAW_OFFSET;
        if (servoNum == 2) return MID_CHAMBER_YAW_OFFSET;
        return BACK_CHAMBER_YAW_OFFSET;
    }

    // ========== LIMELIGHT HELPERS ==========

    LLResultTypes.FiducialResult getTagById(int targetId) {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return null;
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) return null;
        for (LLResultTypes.FiducialResult f : fiducials) {
            if (f.getFiducialId() == targetId) return f;
        }
        return null;
    }

    LLResultTypes.FiducialResult getGoalTag() {
        return getTagById(24); // Red side goal tag
    }

    double calculateDistance(LLResultTypes.FiducialResult tag) {
        if (tag == null) return 0.0;
        return Math.abs((TARGET_HEIGHT_INCHES - CAMERA_HEIGHT_INCHES) /
                Math.tan(Math.toRadians(CAMERA_ANGLE_DEGREES + tag.getTargetYDegrees())));
    }

    double calculateDistanceBasedPower(double distanceInches, double basePower) {
        double powerMultiplier;
        if (distanceInches < CLOSE_DISTANCE_INCHES) {
            powerMultiplier = CLOSE_POWER_MULTIPLIER;
        } else if (distanceInches > FAR_DISTANCE_INCHES) {
            powerMultiplier = FAR_POWER_MULTIPLIER;
        } else {
            double rangeRatio = (distanceInches - CLOSE_DISTANCE_INCHES) /
                    (FAR_DISTANCE_INCHES - CLOSE_DISTANCE_INCHES);
            powerMultiplier = CLOSE_POWER_MULTIPLIER +
                    (rangeRatio * (FAR_POWER_MULTIPLIER - CLOSE_POWER_MULTIPLIER));
        }
        return Math.max(0.6, Math.min(1.0, basePower * powerMultiplier));
    }

    double calculateShooterPower(double voltage) {
        voltage = Math.max(MIN_VOLTAGE, Math.min(MAX_VOLTAGE, voltage));
        return Math.max(0.6, Math.min(1.0, BASE_SHOOTER_POWER * (TARGET_VOLTAGE / voltage)));
    }

    double getYaw(LLResultTypes.FiducialResult tag) {
        if (tag == null) return 0;
        return Math.toRadians(tag.getTargetXDegrees());
    }

    double aimAtGoalAndGetHeading(SampleMecanumDrive drive, double offset) {
        LLResultTypes.FiducialResult tag = getGoalTag();
        if (tag == null) {
            telemetry.addData("Warning", "No AprilTag 24 detected!");
            telemetry.update();
            return drive.getPoseEstimate().getHeading();
        }

        double distance = calculateDistance(tag);
        lastCalculatedDistance = distance;

        double distanceAdjustedPower = calculateDistanceBasedPower(distance, currentShooterPower);
        setShooterPower(distanceAdjustedPower);

        double yawToGoal      = getYaw(tag);
        double currentHeading = drive.getPoseEstimate().getHeading();
        double targetHeading  = currentHeading - yawToGoal + offset;

        turnToHeading(drive, targetHeading);

        telemetry.addData("Distance to Goal", "%.1f in", distance);
        telemetry.addData("Adjusted Power", "%.2f", distanceAdjustedPower);
        telemetry.addData("Yaw Offset", "%.1f°", Math.toDegrees(yawToGoal));
        telemetry.addData("Target Heading", "%.1f°", Math.toDegrees(targetHeading));
        telemetry.update();

        return targetHeading;
    }

    void turnToHeading(SampleMecanumDrive drive, double targetHeading) {
        double currentHeading = drive.getPoseEstimate().getHeading();
        double angleDiff = normalizeAngle(targetHeading - currentHeading);
        if (Math.abs(angleDiff) > Math.toRadians(0.5)) {
            drive.turn(angleDiff);
        }
    }

    double normalizeAngle(double angle) {
        while (angle >  Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    void setShooterPower(double power) {
        lowerShooter.setPower(power);
        upperShooter.setPower(power);
    }

    void stopShooters() {
        lowerShooter.setPower(0);
        upperShooter.setPower(0);
    }

    void shootBall(int n) {
        if (n == 1) {
            front.setPosition(SERVO_UP);
            sleep(650);
            front.setPosition(SERVO_DOWN);
            sleep(150);
        }
        if (n == 2) {
            mid.setPosition(SERVO_UP);
            sleep(675);
            mid.setPosition(SERVO_DOWN);
            sleep(150);
        }
        if (n == 3) {
            back.setPosition(SERVO_UP);
            sleep(650);
            back.setPosition(SERVO_DOWN);
            sleep(150);
        }
    }
}