//Made by Alben Beena 2026 - Version 2.38

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.roadrunnertuning.drive.SampleMecanumDrive;

import java.util.List;

@TeleOp(name = "TeleopTest")
public class TeleopTest extends LinearOpMode {

    // ================= DRIVE MOTORS =================
    DcMotorEx motorLF, motorRF, motorLB, motorRB;

    // ================= MECHANISM MOTORS =================
    DcMotorEx lowerShooter, upperShooter, intake;

    // ================= SERVOS =================
    Servo front, mid, back;

    // ================= COLOR SENSORS =================
    NormalizedColorSensor colorSensorFront, colorSensorMid, colorSensorBack;

    // ================= LIMELIGHT & LED =================
    Limelight3A limelight;
    Servo headlight;

    // ================= COLOR DETECTION =================
    enum BallColor { PURPLE, GREEN, UNKNOWN }
    BallColor[] requiredOrder = new BallColor[3];
    BallColor[] currentBalls  = { BallColor.UNKNOWN, BallColor.UNKNOWN, BallColor.UNKNOWN };
    boolean orderDetected = false;
    boolean orderLocked   = false;
    long lastOrderCheckTime = 0;

    boolean[]  slotLocked        = { false, false, false };
    boolean    allSlotsLocked    = false;
    BallColor[] slotCandidate    = { BallColor.UNKNOWN, BallColor.UNKNOWN, BallColor.UNKNOWN };
    int[]       slotConfirmCount = { 0, 0, 0 };
    static final int LOCK_CONFIRM_COUNT = 5;

    // ================= BUTTON TRACKING =================
    boolean frontLast  = false;
    boolean midLast    = false;
    boolean backLast   = false;
    boolean yLast      = false;
    boolean masterLast = false;

    int queuedServo = -1;

    // ================= MASTER MODE =================
    boolean masterMode = false;

    // ================= SERVO STATE MACHINE =================
    enum ServoState { IDLE, FIRING_SINGLE, FIRING_ALL, FIRING_ORDERED }
    ServoState servoState     = ServoState.IDLE;
    int  currentServoStep     = 0;
    long servoStepStartTime   = 0;
    int  servoToFire          = 0;

    // ================= ORDERED SHOOTING =================
    int[] orderedServoSequence = new int[3];
    int   orderedServoIndex    = 0;

    // ================= AUTO-AIM STATE MACHINE =================
    enum AutoAimState { IDLE, SPINNING_UP, AIMING_MID, SHOOTING_MID, AIMING_FRONT, SHOOTING_FRONT, AIMING_BACK, SHOOTING_BACK, CLEANUP }
    AutoAimState autoAimState         = AutoAimState.IDLE;
    long         autoAimStepStartTime = 0;
    boolean      autoAimIsBlue        = false;
    int          autoAimTargetTagId   = 0;

    // ================= QUICK-AIM STATE MACHINE =================
    enum QuickAimState { IDLE, TURNING, DONE }
    QuickAimState quickAimState = QuickAimState.IDLE;
    boolean       dpadUpLast    = false;
    boolean       dpadDownLast  = false;
    int           quickAimTagId = 0;
    double        quickAimTurnPower = 0.0;

    // ================= SERVO CONSTANTS =================
    static final double SERVO_DOWN  = 1.0;
    static final double SERVO_UP    = 0.0;
    static final double MID_UP_POS  = -0.1;
    static final double BACK_UP_POS = 0.05;

    // Servo timings matched to auto code
    static final int FRONT_UP_TIME    = 440;
    static final int MID_UP_TIME_MS   = 450;
    static final int BACK_UP_TIME_MS  = 450;
    static final int SERVO_DOWN_DELAY = 150;

    // ================= BATTERY COMPENSATION =================
    static final double TARGET_VOLTAGE     = 12.31;
    static final double MIN_VOLTAGE        = 11.5;
    static final double MAX_VOLTAGE        = 13.59;
    static final double BASE_SHOOTER_POWER = 0.90;

    // ================= TRIGGER SHOOTER SPEEDS =================
    static final double CLOSE_SHOOTER_POWER = 0.56;
    static final double FAR_SHOOTER_POWER   = 0.75;

    // ================= LIMELIGHT =================
    static final double CAMERA_HEIGHT_INCHES = 17.42;
    static final double TARGET_HEIGHT_INCHES = 29.5;
    static final double CAMERA_ANGLE_DEGREES = 30.0;

    static final double CLOSE_DISTANCE_INCHES  = 36.0;
    static final double FAR_DISTANCE_INCHES    = 72.0;
    static final double CLOSE_POWER_MULTIPLIER = 0.95;
    static final double FAR_POWER_MULTIPLIER   = 1.05;

    // ================= APRILTAG IDS =================
    static final int BLUE_GOAL_TAG_ID = 20;
    static final int RED_GOAL_TAG_ID  = 24;
    static final int DECODE_TAG_21    = 21;
    static final int DECODE_TAG_22    = 22;
    static final int DECODE_TAG_23    = 23;

    // ================= YAW OFFSETS — BLUE (tag 20) =================
    static final double BLUE_MID_OFFSET   = Math.toRadians(0);
    static final double BLUE_FRONT_OFFSET = Math.toRadians(-5);
    static final double BLUE_BACK_OFFSET  = Math.toRadians(2);

    // ================= YAW OFFSETS — RED (tag 24) =================
    static final double RED_MID_OFFSET   = Math.toRadians(-2.83);
    static final double RED_FRONT_OFFSET = Math.toRadians(0);
    static final double RED_BACK_OFFSET  = Math.toRadians(0);

    // ================= AUTO-AIM =================
    static final double MIN_SHOOTING_DISTANCE = 24.0;
    static final double MAX_SHOOTING_DISTANCE = 96.0;
    static final double AUTO_AIM_TURN_POWER   = 0.3;
    static final double AUTO_AIM_TOLERANCE    = Math.toRadians(2.5);

    // ================= TURN CONTROLLER =================
    static final double TURN_KP      = 1.0;
    static final double TURN_MIN_PWR = 0.12;

    // ================= COLOR SENSOR THRESHOLDS =================
    static final float PURPLE_RED_MIN   = 0.03f;
    static final float PURPLE_BLUE_MIN  = 0.03f;
    static final float PURPLE_GREEN_MAX = 0.08f;
    static final float GREEN_GREEN_MIN  = 0.05f;
    static final float GREEN_RED_MAX    = 0.04f;
    static final float GREEN_BLUE_MAX   = 0.04f;
    static final float MIN_ALPHA        = 0.10f;
    static final float SENSOR_GAIN      = 8.0f;

    private SampleMecanumDrive drive;

    @Override
    public void runOpMode() {

        motorLF = hardwareMap.get(DcMotorEx.class, "leftFront");
        motorLB = hardwareMap.get(DcMotorEx.class, "leftBack");
        motorRF = hardwareMap.get(DcMotorEx.class, "rightFront");
        motorRB = hardwareMap.get(DcMotorEx.class, "rightBack");

        lowerShooter = hardwareMap.get(DcMotorEx.class, "lowerShooter");
        upperShooter = hardwareMap.get(DcMotorEx.class, "upperShooter");
        intake       = hardwareMap.get(DcMotorEx.class, "intake");

        front = hardwareMap.get(Servo.class, "front");
        mid   = hardwareMap.get(Servo.class, "mid");
        back  = hardwareMap.get(Servo.class, "back");

        colorSensorFront = hardwareMap.get(NormalizedColorSensor.class, "colorFront");
        colorSensorMid   = hardwareMap.get(NormalizedColorSensor.class, "colorMid");
        colorSensorBack  = hardwareMap.get(NormalizedColorSensor.class, "colorBack");

        colorSensorFront.setGain(SENSOR_GAIN);
        colorSensorMid.setGain(SENSOR_GAIN);
        colorSensorBack.setGain(SENSOR_GAIN);

        if (colorSensorFront instanceof com.qualcomm.hardware.rev.RevColorSensorV3)
            ((com.qualcomm.hardware.rev.RevColorSensorV3) colorSensorFront).enableLed(true);
        if (colorSensorMid instanceof com.qualcomm.hardware.rev.RevColorSensorV3)
            ((com.qualcomm.hardware.rev.RevColorSensorV3) colorSensorMid).enableLed(true);
        if (colorSensorBack instanceof com.qualcomm.hardware.rev.RevColorSensorV3)
            ((com.qualcomm.hardware.rev.RevColorSensorV3) colorSensorBack).enableLed(true);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        headlight = hardwareMap.get(Servo.class, "headlight");
        drive = new SampleMecanumDrive(hardwareMap);

        motorLF.setDirection(DcMotor.Direction.REVERSE);
        motorLB.setDirection(DcMotor.Direction.REVERSE);
        motorRF.setDirection(DcMotor.Direction.FORWARD);
        motorRB.setDirection(DcMotor.Direction.FORWARD);

        motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        front.setPosition(SERVO_DOWN);
        mid.setPosition(SERVO_DOWN);
        back.setPosition(SERVO_DOWN);

        lowerShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        upperShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Falcons TeleOp V2.38 - Ready");
        telemetry.addLine("LEFT  TRIGGER = Far range   (0.75)");
        telemetry.addLine("RIGHT TRIGGER = Close range (0.56)");
        telemetry.addLine("Y = Shoot in DECODE order");
        telemetry.addLine("DPAD UP   = Quick-aim BLUE (tag 20) | move stick to cancel");
        telemetry.addLine("DPAD DOWN = Quick-aim RED  (tag 24) | move stick to cancel");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            drive.update();

            long currentTime = System.currentTimeMillis();
            if (!orderLocked && currentTime - lastOrderCheckTime > 500) {
                detectDecodeOrder();
                lastOrderCheckTime = currentTime;
            }

            updateCurrentBallColors();

            if (gamepad1.start && gamepad1.y && !masterLast) masterMode = !masterMode;
            masterLast = gamepad1.start && gamepad1.y;

            // LED / Limelight
            LLResultTypes.FiducialResult blueTag = getGoalTag(BLUE_GOAL_TAG_ID);
            LLResultTypes.FiducialResult redTag  = getGoalTag(RED_GOAL_TAG_ID);

            double  blueDistance = 0.0, redDistance = 0.0;
            boolean blueInRange  = false, redInRange  = false;

            if (blueTag != null) {
                blueDistance = calculateDistance(blueTag);
                blueInRange  = (blueDistance >= MIN_SHOOTING_DISTANCE && blueDistance <= MAX_SHOOTING_DISTANCE);
            }
            if (redTag != null) {
                redDistance = calculateDistance(redTag);
                redInRange  = (redDistance >= MIN_SHOOTING_DISTANCE && redDistance <= MAX_SHOOTING_DISTANCE);
            }

            headlight.setPosition((blueTag != null || redTag != null) ? 1.0 : 0.0);

            updateAutoAimStateMachine();

            // ================= QUICK-AIM =================
            boolean dpadUp   = gamepad1.dpad_up;
            boolean dpadDown = gamepad1.dpad_down;

            boolean driverActive = (Math.abs(masterMode ? -gamepad1.left_stick_y : -gamepad2.left_stick_y) > 0.05
                    || Math.abs(masterMode ?  gamepad1.left_stick_x :  gamepad2.left_stick_x) > 0.05
                    || Math.abs(masterMode ?  gamepad1.right_stick_x :  gamepad2.right_stick_x) > 0.05);

            if (dpadUp && !dpadUpLast && quickAimState == QuickAimState.IDLE && autoAimState == AutoAimState.IDLE)
                startQuickAim(BLUE_GOAL_TAG_ID);

            if (dpadDown && !dpadDownLast && quickAimState == QuickAimState.IDLE && autoAimState == AutoAimState.IDLE)
                startQuickAim(RED_GOAL_TAG_ID);

            if (quickAimState == QuickAimState.TURNING && driverActive) cancelQuickAim();
            if (quickAimState == QuickAimState.DONE    && driverActive) quickAimState = QuickAimState.IDLE;

            updateQuickAimStateMachine();
            dpadUpLast   = dpadUp;
            dpadDownLast = dpadDown;

            // ================= DRIVE =================
            double driveY    = masterMode ? -gamepad1.left_stick_y  : -gamepad2.left_stick_y;
            double driveX    = masterMode ?  gamepad1.left_stick_x  :  gamepad2.left_stick_x;
            double driveTurn = masterMode ?  gamepad1.right_stick_x :  gamepad2.right_stick_x;

            if (quickAimState != QuickAimState.TURNING) {
                if (driverActive && autoAimState != AutoAimState.IDLE) {
                    double lf = driveY + driveX + driveTurn;
                    double lb = driveY - driveX + driveTurn;
                    double rf = driveY - driveX - driveTurn;
                    double rb = driveY + driveX - driveTurn;
                    double max = Math.max(1.0, Math.max(Math.abs(lf), Math.max(Math.abs(lb), Math.max(Math.abs(rf), Math.abs(rb)))));
                    motorLF.setPower(lf/max); motorLB.setPower(lb/max);
                    motorRF.setPower(rf/max); motorRB.setPower(rb/max);
                } else if (autoAimState == AutoAimState.IDLE) {
                    double lf = driveY + driveX + driveTurn;
                    double lb = driveY - driveX + driveTurn;
                    double rf = driveY - driveX - driveTurn;
                    double rb = driveY + driveX - driveTurn;
                    double max = Math.max(1.0, Math.max(Math.abs(lf), Math.max(Math.abs(lb), Math.max(Math.abs(rf), Math.abs(rb)))));
                    motorLF.setPower(lf/max); motorLB.setPower(lb/max);
                    motorRF.setPower(rf/max); motorRB.setPower(rb/max);
                }
            }

            // ================= INTAKE =================
            boolean intakeIn  = masterMode ? gamepad1.left_bumper  : gamepad2.left_bumper;
            boolean intakeOut = masterMode ? gamepad1.right_bumper : gamepad2.right_bumper;
            if (intakeIn)       intake.setPower(0.75);
            else if (intakeOut) intake.setPower(-1.0);
            else                intake.setPower(0.0);

            // ================= SHOOTER =================
            double batteryVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
            double voltage        = Math.max(MIN_VOLTAGE, Math.min(MAX_VOLTAGE, batteryVoltage));
            double shooterPower   = 0.0;

            if (gamepad1.left_trigger > 0.01) {
                double compensated = Math.max(0.40, Math.min(1.0, FAR_SHOOTER_POWER * (TARGET_VOLTAGE / voltage)));
                shooterPower = gamepad1.left_trigger * compensated;
                lowerShooter.setPower(shooterPower);
                upperShooter.setPower(shooterPower);
            } else if (gamepad1.right_trigger > 0.01) {
                double compensated = Math.max(0.40, Math.min(1.0, CLOSE_SHOOTER_POWER * (TARGET_VOLTAGE / voltage)));
                shooterPower = gamepad1.right_trigger * compensated;
                lowerShooter.setPower(shooterPower);
                upperShooter.setPower(shooterPower);
            } else {
                lowerShooter.setPower(0);
                upperShooter.setPower(0);
            }

            // ================= BUTTON INPUT WITH QUEUING =================
            boolean bBtn = gamepad1.b;
            boolean aBtn = gamepad1.a;
            boolean xBtn = gamepad1.x;
            boolean yBtn = gamepad1.y;

            if (yBtn && !yLast) {
                int request = orderDetected ? 5 : 4;
                if (servoState == ServoState.IDLE) fireQueued(request);
                else queuedServo = request;
            }
            yLast = yBtn;

            if (bBtn && !frontLast) {
                if (servoState == ServoState.IDLE) startServoSequence(1);
                else queuedServo = 1;
            }
            frontLast = bBtn;

            if (aBtn && !midLast) {
                if (servoState == ServoState.IDLE) startServoSequence(2);
                else queuedServo = 2;
            }
            midLast = aBtn;

            if (xBtn && !backLast) {
                if (servoState == ServoState.IDLE) startServoSequence(3);
                else queuedServo = 3;
            }
            backLast = xBtn;

            if (servoState == ServoState.IDLE && queuedServo != -1) {
                fireQueued(queuedServo);
                queuedServo = -1;
            }

            updateServoStateMachine();

            // ================= TELEMETRY =================
            telemetry.addData("Master Mode", masterMode ? "ON (gamepad1)" : "OFF (gamepad2)");
            telemetry.addData("Battery", "%.2f V", batteryVoltage);
            telemetry.addData("Shooter Output", "%.2f  [L=far 0.75 | R=close 0.56]", shooterPower);
            if (blueTag != null) telemetry.addData("Blue Target (tag 20)", "%.1f in - %s", blueDistance, blueInRange ? "IN RANGE" : "Out of range");
            if (redTag  != null) telemetry.addData("Red Target  (tag 24)", "%.1f in - %s", redDistance,  redInRange  ? "IN RANGE" : "Out of range");
            telemetry.addData("Queued Servo", queuedServo == -1 ? "none" : String.valueOf(queuedServo));

            telemetry.addLine();
            if (quickAimState == QuickAimState.TURNING) {
                String alliance = (quickAimTagId == BLUE_GOAL_TAG_ID) ? "BLUE (tag 20)" : "RED (tag 24)";
                telemetry.addLine(">> QUICK-AIM: TURNING -> " + alliance + " <<");
            } else if (quickAimState == QuickAimState.DONE) {
                String alliance = (quickAimTagId == BLUE_GOAL_TAG_ID) ? "BLUE (tag 20)" : "RED (tag 24)";
                telemetry.addLine(">> QUICK-AIM: LOCKED ON [" + alliance + "] - Shoot with triggers! Move stick to cancel. <<");
            } else {
                telemetry.addLine("QUICK-AIM: idle  [dpad_up=BLUE | dpad_down=RED]");
            }

            telemetry.addLine();
            telemetry.addLine("=== RED AIM TUNING ===");
            if (redTag != null) {
                double rawYawDeg = redTag.getTargetXDegrees();
                telemetry.addData("RED RAW YAW (deg)", "%.2f  <-- use this for offsets", rawYawDeg);
                telemetry.addData("RED_MID_OFFSET   currently", "%.1f deg", Math.toDegrees(RED_MID_OFFSET));
                telemetry.addData("RED_FRONT_OFFSET currently", "%.1f deg", Math.toDegrees(RED_FRONT_OFFSET));
                telemetry.addData("RED_BACK_OFFSET  currently", "%.1f deg", Math.toDegrees(RED_BACK_OFFSET));
            } else {
                telemetry.addLine("RED tag (24) not visible");
            }

            telemetry.addLine();
            telemetry.addData("DECODE Order", orderLocked ? "LOCKED: " + getOrderString() : (orderDetected ? getOrderString() : "Searching..."));
            telemetry.addData("Front Ball", currentBalls[0] + (slotLocked[0] ? " LOCKED" : " (" + slotConfirmCount[0] + "/" + LOCK_CONFIRM_COUNT + ")"));
            telemetry.addData("Mid Ball",   currentBalls[1] + (slotLocked[1] ? " LOCKED" : " (" + slotConfirmCount[1] + "/" + LOCK_CONFIRM_COUNT + ")"));
            telemetry.addData("Back Ball",  currentBalls[2] + (slotLocked[2] ? " LOCKED" : " (" + slotConfirmCount[2] + "/" + LOCK_CONFIRM_COUNT + ")"));
            telemetry.addData("All Slots", allSlotsLocked ? "ALL LOCKED - auto resets on Y" : "Scanning...");

            telemetry.addLine();
            telemetry.addLine("=== NORMALIZED COLOR VALUES ===");
            NormalizedRGBA fc = colorSensorFront.getNormalizedColors();
            float fR = fc.alpha > 0 ? fc.red/fc.alpha : 0, fG = fc.alpha > 0 ? fc.green/fc.alpha : 0, fB = fc.alpha > 0 ? fc.blue/fc.alpha : 0;
            telemetry.addData("Front", "R:%.2f G:%.2f B:%.2f A:%.2f", fR, fG, fB, fc.alpha);
            NormalizedRGBA mc = colorSensorMid.getNormalizedColors();
            float mR = mc.alpha > 0 ? mc.red/mc.alpha : 0, mG = mc.alpha > 0 ? mc.green/mc.alpha : 0, mB = mc.alpha > 0 ? mc.blue/mc.alpha : 0;
            telemetry.addData("Mid",   "R:%.2f G:%.2f B:%.2f A:%.2f", mR, mG, mB, mc.alpha);
            NormalizedRGBA bc = colorSensorBack.getNormalizedColors();
            float bR = bc.alpha > 0 ? bc.red/bc.alpha : 0, bG = bc.alpha > 0 ? bc.green/bc.alpha : 0, bB = bc.alpha > 0 ? bc.blue/bc.alpha : 0;
            telemetry.addData("Back",  "R:%.2f G:%.2f B:%.2f A:%.2f", bR, bG, bB, bc.alpha);

            telemetry.addLine();
            telemetry.addLine("=== DEBUG INFO ===");
            telemetry.addData("Front Alpha", "%.2f %s", fc.alpha, fc.alpha >= MIN_ALPHA ? "OK" : "TOO LOW!");
            telemetry.addData("Mid Alpha",   "%.2f %s", mc.alpha, mc.alpha >= MIN_ALPHA ? "OK" : "TOO LOW!");
            telemetry.addData("Back Alpha",  "%.2f %s", bc.alpha, bc.alpha >= MIN_ALPHA ? "OK" : "TOO LOW!");

            if (fc.alpha >= MIN_ALPHA) {
                boolean isPurple = fR > PURPLE_RED_MIN && fB > PURPLE_BLUE_MIN && fG < PURPLE_GREEN_MAX;
                boolean isGreen  = fG > GREEN_GREEN_MIN && fR < GREEN_RED_MAX && fB < GREEN_BLUE_MAX;
                if (isPurple)     telemetry.addData("Front SHOULD detect", "PURPLE");
                else if (isGreen) telemetry.addData("Front SHOULD detect", "GREEN");
                else              telemetry.addLine("Front FAILS all color checks");
            } else {
                telemetry.addData("Front FAIL", "Alpha too low");
            }

            telemetry.update();
        }

        limelight.stop();
        headlight.setPosition(0.0);
    }

    // =========================================================
    //  QUICK-AIM
    // =========================================================

    void startQuickAim(int tagId) {
        LLResultTypes.FiducialResult tag = getGoalTag(tagId);
        if (tag == null) {
            String alliance = (tagId == BLUE_GOAL_TAG_ID) ? "BLUE (tag 20)" : "RED (tag 24)";
            telemetry.addLine("QUICK-AIM: " + alliance + " tag not visible!");
            telemetry.update();
            return;
        }
        quickAimTagId = tagId;
        quickAimState = QuickAimState.TURNING;
    }

    void updateQuickAimStateMachine() {
        if (quickAimState != QuickAimState.TURNING) return;

        double yawOffset = (quickAimTagId == BLUE_GOAL_TAG_ID) ? BLUE_MID_OFFSET : RED_MID_OFFSET;
        LLResultTypes.FiducialResult tag = getGoalTag(quickAimTagId);

        if (tag == null) { stopTurning(); return; }

        double yawError  = normalizeAngle(getYaw(tag) - yawOffset);
        double turnPower = calculateTurnPower(yawError);

        if (Math.abs(yawError) < AUTO_AIM_TOLERANCE) {
            stopTurning();
            quickAimState = QuickAimState.DONE;
        } else {
            motorLF.setPower( turnPower); motorLB.setPower( turnPower);
            motorRF.setPower(-turnPower); motorRB.setPower(-turnPower);
        }
    }

    void cancelQuickAim() {
        stopTurning();
        quickAimState = QuickAimState.IDLE;
    }

    // =========================================================

    void fireQueued(int request) {
        if (request == 5) startOrderedServoSequence();
        else              startServoSequence(request);
    }

    void detectDecodeOrder() {
        if (orderLocked) return;
        LLResultTypes.FiducialResult tag21 = getGoalTag(DECODE_TAG_21);
        LLResultTypes.FiducialResult tag22 = getGoalTag(DECODE_TAG_22);
        LLResultTypes.FiducialResult tag23 = getGoalTag(DECODE_TAG_23);
        if (tag21 != null) {
            requiredOrder[0] = BallColor.GREEN;  requiredOrder[1] = BallColor.PURPLE; requiredOrder[2] = BallColor.PURPLE;
            orderDetected = true; orderLocked = true;
            telemetry.addLine("LOCKED: Green -> Purple -> Purple"); telemetry.update(); sleep(500);
        } else if (tag22 != null) {
            requiredOrder[0] = BallColor.PURPLE; requiredOrder[1] = BallColor.GREEN;  requiredOrder[2] = BallColor.PURPLE;
            orderDetected = true; orderLocked = true;
            telemetry.addLine("LOCKED: Purple -> Green -> Purple"); telemetry.update(); sleep(500);
        } else if (tag23 != null) {
            requiredOrder[0] = BallColor.PURPLE; requiredOrder[1] = BallColor.PURPLE; requiredOrder[2] = BallColor.GREEN;
            orderDetected = true; orderLocked = true;
            telemetry.addLine("LOCKED: Purple -> Purple -> Green"); telemetry.update(); sleep(500);
        } else {
            orderDetected = false;
        }
    }

    void updateCurrentBallColors() {
        if (allSlotsLocked) return;
        NormalizedColorSensor[] sensors = { colorSensorFront, colorSensorMid, colorSensorBack };
        for (int i = 0; i < 3; i++) {
            if (slotLocked[i]) continue;
            BallColor reading = detectColor(sensors[i]);
            if (reading != BallColor.UNKNOWN) {
                if (reading == slotCandidate[i]) {
                    slotConfirmCount[i]++;
                    if (slotConfirmCount[i] >= LOCK_CONFIRM_COUNT) {
                        currentBalls[i] = reading;
                        slotLocked[i]   = true;
                    }
                } else {
                    slotCandidate[i]    = reading;
                    slotConfirmCount[i] = 1;
                }
            } else {
                slotCandidate[i]    = BallColor.UNKNOWN;
                slotConfirmCount[i] = 0;
            }
        }
        allSlotsLocked = slotLocked[0] && slotLocked[1] && slotLocked[2];
    }

    void resetBallColors() {
        for (int i = 0; i < 3; i++) {
            currentBalls[i]     = BallColor.UNKNOWN;
            slotCandidate[i]    = BallColor.UNKNOWN;
            slotConfirmCount[i] = 0;
            slotLocked[i]       = false;
        }
        allSlotsLocked = false;
    }

    BallColor detectColor(NormalizedColorSensor sensor) {
        NormalizedRGBA colors = sensor.getNormalizedColors();
        if (colors.alpha < MIN_ALPHA || colors.alpha == 0) return BallColor.UNKNOWN;
        float red = colors.red/colors.alpha, green = colors.green/colors.alpha, blue = colors.blue/colors.alpha;
        if (red > PURPLE_RED_MIN && blue > PURPLE_BLUE_MIN && green < PURPLE_GREEN_MAX) return BallColor.PURPLE;
        if (green > GREEN_GREEN_MIN && red < GREEN_RED_MAX && blue < GREEN_BLUE_MAX)    return BallColor.GREEN;
        if (green > red && green > blue && green > 0.15f)                               return BallColor.GREEN;
        if ((red > green || blue > green) && (red > 0.10f || blue > 0.10f))            return BallColor.PURPLE;
        return BallColor.UNKNOWN;
    }

    String getOrderString() {
        if (!orderDetected) return "UNKNOWN";
        return String.format("%s -> %s -> %s", requiredOrder[0], requiredOrder[1], requiredOrder[2]);
    }

    void startOrderedServoSequence() {
        orderedServoSequence = new int[3];
        int sequenceIndex = 0;
        for (int i = 0; i < 3; i++) {
            BallColor needed = requiredOrder[i];
            for (int j = 0; j < 3; j++) {
                if (currentBalls[j] == needed) {
                    orderedServoSequence[sequenceIndex++] = j + 1;
                    currentBalls[j] = BallColor.UNKNOWN;
                    break;
                }
            }
        }
        updateCurrentBallColors();
        servoState = ServoState.FIRING_ORDERED;
        orderedServoIndex = 0; currentServoStep = 0;
        servoStepStartTime = System.currentTimeMillis();
    }

    void startServoSequence(int servoNumber) {
        servoToFire = servoNumber;
        servoState  = (servoNumber == 4) ? ServoState.FIRING_ALL : ServoState.FIRING_SINGLE;
        currentServoStep   = 0;
        servoStepStartTime = System.currentTimeMillis();
    }

    void updateServoStateMachine() {
        long elapsed = System.currentTimeMillis() - servoStepStartTime;
        if      (servoState == ServoState.FIRING_SINGLE)  updateSingleServoFiring(elapsed);
        else if (servoState == ServoState.FIRING_ALL)     updateAllServosFiring(elapsed);
        else if (servoState == ServoState.FIRING_ORDERED) updateOrderedServoFiring(elapsed);
    }

    void updateOrderedServoFiring(long elapsed) {
        if (orderedServoIndex >= 3) { servoState = ServoState.IDLE; currentServoStep = 0; resetBallColors(); return; }
        int s = orderedServoSequence[orderedServoIndex];
        int upTime = (s == 2) ? MID_UP_TIME_MS : (s == 3) ? BACK_UP_TIME_MS : FRONT_UP_TIME;
        if (currentServoStep == 0) {
            if (s == 1) front.setPosition(SERVO_UP);
            if (s == 2) mid.setPosition(MID_UP_POS);
            if (s == 3) back.setPosition(SERVO_UP);
            if (elapsed >= upTime) { currentServoStep = 1; servoStepStartTime = System.currentTimeMillis(); }
        } else if (currentServoStep == 1) {
            if (s == 1) front.setPosition(SERVO_DOWN);
            if (s == 2) mid.setPosition(SERVO_DOWN);
            if (s == 3) back.setPosition(SERVO_DOWN);
            if (elapsed >= SERVO_DOWN_DELAY) { orderedServoIndex++; currentServoStep = 0; servoStepStartTime = System.currentTimeMillis(); }
        }
    }

    void updateSingleServoFiring(long elapsed) {
        int upTime = (servoToFire == 2) ? MID_UP_TIME_MS : (servoToFire == 3) ? BACK_UP_TIME_MS : FRONT_UP_TIME;
        if (currentServoStep == 0) {
            if (servoToFire == 1) front.setPosition(SERVO_UP);
            if (servoToFire == 2) mid.setPosition(MID_UP_POS);
            if (servoToFire == 3) back.setPosition(SERVO_UP);
            if (elapsed >= upTime) { currentServoStep = 1; servoStepStartTime = System.currentTimeMillis(); }
        } else if (currentServoStep == 1) {
            if (servoToFire == 1) front.setPosition(SERVO_DOWN);
            if (servoToFire == 2) mid.setPosition(SERVO_DOWN);
            if (servoToFire == 3) back.setPosition(SERVO_DOWN);
            if (elapsed >= SERVO_DOWN_DELAY) { servoState = ServoState.IDLE; currentServoStep = 0; }
        }
    }

    void updateAllServosFiring(long elapsed) {
        if      (currentServoStep == 0) { mid.setPosition(MID_UP_POS);   if (elapsed >= MID_UP_TIME_MS)   { currentServoStep = 1; servoStepStartTime = System.currentTimeMillis(); } }
        else if (currentServoStep == 1) { mid.setPosition(SERVO_DOWN);   if (elapsed >= SERVO_DOWN_DELAY) { currentServoStep = 2; servoStepStartTime = System.currentTimeMillis(); } }
        else if (currentServoStep == 2) { front.setPosition(SERVO_UP);   if (elapsed >= FRONT_UP_TIME)    { currentServoStep = 3; servoStepStartTime = System.currentTimeMillis(); } }
        else if (currentServoStep == 3) { front.setPosition(SERVO_DOWN); if (elapsed >= SERVO_DOWN_DELAY) { currentServoStep = 4; servoStepStartTime = System.currentTimeMillis(); } }
        else if (currentServoStep == 4) { back.setPosition(SERVO_UP);    if (elapsed >= BACK_UP_TIME_MS)  { currentServoStep = 5; servoStepStartTime = System.currentTimeMillis(); } }
        else if (currentServoStep == 5) { back.setPosition(SERVO_DOWN);  if (elapsed >= SERVO_DOWN_DELAY) { servoState = ServoState.IDLE; currentServoStep = 0; } }
    }

    void startAutoAimSequence(LLResultTypes.FiducialResult tag, boolean isBlue) {
        if (tag == null) return;
        autoAimIsBlue      = isBlue;
        autoAimTargetTagId = isBlue ? BLUE_GOAL_TAG_ID : RED_GOAL_TAG_ID;
        autoAimState       = AutoAimState.SPINNING_UP;
        autoAimStepStartTime = System.currentTimeMillis();
        headlight.setPosition(0.0);
        setShooterPower(calculateDistanceBasedPower(calculateDistance(tag), FAR_SHOOTER_POWER));
    }

    void updateAutoAimStateMachine() {
        if (autoAimState == AutoAimState.IDLE) return;
        long elapsed = System.currentTimeMillis() - autoAimStepStartTime;
        switch (autoAimState) {
            case SPINNING_UP:
                headlight.setPosition(1.0);
                if (elapsed >= 300) { autoAimState = AutoAimState.AIMING_MID; autoAimStepStartTime = System.currentTimeMillis(); headlight.setPosition(0.5); }
                break;
            case AIMING_MID:
                aimAtTarget(autoAimIsBlue ? BLUE_MID_OFFSET : RED_MID_OFFSET);
                if (elapsed >= 400) { stopTurning(); autoAimState = AutoAimState.SHOOTING_MID; autoAimStepStartTime = System.currentTimeMillis(); startServoSequence(2); }
                break;
            case SHOOTING_MID:
                if (servoState == ServoState.IDLE && elapsed >= 150) { autoAimState = AutoAimState.AIMING_FRONT; autoAimStepStartTime = System.currentTimeMillis(); headlight.setPosition(1.0); }
                break;
            case AIMING_FRONT:
                aimAtTarget(autoAimIsBlue ? BLUE_FRONT_OFFSET : RED_FRONT_OFFSET);
                if (elapsed >= 400) { stopTurning(); autoAimState = AutoAimState.SHOOTING_FRONT; autoAimStepStartTime = System.currentTimeMillis(); startServoSequence(1); }
                break;
            case SHOOTING_FRONT:
                if (servoState == ServoState.IDLE && elapsed >= 150) { autoAimState = AutoAimState.AIMING_BACK; autoAimStepStartTime = System.currentTimeMillis(); headlight.setPosition(0.5); }
                break;
            case AIMING_BACK:
                aimAtTarget(autoAimIsBlue ? BLUE_BACK_OFFSET : RED_BACK_OFFSET);
                if (elapsed >= 400) { stopTurning(); autoAimState = AutoAimState.SHOOTING_BACK; autoAimStepStartTime = System.currentTimeMillis(); startServoSequence(3); }
                break;
            case SHOOTING_BACK:
                if (servoState == ServoState.IDLE && elapsed >= 200) { autoAimState = AutoAimState.CLEANUP; autoAimStepStartTime = System.currentTimeMillis(); }
                break;
            case CLEANUP:
                setShooterPower(0);
                headlight.setPosition(1.0);
                if (elapsed >= 200) { autoAimState = AutoAimState.IDLE; }
                break;
        }
    }

    void aimAtTarget(double offset) {
        LLResultTypes.FiducialResult tag = getGoalTag(autoAimTargetTagId);
        if (tag != null) {
            double yawError  = normalizeAngle(getYaw(tag) - offset);
            double turnPower = calculateTurnPower(yawError);
            motorLF.setPower( turnPower); motorLB.setPower( turnPower);
            motorRF.setPower(-turnPower); motorRB.setPower(-turnPower);
        }
    }

    void stopTurning() {
        motorLF.setPower(0); motorLB.setPower(0);
        motorRF.setPower(0); motorRB.setPower(0);
    }

    double calculateTurnPower(double angleError) {
        if (Math.abs(angleError) < AUTO_AIM_TOLERANCE) return 0.0;
        double power = angleError * TURN_KP;
        if (power > 0) power = Math.max(power,  TURN_MIN_PWR);
        else           power = Math.min(power, -TURN_MIN_PWR);
        return Math.max(-AUTO_AIM_TURN_POWER, Math.min(AUTO_AIM_TURN_POWER, power));
    }

    LLResultTypes.FiducialResult getGoalTag(int targetTagId) {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return null;
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) return null;
        for (LLResultTypes.FiducialResult f : fiducials) { if (f.getFiducialId() == targetTagId) return f; }
        return null;
    }

    double calculateDistance(LLResultTypes.FiducialResult tag) {
        if (tag == null) return 0.0;
        return Math.abs((TARGET_HEIGHT_INCHES - CAMERA_HEIGHT_INCHES) / Math.tan(Math.toRadians(CAMERA_ANGLE_DEGREES + tag.getTargetYDegrees())));
    }

    double getYaw(LLResultTypes.FiducialResult tag) {
        return tag == null ? 0 : Math.toRadians(tag.getTargetXDegrees());
    }

    double normalizeAngle(double angle) {
        while (angle >  Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    double calculateDistanceBasedPower(double distanceInches, double basePower) {
        double powerMultiplier;
        if (distanceInches < CLOSE_DISTANCE_INCHES)      powerMultiplier = CLOSE_POWER_MULTIPLIER;
        else if (distanceInches > FAR_DISTANCE_INCHES)   powerMultiplier = FAR_POWER_MULTIPLIER;
        else powerMultiplier = CLOSE_POWER_MULTIPLIER + ((distanceInches - CLOSE_DISTANCE_INCHES) /
                    (FAR_DISTANCE_INCHES - CLOSE_DISTANCE_INCHES)) * (FAR_POWER_MULTIPLIER - CLOSE_POWER_MULTIPLIER);
        return Math.max(0.40, Math.min(1.0, basePower * powerMultiplier));
    }

    double calculateShooterPower(double voltage) {
        voltage = Math.max(MIN_VOLTAGE, Math.min(MAX_VOLTAGE, voltage));
        return Math.max(0.40, Math.min(1.0, BASE_SHOOTER_POWER * (TARGET_VOLTAGE / voltage)));
    }

    void setShooterPower(double power) {
        lowerShooter.setPower(power);
        upperShooter.setPower(power);
    }
}