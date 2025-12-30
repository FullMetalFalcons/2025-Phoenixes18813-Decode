package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "FalconsTeleOp")
public class FalconsTeleOp extends LinearOpMode {

    // ================= DRIVE MOTORS =================
    DcMotorEx motorLF, motorRF, motorLB, motorRB;

    // ================= MECHANISM MOTORS =================
    DcMotorEx lowerShooter, upperShooter, intake;

    // ================= SERVOS =================
    Servo front, mid, back;

    // ================= SERVO TOGGLE STATE =================
    boolean frontUp = false;
    boolean midUp   = false;
    boolean backUp  = false;

    boolean frontLast = false;
    boolean midLast   = false;
    boolean backLast  = false;

    // ================= MASTER MODE =================
    boolean masterMode = false;
    boolean masterLast = false;

    // ================= CONSTANTS =================
    static final double SERVO_DOWN = 1.0;
    static final double SERVO_UP   = 0.2;
    static final double MID_UP_POS  = -0.1;
    static final double BACK_UP_POS = 0.35;

    public static MecanumDrive.Params DRIVE_PARAMS = new MecanumDrive.Params();

    @Override
    public void runOpMode() {

        /* ================= HARDWARE MAP ================= */
        motorLF = hardwareMap.get(DcMotorEx.class, DRIVE_PARAMS.leftFrontDriveName);
        motorLB = hardwareMap.get(DcMotorEx.class, DRIVE_PARAMS.leftBackDriveName);
        motorRF = hardwareMap.get(DcMotorEx.class, DRIVE_PARAMS.rightFrontDriveName);
        motorRB = hardwareMap.get(DcMotorEx.class, DRIVE_PARAMS.rightBackDriveName);

        lowerShooter = hardwareMap.get(DcMotorEx.class, "lowerShooter");
        upperShooter = hardwareMap.get(DcMotorEx.class, "upperShooter");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        front = hardwareMap.get(Servo.class, "front");
        mid   = hardwareMap.get(Servo.class, "mid");
        back  = hardwareMap.get(Servo.class, "back");

        /* ================= MOTOR SETUP ================= */
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

        telemetry.addLine("TeleOp Ready – Split + Master Mode");
        telemetry.update();

        waitForStart();

        /* ================= TELEOP LOOP ================= */
        while (opModeIsActive()) {

            /* ---------- MASTER MODE TOGGLE ---------- */
            if (gamepad1.start && gamepad1.y && !masterLast) {
                masterMode = !masterMode;
            }
            masterLast = gamepad1.start && gamepad1.y;

            telemetry.addData("MASTER MODE", masterMode ? "ON" : "OFF");

            /* ============================================================ */
            /* ======================== DRIVE ============================= */
            /* ============================================================ */
            // Driver 2 normally, Driver 1 in master mode
            double driveY    = masterMode ? -gamepad1.left_stick_y : -gamepad2.left_stick_y;
            double driveX    = masterMode ?  gamepad1.left_stick_x :  gamepad2.left_stick_x;
            double driveTurn = masterMode ?  gamepad1.right_stick_x :  gamepad2.right_stick_x;

            double lf = driveY + driveX + driveTurn;
            double lb = driveY - driveX + driveTurn;
            double rf = driveY - driveX - driveTurn;
            double rb = driveY + driveX - driveTurn;

            double max = Math.max(1.0,
                    Math.max(Math.abs(lf), Math.max(Math.abs(lb), Math.max(Math.abs(rf), Math.abs(rb)))));

            motorLF.setPower(lf / max);
            motorLB.setPower(lb / max);
            motorRF.setPower(rf / max);
            motorRB.setPower(rb / max);

            /* ============================================================ */
            /* ======================== INTAKE ============================ */
            /* ============================================================ */
            // Driver 2 normally, Driver 1 in master mode
            boolean intakeIn  = masterMode ? gamepad1.left_bumper  : gamepad2.left_bumper;
            boolean intakeOut = masterMode ? gamepad1.right_bumper : gamepad2.right_bumper;

            if (intakeIn) {
                intake.setPower(0.6);
            } else if (intakeOut) {
                intake.setPower(-1.0);
            } else {
                intake.setPower(0.0);
            }

            /* ============================================================ */
            /* ======================== SHOOTER =========================== */
            /* ============================================================ */
            // Driver 1 normally, Driver 1 also in master mode
            double shooterPower = (gamepad1.left_trigger > 0.2) ? 1.0 : 0.0;

            lowerShooter.setPower(shooterPower);
            upperShooter.setPower(shooterPower);

            // safety disabled: intake allowed while shooter is running

            /* ============================================================ */
            /* ======================== SERVOS ============================ */
            /* ============================================================ */
            // Driver 1 normally, Driver 1 also in master mode
            boolean b = gamepad1.b;
            boolean a = gamepad1.a;
            boolean xBtn = gamepad1.x;

            if (b && !frontLast) frontUp = !frontUp;
            if (a && !midLast)   midUp   = !midUp;
            if (xBtn && !backLast) backUp = !backUp;

            frontLast = b;
            midLast   = a;
            backLast  = xBtn;

            front.setPosition(frontUp ? SERVO_UP : SERVO_DOWN);
            mid.setPosition(midUp ? MID_UP_POS : SERVO_DOWN);
            back.setPosition(backUp ? BACK_UP_POS : SERVO_DOWN);

            telemetry.update();
        }
    }
}
