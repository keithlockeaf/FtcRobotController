package org.firstinspires.ftc.teamcode.newrobot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


/**
 * Control Hub Hardware Profile:
 *   Hardware Variables:
 *      DcMotor:  motor_right_rear, control hub, port 0, GoBILDA Series 5203
 *      DcMotor:  motor_right_forward, control hub, port 1, GoBILDA Series 5203
 *      DcMotor:  motor_left_forward, control hub, port 2, GoBILDA Series 5203
 *      DcMotor:  motor_left_rear, control hub, port 3, GoBILDA Series 5203
 *      DcMotor:  motor_intake, expansion hub, port 1, GoBILDA Series 5203
 *      DcMotor:  motor_main_flywheel, expansion hub, port 2, GoBILDA Series 5203
 *      DcMotor:  motor_aux_flywheel, expansion hub, port 3, GoBILDA Series 5203
 *      DcMotor:  motor_top_shooter,
 *      Servo:    servo_flipper,
 * Notes:
 *      Lowest Max Velocity of drive motors:  1880
 * Syntax types:
 *   Class - Pascal Case:     ThisIsPascalCase
 *   Functions - Camel Case:  thisIsCamelCase
 *   Variables - Camel Case:  thisIsCamelCase
 *   Hardware - Snake Case:   this_is_snake_case
 *   Constants - Upper Snake Case:  THIS_IS_UPPER_CASE
 */

@SuppressWarnings("FieldMayBeFinal")
@Disabled
@TeleOp(group="Test_Group")
public class Main_TeleOP extends LinearOpMode {

    private double motorSensitivity = 1.0;
    private double initMotorPower = 0.0;
    private double currentVelocityMLF = 0.0;
    private double currentVelocityMLR = 0.0;
    private double currentVelocityMRF = 0.0;
    private double currentVelocityMRR = 0.0;
    private double maxForwardVelocityMLF = 0.0;
    private double maxForwardVelocityMLR = 0.0;
    private double maxForwardVelocityMRF = 0.0;
    private double maxForwardVelocityMRR = 0.0;
    private double maxReverseVelocityMLF = 0.0;
    private double maxReverseVelocityMLR = 0.0;
    private double maxReverseVelocityMRF = 0.0;
    private double maxReverseVelocityMRR = 0.0;

/**
 *    PID tuning for all 4 drivetrain motors
 *    https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit?pli=1&tab=t.0#heading=h.61g9ixenznbx
 *    https://www.youtube.com/watch?v=6OH-wOsVVjg
 *
 */

    private double targetVelocity = 800;
    private double resultsMaxVelocityTest = 1880;
//    private double maxDriveTrainVelocity = 1880;
//    private double F = 32767.0 / resultsMaxVelocityTest;
//    private double kP = F * 0.1; // Large increase of acceleration or deceleration
//    private double kI = kP * 0.1; // Small increment of acceleration or deceleration
//    private double kD = kP * 0.01; // Slows down the motor once the kP gets closer to it goal, minimizing overshoot
//    f 17.4 kP 1.74, kI 0.17, kD 0.0174
    private double F = 32767.0 / resultsMaxVelocityTest;
    private double kP = 1.0; //
    private double kI = 0.2; //
    private double kD = 0.02; //
    private double position = 5.0; //
    private double motorLeftForwardCurrentVelocity = 0.0;
    private double motorLeftForwardMaxVelocity = 0.0;





    private DcMotorEx motorLeftForward;
    private DcMotorEx motorLeftRear;
    private DcMotorEx motorRightForward;
    private DcMotorEx motorRightRear;



    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        while (!isStarted()) {
            opModeTelemetry();
        }
        waitForStart();
        while (opModeIsActive()) {
//            teleOpControls();
            runMotorLeftForward(targetVelocity);

            if (gamepad1.a) {
                targetVelocity = 800;
            }
            if (gamepad1.b) {
                targetVelocity = 1800;
            }


            opModeTelemetry();
        }
    }
    public void initHardware() {
        initMotorLeftForward(kP, kI, kD, F, position);
        initMotorLeftRear();
        initMotorRightForward();
        initMotorRightRear();
    }

    private void initMotorLeftForward(double kP, double kI, double kD, double F, double position) {
        motorLeftForward = hardwareMap.get(DcMotorEx.class, "motor_left_forward");
        motorLeftForward.setPower(initMotorPower);
        motorLeftForward.setDirection(DcMotorEx.Direction.FORWARD);
        motorLeftForward.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        motorLeftForward.setVelocityPIDFCoefficients(kP, kI, kD, F);
//        motorLeftForward.setPositionPIDFCoefficients(position);
        motorLeftForward.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftForward.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    private void initMotorLeftRear() {
        motorLeftRear = hardwareMap.get(DcMotorEx.class, "motor_left_rear");
        motorLeftRear.setPower(initMotorPower);
        motorLeftRear.setDirection(DcMotorEx.Direction.FORWARD);
        motorLeftRear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        motorLeftRear.setVelocityPIDFCoefficients(kP, kI, kD, F);
//        motorLeftRear.setPositionPIDFCoefficients(position);
        motorLeftRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftRear.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    private void initMotorRightForward() {
        motorRightForward = hardwareMap.get(DcMotorEx.class, "motor_right_forward");
        motorRightForward.setPower(initMotorPower);
        motorRightForward.setDirection(DcMotorEx.Direction.REVERSE);
        motorRightForward.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        motorRightForward.setVelocityPIDFCoefficients(kP, kI, kD, F);
//        motorRightForward.setPositionPIDFCoefficients(position);
        motorRightForward.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorRightForward.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    private void initMotorRightRear() {
        motorRightRear = hardwareMap.get(DcMotorEx.class, "motor_right_rear");
        motorRightRear.setPower(initMotorPower);
        motorRightRear.setDirection(DcMotorEx.Direction.REVERSE);
        motorRightRear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        motorRightRear.setVelocityPIDFCoefficients(kP, kI, kD, F);
//        motorRightRear.setPositionPIDFCoefficients(position);
        motorRightRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorRightRear.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void teleOpControls() {
        motorLeftForward.setPower(gamepad1.left_stick_y * motorSensitivity * -1);
        motorLeftRear.setPower(gamepad1.left_stick_y * motorSensitivity * -1);
        motorRightForward.setPower(gamepad1.left_stick_y * motorSensitivity * -1);
        motorRightRear.setPower(gamepad1.left_stick_y * motorSensitivity * -1);

//        if (motorLeftForward.getVelocity() >= maxDriveTrainVelocity) {
//            motorLeftForward.setPower(maxDriveTrainVelocity);
//        }
//        if (motorLeftRear.getVelocity() >= maxDriveTrainVelocity) {
//            motorLeftRear.setPower(maxDriveTrainVelocity);
//        }
//        if (motorRightForward.getVelocity() >= maxDriveTrainVelocity) {
//            motorRightForward.setPower(maxDriveTrainVelocity);
//        }
//        if (motorRightRear.getVelocity() >= maxDriveTrainVelocity) {
//            motorRightRear.setPower(maxDriveTrainVelocity);
//        }


        currentVelocityMLF = motorLeftForward.getVelocity();
        if (currentVelocityMLF > 0 && currentVelocityMLF > maxForwardVelocityMLF) {
            maxForwardVelocityMLF = currentVelocityMLF;
        }
        if (currentVelocityMLF < 0 && currentVelocityMLF < maxReverseVelocityMLF) {
            maxReverseVelocityMLF = currentVelocityMLF;
        }

        currentVelocityMLR = motorLeftRear.getVelocity();
        if (currentVelocityMLR > 0 && currentVelocityMLR > maxForwardVelocityMLR) {
            maxForwardVelocityMLR = currentVelocityMLR;
        }
        if (currentVelocityMLR < 0 && currentVelocityMLR < maxReverseVelocityMLR) {
            maxReverseVelocityMLR = currentVelocityMLR;
        }

        currentVelocityMRF = motorRightForward.getVelocity();
        if (currentVelocityMRF > 0 && currentVelocityMRF > maxForwardVelocityMRF) {
            maxForwardVelocityMRF = currentVelocityMRF;
        }
        if (currentVelocityMRF < 0 && currentVelocityMRF < maxReverseVelocityMRF) {
            maxReverseVelocityMRF = currentVelocityMRF;
        }

        currentVelocityMRR = motorRightRear.getVelocity();
        if (currentVelocityMRR > 0 && currentVelocityMRR > maxForwardVelocityMRR) {
            maxForwardVelocityMRR = currentVelocityMRR;
        }
        if (currentVelocityMRR < 0 && currentVelocityMRR < maxReverseVelocityMRR) {
            maxReverseVelocityMRR = currentVelocityMRR;
        }
    }

    public void runMotorLeftForward(double velocity) {
        motorLeftForward.setVelocity(velocity);
        motorLeftForwardCurrentVelocity = motorLeftForward.getVelocity();
        if (motorLeftForwardCurrentVelocity > motorLeftForwardMaxVelocity) {
            motorLeftForwardMaxVelocity = motorLeftForwardCurrentVelocity;
        }
    }

    private void opModeTelemetry() {
        telemetry.addData("mlf power", "Power: %.2f", motorLeftForward.getPower());
//        telemetry.addData("mlf controller: ", motorLeftForward.getController());
//        telemetry.addData("mlf port#: ", motorLeftForward.getPortNumber());
//        telemetry.addData("mlf connection info: ", motorLeftForward.getConnectionInfo());
//        telemetry.addData("mlf device name: ", motorLeftForward.getDeviceName());
//        telemetry.addData("mlf manufacture: ", motorLeftForward.getManufacturer());
//        telemetry.addData("mlf version: ", motorLeftForward.getVersion());
//        telemetry.addData("mlf class: ", motorLeftForward.getClass());
//        telemetry.addData("mlf mode", motorLeftForward.getMode());
//        telemetry.addData("mlf cv ", currentVelocityMLF);
//        telemetry.addData("mlf mfv", maxForwardVelocityMLF);
//        telemetry.addData("mlf mrv", maxReverseVelocityMLF);
//        telemetry.addData("mlr cv ", currentVelocityMLR);
//        telemetry.addData("mlr mfv", maxForwardVelocityMLR);
//        telemetry.addData("mlr mtv", maxReverseVelocityMLR);
//        telemetry.addData("mrf cv ", currentVelocityMRF);
//        telemetry.addData("mrf mfv", maxForwardVelocityMRF);
//        telemetry.addData("mrf mrv", maxReverseVelocityMRF);
//        telemetry.addData("mrr cv ", currentVelocityMRR);
//        telemetry.addData("mrr mfv", maxForwardVelocityMRR);
//        telemetry.addData("mrr mrv", maxReverseVelocityMRR);
        telemetry.addData("Power", motorLeftForward.getPower());
        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Current Velocity", motorLeftForward.getVelocity());
        telemetry.addData("Max Velocity", resultsMaxVelocityTest);
        telemetry.addData("F", F);
        telemetry.addData("kP", kP);
        telemetry.addData("kI", kI);
        telemetry.addData("kD", kD);

        telemetry.update();
    }
}
