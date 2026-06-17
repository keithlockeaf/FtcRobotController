package org.firstinspires.ftc.teamcode;


import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.newrobot.AprilTag;
import org.firstinspires.ftc.teamcode.newrobot.DriveTrain;
import org.firstinspires.ftc.teamcode.newrobot.Initiate_Hardware;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import java.text.NumberFormat;

/**
 * Control Hub Hardware Profile:
 *   Hardware Variables:
 *     Control Hub
 *       DcMotor:  motor_right_rear, motor port 0, GoBILDA Series 5203
 *       DcMotor:  motor_right_forward, motor port 1, GoBILDA Series 5203
 *       DcMotor:  motor_left_forward, motor port 2, GoBILDA Series 5203
 *       DcMotor:  motor_left_rear, motor port 3, GoBILDA Series 5203
 *       Servo:    servo_rgb_light, servo port 0, GoBILDA RGB Indicator Light
 *       I2C:      pinpoint_odometry_computer, I2C port 2, GoBILDA Pinpoint Odometry Computer
 *       Network Device:  limelight, eth0: 172.29.0.26, Limelight 3A
 *       Digital Device:  sensor_laser_distance, digital port 0, GoBILDA Distance Sensor
 *     Expansion Hub
 *       DcMotor:  motor_turret, port, motor port 0, GoBILDA Series 5203
 *       DcMotor:  motor_artifact_intake, motor port 1, GoBILDA Series 5203
 *       DcMotor:  motor_main_flywheel, motor port 2, GoBILDA Series 5203
 *       DcMotor:  motor_aux_flywheel, motor port 3, GoBILDA Series 5203
 *       Servo:    servo_flipper, servo port 0, Axon MINI MK2
 *     Limelight:
 *       Red Goal:  April-Tag 24
 *       Blue Goal: April-Tag
 *       Artifact Order:  April-Tag
 * Notes:
 *      Lowest Max Velocity of drive motors:  1880
 * Syntax types:
 *   Class - Pascal Case:     ThisIsPascalCase
 *   Functions - Camel Case:  thisIsCamelCase
 *   Variables - Camel Case:  thisIsCamelCase
 *   Hardware - Snake Case:   this_is_snake_case
 *   Constants - Upper Snake Case:  THIS_IS_UPPER_CASE
 */

@SuppressWarnings({"FieldMayBeFinal","FieldCanBeLocal"})
//@Disabled
@Autonomous
public class AutonomousSimple extends LinearOpMode {

    // DriveTrain Hardware Variables
    private DcMotorEx motorRightForward;
    private DcMotorEx motorRightRear;
    private DcMotorEx motorLeftForward;
    private DcMotorEx motorLeftRear;

    // Required var for the DriveTrain
    private double customDriveTrainPower = 1.0;

    // Artifact Intake Counter Variables
    private int artifactIntakeCount = 3;

    // Shooter Hardware Variables
    DcMotorEx motorArtifactIntake;
    DcMotorEx motorMainFlywheel;
    Servo servoFlipper;

    // Shooter Software Variables
    double flywheelTargetVelocity = 1350;
    double F = 14.098; // Feedforward gain to counteract constant forces like friction.
    double P = 265;    // Proportional gain to correct error based on how far off the velocity is.
    private double SERVO_FLIPPER_STARTING_ANGLE = 0.93;
    private double SERVO_FLIPPER_ENDING_ANGLE = 0.7;
    private int SERVO_FLIPPER_TRAVEL_TIME = 250;
    Timer distanceTimer = new Timer();
    double DISTANCE_TIMEOUT_SECONDS = 3;
    boolean atShootingDistance = false;

    // LimeLight Variables
    private Limelight3A limelight;
    private IMU imu;
    private double distance;
    private int teamColor = 8;  // Default is 8 which is RED, 9 is BLUE.  The numbers correspond to the limelight index to use for each april tag color.

    // Autonomous Specific Variables
    boolean isFinishedShooting = false;
    boolean finalMove = false;

    AprilTag aprilTag = new AprilTag();
    DriveTrain driveTrain = new DriveTrain();

    double forward, strafe, rotate = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        while (!isStarted()) {
            changeTeamColor(limelight);
            initTelemetry();
        }
        waitForStart();
        limelight.start();
        distanceTimer.resetTimer();
        while (opModeIsActive()) {
            AutonomousControls();
            opModeTelemetry();
        }
    }

    // Initiates the hardware of the robot
    public void initHardware() {
        // Generate an instance of the Initiate_Hardware file that is used to set up all the hardware for the robot
        Initiate_Hardware initHardware = new Initiate_Hardware();

        // Initiate drive train hardware (all 4 motors that control the wheels)
        motorRightForward = initHardware.initMotor(hardwareMap,"motor_right_forward", "REVERSE", "BRAKE", true);
        motorRightRear = initHardware.initMotor(hardwareMap,"motor_right_rear", "REVERSE", "BRAKE", true);
        motorLeftForward = initHardware.initMotor(hardwareMap,"motor_left_forward", "FORWARD", "BRAKE", true);
        motorLeftRear = initHardware.initMotor(hardwareMap,"motor_left_rear", "FORWARD", "BRAKE", true);

        // Initiate shooter hardware
        motorArtifactIntake = initHardware.initMotor(hardwareMap,"motor_artifact_intake", "REVERSE", "FLOAT", false);
        motorMainFlywheel = initHardware.initMotor(hardwareMap,"motor_main_flywheel", "FORWARD", "FLOAT", true);
        // motorAuxFlywheel = initHardware.initMotor(hardwareMap,"motor_aux_flywheel", "FORWARD", "FLOAT", true);
        servoFlipper = initHardware.initServo(hardwareMap,"servo_flipper", SERVO_FLIPPER_STARTING_ANGLE);

        // Initiate PIDF Coefficients and apply them to the Flywheels
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        motorMainFlywheel.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        // motorAuxFlywheel.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        // Initiate LimeLight
        limelight = initHardware.initLimelight(hardwareMap, "limelight", teamColor);

        // Initiate IMU
        imu = initHardware.initIMU(hardwareMap,"imu","LEFT","UP");

        telemetry.addLine("Init complete");
    }

    private void changeTeamColor(Limelight3A limelight) {
        if (gamepad1.aWasPressed()) {
            if (teamColor == 8) {
                teamColor = 9;
            } else {
                teamColor = 8;
            }
            limelight.pipelineSwitch(teamColor);
        }
    }

    private void AutonomousControls() {
        // Drive Train controls
//        forward = gamepad1.left_stick_y;
//        strafe = gamepad1.left_stick_x;
//        rotate = gamepad1.right_stick_x;
//        driveTrain.drive(motorRightForward, motorRightRear, motorLeftForward, motorLeftRear, forward, strafe, rotate, customDriveTrainPower);

        // Get distance in inches from April Tag.  Distance returns in CMs, and the / 2.54 converts it into Inches
//        distance = aprilTag.getDistance(limelight, imu) / 2.54;

        while (!atShootingDistance) {
            driveTrain.drive(motorRightForward, motorRightRear, motorLeftForward, motorLeftRear, 0.4, strafe, rotate, customDriveTrainPower);
//            forward = 0.4;
            distance = aprilTag.getDistance(limelight, imu) / 2.54;
            if (distance > 35.0) {
                atShootingDistance = true;
                driveTrain.drive(motorRightForward, motorRightRear, motorLeftForward, motorLeftRear, 0.0, strafe, rotate, customDriveTrainPower);
//                forward = 0.0;
            }
        }



//        if (distance < 35.0 && !isFinished) {
//            // go backwards
//            driveTrain.drive(motorRightForward, motorRightRear, motorLeftForward, motorLeftRear, 1.0, strafe, rotate, customDriveTrainPower);
//        } else if (distance >= 30.0 && !isFinished) {
//            // stop
//            driveTrain.drive(motorRightForward, motorRightRear, motorLeftForward, motorLeftRear, 0.0, strafe, rotate, customDriveTrainPower);
//        }

        // Start flywheel to shoot artifacts and don't stop till shooting sequence is complete
        if (distance >= 30.0 && !isFinishedShooting) {
            motorMainFlywheel.setVelocity(flywheelTargetVelocity);  // Command the motor to run at the current target velocity.
            sleep(1000);  // Give the flywheel 1 second to spin up to target velocity
        }

        while (artifactIntakeCount != 0 && distance >= 30.0) {
            shootArtifact();
            artifactIntakeCount--;
        }

        // Shake robot in case last artifact didn't fire
        if (artifactIntakeCount == 0 && !isFinishedShooting) {
            motorArtifactIntake.setPower(1);
            driveTrain.drive(motorRightForward, motorRightRear, motorLeftForward, motorLeftRear, 0.4, strafe, rotate, customDriveTrainPower);
            sleep(250);
            motorArtifactIntake.setPower(0);
            driveTrain.drive(motorRightForward, motorRightRear, motorLeftForward, motorLeftRear, 0.0, strafe, rotate, customDriveTrainPower);
            sleep(100);
            shootArtifact();
            isFinishedShooting = true;
        }

        if (isFinishedShooting && !finalMove) {
            motorMainFlywheel.setPower(0);
            motorArtifactIntake.setPower(0);
            // Where the robot should move out of the way depending on team color
            if (teamColor == 8) {
                driveTrain.drive(motorRightForward, motorRightRear, motorLeftForward, motorLeftRear, forward, -0.4, rotate, customDriveTrainPower);
            } else {
                driveTrain.drive(motorRightForward, motorRightRear, motorLeftForward, motorLeftRear, forward, 0.4, rotate, customDriveTrainPower);
            }
            sleep (2000);
            driveTrain.drive(motorRightForward, motorRightRear, motorLeftForward, motorLeftRear, 0.0, strafe, rotate, customDriveTrainPower);
            finalMove = true;
        }


    }

    // Artifact launch sequence
    private void shootArtifact() {
        motorArtifactIntake.setPower(0);
        servoFlipper.setPosition(SERVO_FLIPPER_ENDING_ANGLE);  // Use the servo arm to lift the artifact up to the main flywheel
        sleep(SERVO_FLIPPER_TRAVEL_TIME);  // Time it takes for the servo to fully lift the artifact
        servoFlipper.setPosition(SERVO_FLIPPER_STARTING_ANGLE);  // Reset the servo arm to its starting position
        sleep(SERVO_FLIPPER_TRAVEL_TIME);  // Time it takes for the servo to fully lift the artifact
        motorArtifactIntake.setPower(1);  // Activate the artifact intake motor to move any artifacts further into the robot
        sleep(500);
    }

    // Allows the user to set their preferred drivetrain power
    private void initTelemetry() {
        NumberFormat percentFormatter = NumberFormat.getPercentInstance();
        percentFormatter.setMaximumFractionDigits(0); // e.g., 2 decimal places

        if (teamColor == 8) {
            telemetry.addData("Team Color", "Red");
        } else {
            telemetry.addData("Team Color", "Blue");
        }
        telemetry.addData("---------------", "-------------");
        telemetry.addData("Press A", "Change Team Color");
        telemetry.update();
    }

    // Display during teleop mode
    private void opModeTelemetry() {
        telemetry.addData("Artifact Intake Count", artifactIntakeCount);
        if (distanceTimer.getElapsedTimeSeconds() > DISTANCE_TIMEOUT_SECONDS && teamColor == 8) {
            telemetry.addData("Distance in inch", "Cannot see Red Team April tag");
        } else if (distanceTimer.getElapsedTimeSeconds() > DISTANCE_TIMEOUT_SECONDS  && teamColor == 9) {
            telemetry.addData("Distance in inch", "Cannot see Blue Team April tag");
        } else {
            telemetry.addData("Distance in inch", distance);
        }
        telemetry.addData("Flywheel Actual Velocity", motorMainFlywheel.getVelocity());
        telemetry.addData("Flywheel Target Velocity", flywheelTargetVelocity);
        telemetry.update();
    }
}
