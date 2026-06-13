package org.firstinspires.ftc.teamcode.newrobot;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
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

@TeleOp(name="Drive Training",group="Training")
public class Training_Driving_Only extends LinearOpMode {

    //    DriveTrain Hardware Variables
    private DcMotorEx motorRightForward;
    private DcMotorEx motorRightRear;
    private DcMotorEx motorLeftForward;
    private DcMotorEx motorLeftRear;

    //    User Power Customisation for the DriveTrain
    private boolean lbWasPressed = false;
    private boolean rbWasPressed = false;
    private double driveTrainPowerCustomisation = 0.5;

    //    Artifact Intake Counter Variables
    private DigitalChannel sensorLaserDistance;
    private int artifactIntakeCount = 0;
    private boolean lastState = false;

    //    Shooter Hardware Variables
//    private DcMotorEx motorTurret;
    DcMotorEx motorArtifactIntake;
    DcMotorEx motorMainFlywheel;
//    DcMotorEx motorAuxFlywheel;
    Servo servoFlipper;

    //    Shooter Software Variables
    double flywheelTargetVelocity;
    double F = 14.098; // Feedforward gain to counteract constant forces like friction.
    double P = 265;    // Proportional gain to correct error based on how far off the velocity is.
    private double servoFlipperStartingAngle = 0.93;
    private double servoFlipperEndingAngle = 0.7;
    private int SERVO_FLIPPER_TRAVEL_TIME = 250;
    double adjustFocusPower = 0.5;

    //    LimeLight Variables
    private Limelight3A limelight;
    private IMU imu;
    private double distance;
    private int limelightIndexToUse = 8;

    AprilTag aprilTag = new AprilTag();
    DriveTrain driveTrain = new DriveTrain();

    double forward, strafe, rotate;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        while (!isStarted()) {
            setUserDrivetrainPower();
            setUserDrivetrainPowerTelemetry();
        }
        waitForStart();
        limelight.start();
        while (opModeIsActive()) {
            teleOpControls();
            opModeTelemetry();
        }
    }

    public void initHardware() {
//        Generate an instance of the Initiate_Hardware file that is used to set up all the hardware for the robot
        Initiate_Hardware initHardware = new Initiate_Hardware();

//        Initiate drive train hardware (all 4 motors that control the wheels)
        motorRightForward = initHardware.initMotor(hardwareMap,"motor_right_forward", "REVERSE", "BRAKE", true);
        motorRightRear = initHardware.initMotor(hardwareMap,"motor_right_rear", "REVERSE", "BRAKE", true);
        motorLeftForward = initHardware.initMotor(hardwareMap,"motor_left_forward", "FORWARD", "BRAKE", true);
        motorLeftRear = initHardware.initMotor(hardwareMap,"motor_left_rear", "FORWARD", "BRAKE", true);

//        Initiate laser distance sensor (counts the number of artifacts gathered)
        sensorLaserDistance = hardwareMap.get(DigitalChannel.class, "sensor_laser_distance");
        sensorLaserDistance.setMode(DigitalChannel.Mode.INPUT);

//        Initiate shooter hardware
        motorArtifactIntake = initHardware.initMotor(hardwareMap,"motor_artifact_intake", "REVERSE", "FLOAT", false);
        motorMainFlywheel = initHardware.initMotor(hardwareMap,"motor_main_flywheel", "FORWARD", "FLOAT", true);
//        motorAuxFlywheel = initHardware.initMotor(hardwareMap,"motor_aux_flywheel", "FORWARD", "FLOAT", true);
        servoFlipper = initHardware.initServo(hardwareMap,"servo_flipper", servoFlipperStartingAngle);

//        Initiate PIDF Coefficients and apply them to the Flywheels
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        motorMainFlywheel.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);
//        motorAuxFlywheel.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);

//        Initiate LimeLight
        limelight = initHardware.initLimelight(hardwareMap, "limelight", limelightIndexToUse);

//        Initiate IMU
        imu = initHardware.initIMU(hardwareMap,"imu","LEFT","UP");

        telemetry.addLine("Init complete");
    }

    //        Configure preferred custom drive train max power
    private void setUserDrivetrainPower() {
        boolean lbPressed = gamepad1.left_bumper;
        if (lbPressed && !lbWasPressed) {
            if (driveTrainPowerCustomisation != 0) {
                driveTrainPowerCustomisation -= 0.1;
            }
        }
        lbWasPressed = lbPressed;

        boolean rbPressed = gamepad1.right_bumper;
        if (rbPressed && !rbWasPressed) {
            if (driveTrainPowerCustomisation != 1.0) {
                driveTrainPowerCustomisation += 0.1;
            }
        }
        rbWasPressed = rbPressed;
    }

    private void teleOpControls() {
//        Drive Train controls
        forward = gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;
        driveTrain.drive(motorRightForward, motorRightRear, motorLeftForward, motorLeftRear, forward, strafe, rotate, driveTrainPowerCustomisation);

//        Artifact Intake
        boolean artifactDetected = sensorLaserDistance.getState();
        if (artifactDetected && !lastState) {
            artifactIntakeCount++;
        }
        lastState = artifactDetected;

        if (artifactIntakeCount < 3) {
            motorArtifactIntake.setPower(1);
        } else {
            sleep(250);  // sleep for 0.25 second before disabling intake so the 3rd artifact can get fully pulled in
            motorArtifactIntake.setPower(0);
        }

//        Get distance in inches from April Tag.  Distance returns in CMs, and the / 2.54 converts it into Inches
        distance = aprilTag.getDistance(limelight, imu) / 2.54;

//        Determine flywheels target velocity based on distance
        if (0.0 == distance) {
            flywheelTargetVelocity = 0.0;
        } else if (20.0 <= distance && distance < 40.0) {
            flywheelTargetVelocity = 1300;
        } else if (40.0 <= distance && distance < 50.0) {
            flywheelTargetVelocity = 1400;
        } else if (50.0 <= distance && distance < 70.0) {
            flywheelTargetVelocity = 1450;
//        } else if (60.0 <= distance && distance < 70.0) {
//            flywheelTargetVelocity = 1400;
        }

        // D-pad left/right adjusts the angle of the robot at small increments
        if (gamepad1.dpadLeftWasPressed()) {
            motorLeftRear.setPower(adjustFocusPower);
            motorRightRear.setPower(-adjustFocusPower);
            sleep(200);
            motorLeftRear.setPower(0);
            motorRightRear.setPower(0);
        }
        if (gamepad1.dpadRightWasPressed()) {
            motorLeftRear.setPower(-adjustFocusPower);
            motorRightRear.setPower(adjustFocusPower);
            sleep(200);
            motorLeftRear.setPower(0);
            motorRightRear.setPower(0);
        }

//        User ready to launch artifact at goal
        if (gamepad1.aWasPressed() || gamepad2.aWasPressed()) {
            motorMainFlywheel.setVelocity(flywheelTargetVelocity);  // Command the motor to run at the current target velocity.
            sleep (1000);  // Give the flywheel 1 second to spin up to target velocity
            motorArtifactIntake.setPower(1);  // Activate artifact intake motor to push any artifacts towards launch servo
            sleep(250);  // Only run the artifact intake motor for .25 seconds
            shootArtifact();  // Start artifact firing sequence
            sleep(1000);  // Give the flywheel 1 second to continue to spin at target velocity before turning off power
            artifactIntakeCount = 0;  // Reset artifact count
            motorMainFlywheel.setVelocity(0);  // Turn off Main flywheel
        }
    }

    // Artifact launch sequence
    private void shootArtifact() {
        motorArtifactIntake.setPower(0);
        servoFlipper.setPosition(servoFlipperEndingAngle);  // Use the servo arm to lift the artifact up to the main flywheel
        sleep(SERVO_FLIPPER_TRAVEL_TIME);  // Time it takes for the servo to fully lift the artifact
        servoFlipper.setPosition(servoFlipperStartingAngle);  // Reset the servo arm to its starting position
        sleep(SERVO_FLIPPER_TRAVEL_TIME);  // Time it takes for the servo to fully lift the artifact
        motorArtifactIntake.setPower(1);  // Activate the artifact intake motor to move any artifacts further into the robot
        sleep(250);
    }

    private void setUserDrivetrainPowerTelemetry() {
        NumberFormat percentFormatter = NumberFormat.getPercentInstance();
        percentFormatter.setMaximumFractionDigits(0); // e.g., 2 decimal places

        telemetry.addData("Set DriveTrain Power @", percentFormatter.format(driveTrainPowerCustomisation));
        telemetry.addData("Left Bumper", "Decrease DriveTrain Power by 10%");
        telemetry.addData("Right Bumper", "Increase DriveTrain Power by 10%");
        telemetry.update();
    }

    private void opModeTelemetry() {
        telemetry.addData("Artifact Intake Count", artifactIntakeCount);
        if (distance == 0.0) {
            telemetry.addData("Distance in inch", "Cannot see April Tag0");
        } else {
            telemetry.addData("Distance in inch", distance);
        }
        telemetry.addData("Flywheel Velocity", flywheelTargetVelocity);
        telemetry.addData("---------------", "-------------");
        telemetry.addData("Controls", "Listed Below");
        telemetry.addData("Left Joystick Up", "Forward");
        telemetry.addData("Left Joystick Down", "Backwards");
        telemetry.addData("Left Joystick Left", "Strafe Left");
        telemetry.addData("Left Joystick Right", "Strafe Right");
        telemetry.addData("Right Joystick Left", "Rotate Left");
        telemetry.addData("Right Joystick Right", "Rotate Right");
        telemetry.addData("Button A", "Shoot");
        telemetry.addData("D Pad Left", "Rotate left 5%");
        telemetry.addData("D Pad Right", "Rotate right 5%");
        telemetry.update();
    }
}
