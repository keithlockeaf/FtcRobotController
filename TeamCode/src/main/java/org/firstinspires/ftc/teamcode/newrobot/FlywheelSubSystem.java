package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@SuppressWarnings({"FieldMayBeFinal","FieldCanBeLocal"})

@Disabled
@TeleOp(name = "Limelight Flywheel Aim")
public class FlywheelSubSystem extends LinearOpMode {

    private Limelight3A limelight;
    private DcMotor flywheel;
    private Servo servoFlipper;
    private double servoFlipperStartingAngle = 0.93;
    private double servoFlipperEndingAngle = 0.7;
    private int SERVO_FLIPPER_TRAVEL_TIME = 250;

    // Constants to tune (Update these for your specific robot/game)
    private final double STEER_GAIN = 0.03;      // Proportional gain for turning
    private final double FLYWHEEL_SCALE = 5000.0; // Example scale factor for distance to ticks
    private final int APRIL_TAG_PIPELINE = 8;   // The pipeline you configured

    @Override
    public void runOpMode() {
        // 1. Initialize Hardware
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        flywheel = hardwareMap.get(DcMotor.class, "motor_main_flywheel");
//        servoFlipper = initHardware.initServo(hardwareMap,"servo_flipper", servoFlipperStartingAngle);
        servoFlipper = hardwareMap.get(Servo.class, "servo_flipper");

        // Ensure the motor runs without encoders or set to run with encoders
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // 2. Start the Limelight & Set Pipeline
        limelight.start();
        limelight.pipelineSwitch(APRIL_TAG_PIPELINE);

        telemetry.addData("Status", "Initialized. Waiting for Start.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.aWasPressed()) {
                servoFlipper.setPosition(servoFlipperEndingAngle);
                sleep(SERVO_FLIPPER_TRAVEL_TIME);
                servoFlipper.setPosition(servoFlipperStartingAngle);
                sleep(SERVO_FLIPPER_TRAVEL_TIME);
            }

            // 3. Get the latest detection result
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                // We have a valid AprilTag in view

                // --- AIMING SECTION ---
                // tx = Horizontal offset from crosshair to target in degrees (-29.8 to 29.8)
                double tx = result.getTx();
                double turnPower = tx * STEER_GAIN;

                // --- FLYWHEEL SECTION ---
                // ty = Vertical offset from crosshair to target in degrees
                // Use a curve/scale based on your team's specific distance-to-area relationship
                double ty = result.getTy();

                // Simple distance estimation based on vertical angle
                // (This is just an example; your game's formula may differ)
                double targetSpeed = FLYWHEEL_SCALE / Math.abs(ty);
                flywheel.setPower(targetSpeed);

                telemetry.addData("Target Found!", result.getFiducialResults());
                telemetry.addData("TX (Offset)", tx);
                telemetry.addData("TY (Vertical)", ty);
                telemetry.addData("Flywheel Power", targetSpeed);
            } else {
                // No target detected; stop/idle
                flywheel.setPower(0);
                telemetry.addData("Vision", "No valid AprilTag found.");
            }

            telemetry.update();
        }
        limelight.stop();
    }
}
