package org.firstinspires.ftc.teamcode.newrobot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@SuppressWarnings({"FieldMayBeFinal","FieldCanBeLocal"})
@Disabled
@TeleOp(group="Test_Group")
public class Shooter_Test extends LinearOpMode {

    private double servoFlipperStartingAngle = 0.93;
    private double servoFlipperEndingAngle = 0.7;
    private int TRAVEL_TIME = 250;
    private boolean bumperWasPressed = false;


    private Servo servoFlipper;

    @Override
    public void runOpMode() {


        servoFlipper = hardwareMap.get(Servo.class, "servo_flipper");

        servoFlipper.setPosition(servoFlipperStartingAngle);



        waitForStart();

        while (opModeIsActive()) {
            boolean bumperPressed = gamepad1.right_bumper;

            // Detect rising edge (press, not hold)
            if (bumperPressed && !bumperWasPressed) {
                servoFlipper.setPosition(servoFlipperEndingAngle);

                sleep(TRAVEL_TIME);

                servoFlipper.setPosition(servoFlipperStartingAngle);
            }

            bumperWasPressed = bumperPressed;






            telemetry.addData("Servo Position", servoFlipper.getPosition());
            telemetry.update();
        }
    }
}

