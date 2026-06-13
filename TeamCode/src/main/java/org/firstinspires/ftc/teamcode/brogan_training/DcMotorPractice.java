package org.firstinspires.ftc.teamcode.brogan_training;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.brogan_training.mechanisms.DecodeHardwareSetup;

@Disabled
@TeleOp
public class DcMotorPractice extends OpMode {

    DecodeHardwareSetup robot = new DecodeHardwareSetup();

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        double motorSpeed = gamepad1.left_stick_y;
        robot.setMotorSpeed(motorSpeed);
    }
}
