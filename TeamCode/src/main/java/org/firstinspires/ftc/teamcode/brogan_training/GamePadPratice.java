package org.firstinspires.ftc.teamcode.brogan_training;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp
public class GamePadPratice extends OpMode {

    @Override
    public void init() {

    }

    @Override
    public void loop() {

        double speedForward = -gamepad1.left_stick_y /2.0;


        telemetry.addData("x", gamepad1.left_stick_x);
        telemetry.addData("y", speedForward);
        telemetry.addData("a", gamepad1.a);


    }
}
