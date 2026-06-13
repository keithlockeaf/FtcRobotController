package org.firstinspires.ftc.teamcode.brogan_training;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Disabled
@TeleOp
public class IfPractice extends OpMode {

    @Override
    public void init(){
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop(){
        boolean aButton = gamepad1.a; // press true, depressed false

        if (aButton){
            telemetry.addData("Status", "A button is pressed");
        } else {
            telemetry.addData("Status", "A button is not pressed");
        }

        telemetry.update();

    }
}
