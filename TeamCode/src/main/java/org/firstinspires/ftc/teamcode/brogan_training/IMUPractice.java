package org.firstinspires.ftc.teamcode.brogan_training;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.brogan_training.mechanisms.RobotIMU;

@Disabled
@TeleOp
public class IMUPractice extends OpMode {

    RobotIMU RIMU = new RobotIMU();



    @Override
    public void init() {
        RIMU.init(hardwareMap);
    }

    @Override
    public void loop() {
        telemetry.addData("Heading", RIMU.getHeading(AngleUnit.DEGREES));
    }
}
