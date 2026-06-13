package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@TeleOp(group="Testing LinearOpMode")
public class Keith_Main_TeleOP_Template extends LinearOpMode {

    DcMotor motorLeftFront;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        while (!isStarted()) {
//            telemetry();
        }
        waitForStart();
        while (opModeIsActive()) {
//            telemetry();
        }
    }
    public void initHardware() {
        initMotorLeftFront();
    }

    private void initMotorLeftFront() {
        motorLeftFront = hardwareMap.get(DcMotor.class, "motorLeftFront");
        motorLeftFront.setDirection(DcMotor.Direction.FORWARD);
//        motorLeftFront.setPosition(servoOneInitPosition);
    }
}

