package org.firstinspires.ftc.teamcode.newrobot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

public class Shooter {

    DcMotorEx initMotor(HardwareMap hwMap, String motorHardwareName, String motorDirection) {
        if (!"FORWARD".equalsIgnoreCase(motorDirection) && !"REVERSE".equalsIgnoreCase(motorDirection)) {
            throw new IllegalArgumentException("Only 'FORWARD' or 'REVERSE' are allowed for motor direction.");
        }
        DcMotorEx motor;
        motor = hwMap.get(DcMotorEx.class, motorHardwareName);
        motor.setPower(0);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        if (motorDirection.equalsIgnoreCase("REVERSE")) {
            motor.setDirection(DcMotorEx.Direction.REVERSE);
        }
        return motor;
    }

//    ServoControllerEx initServo(HardwareMap hwMap, String motorHardwareName, String motorDirection) {
//
//    }
}
