package org.firstinspires.ftc.teamcode.newrobot;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;


public class Initiate_Hardware {

    DcMotorEx initMotor(
            HardwareMap hwMap,
            String motorHardwareName,
            String motorDirection,
            String breakBehavior,
            boolean useEncoder
    ) {

        if (!"FORWARD".equalsIgnoreCase(motorDirection) && !"REVERSE".equalsIgnoreCase(motorDirection)) {
            throw new IllegalArgumentException("Only 'FORWARD' or 'REVERSE' are allowed for motor direction.");
        }
        DcMotorEx motor;
        motor = hwMap.get(DcMotorEx.class, motorHardwareName);
        motor.setPower(0);

//        Default Direction is FORWARD
        if (motorDirection.equalsIgnoreCase("REVERSE")) {
            motor.setDirection(DcMotorEx.Direction.REVERSE);
        }

//        Default ZeroPowerBehavior is FLOAT
        if (breakBehavior.equalsIgnoreCase("Break")) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

//        If useEncoder is set to true
        if (useEncoder) {
            motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }

        return motor;
    }

    DcMotorEx initTurretMotor(
            HardwareMap hwMap,
            String motorHardwareName,
            double kP,
            double kI,
            double kD,
            double F,
            double position
    ) {
        DcMotorEx motor;
        motor = hwMap.get(DcMotorEx.class, motorHardwareName);
        motor.setPower(0);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motor.setVelocityPIDFCoefficients(kP, kI, kD, F);
        motor.setPositionPIDFCoefficients(position);
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        return motor;
    }

    Servo initServo(
            HardwareMap hwMap,
            String servoHardwareName,
            double servoStartingAngle
    ) {
        Servo servo = hwMap.get(Servo.class, servoHardwareName);
        servo.setPosition(servoStartingAngle);
        return servo;
    }

    Limelight3A initLimelight(
            HardwareMap hwMap,
            String limelightHardwareName,
            int pipelineIndexToUse
    ) {
        Limelight3A limelight = hwMap.get(Limelight3A.class, limelightHardwareName);
        limelight.pipelineSwitch(pipelineIndexToUse);
        return limelight;
    }

    IMU initIMU(
            HardwareMap hwMap,
            String imuDeviceName,
            String logoFacingDirection,
            String usbFacingDirection
    ) {
        IMU imu = hwMap.get(IMU.class, imuDeviceName);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD; // default value
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD; // default value

        switch (logoFacingDirection) {
//            FORWARD is set by default
            case "BACKWARD": logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD; break;
            case "UP": logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP; break;
            case "DOWN":  logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.DOWN; break;
            case "RIGHT":  logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT; break;
            case "LEFT":  logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT; break;
        }

        switch (usbFacingDirection) {
//            FORWARD is set by default
            case "BACKWARD": usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD; break;
            case "UP": usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP; break;
            case "DOWN": usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.DOWN; break;
            case "RIGHT": usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT; break;
            case "LEFT": usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.LEFT; break;
        }

        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
        return imu;
    }
}
