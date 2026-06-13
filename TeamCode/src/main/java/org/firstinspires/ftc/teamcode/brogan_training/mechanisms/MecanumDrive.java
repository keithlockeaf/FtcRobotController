package org.firstinspires.ftc.teamcode.brogan_training.mechanisms;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumDrive {

    private DcMotor motorLeftFront;
    private DcMotor motorLeftBack;
    private DcMotor motorRightFront;
    private DcMotor motorRightBack;

    private IMU imu;

    public void init(HardwareMap hwMap) {
        motorLeftFront = hwMap.get(DcMotor.class, "motor_left_front");
        motorLeftBack = hwMap.get(DcMotor.class, "motor_left_back");
        motorRightFront = hwMap.get(DcMotor.class, "motor_right_front");
        motorRightBack = hwMap.get(DcMotor.class, "motor_right_back");

        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorRightFront.setDirection(DcMotor.Direction.REVERSE);
        motorRightBack.setDirection(DcMotor.Direction.REVERSE);

        imu = hwMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );

        imu.initialize(new IMU.Parameters(RevOrientation));
    }

    public void drive (double forward, double strafe, double rotate) {
        double powerRightFront = forward + strafe + rotate;
        double powerRightBack = forward - strafe + rotate;
        double powerLeftFront = forward - strafe - rotate;
        double powerLeftBack = forward + strafe - rotate;

        double maxPower = 0.3;
        double maxSpeed = 0.3;

        maxPower = Math.max(maxPower, Math.abs(powerLeftFront));
        maxPower = Math.max(maxPower, Math.abs(powerLeftBack));
        maxPower = Math.max(maxPower, Math.abs(powerRightFront));
        maxPower = Math.max(maxPower, Math.abs(powerRightBack));

        motorLeftFront.setPower(maxSpeed * (powerLeftFront / maxPower));
        motorLeftBack.setPower(maxSpeed * (powerLeftBack / maxPower));
        motorRightFront.setPower(maxSpeed * (powerRightFront / maxPower));
        motorRightBack.setPower(maxSpeed * (powerRightBack / maxPower));
    }

    public void driveFieldRelative(double forward, double strafe, double rotate) {
        double theta = Math.atan2(forward, strafe);
        double r = Math.hypot(forward, strafe);

        theta = AngleUnit.normalizeRadians( theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        double newForward = r * Math.sin(theta);
        double newStrafe = r * Math.cos(theta);

        this.drive(newForward, newStrafe, rotate);
    }
}
