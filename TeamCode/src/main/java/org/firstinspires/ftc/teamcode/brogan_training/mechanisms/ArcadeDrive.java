package org.firstinspires.ftc.teamcode.brogan_training.mechanisms;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArcadeDrive {

    private DcMotor motorLeftFront;
    private DcMotor motorLeftBack;
    private DcMotor motorRightFront;
    private DcMotor motorRightBack;

    public void init(HardwareMap hwMap) {
        motorLeftFront = hwMap.get(DcMotor.class, "motor_left_front");
        motorLeftBack = hwMap.get(DcMotor.class, "motor_left_rear");
        motorRightFront = hwMap.get(DcMotor.class, "motor_right_front");
        motorRightBack = hwMap.get(DcMotor.class, "motor_right_rear");

        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorLeftFront.setDirection(DcMotor.Direction.REVERSE);
        motorLeftBack.setDirection(DcMotor.Direction.REVERSE);
    }

    public void drive(double Throttle, double Spin) {
        double leftPower = Throttle + Spin;
        double rightPower = Throttle - Spin;
        double largest = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (largest > 1.0) {
            leftPower /= largest;
            rightPower /= largest;
        }

        motorLeftFront.setPower(leftPower);
        motorLeftBack.setPower(leftPower);
        motorRightFront.setPower(rightPower);
        motorRightBack.setPower(rightPower);
    
    }
}
