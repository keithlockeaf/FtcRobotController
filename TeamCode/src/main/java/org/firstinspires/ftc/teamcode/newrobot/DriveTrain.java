package org.firstinspires.ftc.teamcode.newrobot;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveTrain {

    public void drive (
            DcMotorEx motorRightForward,
            DcMotorEx motorRightRear,
            DcMotorEx motorLeftForward,
            DcMotorEx motorLeftRear,
            double forward,
            double strafe,
            double rotate,
            double driveTrainPowerCustomisation
    ) {
        double powerRightFront = forward + strafe + rotate;
        double powerRightBack = forward - strafe + rotate;
        double powerLeftFront = forward - strafe - rotate;
        double powerLeftBack = forward + strafe - rotate;

        double maxPower = driveTrainPowerCustomisation;

        maxPower = Math.max(maxPower, Math.abs(powerLeftFront));
        maxPower = Math.max(maxPower, Math.abs(powerLeftBack));
        maxPower = Math.max(maxPower, Math.abs(powerRightFront));
        maxPower = Math.max(maxPower, Math.abs(powerRightBack));

        motorLeftForward.setPower(driveTrainPowerCustomisation * (powerLeftFront / maxPower));
        motorLeftRear.setPower(driveTrainPowerCustomisation * (powerLeftBack / maxPower));
        motorRightForward.setPower(driveTrainPowerCustomisation * (powerRightFront / maxPower));
        motorRightRear.setPower(driveTrainPowerCustomisation * (powerRightBack / maxPower));
    }
}
