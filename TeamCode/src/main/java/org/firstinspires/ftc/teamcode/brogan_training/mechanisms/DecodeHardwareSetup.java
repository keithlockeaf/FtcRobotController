package org.firstinspires.ftc.teamcode.brogan_training.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DecodeHardwareSetup {

    private DcMotor motor_right_front;

    public void init (HardwareMap hwMap) {
        motor_right_front = hwMap.get(DcMotor.class, "motor_right_front");
        motor_right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setMotorSpeed (double speed) {
        // acceptable values are between -1.0 and 1.0
        motor_right_front.setPower(speed);
    }

}
