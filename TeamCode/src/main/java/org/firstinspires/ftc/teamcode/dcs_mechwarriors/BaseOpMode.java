package org.firstinspires.ftc.teamcode.dcs_mechwarriors;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Control Hub Hardware Profile:
 *   Hardware Variables:
 *     DcMotor:  motor_left_frontr
 * Syntax types:
 *   Class - Pascal Case:     ThisIsPascalCase
 *   Variables - Camel Case:  thisIsCamelCase
 *   Hardware - Snake Case:   this_is_snake_case
 *   Constants - Upper Case:  THIS_IS_UPPER_CASE
 *
 *
 */


@Disabled
@TeleOp(name="_Main", group="Iterative OpMode")
public class BaseOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        while (!isStarted()) {}
        waitForStart();
        while (opModeIsActive()) {}
    }

    public void initHardware() {
    }
}

