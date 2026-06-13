package org.firstinspires.ftc.teamcode.dcs_mechwarriors;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

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
@TeleOp(group="Examples")

public class ServoOne extends LinearOpMode {

    private Servo servoOne;
    private double servoOneInitPosition = 0.5;
    private double servoOnePositionOne = 0.0;
    private double servoOnePositionTwo = 1.0;
    private int servoOneDelay = 30;


    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        while (!isStarted()) {
            servoTelemetry();
        }
        waitForStart();
        while (opModeIsActive()) {
            servoTelemetry();
        }
    }

    public void initHardware() {
        initServoOne();
    }

    public void initServoOne() {
        servoOne = hardwareMap.get(Servo.class, "ServoOne");
        servoOne.setDirection(Servo.Direction.FORWARD);
        servoOne.setPosition(servoOneInitPosition);
    }

    public void teleOpControls() {
        if (gamepad1.a) {
            servoOne.setPosition(servoOnePositionOne);
        }
        if (gamepad1.b) {
            servoOne.setPosition(servoOnePositionTwo);
            sleep(servoOneDelay);
        }
        if (gamepad1.left_bumper) {
            serverOneSlower(servoOnePositionOne, servoOnePositionTwo, servoOneDelay);
        }
    }

    public void serverOneSlower(double startPosition, double endPosition, int delay) {
        double range = ((endPosition - startPosition) / 100);
        for (int i = 0; i <= range; i++) {
            servoOne.setPosition(startPosition);
            sleep(delay);
            startPosition = startPosition + 0.01;
        }
    }

    public void servoTelemetry() {
        telemetry.addData("Servo One Position", servoOne.getPosition());
        telemetry.addData("Direction", servoOne.getDirection());
        telemetry.addData("Controller", servoOne.getController());
        telemetry.addData("Port Number",servoOne.getPortNumber());
        telemetry.addData("Connection Info", servoOne.getConnectionInfo());
        telemetry.addData("Device Name", servoOne.getDeviceName());
        telemetry.addData("Manufacture", servoOne.getManufacturer());
        telemetry.addData("Version", servoOne.getVersion());
        telemetry.addData("Class", servoOne.getClass());
        telemetry.update();
    }
}

