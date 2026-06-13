package org.firstinspires.ftc.teamcode.dcs_mechwarriors;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

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
public class Test extends LinearOpMode {
    // Define Global Variables/Members/Fields
    // int, double, boolean, are the standard vars for FTC

    // batteryVoltageSensor is used to determine the voltage of the battery (its power level)
    private VoltageSensor batteryVoltageSensor;

    @Override
    public void runOpMode() throws InterruptedException {

        // What happens when you press "init" on the driver station
        // Initialize the hardware
        initHardware();

        while (!isStarted()) {
            // While waiting to start (play) do this
            sensorTelemetry();
        }

        waitForStart(); // Wait for the play button to be pressed

        while (opModeIsActive()) {
            sensorTelemetry();
        }
    }

    public void initHardware() {
        // All hardware should be initialized here (Motors, Sensors, etc.)
        // Each piece of hardware should have its own init method
        initVoltageSensor();
    }

    public void initVoltageSensor() {
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    public void sensorTelemetry() {
        telemetry.addData("Battery Voltage", batteryVoltageSensor.getVoltage());
        telemetry.update();
    }
}



