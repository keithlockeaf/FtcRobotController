package org.firstinspires.ftc.teamcode.newrobot;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Disabled
@TeleOp
public class AprilTagLimelightTest extends OpMode {
    private Limelight3A limelight;
    private IMU imu;
    private double taMax = -100.0;
    private double taMin = 100.0;
    private double taAverage;
    private double distance;


    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8);
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            if (taMax < llResult.getTa()) {
                taMax = llResult.getTa();
            }
            if (taMin > llResult.getTa()) {
                taMin = llResult.getTa();
            }
            taAverage = (taMax + taMin) / 2;
            Pose3D botPose = llResult.getBotpose_MT2();

            distance = getDistanceFromTarget((llResult.getTa()));
            telemetry.addData("Distance in cm", distance);
            telemetry.addData("Distance in in", distance / 2.54);
            telemetry.addData("Tx", llResult.getTx());
            telemetry.addData("Ty", llResult.getTy());
            telemetry.addData("Ta", llResult.getTa());
            telemetry.addData("taMax", taMax);
            telemetry.addData("taMin", taMin);
            telemetry.addData("taAverage", taAverage);
            telemetry.addData("BotPose", botPose.toString());
            telemetry.addData("Yaw", botPose.getOrientation().getYaw());

        } else {
            telemetry.addData("llResult", llResult);
        }

    }

    public double getDistanceFromTarget (double ta) {
//        double scale = 1855.174;
//        double power = -1.466193;
        double scale = 166.8733;
        double power = -0.5951379;
        double distance = scale * Math.pow(ta, power);
        return distance;
    }
}
