package org.firstinspires.ftc.teamcode.newrobot;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class AprilTag {

    private double taMax = -100.0;
    private double taMin = 100.0;
    double scale = 166.8733;
    double power = -0.5951379;

    public double getDistance(
            Limelight3A limelight,
            IMU imu
    ) {
        double distance;

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
            double taAverage = (taMax + taMin) / 2;
            Pose3D botPose = llResult.getBotpose_MT2();


            distance = getDistanceFromTarget((llResult.getTa()));
        } else {
            distance = 0.0;
        }
        return distance;
    }

    public double getDistanceFromTarget (double ta) {
//        double scale = 1855.174;
//        double power = -1.466193;
//        double scale = 166.8733;
//        double power = -0.5951379;
        double distance = scale * Math.pow(ta, power);
        return distance;
    }
}