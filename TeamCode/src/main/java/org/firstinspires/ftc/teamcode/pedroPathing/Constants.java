package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10.5) // Weight of the robot in kgs.  example:  5.1
            ;

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rf")  // Update with correct motor names
            .rightRearMotorName("rr")  // Update with correct motor names
            .leftRearMotorName("lr")  // Update with correct motor names
            .leftFrontMotorName("lf")  // Update with correct motor names
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            ;

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-5)  // Update distance in inches from center of robot to dead wheel
            .strafePodX(0.5)  // Update distance in inches from center of robot to dead wheel
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)  // physical test of encoder direction needed.  Follow instruction in https://www.youtube.com/watch?v=vihb2LPtSK0
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)  // physical test of encoder direction needed.  Follow instruction in https://www.youtube.com/watch?v=vihb2LPtSK0
            ;
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build()
                ;
    }
}
