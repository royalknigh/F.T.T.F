package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .forwardZeroPowerAcceleration(-34.39)
            .lateralZeroPowerAcceleration(-64.1) //-78.62  -68.60  -55.29   -74.72   -43.69
            .translationalPIDFCoefficients(new PIDFCoefficients(0.05, 0, 0.01, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(0.6, 0, 0.01, 0.005))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.01, 0, 0.00001, 0.6, 0.01))
            .centripetalScaling(0.00043)
            .mass(12);


    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .threeWheelLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("fr")
            .rightRearMotorName("br")
            .leftRearMotorName("bl")
            .leftFrontMotorName("fl")
            .xVelocity(77.82)
            .yVelocity(62.59)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
            .forwardTicksToInches(.002)//.001981480234
            .strafeTicksToInches(.002) //.00197860454
            .turnTicksToInches(.002) //0.002116752641
            .leftPodY(0)
            .rightPodY(-3.1496)
            .strafePodX(-7.08)
            .leftEncoder_HardwareMapName("fr")
            .rightEncoder_HardwareMapName("fl")
            .strafeEncoder_HardwareMapName("im")
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.REVERSE)
            .strafeEncoderDirection(Encoder.REVERSE);
}

