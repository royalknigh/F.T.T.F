package org.firstinspires.ftc.teamcode.comp.tele;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.configs.Configuration;
import org.firstinspires.ftc.teamcode.configs.LaunchSystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

@Configurable
@TeleOp
public class Tele extends OpMode {
    private Follower follower;
    private LaunchSystem launchSystem;
    public static Pose startPose; // Hand-off variable

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        if (startPose != null) {
            follower.setStartingPose(startPose);
            startPose = null;
        } else {
            follower.setStartingPose(new Pose(8, 7, Math.toRadians(180)));
        }

        launchSystem = new LaunchSystem(new Configuration(hardwareMap));
    }

    @Override
    public void loop() {
        follower.update();
        launchSystem.updateTurret(follower.getPose());
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);


        telemetry.update();
    }
}