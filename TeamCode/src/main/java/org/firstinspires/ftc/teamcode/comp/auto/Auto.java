package org.firstinspires.ftc.teamcode.comp.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.configs.Configuration;
import org.firstinspires.ftc.teamcode.configs.LaunchSystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.comp.tele.Tele;

@Autonomous(name = "Auto ")
public class Auto extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private int pathState = 0;
    private final Pose startPose = new Pose(56, 8, Math.toRadians(90));
    private LaunchSystem launchSystem;
    private PathChain scorePreload;

    public void buildPaths() {
            }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
        }
    }

    public void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        launchSystem = new LaunchSystem(new Configuration(hardwareMap));

        buildPaths();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        launchSystem.updateTurret(follower.getPose());
        autonomousPathUpdate();
        telemetry.addData("Path State", pathState);
        telemetry.addData("turret ticks: ", launchSystem.getTurretTicks());
        telemetry.update();
    }

    @Override
    public void stop(){
        Tele.startPose = follower.getPose();
    }
}