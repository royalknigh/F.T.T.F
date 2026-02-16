package org.firstinspires.ftc.teamcode.comp.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.configs.Configuration;
import org.firstinspires.ftc.teamcode.configs.LaunchSystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.comp.tele.Tele;

@Autonomous(name = "Auto Red Long")
public class AutoRedLong extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private int pathState = 0;
    private final Pose startPose = new Pose(89, 9, Math.toRadians(90));
    private final Pose scorePose = new Pose(84, 13, Math.toRadians(70));
    private final Pose lineup = new Pose(101, 35, Math.toRadians(0));
    private final Pose pickupPose = new Pose(132, 35, Math.toRadians(0));
    private final Pose bottomPose = new Pose(134, 9, Math.toRadians(0));
    private final Pose gatePickup = new Pose(136, 10, Math.toRadians(90));
    private Configuration configuration;
    private boolean hasStartedLaunch = false;
    private LaunchSystem launchSystem;
    private PathChain scorePreload, alignRow, pickupRow, score, pickupBottom, score2, bottomLineup;
//    startPose -> score0 ( scorePreload )
//    scorePreload -> lineup ( alignRow )
//    lineup -> rowPickup ( pickupRow )
//    rowPickup -> scorePose ( score )
//    score1 -> bottomPickup ( pickupBottom )
//    bottomPickup -> scorePose ( score2 )
//    score2 -> gatePickup ( gateLineup )
    public void buildPaths() {
            scorePreload = follower.pathBuilder()
                    .addPath(new BezierLine(startPose, scorePose))
                   // .addParametricCallback(0.5, () -> launchSystem.start(LaunchSystem.highVelocity, interval)) -> change 41, 51
                    .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                    .build();
            alignRow = follower.pathBuilder()
                    .addPath(new BezierLine(scorePose, lineup))
                    .setLinearHeadingInterpolation(scorePose.getHeading(), lineup.getHeading())
                    .build();
            pickupRow = follower.pathBuilder()
                    .addPath(new BezierLine(lineup, pickupPose))
                    .setLinearHeadingInterpolation(lineup.getHeading(), pickupPose.getHeading())
                    .build();
            score = follower.pathBuilder()
                    .addPath(new BezierLine(pickupPose, scorePose))
                    // .addParametricCallback(0.9, () -> launchSystem.start(LaunchSystem.highVelocity, interval))
                    .setLinearHeadingInterpolation(pickupPose.getHeading(), scorePose.getHeading())
                    .build();
            pickupBottom = follower.pathBuilder()
                    .addPath(new BezierLine(scorePose, bottomPose))
                    .setLinearHeadingInterpolation(scorePose.getHeading(), bottomPose.getHeading())
                    .build();

            score2 = follower.pathBuilder()
                    .addPath(new BezierLine(bottomPose, scorePose))
//                    .addParametricCallback(0.5, () -> launchSystem.start(LaunchSystem.highVelocity, interval))
                    .setLinearHeadingInterpolation(bottomPose.getHeading(), scorePose.getHeading())
                    .build();
            bottomLineup = follower.pathBuilder()
                    .addPath(new BezierLine(scorePose, gatePickup))
                    .setLinearHeadingInterpolation(scorePose.getHeading(), gatePickup.getHeading())
                    .build();
    }

    public void autonomousPathUpdate() {
        switch(pathState) {
            case 0:
                follower.followPath(scorePreload);
                follower.setMaxPower(1);
                hasStartedLaunch=false;
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    if(!hasStartedLaunch) {
                        launchSystem.start(Tele.speed);
                        hasStartedLaunch = true;
                        launchSystem.toggleTracking();
                    }
                    if(launchSystem.update(launchSystem.returnDistance(follower.getPose()))) {
                        follower.followPath(alignRow);
                        hasStartedLaunch = false;
                        launchSystem.toggleTracking();
                        setPathState(2);
                    }
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(pickupRow);
                    configuration.intakeMotor.setPower(0.8);
                    follower.setMaxPower(0.4);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(score);
                    configuration.intakeMotor.setPower(0);
                    follower.setMaxPower(0.8);
                    hasStartedLaunch=false;
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    if(!launchSystem.isLaunching() && !hasStartedLaunch) {
                        launchSystem.toggleTracking();
                        launchSystem.start(Tele.speed);
                        hasStartedLaunch = true;
                    }
                    if(launchSystem.update(launchSystem.returnDistance(follower.getPose()))) {
                        follower.followPath(pickupBottom);
                        hasStartedLaunch = false;
                        launchSystem.toggleTracking();
                        setPathState(5);
                    }
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(score2);
                    configuration.intakeMotor.setPower(0);
                    follower.setMaxPower(0);
                    setPathState(6);
                }
            case 6:
                if(!follower.isBusy()) {
                    if(!launchSystem.isLaunching() && !hasStartedLaunch) {
                        launchSystem.toggleTracking();
                        launchSystem.start(Tele.speed);
                        hasStartedLaunch = true;
                    }
                    if(launchSystem.update(launchSystem.returnDistance(follower.getPose()))) {
                        follower.followPath(bottomLineup);
                        hasStartedLaunch = false;
                        launchSystem.toggleTracking();
                        setPathState(-1);
                    }
                }
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
        launchSystem = new LaunchSystem(new Configuration(hardwareMap), LaunchSystem.redGoalPose);

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
        telemetry.update();
    }

    @Override
    public void stop(){
        Tele.startPose = follower.getPose();
    }

//    public void
}