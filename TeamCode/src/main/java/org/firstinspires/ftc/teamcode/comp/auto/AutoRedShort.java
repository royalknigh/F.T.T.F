package org.firstinspires.ftc.teamcode.comp.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.comp.tele.Tele;
import org.firstinspires.ftc.teamcode.configs.Configuration;
import org.firstinspires.ftc.teamcode.configs.LaunchSystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Auto Red Short")
public class AutoRedShort extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private int pathState = 0;
//    private final Pose startPose = new Pose(56, 8, Math.toRadians(90));
//    public final Pose redGoalPose = new Pose(130, 136);
    private final Pose startPose = new Pose(117, 120, Math.toRadians(37));
    private final Pose scorePose = new Pose(85, 89, Math.toRadians(30));
    private final Pose fisrtLinePose = new Pose(96, 82, Math.toRadians(0));
    private final Pose pickup1Pose = new Pose(124, 82, Math.toRadians(0));
    private final Pose secondLinePose = new Pose(96, 59, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(131, 59, Math.toRadians(0));
    private final Pose openGatePose = new Pose(125, 65, Math.toRadians(0)); //gate
    private final Pose thirdLinePose = new Pose(96, 35, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(131, 35, Math.toRadians(0));

    private Configuration configuration;

    private LaunchSystem launchSystem;
    private PathChain scorePreload, alignRow1, pickupRow1, score1, alignRow2, pickupRow2,
            openGate, score2, alignRow3, pickupRow3, score3, park;

    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        alignRow1 = follower.pathBuilder().addPath(new BezierLine(scorePose, fisrtLinePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), fisrtLinePose.getHeading())
                .build();

        pickupRow1 = follower.pathBuilder().addPath(new BezierLine(fisrtLinePose, pickup1Pose))
                .setLinearHeadingInterpolation(fisrtLinePose.getHeading(), pickup1Pose.getHeading())
                .build();

        score1 = follower.pathBuilder().addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        alignRow2 = follower.pathBuilder().addPath(new BezierLine(scorePose, secondLinePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), secondLinePose.getHeading())
                .build();

        pickupRow2 = follower.pathBuilder().addPath(new BezierLine(secondLinePose, pickup2Pose))
                .setLinearHeadingInterpolation(secondLinePose.getHeading(), pickup2Pose.getHeading())
                .build();

        openGate = follower.pathBuilder()
                .addPath(new BezierCurve(pickup2Pose, new Pose(114, 61), openGatePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), openGatePose.getHeading()).build();

        score2 = follower.pathBuilder().addPath(new BezierLine(openGatePose, scorePose))
                .setLinearHeadingInterpolation(openGatePose.getHeading(), scorePose.getHeading())
                .build();

        alignRow3 = follower.pathBuilder().addPath(new BezierLine(scorePose, thirdLinePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), thirdLinePose.getHeading())
                .build();

        pickupRow3 = follower.pathBuilder().addPath(new BezierLine(thirdLinePose, pickup3Pose))
                .setLinearHeadingInterpolation(thirdLinePose.getHeading(), pickup3Pose.getHeading())
                .build();

        score3 = follower.pathBuilder().addPath(new BezierLine(pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, secondLinePose))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if(!follower.isBusy()) {
                    follower.followPath(scorePreload);
                    setPathState(1);
                }
                break;
            case 1:
                if(!follower.isBusy()) {
                    follower.followPath(alignRow1);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    follower.followPath(pickupRow1);
                    configuration.intakeMotor.setPower(0.8);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    follower.followPath(score1);
                    configuration.intakeMotor.setPower(0);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    follower.followPath(alignRow2);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    follower.followPath(pickupRow2);
                    configuration.intakeMotor.setPower(0.8);
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    follower.followPath(openGate);
                    configuration.intakeMotor.setPower(0);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    follower.followPath(score2);
                    setPathState(8);
                }
                break;
            case 8:
                if(!follower.isBusy()) {
                    follower.followPath(alignRow3);
                    setPathState(9);
                }
                break;
            case 9:
                if(!follower.isBusy()) {
                    follower.followPath(pickupRow3);
                    configuration.intakeMotor.setPower(0.8);
                    setPathState(10);
                }
                break;
            case 10:
                if(!follower.isBusy()) {
                    follower.followPath(score3);
                    configuration.intakeMotor.setPower(0);
                    setPathState(11);
                }
                break;
            case 11:
                if(!follower.isBusy()) {
                    follower.followPath(park);
                    setPathState(-1);
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
        telemetry.update();
    }

    @Override
    public void stop(){
        Tele.startPose = follower.getPose();
    }

//    public void
}