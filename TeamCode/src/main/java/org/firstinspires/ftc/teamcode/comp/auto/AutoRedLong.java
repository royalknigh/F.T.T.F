package org.firstinspires.ftc.teamcode.comp.auto;

import static org.firstinspires.ftc.teamcode.configs.LaunchSystem.holdBall;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.configs.Configuration;
import org.firstinspires.ftc.teamcode.configs.LaunchSystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.comp.tele.Tele;
@Disabled
@Configurable
@Autonomous(name = "Auto Red Long")
public class AutoRedLong extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private int pathState = 0;
    private final Pose startPose = new Pose(89, 9, Math.toRadians(90));
    private final Pose scorePose = new Pose(83, 21, Math.toRadians(70));
    private final Pose lineup = new Pose(132, 35.5, Math.toRadians(0));
    private final Pose pickupPose = new Pose(125, 35.5, Math.toRadians(0));
    private final Pose bottomPose = new Pose(135, 9.5, Math.toRadians(0)); // x 135
    private final Pose gatePickup = new Pose(135, 20, Math.toRadians(0));
    private Configuration configuration;
    private boolean hasStartedLaunch = false;
//    private boolean failsafe = false;
    private LaunchSystem launchSystem;
    private PathChain scorePreload, alignRow, pickupRow, score, pickupBottom,
            score2, bottomLineup, score3, leave, scoreLineup;

    private ElapsedTime launchTimer = new ElapsedTime();

    public void buildPaths() {
            scorePreload = follower.pathBuilder()
                    .addPath(new BezierLine(startPose, scorePose))
                   // .addParametricCallback(0.5, () -> launchSystem.start(LaunchSystem.highVelocity, interval)) -> change 41, 51
                    .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                    .build();
            alignRow = follower.pathBuilder()
                    .addPath(new BezierCurve(scorePose, new Pose(89, 34), lineup))
                    .addParametricCallback(0.2, () -> configuration.intakeMotor.setPower(1))

                    .setTangentHeadingInterpolation()
                    .build();
//            pickupRow = follower.pathBuilder()
//                    .addPath(new BezierLine(lineup, pickupPose))
//                    .setLinearHeadingInterpolation(lineup.getHeading(), pickupPose.getHeading())
//                    .build();
            score = follower.pathBuilder()
                    .addPath(new BezierLine(pickupPose, scorePose))
                    // .addParametricCallback(0.9, () -> launchSystem.start(LaunchSystem.highVelocity, interval))
                    .setConstantHeadingInterpolation(pickupPose.getHeading())
                    .build();
        scoreLineup = follower.pathBuilder()
                .addPath(new BezierLine(lineup, scorePose))
                // .addParametricCallback(0.9, () -> launchSystem.start(LaunchSystem.highVelocity, interval))
                .setConstantHeadingInterpolation(lineup.getHeading())
                .build();
            pickupBottom = follower.pathBuilder()
                    .addPath(new BezierLine(scorePose, bottomPose)) //116, 41
                    .addParametricCallback(0.65, () -> follower.setMaxPower(0.75))
//                    .addTemporalCallback(2000, () -> {
//                        failsafe = true;
//                    })
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            score2 = follower.pathBuilder()
                    .addPath(new BezierCurve(bottomPose, new Pose(111,  27), scorePose))
                    .addParametricCallback(0.6, () -> configuration.intakeMotor.setPower(0))
//                    .addParametricCallback(0.5, () -> launchSystem.start(LaunchSystem.highVelocity, interval))
                    .setConstantHeadingInterpolation(bottomPose.getHeading())
                    .build();
            bottomLineup = follower.pathBuilder()
                    .addPath(new BezierCurve(scorePose, new Pose(116,  19), gatePickup))
//                    .addTemporalCallback(2000, () ->{
//                        failsafe = true;
//                    })
                    .setConstantHeadingInterpolation(bottomPose.getHeading())
                    .build();
            score3 = follower.pathBuilder()
                .addPath(new BezierCurve(gatePickup, new Pose(111,  25), scorePose))
                .addParametricCallback(0.6, () -> configuration.intakeMotor.setPower(0))
//                    .addParametricCallback(0.5, () -> launchSystem.start(LaunchSystem.highVelocity, interval))
                .setConstantHeadingInterpolation(gatePickup.getHeading())
                .build();
            leave = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, new Pose(110, 25)))
                .addParametricCallback(0.6, () -> configuration.intakeMotor.setPower(0))
//                    .addParametricCallback(0.5, () -> launchSystem.start(LaunchSystem.highVelocity, interval))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch(pathState) {
            case 0:
                follower.followPath(scorePreload);
                follower.setMaxPower(1);
                hasStartedLaunch=false;
                setPathState(1); //1
                launchTimer.reset();

                launchSystem.start(Tele.speed);
                hasStartedLaunch = true;
                launchSystem.toggleTracking();
                break;
            case 1:
                if(!follower.isBusy()&& launchTimer.seconds()>1.5) {
                    if(!hasStartedLaunch) {

                    }
                    if(launchSystem.update(launchSystem.returnDistance(follower.getPose()), Tele.speed)) {
                        follower.followPath(alignRow);
                        hasStartedLaunch = false;
                        launchSystem.toggleTracking();
                        setPathState(2);
                    }
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(scoreLineup);
                    configuration.intakeMotor.setPower(0);
                    follower.setMaxPower(1);
                    hasStartedLaunch=false;
                    launchSystem.toggleTracking();
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    if(!launchSystem.isLaunching() && !hasStartedLaunch) {
                        launchSystem.start(Tele.speed);
                        hasStartedLaunch = true;
                    }
                    if(launchSystem.update(launchSystem.returnDistance(follower.getPose()), Tele.speed)) {
                        follower.setMaxPower(1);
                        follower.followPath(pickupBottom);
                        configuration.intakeMotor.setPower(1);
                        hasStartedLaunch = false;
                        launchSystem.toggleTracking();
                        setPathState(4);
                    }
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    follower.followPath(score2);
                    launchSystem.toggleTracking();
                    setPathState(5);
                }
            case 5:
                if(!follower.isBusy()) {
                    if(!launchSystem.isLaunching() && !hasStartedLaunch) {

                        launchSystem.start(Tele.speed);
                        hasStartedLaunch = true;
                    }
                    if(launchSystem.update(launchSystem.returnDistance(follower.getPose()), Tele.speed)) {
                        follower.followPath(bottomLineup);
                        hasStartedLaunch = false;
                        configuration.intakeMotor.setPower(1);
                        launchSystem.toggleTracking();
                        follower.setMaxPower(1);
                        setPathState(6);
                    }
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(score3);
                    launchSystem.toggleTracking();
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    if(!launchSystem.isLaunching() && !hasStartedLaunch) {

                        launchSystem.start(Tele.speed);
                        hasStartedLaunch = true;
                    }
                    if(launchSystem.update(launchSystem.returnDistance(follower.getPose()), Tele.speed)) {
                        follower.followPath(pickupBottom);
                        hasStartedLaunch = false;
                        configuration.intakeMotor.setPower(1);
                        launchSystem.toggleTracking();
                        setPathState(8);
                    }
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(score3);
                    launchSystem.toggleTracking();
                    setPathState(9);
                }
                break;
            case 9:
                if(!follower.isBusy()) {
                    if(!launchSystem.isLaunching() && !hasStartedLaunch) {

                        launchSystem.start(Tele.speed);
                        hasStartedLaunch = true;
                    }
                    if(launchSystem.update(launchSystem.returnDistance(follower.getPose()), Tele.speed)) {
                        follower.followPath(leave);
                        hasStartedLaunch = false;
                        configuration.intakeMotor.setPower(0);
//                        launchSystem.toggleTracking();
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
        configuration.stopper.setPosition(holdBall);
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
        launchSystem.updateTurret(follower.getPose(), follower.getVelocity().getXComponent(), follower.getVelocity().getYComponent());

        double currentDist = launchSystem.returnDistance(follower.getPose());
        Tele.speedCalculator(currentDist);
        configuration.marco.setPosition(Tele.angleCalculator(currentDist));
        autonomousPathUpdate();
        telemetry.addData("Path State", pathState);
        telemetry.addData("Distance: ", currentDist);
        telemetry.update();
    }

    @Override
    public void stop(){
        Tele.startPose = follower.getPose();
    }

//    public void
}