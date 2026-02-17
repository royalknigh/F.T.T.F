package org.firstinspires.ftc.teamcode.comp.auto;

import static org.firstinspires.ftc.teamcode.configs.LaunchSystem.blueGoalPose;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.configs.Configuration;
import org.firstinspires.ftc.teamcode.configs.LaunchSystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.comp.tele.Tele;

@Autonomous(name = "Auto Blue Short playoff")
public class AutoBlueShortPlayoff extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private ElapsedTime getOpenGateTimer = new ElapsedTime();
    private Timer openGateTimer;
    private int pathState = 0;
    private boolean hasStartedLaunch = false;

    private final Pose startPose = new Pose(36, 137, Math.toRadians(180));
    private final Pose scorePose = new Pose(56, 84, Math.toRadians(180));

    private final Pose fisrtLinePose = new Pose(48, 84, Math.toRadians(180));
    private final Pose pickup1Pose = new Pose(17, 84, Math.toRadians(180));

    private final Pose secondLinePose = new Pose(48, 62, Math.toRadians(180));
    private final Pose pickup2Pose = new Pose(10, 62, Math.toRadians(180));
    private final Pose openGatePose = new Pose(15, 69, Math.toRadians(180));

    private final Pose pickupGate = new Pose(10, 57, Math.toRadians(140));

    private final Pose thirdLinePose = new Pose(48, 38, Math.toRadians(180));
    private final Pose pickup3Pose = new Pose(10, 38, Math.toRadians(180));

    private final Pose human = new Pose(6, 10, Math.toRadians(230));

    private Configuration configuration;
    private LaunchSystem launchSystem;

    private PathChain scorePreload, middleRow, pickupMiddleRow, scoreMiddleRow, alignFirstRow,
            firstRow, openGate, scoreFirstRow, scoreGate, gateCollect, park;

    @Override
    public void init() {
        pathTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        configuration = new Configuration(hardwareMap);
        launchSystem = new LaunchSystem(new Configuration(hardwareMap), blueGoalPose);
        buildPaths();
    }

    @Override
    public void start() {
         // Ensure turret is aiming
        launchSystem.idle();
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();

        // Target calculations
        launchSystem.updateTurret(follower.getPose());
        double currentDist = launchSystem.returnDistance(follower.getPose());
        Tele.speedCalculator(currentDist);
        configuration.marco.setPosition(Tele.angleCalculator(currentDist));

        autonomousPathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("Velo", "%.0f / %.0f", launchSystem.getVelocity(), Tele.speed);
        telemetry.addData("Target Pose", "48, 85");
        telemetry.update();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Preload Path
                if(!follower.isBusy()) {
                    follower.followPath(scorePreload);
                    follower.setMaxPower(1);
                    hasStartedLaunch = false;
                    setPathState(1);
                }
                break;

            case 1: // Preload Launch, middle row pickup
                if(!follower.isBusy()) {
                    if(!hasStartedLaunch) {
                        launchSystem.start(Tele.speed);
                        hasStartedLaunch = true;
                        launchSystem.toggleTracking();
                    }
                    if(launchSystem.update(launchSystem.returnDistance(follower.getPose()), Tele.speed)){
                        follower.followPath(middleRow);
                        hasStartedLaunch = false;
                        launchSystem.toggleTracking();
                        setPathState(2);
                    }
                }
                break;
            case 2: //middle row launch
                if(!follower.isBusy()) {
                    if(!hasStartedLaunch) {
                        launchSystem.start(Tele.speed);
                        hasStartedLaunch = true;
                        launchSystem.toggleTracking();
                    }
                    if(launchSystem.update(launchSystem.returnDistance(follower.getPose()), Tele.speed)){
                        follower.followPath(openGate);
                        hasStartedLaunch = false;
                        launchSystem.toggleTracking();
                        setPathState(3);
                    }
                }
                break;

            case 3:
                if(!follower.isBusy()){
                    follower.followPath(gateCollect);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()){
                    follower.followPath(scoreGate);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    if(!hasStartedLaunch) {
                        launchSystem.start(Tele.speed);
                        hasStartedLaunch = true;
                        launchSystem.toggleTracking();
                    }
                    if(launchSystem.update(launchSystem.returnDistance(follower.getPose()), Tele.speed)){
                        follower.followPath(firstRow);
                        hasStartedLaunch = false;
                        launchSystem.toggleTracking();
                        setPathState(7);
                    }
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    if(!hasStartedLaunch) {
                        launchSystem.start(Tele.speed);
                        hasStartedLaunch = true;
                        launchSystem.toggleTracking();
                    }
                    if(launchSystem.update(launchSystem.returnDistance(follower.getPose()), Tele.speed)){
                        follower.followPath(openGate);
                        hasStartedLaunch = false;
                        launchSystem.toggleTracking();
                        setPathState(8);
                    }
                }
                break;
            case 8:
                if(!follower.isBusy()){
                    follower.followPath(gateCollect);
                    setPathState(9);
                }
                break;
            case 9:
                if(!follower.isBusy()){
                    follower.followPath(scoreGate);
                    setPathState(9);
                }
                break;
            case 10:
                if(!follower.isBusy()) {
                    if(!hasStartedLaunch) {
                        launchSystem.start(Tele.speed);
                        hasStartedLaunch = true;
                        launchSystem.toggleTracking();
                    }
                    if(launchSystem.update(launchSystem.returnDistance(follower.getPose()), Tele.speed)){
                        follower.followPath(park);
                        hasStartedLaunch = false;
                        launchSystem.toggleTracking();
                        setPathState(11);
                    }
                }
                break;

        }
    }

    public void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
    }

    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
//                .addParametricCallback(0.3, () -> launchSystem.adjustOffset(-5))
                .build();
        middleRow = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, secondLinePose))
                .setConstantHeadingInterpolation(scorePose.getHeading())

                .addPath(new BezierLine(secondLinePose, pickup2Pose))
                .setConstantHeadingInterpolation(scorePose.getHeading())

                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .build();

        
        firstRow = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setConstantHeadingInterpolation(scorePose.getHeading())

                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        openGate = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, openGatePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), openGatePose.getHeading())
                .setTimeoutConstraint(500)
                .build();

        gateCollect = follower.pathBuilder()
                .addPath(new BezierLine(openGatePose, pickupGate))
                .setLinearHeadingInterpolation(openGatePose.getHeading(), pickupGate.getHeading())
                .setTimeoutConstraint(500)
                .build();

        scoreGate = follower.pathBuilder()
                .addPath(new BezierLine(pickupGate, scorePose))
                .setLinearHeadingInterpolation(pickupGate.getHeading(), scorePose.getHeading())
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, secondLinePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), secondLinePose.getHeading())
                .build();
        
    }

    @Override
    public void stop(){
        Tele.startPose = follower.getPose();
        launchSystem.fullStop();
    }
}