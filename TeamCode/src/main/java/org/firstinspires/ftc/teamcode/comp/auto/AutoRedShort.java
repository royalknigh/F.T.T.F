package org.firstinspires.ftc.teamcode.comp.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.configs.Configuration;
import org.firstinspires.ftc.teamcode.configs.LaunchSystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.comp.tele.Tele;

@Autonomous(name = "Auto Red Short")
public class AutoRedShort extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private ElapsedTime getOpenGateTimer = new ElapsedTime();
    private Timer openGateTimer;
    private int pathState = 0;
    private boolean hasStartedLaunch = false;

    // --- Poses ---
    private final Pose startPose = new Pose(108, 137, Math.toRadians(0));
    private final Pose scorePose = new Pose(92, 85, Math.toRadians(0));
    private final Pose fisrtLinePose = new Pose(96, 85, Math.toRadians(0));
    private final Pose pickup1Pose = new Pose(130, 85, Math.toRadians(0));
    private final Pose secondLinePose = new Pose(96, 62, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(132, 62, Math.toRadians(0));
    private final Pose openGatePose = new Pose(130, 72, Math.toRadians(0)); //gate
    private final Pose thirdLinePose = new Pose(96, 36, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(132, 36, Math.toRadians(0));

    private final Pose human = new Pose(136, 16, Math.toRadians(-70));

    private Configuration configuration;
    private LaunchSystem launchSystem;

    private PathChain scorePreload, alignRow1, pickupRow1, score1, alignRow2, pickupRow2,
            openGate, score2, alignRow3, pickupRow3, score3, park, humanPickup;

    @Override
    public void init() {
        pathTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        configuration = new Configuration(hardwareMap);
        launchSystem = new LaunchSystem(configuration, LaunchSystem.redGoalPose);

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

            case 1: // Preload Launch
                if(!follower.isBusy()) {
                    if(!hasStartedLaunch) {
                        launchSystem.start(Tele.speed);
                        hasStartedLaunch = true;
                        launchSystem.toggleTracking();
                    }
                    if(launchSystem.update(launchSystem.returnDistance(follower.getPose()))) {
                        follower.followPath(alignRow1);
                        hasStartedLaunch = false;
                        launchSystem.toggleTracking();
                        setPathState(2);
                    }
                }
                break;

            case 2: // Align 1 -> Pickup 1
                if(!follower.isBusy()) {
                    follower.followPath(pickupRow1);
                    configuration.intakeMotor.setPower(0.8);
                    setPathState(3);
                }
                break;

            case 3: // Pickup 1 -> Score 1
                if(!follower.isBusy()) {
                    follower.followPath(score1);
                    hasStartedLaunch = false;
                    setPathState(4);
                }
                break;

            case 4: // Score 1 Launch
                if(!follower.isBusy()) {
                    if(!launchSystem.isLaunching() && !hasStartedLaunch) {
                        launchSystem.toggleTracking();
                        launchSystem.start(Tele.speed);
                        hasStartedLaunch = true;
                    }
                    if(launchSystem.update(launchSystem.returnDistance(follower.getPose()))) {
                        follower.followPath(alignRow2);
                        hasStartedLaunch = false;
                        launchSystem.toggleTracking();
                        setPathState(5);
                    }
                }
                break;

            case 5: // Align 2 -> Pickup 2
                if(!follower.isBusy()) {
                    follower.followPath(pickupRow2);
                    configuration.intakeMotor.setPower(1);
                    setPathState(6);
                }
                break;

            case 6: // Pickup 2 -> Gate
                if(!follower.isBusy()) {

                    follower.followPath(openGate);
                    follower.setMaxPower(0.8);
                    setPathState(7);
                    getOpenGateTimer.reset();
                }
                break;

            case 7: // Gate -> Score 2
                if(!follower.isBusy() && getOpenGateTimer.seconds()>2.7) {

                    follower.followPath(score2);
                    follower.setMaxPower(1);
                    hasStartedLaunch = false;

                    setPathState(8);
                }
                break;

            case 8: // Score 2 Launch
                if(!follower.isBusy()) {
                    if(!launchSystem.isLaunching() && !hasStartedLaunch) {
                        launchSystem.start(Tele.speed);
                        launchSystem.toggleTracking();
                        hasStartedLaunch = true;
                    }
                    if(launchSystem.update(launchSystem.returnDistance(follower.getPose()))) {
                        follower.followPath(alignRow3);
                        hasStartedLaunch = false;
                        launchSystem.toggleTracking();
                        setPathState(9);
                    }
                }
                break;

            case 9: // Align 3 -> Pickup 3
                if(!follower.isBusy()) {
                    follower.followPath(pickupRow3);
                    configuration.intakeMotor.setPower(1);
                    setPathState(10);
                }
                break;

            case 10: // Pickup 3 -> Score 3
                if(!follower.isBusy()) {
                    follower.followPath(score3);

                    hasStartedLaunch = false;
                    setPathState(11);
                }
                break;

            case 11: // Score 3 Launch
                if(!follower.isBusy()) {
                    if(!launchSystem.isLaunching() && !hasStartedLaunch) {
                        launchSystem.start(Tele.speed); // Fixed high speed for last shot
                        launchSystem.toggleTracking();
                        hasStartedLaunch = true;
                    }
                    if(launchSystem.update(launchSystem.returnDistance(follower.getPose()))) {
                        follower.followPath(humanPickup);
                        launchSystem.toggleTracking();
                        hasStartedLaunch = false;
                        setPathState(12);
                    }
                }
                break;
            case 12: // Score 3 Launch
                if(!follower.isBusy()) {
                    if(!launchSystem.isLaunching() && !hasStartedLaunch) {
                        launchSystem.start(Tele.speed); // Fixed high speed for last shot
                        launchSystem.toggleTracking();
                        hasStartedLaunch = true;
                    }
                    if(launchSystem.update(launchSystem.returnDistance(follower.getPose()))) {
                        follower.followPath(park);
                        launchSystem.toggleTracking();
                        hasStartedLaunch = false;
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

    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
//                .addParametricCallback(0.3, () -> launchSystem.adjustOffset(-5))
                .build();

        alignRow1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, fisrtLinePose))
                .setNoDeceleration()
                .setLinearHeadingInterpolation(scorePose.getHeading(), fisrtLinePose.getHeading())
                .build();

        pickupRow1 = follower.pathBuilder()
                .addPath(new BezierLine(fisrtLinePose, pickup1Pose))
                .setLinearHeadingInterpolation(fisrtLinePose.getHeading(), pickup1Pose.getHeading())
                .build();

        score1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .addParametricCallback(0.6, () -> configuration.intakeMotor.setPower(0))
                .setTValueConstraint(0.9)
                .build();

        alignRow2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, secondLinePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), secondLinePose.getHeading())
                .build();

        pickupRow2 = follower.pathBuilder()
                .addPath(new BezierLine(secondLinePose, pickup2Pose))
                .setLinearHeadingInterpolation(secondLinePose.getHeading(), pickup2Pose.getHeading())
                .build();

        openGate = follower.pathBuilder()
                .addPath(new BezierCurve(pickup2Pose, new Pose(110, 60), openGatePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), openGatePose.getHeading())
                .addParametricCallback(0.5, () -> configuration.intakeMotor.setPower(0.7))
                .addParametricCallback(0.7, () -> follower.setMaxPower(0.6))
                .build();

        score2 = follower.pathBuilder()
                .addPath(new BezierLine(openGatePose, scorePose))
                .setLinearHeadingInterpolation(openGatePose.getHeading(), scorePose.getHeading())
                .addParametricCallback(0.3, () -> configuration.intakeMotor.setPower(0))
                .setTValueConstraint(0.9)
                .build();

        alignRow3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, thirdLinePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), thirdLinePose.getHeading())
                .build();

        pickupRow3 = follower.pathBuilder()
                .addPath(new BezierLine(thirdLinePose, pickup3Pose))
                .setLinearHeadingInterpolation(thirdLinePose.getHeading(), pickup3Pose.getHeading())
                .build();

        score3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .addParametricCallback(0, () -> configuration.intakeMotor.setPower(0.7))
                .addParametricCallback(0.6, () -> configuration.intakeMotor.setPower(0))
                .setTValueConstraint(0.9)
                .build();


        humanPickup =  follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(142, 63), human))
                .setConstantHeadingInterpolation(human.getHeading())
                .addParametricCallback(0.5, () -> configuration.intakeMotor.setPower(1))
                .addParametricCallback(0.8, () -> follower.setMaxPower(0.5))
                .setTimeoutConstraint(500)

                .addPath(new BezierLine(human, scorePose))
                .setLinearHeadingInterpolation(human.getHeading(), scorePose.getHeading())
                .addParametricCallback(0.5, () -> configuration.intakeMotor.setPower(0))
                .addParametricCallback(0, () -> follower.setMaxPower(1))
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, secondLinePose))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .build();


    }

    @Override
    public void stop(){
        Tele.startPose = follower.getPose();
        launchSystem.fullStop();
    }
}