package org.firstinspires.ftc.teamcode.comp.auto;

import static org.firstinspires.ftc.teamcode.configs.LaunchSystem.blueGoalPose;

import com.bylazar.configurables.annotations.Configurable;
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

@Configurable
@Autonomous(name = "Auto Blue Short playoff")
public class AutoBlueShortPlayoff extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private ElapsedTime gateTimer = new ElapsedTime();
    private int pathState = 0;

    public static double gateX = 15, gateY = 67, gateHeading = 180;
    public static double pickupX = 12, pickupY = 57, pickupHeading = 130;

    private final Pose startPose    = new Pose(34, 136, Math.toRadians(180));
    private final Pose scorePose    = new Pose(50, 86,    Math.toRadians(180));

    private final Pose fisrtLinePose  = new Pose(42, 84, Math.toRadians(180));
    private final Pose pickup1Pose    = new Pose(26, 84, Math.toRadians(180));

    private final Pose secondLinePose = new Pose(42, 60, Math.toRadians(180));
    private final Pose pickup2Pose    = new Pose(23, 60, Math.toRadians(180));
    public static final Pose openGatePose   = new Pose(gateX, gateY, Math.toRadians(gateHeading));

    private final Pose pickupGate = new Pose(pickupX, pickupY,  Math.toRadians(pickupHeading));
    private final Pose scoreLeave = new Pose(58, 95,  Math.toRadians(135));

    private Configuration configuration;
    private LaunchSystem launchSystem;

    private PathChain scorePreload, middleRow, firstRow, openGate, gateCollect, scoreGate, leave, park;

    @Override
    public void init() {
        pathTimer = new Timer();
        follower   = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        configuration = new Configuration(hardwareMap);
        launchSystem  = new LaunchSystem(new Configuration(hardwareMap), blueGoalPose);
        buildPaths();
    }

    @Override
    public void start() {
        launchSystem.idle();
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();

        launchSystem.updateTurret(follower.getPose());
        double currentDist = launchSystem.returnDistance(follower.getPose());
        Tele.speedCalculator(currentDist);
        configuration.marco.setPosition(Tele.angleCalculator(currentDist) + 0.05);

        autonomousPathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("Velo", "%.0f / %.0f", launchSystem.getVelocity(), Tele.speed);
        telemetry.addData("Target Pose", "48, 85");
        telemetry.update();
    }

    @Override
    public void stop() {
        Tele.startPose = follower.getPose();
        launchSystem.adjustOffset(15);
        launchSystem.fullStop();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0: // Begin: drive to score pose (callback fires launch at end of path)
                if (!follower.isBusy()) {
                    follower.followPath(scorePreload);
                    follower.setMaxPower(1);
                    setPathState(1);
                }
                break;

            case 1: // Wait for preload launch, then sweep middle row (callback fires launch)
                if (!follower.isBusy()) {
                    if (launchSystem.update(launchSystem.returnDistance(follower.getPose()), Tele.speed)) {
                        launchSystem.toggleTracking();
                        follower.followPath(middleRow);
                        setPathState(2);
                    }
                }
                break;

            case 2: // Wait for middle-row launch, then open gate
                if (!follower.isBusy()) {
                    if (launchSystem.update(launchSystem.returnDistance(follower.getPose()), Tele.speed)) {
                        launchSystem.toggleTracking();
                        follower.followPath(openGate);
                        gateTimer.reset();
                        setPathState(3);
                    }
                }
                break;

            case 3: // Wait for gate to open, then collect
                if (!follower.isBusy() && gateTimer.seconds() > 1.8) {
                    follower.followPath(gateCollect);
                    gateTimer.reset();
                    setPathState(4);
                }
                break;

            case 4: // Wait for collection, then return to score (callback fires launch)
                if (!follower.isBusy() && gateTimer.seconds() > 1.8) {
                    follower.followPath(scoreGate);
                    setPathState(5);
                }
                break;

            case 5: // Wait for gate-ball launch, then sweep first row (callback fires launch)
                if (!follower.isBusy()) {
                    if (launchSystem.update(launchSystem.returnDistance(follower.getPose()), Tele.speed)) {
                        launchSystem.toggleTracking();
                        follower.followPath(firstRow);
                        setPathState(7);
                    }
                }
                break;

            case 7: // Wait for first-row launch, then open gate again
                if (!follower.isBusy()) {
                    if (launchSystem.update(launchSystem.returnDistance(follower.getPose()), Tele.speed)) {
                        launchSystem.toggleTracking();
                        follower.followPath(openGate);
                        gateTimer.reset();
                        setPathState(8);
                    }
                }
                break;

            case 8: // Wait for gate, then collect
                if (!follower.isBusy() && gateTimer.seconds() > 1.8) {
                    follower.followPath(gateCollect);
                    gateTimer.reset();
                    setPathState(9);
                }
                break;

            case 9: // Wait for collection, then return to score (callback fires launch)
                if (!follower.isBusy() && gateTimer.seconds() > 1.8) {
                    follower.followPath(scoreGate);
                    setPathState(10);
                }
                break;
            case 10: // Wait for first-row launch, then open gate again
                if (!follower.isBusy()) {
                    if (launchSystem.update(launchSystem.returnDistance(follower.getPose()), Tele.speed)) {
                        launchSystem.toggleTracking();
                        follower.followPath(openGate);
                        gateTimer.reset();
                        setPathState(11);
                    }
                }
                break;

            case 11: // Wait for gate, then collect
                if (!follower.isBusy() && gateTimer.seconds() > 1.8) {
                    follower.followPath(gateCollect);
                    gateTimer.reset();
                    setPathState(12);
                }
                break;

            case 12: // Wait for collection, then return to score (callback fires launch)
                if (!follower.isBusy() && gateTimer.seconds() > 1.8) {
                    follower.followPath(leave);
                    setPathState(13);
                }
                break;

            case 13:
                if (!follower.isBusy()) {
                    if (launchSystem.update(launchSystem.returnDistance(follower.getPose()), Tele.speed)) {
                        launchSystem.toggleTracking();
                        gateTimer.reset();
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

    // -------------------------------------------------------------------------
    // Paths  —  every path that ends at scorePose fires launch() at t=1
    // -------------------------------------------------------------------------

    public void buildPaths() {

        // Drives from start → scorePose; launches at arrival
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .addParametricCallback(0.8, () -> launch())
                .build();

        // Sweeps middle row then curves back to scorePose; launches at arrival
        middleRow = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(50,53), new Pose(43, 60), pickup2Pose))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .addParametricCallback(0.4, () -> configuration.intakeMotor.setPower(1))

                .addPath(new BezierCurve(pickup2Pose, new Pose(43, 68), scorePose))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .addParametricCallback(0.4, () -> configuration.intakeMotor.setPower(0))
                .addParametricCallback(0.9, () -> launch())
                .build();

        // Collects first row then returns to scorePose; launches at arrival
        firstRow = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .addParametricCallback(0, () -> configuration.intakeMotor.setPower(1))

                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .addParametricCallback(0.5, () -> configuration.intakeMotor.setPower(0))
                .addParametricCallback(0.7, () -> launch())
                .build();

        // Curves to gate position; no launch here
        openGate = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(45, 65), openGatePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), openGatePose.getHeading())
                .build();

        // Drives to pickupGate to collect balls
        gateCollect = follower.pathBuilder()
                .addPath(new BezierLine(openGatePose, pickupGate))
                .setLinearHeadingInterpolation(openGatePose.getHeading(), pickupGate.getHeading())
                .addParametricCallback(0, () -> configuration.intakeMotor.setPower(1))
                .build();

        // Returns from gate to scorePose; launches at arrival
        scoreGate = follower.pathBuilder()
                .addPath(new BezierCurve(pickupGate, new Pose(45, 65), scorePose))
                .setLinearHeadingInterpolation(pickupGate.getHeading(), scorePose.getHeading())
                .addParametricCallback(0.2, () -> configuration.intakeMotor.setPower(0))
                .addParametricCallback(0.9, () -> launch())
                .build();
        leave = follower.pathBuilder()
                .addPath(new BezierCurve(pickupGate, new Pose(45, 65), scoreLeave))
                .setLinearHeadingInterpolation(pickupGate.getHeading(), scoreLeave.getHeading())
                .addParametricCallback(0.2, () -> configuration.intakeMotor.setPower(0))
                .addParametricCallback(0.7, () -> launch())
                .build();
    }

    /** Called by path callbacks; spins up the flywheel and enables turret tracking. */
    public void launch() {
        launchSystem.start(Tele.speed);
        launchSystem.toggleTracking();
    }
}