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

import org.firstinspires.ftc.teamcode.comp.tele.Tele;
import org.firstinspires.ftc.teamcode.configs.Configuration;
import org.firstinspires.ftc.teamcode.configs.LaunchSystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous(name = "Auto Blue Short *Shared* FRI")
public class AutoBlueShortSharedFRI extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private ElapsedTime gateTimer = new ElapsedTime();
    private int pathState = 0;

    public static double gateX = 15, gateY = 67, gateHeading = 180;
    public static double pickupX = 11, pickupY = 57, pickupHeading = 130;

    private final Pose startPose    = new Pose(61, 32, Math.toRadians(310));
    private final Pose scorePose    = new Pose(44, 55,    Math.toRadians(180));//-50
    private final Pose pickupBottom = new Pose(9.5,14.5, Math.toRadians(270));
    //private final Pose fisrtLinePose  = new Pose(42, 84, Math.toRadians(180));
    private final Pose pickup1Pose    = new Pose(22, 63, Math.toRadians(180));

    private final Pose secondLinePose = new Pose(46, 87, Math.toRadians(180));
    private final Pose pickup2Pose    = new Pose(24, 87, Math.toRadians(180));
    private final Pose thirdLinePose = new Pose(45, 112, Math.toRadians(180));
    private final Pose pickup3Pose    = new Pose(23, 112, Math.toRadians(180));
//    public static final Pose openGatePose   = new Pose(gateX, gateY, Math.toRadians(gateHeading));
//
//    private final Pose pickupGate = new Pose(pickupX, pickupY,  Math.toRadians(pickupHeading));
    private final Pose leave = new Pose(57, 98,  Math.toRadians(320));

    private Configuration configuration;
    private LaunchSystem launchSystem;

    private PathChain scorePreload, bottomRow, firstRow, secondRow, thirdRow, leavePath;

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

        launchSystem.updateTurret(follower.getPose(), follower.getVelocity().getXComponent(), follower.getVelocity().getYComponent());

        double currentDist = launchSystem.returnDistance(follower.getPose());
        Tele.speedCalculator(currentDist);
        configuration.marco.setPosition(Tele.angleCalculator(currentDist) );

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
//        switch (pathState) {
//
//            case 0: // Begin: drive to score pose (callback fires launch at end of path)
//                if (!follower.isBusy()) {
//                    follower.followPath(scorePreload);
//                    follower.setMaxPower(1);
//                    setPathState(1);
//                }
//                break;
//
//            case 1: // Wait for preload launch, then sweep middle row (callback fires launch)
//                if (!follower.isBusy()) {
//                    if (launchSystem.update(launchSystem.returnDistance(follower.getPose()), Tele.speed)) {
//                        launchSystem.toggleTracking();
//                        follower.followPath(middleRow);
//                        setPathState(2);
//                    }
//                }
//                break;
//
//            case 2: // Wait for middle-row launch, then open gate
//                if (!follower.isBusy()) {
//                    if (launchSystem.update(launchSystem.returnDistance(follower.getPose()), Tele.speed)) {
//                        launchSystem.toggleTracking();
//                        follower.followPath(openGate);
//                        gateTimer.reset();
//                        setPathState(3);
//                    }
//                }
//                break;
//
//            case 3: // Wait for gate to open, then collect
//                if (!follower.isBusy() && gateTimer.seconds() > 1.8) {
//                    follower.followPath(gateCollect);
//                    gateTimer.reset();
//                    setPathState(4);
//                }
//                break;
//
//            case 4: // Wait for collection, then return to score (callback fires launch)
//                if (!follower.isBusy() && gateTimer.seconds() > 1.8) {
//                    follower.followPath(scoreGate);
//                    setPathState(7);
//                }
//                break;
//
//
//            case 7: // Wait for first-row launch, then open gate again
//                if (!follower.isBusy()) {
//                    if (launchSystem.update(launchSystem.returnDistance(follower.getPose()), Tele.speed)) {
//                        launchSystem.toggleTracking();
//                        follower.followPath(openGate);
//                        gateTimer.reset();
//                        setPathState(8);
//                    }
//                }
//                break;
//
//            case 8: // Wait for gate, then collect
//                if (!follower.isBusy() && gateTimer.seconds() > 1.8) {
//                    follower.followPath(gateCollect);
//                    gateTimer.reset();
//                    setPathState(9);
//                }
//                break;
//
//            case 9: // Wait for collection, then return to score (callback fires launch)
//                if (!follower.isBusy() && gateTimer.seconds() > 1.8) {
//                    follower.followPath(scoreGate);
//                    setPathState(10);
//                }
//                break;
//            case 10: // Wait for first-row launch, then open gate again
//                if (!follower.isBusy()) {
//                    if (launchSystem.update(launchSystem.returnDistance(follower.getPose()), Tele.speed)) {
//                        launchSystem.toggleTracking();
//                        follower.followPath(openGate);
//                        gateTimer.reset();
//                        setPathState(11);
//                    }
//                }
//                break;
//
//            case 11: // Wait for gate, then collect
//                if (!follower.isBusy() && gateTimer.seconds() > 1.8) {
//                    follower.followPath(gateCollect);
//                    gateTimer.reset();
//                    setPathState(12);
//                }
//                break;
//
//            case 12: // Wait for collection, then return to score (callback fires launch)
//                if (!follower.isBusy() && gateTimer.seconds() > 1.8) {
//                    follower.followPath(scoreGate);
//                    setPathState(13);
//                }
//                break;
//
//            case 13:
//                if (!follower.isBusy()) {
//                    if (launchSystem.update(launchSystem.returnDistance(follower.getPose()), Tele.speed)) {
//                        follower.followPath(firstRow);
//                        launchSystem.toggleTracking();
//                        gateTimer.reset();
//                        setPathState(14);
//                    }
//                }
//                break;
//            case 14:
//                if (!follower.isBusy()) {
//                    if (launchSystem.update(launchSystem.returnDistance(follower.getPose()), Tele.speed)) {
////                        launchSystem.toggleTracking();
//                        launchSystem.adjustOffset(-48);
//                        setPathState(-1);
//                    }
//                }
//                break;
//        }
   }

    public void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
    }

    // -------------------------------------------------------------------------
    // Paths  —  every path that ends at scorePose fires launch() at t=1
    // -------------------------------------------------------------------------

    public void buildPaths() {
//    private PathChain scorePreload, bottomRow, firstRow, secondRow, thirdRow, leavePath;
        // Drives from start → scorePose; launches at arrival
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .addParametricCallback(0, () -> launchSystem.adjustOffset(-3))
                .addParametricCallback(0.8, () -> launch())
                .build();

        // Sweeps middle row then curves back to scorePose; launches at arrival
        bottomRow = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(55,41), new Pose(22, 30), pickupBottom))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickupBottom.getHeading())
                .addParametricCallback(0.4, () -> configuration.intakeMotor.setPower(1))

                .addPath(new BezierLine(pickupBottom, scorePose))
                .setLinearHeadingInterpolation(pickupBottom.getHeading(), scorePose.getHeading())
                .addParametricCallback(0.2, () -> configuration.intakeMotor.setPower(0))
                .addParametricCallback(0.9, () -> launch())
                .build();
        firstRow = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(46,59), pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .addParametricCallback(0.4, () -> configuration.intakeMotor.setPower(1))

                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .addParametricCallback(0.2, () -> configuration.intakeMotor.setPower(0))
                .addParametricCallback(0.9, () -> launch())
                .build();
        secondRow = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .addParametricCallback(0.4, () -> configuration.intakeMotor.setPower(1))

                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .addParametricCallback(0.2, () -> configuration.intakeMotor.setPower(0))
                .addParametricCallback(0.9, () -> launch())
                .build();
        thirdRow = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .addParametricCallback(0.4, () -> configuration.intakeMotor.setPower(1))

                .addPath(new BezierLine(pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .addParametricCallback(0.2, () -> configuration.intakeMotor.setPower(0))
                .addParametricCallback(0.9, () -> launch())
                .build();

        leavePath = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(45, 65), leave))
                .setLinearHeadingInterpolation(scorePose.getHeading(), leave.getHeading())
                .addParametricCallback(0.1, () -> configuration.intakeMotor.setPower(0))
                .addParametricCallback(0.7, () -> launch())
                .build();

    }
// hai to do list Iulia:
    // path -> bile cand cad din cos - cycles
    // de refacut case-urile
    // verificat curbe bezier si de adaugat
    // be happy
    /** Called by path callbacks; spins up the flywheel and enables turret tracking. */
    public void launch() {
        launchSystem.start(Tele.speed);
        launchSystem.toggleTracking();
    }
}