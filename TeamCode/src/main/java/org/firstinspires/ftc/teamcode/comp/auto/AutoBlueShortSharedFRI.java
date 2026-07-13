package org.firstinspires.ftc.teamcode.comp.auto;

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
import org.firstinspires.ftc.teamcode.comp.tele.TeleBlueFRI;
import org.firstinspires.ftc.teamcode.configs.Configuration;
import org.firstinspires.ftc.teamcode.configs.LaunchSystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous(name = "🔵 Auto Blue *FRI* ----> Shared")
public class AutoBlueShortSharedFRI extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private ElapsedTime gateTimer = new ElapsedTime();
    private int pathState = 0;

    public static double gateX = 15, gateY = 67, gateHeading = 180;
    public static double pickupX = 11, pickupY = 57, pickupHeading = 130;

    private final Pose startPose    = new Pose(61, 32, Math.toRadians(310));
    private final Pose scorePose    = new Pose(47, 63,    Math.toRadians(180));//-50
    private final Pose pickupBottom = new Pose(11,15, Math.toRadians(270));
    private final Pose cyclePose = new Pose(15,15, Math.toRadians(270));
    //private final Pose fisrtLinePose  = new Pose(42, 84, Math.toRadians(180));
    private final Pose pickup1Pose    = new Pose(27, 61, Math.toRadians(180));

//    private final Pose secondLinePose = new Pose(46, 87, Math.toRadians(180));
    private final Pose pickup2Pose    = new Pose(27, 86, Math.toRadians(180));
//    private final Pose thirdLinePose = new Pose(45, 112, Math.toRadians(180));
    private final Pose pickup3Pose    = new Pose(26, 109, Math.toRadians(180));
//    public static final Pose openGatePose   = new Pose(gateX, gateY, Math.toRadians(gateHeading));
//
//    private final Pose pickupGate = new Pose(pickupX, pickupY,  Math.toRadians(pickupHeading));
    private final Pose leave = new Pose(57, 98,  Math.toRadians(275));

    private Configuration configuration;
    private LaunchSystem launchSystem;

    private PathChain scorePreload, bottomRow, firstRow, secondRow, thirdRow, cycle, leavePath;

    @Override
    public void init() {
        pathTimer = new Timer();
        follower   = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        configuration = new Configuration(hardwareMap);
        launchSystem  = new LaunchSystem(new Configuration(hardwareMap), LaunchSystem.bluePurpleGoalPoseAuto);
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
        TeleBlueFRI.speedCalculator(currentDist);
        configuration.marco.setPosition(TeleBlueFRI.angleCalculator(currentDist) );

        autonomousPathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("Velo", "%.0f / %.0f", launchSystem.getVelocity(), TeleBlueFRI.speed);
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

            case 0: // preload duh
                if (!follower.isBusy()) {
                    follower.followPath(scorePreload);
                    follower.setMaxPower(1);
                    setPathState(1);
                }
                break;

            case 1: // arunca preload -> bile jos -> score pose
                if (!follower.isBusy()) {
                    if (launchSystem.update(launchSystem.returnDistance(follower.getPose()), TeleBlueFRI.speed)) {
                        launchSystem.toggleTracking();
                        follower.followPath(bottomRow);
                        setPathState(2);
                    }
                }
                break;

            case 2: // arunca bile -> primul rand -> score pose
                if (!follower.isBusy()) {
                    if (launchSystem.update(launchSystem.returnDistance(follower.getPose()), TeleBlueFRI.speed)) {
                        launchSystem.toggleTracking();
                        follower.followPath(firstRow);
                        setPathState(3);
                    }
                }
                break;
            case 3: // arunca bile -> al doilea rand -> score pose
                if (!follower.isBusy()) {
                    if (launchSystem.update(launchSystem.returnDistance(follower.getPose()), TeleBlueFRI.speed)) {
                        launchSystem.toggleTracking();
                        follower.followPath(secondRow);
                        setPathState(5);
                    }
                }
                break;
            case 4: // daca e de sincronizat case 3 -> case 5 ( this is the 3rd row btw )
                if (!follower.isBusy()) {
                    if (launchSystem.update(launchSystem.returnDistance(follower.getPose()), TeleBlueFRI.speed)) {
                        launchSystem.toggleTracking();
                        follower.followPath(thirdRow);
                        setPathState(5);
                    }
                }
                break;

            case 5: // Gate drop-off pickup ( cycles + )
                if (!follower.isBusy()) {
                    if (launchSystem.update(launchSystem.returnDistance(follower.getPose()), TeleBlueFRI.speed)) {
                        launchSystem.toggleTracking();
                        follower.followPath(bottomRow);
                        setPathState(6);
                    }
                }
                break;

            case 6: // Gate drop-off pickup ( cycles + )
                if (!follower.isBusy()) {
                    if (launchSystem.update(launchSystem.returnDistance(follower.getPose()), TeleBlueFRI.speed)) {
                        launchSystem.toggleTracking();
                        follower.followPath(cycle);
                        setPathState(7);
                    }
                }
                break;
            case 7: // Gate drop-off pickup ( cycles + )
                if (!follower.isBusy()) {
                    if (launchSystem.update(launchSystem.returnDistance(follower.getPose()), TeleBlueFRI.speed)) {
                        launchSystem.toggleTracking();
                        follower.followPath(cycle);
                        setPathState(9);
                    }
                }
                break;
//            case 8: // Gate drop-off pickup ( cycles + )
//                if (!follower.isBusy()) {
//                    if (launchSystem.update(launchSystem.returnDistance(follower.getPose()), TeleBlueFRI.speed)) {
//                        launchSystem.toggleTracking();
//                        follower.followPath(bottomRow);
//                        setPathState(9);
//                    }
//                }
//                break;
            case 9: // Leave, "Trebuie sa schimbam pozitia" -Iulia
                if (!follower.isBusy()) {
                    if (launchSystem.update(launchSystem.returnDistance(follower.getPose()), TeleBlueFRI.speed)) {
//                        launchSystem.toggleTracking();
                        follower.followPath(leavePath);
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
                .addPath(new BezierCurve(scorePose, new Pose(39,41), new Pose(8,45), pickupBottom))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickupBottom.getHeading())
                .addParametricCallback(0.4, () -> configuration.intakeMotor.setPower(1))

                .addPath(new BezierLine(pickupBottom, scorePose))
                .setLinearHeadingInterpolation(pickupBottom.getHeading(), scorePose.getHeading())
                .addParametricCallback(0.5, () -> configuration.intakeMotor.setPower(0))
                .addParametricCallback(0.9, () -> launch())
                .build();
        firstRow = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .addParametricCallback(0.1, () -> configuration.intakeMotor.setPower(1))

                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .addParametricCallback(0.4, () -> configuration.intakeMotor.setPower(0))
                .addParametricCallback(0.9, () -> launch())
                .build();
        secondRow = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(55, 91), pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .addParametricCallback(0.3, () -> configuration.intakeMotor.setPower(1))

                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .addParametricCallback(0.4, () -> configuration.intakeMotor.setPower(0))
                .addParametricCallback(0.9, () -> launch())
                .build();
        thirdRow = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(60, 115), pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .addParametricCallback(0.4, () -> configuration.intakeMotor.setPower(1))

                .addPath(new BezierLine(pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .addParametricCallback(0.4, () -> configuration.intakeMotor.setPower(0))
                .addParametricCallback(0.9, () -> launch())
                .build();
        cycle = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(27,41), cyclePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), cyclePose.getHeading())
                .addParametricCallback(0.4, () -> configuration.intakeMotor.setPower(1))

                .addPath(new BezierLine(cyclePose, scorePose))
                .setLinearHeadingInterpolation(cyclePose.getHeading(), scorePose.getHeading())
                .addParametricCallback(0.2, () -> configuration.intakeMotor.setPower(0))
                .addParametricCallback(0.9, () -> launch())
                .build();
        leavePath = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, leave))
                .setLinearHeadingInterpolation(scorePose.getHeading(), leave.getHeading())
                .addParametricCallback(0.1, () -> configuration.intakeMotor.setPower(0))
                .build();

    }
// hai to do list Iulia:
    // path -> bile cand cad din cos - cycles
    // de refacut case-urile
    // verificat curbe bezier si de adaugat
    // be happy

    /** Called by path callbacks; spins up the flywheel and enables turret tracking. */
    public void launch() {
        launchSystem.start(TeleBlueFRI.speed);
        launchSystem.toggleTracking();
    }
}