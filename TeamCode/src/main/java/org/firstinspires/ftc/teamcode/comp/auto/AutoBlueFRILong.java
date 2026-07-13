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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.comp.tele.Tele;
import org.firstinspires.ftc.teamcode.configs.Configuration;
import org.firstinspires.ftc.teamcode.configs.LaunchSystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous(name = "🔵 Auto Blue *FRI* ----> Long")
public class AutoBlueFRILong extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private int pathState = 0;

    // All x values mirrored: x_red = 144 - x_blue
    // All headings mirrored: heading_red = 180° - heading_blue
    private final Pose startPose = new Pose(50, 7, Math.toRadians(180));
    private final Pose scorePose = new Pose(52, 22, Math.toRadians(110));
    private final Pose lineup = new Pose(49, 35, Math.toRadians(180));
    private final Pose pickupPose = new Pose(10, 35, Math.toRadians(180));
    private final Pose bottomPose = new Pose(8, 10, Math.toRadians(180));
    private final Pose gatePickup = new Pose(8, -9, Math.toRadians(180));
    private final Pose gatePickup2 = new Pose(10, -4, Math.toRadians(180));
    private final Pose gatePickup3 = new Pose(10, -7, Math.toRadians(180));
    private final Pose gatePickup4 = new Pose(10, -8, Math.toRadians(180));
    private final Pose park = new Pose(40,30, Math.toRadians(105));



    private Configuration configuration;
    private LaunchSystem launchSystem;

    private PathChain scorePreload, alignRow, scoreRow, pickupBottom, scoreBottom, bottomLineup, bottomLineup2, bottomLineup3, bottomLineup4, score1, score2, score3, score4, leave;

    /** Called by path callbacks — spins up the flywheel and enables turret tracking. */
    public void launch() {
        launchSystem.start(Tele.speed);
        launchSystem.toggleTracking();
    }

    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .addParametricCallback(0.1, () -> launch())
                .build();

        // Control point: x = 144 - 55 = 89
        alignRow = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(40, 37), pickupPose))
                .setTangentHeadingInterpolation()
                .addParametricCallback(0.2, () -> configuration.intakeMotor.setPower(1))
                .build();

        scoreRow = follower.pathBuilder()
                .addPath(new BezierLine(pickupPose, scorePose))
                .setConstantHeadingInterpolation(pickupPose.getHeading())
                .addParametricCallback(0.2, () -> configuration.intakeMotor.setPower(0))
                .addParametricCallback(0.9, () -> launch())
                .build();


        pickupBottom = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, bottomPose))
                .addParametricCallback(0.65, () -> follower.setMaxPower(0.75))
                .addParametricCallback(0.2, () -> configuration.intakeMotor.setPower(1))
                .setConstantHeadingInterpolation(bottomPose.getHeading())
                .build();

        scoreBottom = follower.pathBuilder()
                .addPath(new BezierLine(bottomPose, scorePose))
                .addParametricCallback(0.6, () -> configuration.intakeMotor.setPower(0))
                .addParametricCallback(0.9, () -> launch())
                .setConstantHeadingInterpolation(bottomPose.getHeading())
                .build();

        bottomLineup = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, gatePickup))
                .addParametricCallback(0.3, () -> configuration.intakeMotor.setPower(1))
                .setConstantHeadingInterpolation(bottomPose.getHeading())
                .build();

        bottomLineup2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(30, 8), gatePickup2))
                .setConstantHeadingInterpolation(bottomPose.getHeading())
                .addParametricCallback(0.3, () -> configuration.intakeMotor.setPower(1))
                .build();

        bottomLineup3 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(30, 8), gatePickup3))
                .setConstantHeadingInterpolation(bottomPose.getHeading())
                .addParametricCallback(0.3, () -> configuration.intakeMotor.setPower(1))
                .build();

        bottomLineup4 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, gatePickup3))
                .setConstantHeadingInterpolation(bottomPose.getHeading())
                .addParametricCallback(0.3, () -> configuration.intakeMotor.setPower(1))
                .build();


        score1 = follower.pathBuilder()
                .addPath(new BezierLine(gatePickup, scorePose))
                .addParametricCallback(0.3, () -> configuration.intakeMotor.setPower(0))
                .addParametricCallback(0.7, () -> launch())
                .setConstantHeadingInterpolation(bottomPose.getHeading())
                .build();

        score2 = follower.pathBuilder()
                .addPath(new BezierLine(gatePickup2, scorePose))
                .addParametricCallback(0.3, () -> configuration.intakeMotor.setPower(0))
                .addParametricCallback(0.7, () -> launch())
                .setConstantHeadingInterpolation(bottomPose.getHeading())
                .build();

        score3 = follower.pathBuilder()
                .addPath(new BezierLine(gatePickup3, scorePose))
                .addParametricCallback(0.3, () -> configuration.intakeMotor.setPower(0))
                .addParametricCallback(0.7, () -> launch())
                .setConstantHeadingInterpolation(bottomPose.getHeading())
                .build();

        score4 = follower.pathBuilder()
                .addPath(new BezierLine(gatePickup4, scorePose))
                .addParametricCallback(0.3, () -> configuration.intakeMotor.setPower(0))
                .addParametricCallback(0.7, () -> launch())
                .setConstantHeadingInterpolation(bottomPose.getHeading())
                .build();

        leave = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, park))
                .addParametricCallback(0.1, () -> configuration.intakeMotor.setPower(0))
                .setConstantHeadingInterpolation(park.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0: // Drive to score pose; callback fires launch() at t=0.1
                follower.followPath(scorePreload);
                follower.setMaxPower(1);
                setPathState(1);
                break;

            case 1: // Wait for preload launch, then align to row
                if (!follower.isBusy()) {
                    if (launchSystem.update(launchSystem.returnDistance(follower.getPose()), Tele.speed)) {
                        launchSystem.toggleTracking();
                        follower.followPath(alignRow);
                        setPathState(2);
                    }
                }
                break;

            case 2: // Align done — intake on, drive to pickup then score; callback fires launch()
                if (!follower.isBusy()) {
                    configuration.intakeMotor.setPower(1);
                    follower.setMaxPower(1);
                    follower.followPath(scoreRow);
                    setPathState(3);
                }
                break;

            case 3: // Wait for row-ball launch, then go collect bottom ball
                if (!follower.isBusy()) {
                    if (launchSystem.update(launchSystem.returnDistance(follower.getPose()), Tele.speed)) {
                        launchSystem.toggleTracking();
                        follower.setMaxPower(0.8);
                        follower.followPath(pickupBottom);
                        setPathState(4);
                    }
                }
                break;

            case 4: // Bottom pickup done — return to score; callback fires launch()
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    follower.followPath(scoreBottom);
                    setPathState(5);
                }
                break;

            case 5: // Wait for bottom-ball launch, then go to gate lineup
                if (!follower.isBusy()) {
                    if (launchSystem.update(launchSystem.returnDistance(follower.getPose()), Tele.speed)) {
                        launchSystem.toggleTracking();
                        follower.followPath(bottomLineup);
                        configuration.intakeMotor.setPower(0.8);
                        follower.setMaxPower(1);
                        setPathState(6);
                    }
                }
                break;

            case 6: // At gate — score gate ball; callback fires launch()
                if (!follower.isBusy()) {
                    follower.followPath(score1);
                    setPathState(7);
                }
                break;

            case 7: // Wait for gate-ball launch, then return to gate
                if (!follower.isBusy()) {
                    if (launchSystem.update(launchSystem.returnDistance(follower.getPose()), Tele.speed)) {
                        launchSystem.toggleTracking();
                        follower.followPath(bottomLineup2);
                        configuration.intakeMotor.setPower(0.8);
                        setPathState(8);
                    }
                }
                break;

            case 8: // At gate again — score gate ball; callback fires launch()
                if (!follower.isBusy()) {
                    follower.followPath(score2);
                    setPathState(9);
                }
                break;

            case 9: // Wait for gate-ball launch, then return to gate
                if (!follower.isBusy()) {
                    if (launchSystem.update(launchSystem.returnDistance(follower.getPose()), Tele.speed)) {
                        launchSystem.toggleTracking();
                        follower.followPath(bottomLineup3);
                        configuration.intakeMotor.setPower(0.8);
                        setPathState(10);
                    }
                }
                break;

            case 10: // At gate again — score gate ball; callback fires launch()
                if (!follower.isBusy()) {
                    follower.followPath(score3);
                    setPathState(13);
                }
                break;

            case 11: // Wait for gate-ball launch, then return to gate
                if (!follower.isBusy()) {
                    if (launchSystem.update(launchSystem.returnDistance(follower.getPose()), Tele.speed)) {
                        launchSystem.toggleTracking();
                        follower.followPath(bottomLineup4);
                        configuration.intakeMotor.setPower(0.8);
                        setPathState(12);
                    }
                }
                break;

            case 12: // At gate again — score gate ball; callback fires launch()
                if (!follower.isBusy()) {
                    follower.followPath(score4);
                    setPathState(13);
                }
                break;

            case 13: // Wait for second gate-ball launch, then leave
                if (!follower.isBusy()) {
                    if (launchSystem.update(launchSystem.returnDistance(follower.getPose()), Tele.speed)) {
//                        launchSystem.toggleTracking();
                        configuration.intakeMotor.setPower(0);
                        follower.followPath(leave);
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
        configuration = new Configuration(hardwareMap);
        launchSystem = new LaunchSystem(configuration, LaunchSystem.blueGoalPose);
        configuration.stopper.setPosition(holdBall);
        buildPaths();
    }

    @Override
    public void init_loop() { }

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
        configuration.marco.setPosition(Tele.angleCalculator(currentDist));
        autonomousPathUpdate();
        telemetry.addData("Path State", pathState);
        telemetry.addData("Velo", "%.0f / %.0f", launchSystem.getVelocity(), Tele.speed);
        telemetry.addData("Distance: ", currentDist);
        telemetry.addData("Velocity", "%.0f", launchSystem.getVelocity());
        telemetry.update();
    }

    @Override
    public void stop() {
        Tele.startPose = follower.getPose();
        launchSystem.fullStop();
    }
}
