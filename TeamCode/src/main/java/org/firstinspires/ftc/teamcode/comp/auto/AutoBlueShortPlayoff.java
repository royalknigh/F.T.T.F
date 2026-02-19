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
    private ElapsedTime gateTimer = new ElapsedTime();
    private int pathState = 0;
    private boolean hasStartedLaunch = false;

    private final Pose startPose = new Pose(36, 137, Math.toRadians(180));
    private final Pose scorePose = new Pose(50, 86, Math.toRadians(180));

    private final Pose fisrtLinePose = new Pose(48, 84, Math.toRadians(180));
    private final Pose pickup1Pose = new Pose(20, 84, Math.toRadians(180));

    private final Pose secondLinePose = new Pose(48, 60, Math.toRadians(180));
    private final Pose pickup2Pose = new Pose(10, 60, Math.toRadians(180));
    private final Pose openGatePose = new Pose(15, 69, Math.toRadians(180));

    private final Pose pickupGate = new Pose(6, 55, Math.toRadians(140));

    private final Pose thirdLinePose = new Pose(48, 38, Math.toRadians(180));
    private final Pose pickup3Pose = new Pose(10, 38, Math.toRadians(180));

//    private final Pose human = new Pose(6, 10, Math.toRadians(230));

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
        configuration.marco.setPosition(Tele.angleCalculator(currentDist)+0.05);

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
                    setPathState(1);
                }
                break;

            case 1: // Preload Launch, middle row pickup
                if(!follower.isBusy()) {
                    if(launchSystem.update(launchSystem.returnDistance(follower.getPose()), Tele.speed)){
                        follower.followPath(middleRow);
                        launchSystem.toggleTracking();
                        setPathState(2);
                    }
                }
                break;
            case 2: //middle row launch
                if(!follower.isBusy()) {
                    if(launchSystem.update(launchSystem.returnDistance(follower.getPose()), Tele.speed)){
                        follower.followPath(openGate);
                        launchSystem.toggleTracking();
                        gateTimer.reset();
                        setPathState(3);
                    }
                }
                break;

            case 3:
                if(!follower.isBusy() && gateTimer.seconds()>1.5){
                    follower.followPath(gateCollect);
                    setPathState(4);
                    gateTimer.reset();
                }
                break;
            case 4:
                if(!follower.isBusy() && gateTimer.seconds()>2){
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
                        gateTimer.reset();
                        setPathState(8);
                    }
                }
                break;
            case 8:
                if(!follower.isBusy()&& gateTimer.seconds()>1.5){
                    follower.followPath(gateCollect);
                    gateTimer.reset();
                    setPathState(9);
                }
                break;
            case 9:
                if(!follower.isBusy() && gateTimer.seconds()>2.){
                    follower.followPath(scoreGate);
                    setPathState(10);
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
                        follower.followPath(openGate);
                        hasStartedLaunch = false;
                        launchSystem.toggleTracking();
                        setPathState(8);
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
//                .addParametricCallback(0.5, () -> launchSystem.adjustOffset(-7))
                .addParametricCallback(0.7, () -> launch())
                .build();
        middleRow = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, secondLinePose))
                .setConstantHeadingInterpolation(scorePose.getHeading())

                .addPath(new BezierLine(secondLinePose, pickup2Pose))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .addParametricCallback(0, ()-> configuration.intakeMotor.setPower(1))

                .addPath(new BezierCurve(pickup2Pose, new Pose(43,68),scorePose))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .addParametricCallback(0.4, ()-> configuration.intakeMotor.setPower(0))
                .addParametricCallback(0.8, () -> launch())
                .build();

        
        firstRow = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .addParametricCallback(0, ()-> configuration.intakeMotor.setPower(1))

                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .addParametricCallback(0.5, ()-> configuration.intakeMotor.setPower(0))
                .build();

        openGate = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(45,65), openGatePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), openGatePose.getHeading())
                .build();

        gateCollect = follower.pathBuilder()
                .addPath(new BezierLine(openGatePose, pickupGate))
                .setLinearHeadingInterpolation(openGatePose.getHeading(), pickupGate.getHeading())
                .addParametricCallback(0, ()-> configuration.intakeMotor.setPower(1))
                .build();

        scoreGate = follower.pathBuilder()
                .addPath(new BezierCurve(pickupGate, new Pose(45,65), scorePose))
                .setLinearHeadingInterpolation(pickupGate.getHeading(), scorePose.getHeading())
                .addParametricCallback(0.2, ()-> configuration.intakeMotor.setPower(0))
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, secondLinePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), secondLinePose.getHeading())
                .build();
        
    }

    public void launch(){
        launchSystem.start(Tele.speed);
        launchSystem.toggleTracking();
    }

    @Override
    public void stop(){
        Tele.startPose = follower.getPose();
        launchSystem.fullStop();
    }
}