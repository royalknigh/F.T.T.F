package org.firstinspires.ftc.teamcode.comp.tele;
import static org.firstinspires.ftc.teamcode.comp.tele.TeleRed.speed;
import static org.firstinspires.ftc.teamcode.configs.LaunchSystem.recoilMult;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.configs.Configuration;
import org.firstinspires.ftc.teamcode.configs.LaunchSystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

@Configurable
@TeleOp
public class TeleBlueFRI extends OpMode {
    private Follower follower;
    private LaunchSystem launchSystem;
    private Configuration config;
    public static Pose startPose;
    public enum State { PIKCUP, LAUNCH }
    public State state = State.PIKCUP;
    public static double angle = 0.5;
    public static double speed = 1500;
    public boolean idle = true;
    public static Servo marco;
    private boolean blue=true;
    public static boolean testing = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(60, 109, Math.toRadians(180)));
        follower.update();
        config = new Configuration(hardwareMap);
        launchSystem = new LaunchSystem(config, LaunchSystem.blueGoalPose);

        this.marco = config.marco;
        angleOffset = 0;
    }
    @Override
    public void init_loop(){
        if(gamepad1.touchpadWasPressed()){
            blue=!blue;
            if(blue){
                launchSystem = new LaunchSystem(config, LaunchSystem.blueGoalPose);
                gamepad1.setLedColor(0,0,255,120000);
                follower.setStartingPose(new Pose(60, 109, Math.toRadians(180)));
            }
            else{
                launchSystem = new LaunchSystem(config, LaunchSystem.bluePurpleGoalPoseAuto);
                gamepad1.setLedColor(255,0,255,120000);
                follower.setStartingPose(new Pose(56.5, -45.1, Math.toRadians(265)));
            }
        }
    }

    @Override
    public void start(){
        follower.startTeleOpDrive();
    }
    @Override
    public void loop() {
        if(gamepad1.touchpadWasPressed())
            blue=!blue;
        if(blue){
            gamepad1.setLedColor(0,0,255,120000);
            launchSystem.setGoalPose(LaunchSystem.blueGoalPose);
        }
        else {
            gamepad1.setLedColor(255,0,255,120000);
            launchSystem.setGoalPose(LaunchSystem.bluepurpleGoalPose);
        }

        if (gamepad1.bWasPressed()){
            launchSystem.adjustOffset(4);
        }

        follower.update();
        // Only one controller drives at a time, so it's safe to sum both gamepads' stick values
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x * 0.5, true);

        if(gamepad1.shareWasPressed())
            testing = !testing;

        // Update Turret and Shooting Logic
        stateMachine();
        launchSystem.updateTurret(follower.getPose(), follower.getVelocity().getXComponent(), follower.getVelocity().getYComponent());

        // --- Nudge Controls (D-Pad Left/Right) ---

        if (gamepad1.xWasPressed()) {
            launchSystem.adjustOffset(-4);
        }

        if (gamepad1.dpadUpWasPressed()) speed += 50;      // Fine-tune speed
        if (gamepad1.dpadDownWasPressed()) speed -= 50;

        if (gamepad1.dpadRightWasPressed()) angleOffset += 0.02; // Fine-tune hood angle
        if (gamepad1.dpadLeftWasPressed()) angleOffset -= 0.02;

        if (config.intakeMotor.isOverCurrent()) {
            gamepad1.rumbleBlips(3);
        }
        if(gamepad1.rightBumperWasPressed()) {
            if(blue){
                follower.setPose(new Pose(19, 122, Math.toRadians(144)));
                launchSystem.manualZeroTurret();
            }
            else{
                follower.setPose(new Pose(11.8, -(131.3), Math.toRadians(180))); // de schimbat, pozitie noua cos mov -131.7
                launchSystem.manualZeroTurret();
            }
        }


        double currentDist = launchSystem.returnDistance(follower.getPose());
        if(blue){
            speedCalculator(currentDist,
                    follower.getVelocity().getXComponent(),
                    follower.getVelocity().getYComponent(),
                    follower.getPose(),
                    LaunchSystem.blueGoalPose);
        }
        else{
            speedCalculator(currentDist,
                    follower.getVelocity().getXComponent(),
                    follower.getVelocity().getYComponent(),
                    follower.getPose(),
                    LaunchSystem.bluepurpleGoalPose); // de modificat in purple goal pose
        }
        if (!launchSystem.isLaunching()) {
            marco.setPosition(angleCalculator(currentDist));
        }

        // --- Hood & Velocity Controls ---
//        speedCalculator(launchSystem.returnDistance(follower.getPose()));
//        marco.setPosition(angleCalculator(launchSystem.returnDistance(follower.getPose())));

        displayData();
    }

    public void stateMachine() {
        if (gamepad1.aWasPressed())
            launchSystem.toggleTracking();
//        if (gamepad1.xWasPressed()) launchSystem.startReset();

//        if(config.intakeMotor.isOverCurrent())
//            gamepad1.rumbleBlips(3);

        switch (state) {
            case PIKCUP:
                if (gamepad1.leftBumperWasPressed()) idle = !idle;
                if (idle) launchSystem.idle(); else launchSystem.fullStop();
                if(gamepad1.left_trigger>0.1)
                    config.intakeMotor.setPower(gamepad1.left_trigger);
                else if( gamepad1.right_trigger>0.2)
                    config.intakeMotor.setPower(-gamepad1.right_trigger);
                else
                    config.intakeMotor.setPower(0);

                if (gamepad1.yWasPressed()) {
                    state = State.LAUNCH;
                    launchSystem.start(speed);
                }
                break;
            case LAUNCH:
                if (launchSystem.update(launchSystem.returnDistance(follower.getPose()), speed)) {
                    state = State.PIKCUP;
                    gamepad1.rumbleBlips(3);

                }
                break;
        }
    }

    public void displayData() {
        telemetry.addData("--- TURRET ---", "");
        telemetry.addData("Mode", launchSystem.isTracking() ? "AUTO-AIM" : "MANUAL/RESET");
        telemetry.addData("Target Deg", "%.2f", launchSystem.getTargetDeg(follower.getPose()));
        telemetry.addData("Current Deg", "%.2f", launchSystem.getCurrentDeg());
        telemetry.addData("Offset (Ticks)", launchSystem.turretOffsetDeg);
        telemetry.addData("Launcher Angle: ", angle);

        telemetry.addData("--- LOCALIZATION ---", "");
        telemetry.addData("Heading", "%.2f Deg", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("x: ", follower.getPose().getX()) ;
        telemetry.addData("y: ", follower.getPose().getY()) ;
        telemetry.addData("Blue", blue);
        telemetry.addData("distance", launchSystem.returnDistance(follower.getPose()));



        telemetry.addData("--- FLYWHEEL ---", "");
        telemetry.addData("servo: ", angle);
        telemetry.addData("Velocity", "%.0f / %.0f", launchSystem.getVelocity(), speed);

        telemetry.addData("current: ", config.intakeMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("testing9: ", testing);
        telemetry.update();
    }

    public static double angleOffset = 0;

    public static double angleCalculator(double x){
        if(!testing)
            angle = -0.000075*x*x+0.01815*x-0.241667+0.02 +angleOffset;
        angle = Range.clip(angle, 0, 0.85);
        return angle;
    }

    public static double speedVelocityGain = 3; // tune this

    public static void speedCalculator(double x){
        speed = -0.0925325*x*x+24.25649*x+728.0303-50;
        LaunchSystem.idleVelocity = speed;
    }

    public static void speedCalculator(double x, double robotVelX, double robotVelY, Pose robotPose, Pose goalPose) {
        if (!testing)
            speed = -0.0925325*x*x+24.25649*x+728.0303-50;

        // Dot product: how much of robot velocity is toward/away from goal
        double dx = goalPose.getX() - robotPose.getX();
        double dy = goalPose.getY() - robotPose.getY();
        double dist = Math.hypot(dx, dy);
        double velTowardGoal = (robotVelX * dx + robotVelY * dy) / dist; // positive = moving toward

        speed -= velTowardGoal * speedVelocityGain; // moving toward = reduce speed, away = increase

        LaunchSystem.idleVelocity = speed - speedDifference;
        speed = Range.clip(speed, 1000, 2500);
    }

    public static void recoilCalculator(double x){
        recoilMult  = 0.5;
    }

    public static double speedDifference = 0; //   was 150


    public Pose getPose(){
        return follower.getPose();
    }
}