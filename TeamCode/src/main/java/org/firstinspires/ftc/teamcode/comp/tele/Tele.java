package org.firstinspires.ftc.teamcode.comp.tele;
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
public class Tele extends OpMode {
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
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(8, 7, Math.toRadians(180)));
        follower.update();
        config = new Configuration(hardwareMap);
        launchSystem = new LaunchSystem(config, LaunchSystem.blueGoalPose);
        this.marco = config.marco;
    }

    @Override
    public void start(){
        follower.startTeleOpDrive();
    }

    @Override
    public void loop() {
        follower.update();
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x * 0.5, true);

        // Update Turret and Shooting Logic
        stateMachine();
        launchSystem.updateTurret(follower.getPose());

        // --- Nudge Controls (D-Pad Left/Right) ---

//        if (gamepad1.dpadUpWasPressed()) {
//            launchSystem.adjustOffset(7);
//        }
//
//        if (gamepad1.dpadDownWasPressed()) {
//            launchSystem.adjustOffset(-7);
//        }

        if (gamepad1.dpadUpWasPressed()) speed += 50;      // Fine-tune speed
        if (gamepad1.dpadDownWasPressed()) speed -= 50;
        if (gamepad1.dpadRightWasPressed()) angle += 0.02; // Fine-tune hood angle
        if (gamepad1.dpadLeftWasPressed()) angle -= 0.02;

        if (config.intakeMotor.isOverCurrent()) gamepad1.rumbleBlips(3);




        if(gamepad1.rightBumperWasPressed()) {
            follower.setPose(new Pose(25, 125, Math.toRadians(143)));
            launchSystem.manualZeroTurret();
        }

        // --- Hood & Velocity Controls ---
        speedCalculator(launchSystem.returnDistance(follower.getPose()));
        marco.setPosition(angleCalculator(launchSystem.returnDistance(follower.getPose())));

        displayData();
    }

    public void stateMachine() {
        if (gamepad1.aWasPressed()) launchSystem.toggleTracking();
        if (gamepad1.xWasPressed()) launchSystem.startReset();

//        if(config.intakeMotor.isOverCurrent())
//            gamepad1.rumbleBlips(3);

        switch (state) {
            case PIKCUP:
                if (gamepad1.leftBumperWasPressed()) idle = !idle;
                if (idle) launchSystem.idle(); else launchSystem.fullStop();
                config.intakeMotor.setPower(gamepad1.left_trigger > 0.1 ? gamepad1.left_trigger : 0);
                LaunchSystem.idleVelocity = speed-300;

                if (gamepad1.yWasPressed()) {
                    state = State.LAUNCH;
                    launchSystem.start(speed);
                }
                break;
            case LAUNCH:
                if (launchSystem.update(launchSystem.returnDistance(follower.getPose()))) {
                    state = State.PIKCUP;
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

        telemetry.addData("distance", launchSystem.returnDistance(follower.getPose()));



        telemetry.addData("--- FLYWHEEL ---", "");
        telemetry.addData("servo: ", angle);
        telemetry.addData("Velocity", "%.0f / %.0f", launchSystem.getVelocity(), speed);

        telemetry.addData("current: ", config.intakeMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.update();
    }

    public static double angleCalculator(double x){
//        if(gamepad1.dpadUpWasPressed()) angle += 0.03;
//        if(gamepad1.dpadDownWasPressed()) angle -= 0.03;
//        angle = -0.0000347794*x*x+0.00953371*x-0.209821+0.06;
        angle = Range.clip(angle, 0.15, 0.85);
        return angle;
    }

    public static void speedCalculator(double x){
//        if (gamepad1.dpadRightWasPressed()) speed += 50;
//        if (gamepad1.dpadLeftWasPressed())  speed -= 50;
//        speed = 7.97132*x+1066.07612+75;
        speed = Range.clip(speed, 1000, 2500);

    }


    public Pose getPose(){
        return follower.getPose();
    }
}