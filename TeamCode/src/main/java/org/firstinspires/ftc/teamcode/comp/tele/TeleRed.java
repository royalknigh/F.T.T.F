package org.firstinspires.ftc.teamcode.comp.tele;
import static org.firstinspires.ftc.teamcode.comp.tele.Tele.speedDifference;
import static org.firstinspires.ftc.teamcode.comp.tele.Tele.speedVelocityGain;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.configs.Configuration;
import org.firstinspires.ftc.teamcode.configs.LaunchSystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp
public class TeleRed extends OpMode {
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

    public static boolean testing = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(8, 7, Math.toRadians(180)));
        follower.update();
        config = new Configuration(hardwareMap);
        launchSystem = new LaunchSystem(config, LaunchSystem.redGoalPose);
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

        if (gamepad1.shareWasPressed())
            testing = !testing;

        stateMachine();
        launchSystem.updateTurret(follower.getPose(), follower.getVelocity().getXComponent(), follower.getVelocity().getYComponent());

        if (gamepad1.bWasPressed()) {
            launchSystem.adjustOffset(4);
        }
        if (gamepad1.xWasPressed()) {
            launchSystem.adjustOffset(-4);
        }

        if (gamepad1.dpadUpWasPressed()) speed += 50;
        if (gamepad1.dpadDownWasPressed()) speed -= 50;
        if (gamepad1.dpadRightWasPressed()) angle += 0.02;
        if (gamepad1.dpadLeftWasPressed()) angle -= 0.02;

        if (config.intakeMotor.isOverCurrent()) gamepad1.rumbleBlips(3);

        if (gamepad1.rightBumperWasPressed()) {
            follower.setPose(new Pose(124, 120, Math.toRadians(36)));
            launchSystem.manualZeroTurret();
        }

        double currentDist = launchSystem.returnDistance(follower.getPose());
        speedCalculator(currentDist,
                follower.getVelocity().getXComponent(),
                follower.getVelocity().getYComponent(),
                follower.getPose(),
                LaunchSystem.redGoalPose);
        if (!launchSystem.isLaunching()) {
            marco.setPosition(angleCalculator(currentDist));
        }

        displayData();
    }

    public void stateMachine() {
        if (gamepad1.aWasPressed()) launchSystem.toggleTracking();

        switch (state) {
            case PIKCUP:
                if (gamepad1.leftBumperWasPressed()) idle = !idle;
                if (idle) launchSystem.idle(); else launchSystem.fullStop();
                config.intakeMotor.setPower(gamepad1.left_trigger > 0.1 ? gamepad1.left_trigger : 0);

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
        telemetry.addData("x: ", follower.getPose().getX());
        telemetry.addData("y: ", follower.getPose().getY());

        telemetry.addData("distance", launchSystem.returnDistance(follower.getPose()));

        telemetry.addData("--- FLYWHEEL ---", "");
        telemetry.addData("servo: ", angle);
        telemetry.addData("Velocity", "%.0f / %.0f", launchSystem.getVelocity(), speed);

        telemetry.addData("current: ", config.intakeMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("testing: ", testing);
        telemetry.update();
    }

    public static double angleCalculator(double x) {
        if (!testing)
            angle = 0.00000190236*x*x*x-0.000602309*x*x+0.0657463*x-1.93061 +0.3;
        angle = Range.clip(angle, 0, 0.85);
        return angle;
    }


//    public static double speedDifference = 0;

    public static void speedCalculator(double x, double robotVelX, double robotVelY, Pose robotPose, Pose goalPose) {
        if (!testing)
            speed = -0.0395022*x*x + 15.15043*x + 900.75758+200;

        double dx = goalPose.getX() - robotPose.getX();
        double dy = goalPose.getY() - robotPose.getY();
        double dist = Math.hypot(dx, dy);
        double velTowardGoal = (robotVelX * dx + robotVelY * dy) / dist;

        speed -= velTowardGoal * speedVelocityGain;

        LaunchSystem.idleVelocity = speed - speedDifference;
        speed = Range.clip(speed, 1000, 2500);
    }

    public Pose getPose() {
        return follower.getPose();
    }
}