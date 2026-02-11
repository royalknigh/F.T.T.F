package org.firstinspires.ftc.teamcode.configs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.pedropathing.geometry.Pose;

public class LaunchSystem {
    private final DcMotorEx lm1, lm2, im, turret;
    private final Servo stopper;
    private ElapsedTime launchTimer = new ElapsedTime();
    private ElapsedTime turretTimer = new ElapsedTime();

    public static double currentTargetVelocity, idleVelocity = 900;
    public double holdBall = 0.71, passBall = 0.95;

    // --- PID Constants ---
    public static double turretP = 0.02;
    public static double turretI = 0.0;
    public static double turretD = 0.0005;
    private double lastError = 0;
    private double integralSum = 0;

    // --- Gearing Logic ---
    // Motor: 145.1 ticks | Gear Ratio: 190/45 (4.22)
    // Formula: (TicksPerRev * GearRatio) / 360
    private final double TICKS_PER_DEGREE = (537.7 * (190.0 / 45.0)) / 360.0;

    public int turretOffset = 0; // Controlled by D-pad in TeleOp
    private boolean trackingEnabled = false;
    private boolean isResetting = false;
    private boolean isLaunching = false;
    private boolean resetTimer = true;

    public final Pose blueGoalPose = new Pose(12, 136);
    public final Pose redGoalPose = new Pose(130, 136);
    private Pose goalPose = blueGoalPose;

    public LaunchSystem(Configuration config) {
        this.lm1 = config.launchMotor1;
        this.lm2 = config.launchMotor2;
        this.im = config.intakeMotor;
        this.turret = config.turretMotor;
        this.stopper = config.stopper;


        lm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void updateTurret(Pose robotPose) {
        double targetLocalDeg = 0;

        if (isResetting) {
            targetLocalDeg = 0;
            // Stop resetting state once within 1 degree of front
            if (Math.abs(getCurrentDeg()) < 1.0) isResetting = false;
        } else if (trackingEnabled) {
            // 1. Calculate Field-Relative Angle to Goal
            double dx = goalPose.getX() - robotPose.getX();
            double dy = goalPose.getY() - robotPose.getY();
            double angleToGoalDeg = Math.toDegrees(Math.atan2(dy, dx));

            // 2. Normalize Robot Heading (Input is Radians, convert to [-180, 180])
            double robotHeadingDeg = Math.toDegrees(robotPose.getHeading());
            double normalizedRobotHeading = ((robotHeadingDeg + 180) % 360 + 360) % 360 - 180;

            // 3. Target Local Angle = Goal Angle - Robot Heading + Manual Offset
            targetLocalDeg = (angleToGoalDeg - normalizedRobotHeading) + (turretOffset / TICKS_PER_DEGREE);
        } else {
            turret.setPower(0);
            return;
        }

        // --- PID Implementation ---
        double currentLocalDeg = getCurrentDeg();
        double error = targetLocalDeg - currentLocalDeg;

        // Ensure turret takes the shortest path
        while (error > 180) error -= 360;
        while (error < -180) error += 360;

        double dt = turretTimer.seconds();
        if (dt <= 0) dt = 0.001;
        turretTimer.reset();

        integralSum = Range.clip(integralSum + (error * dt), -0.5, 0.5);
        double derivative = (error - lastError) / dt;
        lastError = error;

        double power = (error * turretP) + (integralSum * turretI) + (derivative * turretD);

        // --- Soft Limits ---
        // Prevents spinning past ±160 degrees to protect wires
        if (currentLocalDeg > 160 && power > 0) power = 0;
        if (currentLocalDeg < -160 && power < 0) power = 0;

        turret.setPower(Range.clip(power, -1.0, 1.0));
    }

    // --- Diagnostic Getters for Telemetry ---
    public double getTargetDeg(Pose robotPose) {
        double dx = goalPose.getX() - robotPose.getX();
        double dy = goalPose.getY() - robotPose.getY();
        double angleToGoalDeg = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeadingDeg = Math.toDegrees(robotPose.getHeading());
        double normalizedRobotHeading = ((robotHeadingDeg + 180) % 360 + 360) % 360 - 180;
        return (angleToGoalDeg - normalizedRobotHeading) + (turretOffset / TICKS_PER_DEGREE);
    }

    public double getCurrentDeg() { return turret.getCurrentPosition() / TICKS_PER_DEGREE; }
    public boolean isTracking() { return trackingEnabled; }
    public void toggleTracking() { trackingEnabled = !trackingEnabled; isResetting = false; }
    public void startReset() { isResetting = true; trackingEnabled = false; }

    // --- Flywheel Logic ---
    public boolean update() {
        if (!isLaunching) return true;
        if (getVelocity() >= (currentTargetVelocity)) {
            if(resetTimer){ launchTimer.reset(); resetTimer = false; }
            stopper.setPosition(passBall);
            im.setPower(1);
            if (launchTimer.milliseconds() > 1000) {
                isLaunching = false;
                return true;
            }
        }
        return false;
    }

    public void start(double target) {
        this.currentTargetVelocity = target;
        this.isLaunching = true;
        this.resetTimer = true;
        lm1.setVelocity(target);
        lm2.setVelocity(target);
    }

    public void idle() {
        isLaunching = false;
        stopper.setPosition(holdBall);
        im.setPower(0);
        lm1.setVelocity(idleVelocity);
        lm2.setVelocity(idleVelocity);
    }

    public void fullStop() { isLaunching = false; lm1.setVelocity(0); lm2.setVelocity(0); }
    public double getVelocity() { return (lm1.getVelocity() + lm2.getVelocity()) / 2.0; }
    public void setGoal(Pose pose) { goalPose = pose; }
}