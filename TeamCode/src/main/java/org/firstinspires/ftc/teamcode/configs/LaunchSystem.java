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
    public double holdBall = 0.56, passBall = 0.95;

    // --- PID Constants ---
    public static double turretP = 0.02;
    public static double turretI = 0.0;
    public static double turretD = 0.0005;
    private double lastError = 0;
    private double integralSum = 0;

    public double P = 30;
    public double F = 13.5;

    // --- Gearing Logic (537.6 TPR Motor | 190/45 Ratio) ---
    private final double TICKS_PER_DEGREE = (537.6 * (190.0 / 45.0)) / 360.0;

    public int turretOffset = 0;
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

        lm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Turret init
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        updatePIDF();
    }

    public void updatePIDF() {
        lm1.setVelocityPIDFCoefficients(P, 0, 0, F);
        lm2.setVelocityPIDFCoefficients(P, 0, 0, F);
    }

    public void updateTurret(Pose robotPose) {
        double targetLocalDeg = 0;

        if (isResetting) {
            targetLocalDeg = 0;
            if (Math.abs(getCurrentDeg()) < 1.0) isResetting = false;
        } else if (trackingEnabled) {
            // 1. Get Global Angle to Goal (Field Space)
            double dx = goalPose.getX() - robotPose.getX();
            double dy = goalPose.getY() - robotPose.getY();
            double angleToGoalDeg = Math.toDegrees(Math.atan2(dy, dx));

            // 2. Get Robot Heading in Degrees
            double robotHeadingDeg = Math.toDegrees(robotPose.getHeading());

            // 3. Calculation: Target = GoalAngle - RobotHeading
            // NOTE: If the turret turns WITH the robot instead of AGAINST it,
            // change this to: targetLocalDeg = (robotHeadingDeg - angleToGoalDeg);
            targetLocalDeg = (angleToGoalDeg - robotHeadingDeg);
        } else {
            turret.setPower(0);
            return;
        }

        targetLocalDeg += (turretOffset / TICKS_PER_DEGREE);

        double currentLocalDeg = getCurrentDeg();
        double error = targetLocalDeg - currentLocalDeg;

        // Wrap error for shortest path
        while (error > 180) error -= 360;
        while (error < -180) error += 360;

        double dt = turretTimer.seconds();
        if (dt <= 0) dt = 0.001;
        turretTimer.reset();

        integralSum = Range.clip(integralSum + (error * dt), -0.5, 0.5);
        double derivative = (error - lastError) / dt;
        lastError = error;

        double power = (error * turretP) + (integralSum * turretI) + (derivative * turretD);

        // Soft Limits
        if (currentLocalDeg > 160 && power > 0) power = 0;
        if (currentLocalDeg < -160 && power < 0) power = 0;

        turret.setPower(Range.clip(power, -1.0, 1.0));
    }

    public double getTargetDeg(Pose robotPose) {
        double dx = goalPose.getX() - robotPose.getX();
        double dy = goalPose.getY() - robotPose.getY();
        double angleToGoalDeg = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeadingDeg = Math.toDegrees(robotPose.getHeading());
        return (angleToGoalDeg - robotHeadingDeg) + (turretOffset / TICKS_PER_DEGREE);
    }

    public double getCurrentDeg() { return turret.getCurrentPosition() / TICKS_PER_DEGREE; }

    public void resetTurretEncoder() {
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lastError = 0;
        integralSum = 0;
    }

    public boolean isTracking() { return trackingEnabled; }
    public void toggleTracking() { trackingEnabled = !trackingEnabled; isResetting = false; }
    public void startReset() { isResetting = true; trackingEnabled = false; }

    public boolean update() {
        updatePIDF();
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

    public double returnDistance(Pose robotPose){
        double dx = goalPose.getX() - robotPose.getX();
        double dy = goalPose.getY() - robotPose.getY();
        return Math.hypot(dx, dy);
    }
}