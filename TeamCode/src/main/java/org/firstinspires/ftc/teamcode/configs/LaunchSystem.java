package org.firstinspires.ftc.teamcode.configs;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.pedropathing.geometry.Pose;

@Configurable
public class LaunchSystem {
    // Separate the physical motor from the data port
    private final DcMotorEx lm1, lm2, im, turretMotor, turretEncoderPort;
    private final Servo stopper, marco;

    private ElapsedTime launchTimer = new ElapsedTime();
    private ElapsedTime turretTimer = new ElapsedTime();

    public static double currentTargetVelocity, idleVelocity = 900;

    // PERSISTENCE: Saved from Auto.stop()
    public static double lastSavedPosition = 0;

    public double holdBall = 0.38, passBall = 0.7;

    // --- PID Constants ---
    public static double turretP = 0.015;
    public static double turretI = 0.01;
    public static double turretD = 0.00003;
    private double lastError = 0;
    private double integralSum = 0;
    private final double kS = 0.07;

    public static double P = 20;
    public static double F = 13.5;

    // --- Gearing Logic (REV Through-Bore 8192 TPR | 190/30 Ratio) ---
    private final double TICKS_PER_DEGREE = (8192.0 * (190.0 / 30.0)) / 360.0;

    public double turretOffsetDeg = 0;
    private boolean trackingEnabled = false;
    private boolean isResetting = false;
    private boolean isLaunching = false;
    private boolean resetTimer = true;

    public static final Pose blueGoalPose = new Pose(0, 141);
    public static final Pose redGoalPose = new Pose(144, 142);
    private Pose goalPose = blueGoalPose;

    public LaunchSystem(Configuration config, Pose pose) {
        this.lm1 = config.launchMotor1;
        this.lm2 = config.launchMotor2;
        this.im = config.intakeMotor;
        this.stopper = config.stopper;
        this.marco = config.marco;
        this.goalPose = pose;

        // CRITICAL: Decouple Power and Data
        this.turretMotor = config.turretMotor;
        this.turretEncoderPort = config.frontLeftMotor; // Encoder is here

        // Setup Flywheels
        lm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Setup Turret Data Port
//        if (!isTeleop) {
            turretEncoderPort.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        }
        turretEncoderPort.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Setup Turret Motor
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Load the handoff position
        this.turretOffsetDeg = lastSavedPosition;

        updatePIDF();
    }

    public void updateTurret(Pose robotPose) {
        double currentDeg = getCurrentDeg();
        double targetDeg;

        if (trackingEnabled) {
            double dx = goalPose.getX() - robotPose.getX();
            double dy = goalPose.getY() - robotPose.getY();
            double fieldAngle = Math.toDegrees(Math.atan2(dy, dx));
            double robotHeading = Math.toDegrees(robotPose.getHeading());

            // 1. Get the raw relative angle
            double rawTarget = betterNormalize(fieldAngle - robotHeading);

            // 2. INTELLIGENT CLAMPING
            // Instead of just clipping, we check if the snap is happening
            if (rawTarget > 140 || rawTarget < -140) {
                // If the goal is in the "dead zone" behind the robot,
                // stay at the limit we are currently closest to.
                targetDeg = (currentDeg > 0) ? 140 : -140;
            } else {
                targetDeg = rawTarget;
            }
        } else {
            targetDeg = isResetting ? 0 : currentDeg;
        }

        // 3. Calculate error with the standard 180 normalization
        double error = betterNormalize(targetDeg - currentDeg);

        // PID Math
        double dt = turretTimer.seconds();
        if (dt <= 0) dt = 0.001;
        turretTimer.reset();

        if (trackingEnabled || isResetting) integralSum += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;

        double power = (error * turretP) + (integralSum * turretI) + (derivative * turretD);

        // Static friction compensation
        if (Math.abs(error) > 1.0) power += Math.signum(error) * kS;

        // Deadzone to prevent jitter at the limits
        if (Math.abs(error) < 0.5) power = 0;

        turretMotor.setPower(Range.clip(power, -1, 1));
    }

    private double betterNormalize(double degrees) {
        while (degrees > 180) degrees -= 360;
        while (degrees < -180) degrees += 360;
        return degrees;
    }

    public double getCurrentDeg() {
        return (turretEncoderPort.getCurrentPosition() / TICKS_PER_DEGREE) + turretOffsetDeg;
    }

    public void manualZeroTurret() {
        turretEncoderPort.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretEncoderPort.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretOffsetDeg = 0;
        lastSavedPosition = 0;
    }

    public void toggleTracking() { trackingEnabled = !trackingEnabled; isResetting = false; }
    public void startReset() { isResetting = true; trackingEnabled = false; }
    public boolean isTracking() { return trackingEnabled; }

    public boolean update(double distance) {
        updatePIDF();
        if (!isLaunching) return true;
        if (getVelocity() >= currentTargetVelocity) {
            if(resetTimer) { launchTimer.reset(); resetTimer = false; }
            stopper.setPosition(passBall);
            if(distance<=90) im.setPower(0.85);
            if(distance> 90 && distance<110) im.setPower(0.8);
            if(distance >110) im.setPower(0.7);
            if (launchTimer.milliseconds() > 600) {
                isLaunching = false;
                resetTimer = true;
                stopper.setPosition(holdBall);
                im.setPower(0);
                idle();
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
    public double returnDistance(Pose robotPose){
        return Math.hypot(goalPose.getX() - robotPose.getX(), goalPose.getY() - robotPose.getY());
    }
    public void adjustOffset(double delta) { turretOffsetDeg += delta; }
    public void updatePIDF() {
        lm1.setVelocityPIDFCoefficients(P, 0, 0, F);
        lm2.setVelocityPIDFCoefficients(P, 0, 0, F);
    }
    public boolean isLaunching(){
        return isLaunching;
    }
    public double getTargetDeg(Pose robotPose) {

        double dx = goalPose.getX() - robotPose.getX();
        double dy = goalPose.getY() - robotPose.getY();
        double angleToGoalDeg = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeadingDeg = Math.toDegrees(robotPose.getHeading());
        return (angleToGoalDeg - robotHeadingDeg) + turretOffsetDeg;

    }
}