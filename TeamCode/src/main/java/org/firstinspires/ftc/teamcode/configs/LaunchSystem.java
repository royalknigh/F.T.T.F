package org.firstinspires.ftc.teamcode.configs;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.comp.tele.Tele;

@Configurable
public class LaunchSystem {
    private final DcMotorEx lm1, lm2, im, turret;
    private final Servo stopper, marco;
    private ElapsedTime launchTimer = new ElapsedTime();
    private ElapsedTime turretTimer = new ElapsedTime();

    public static double currentTargetVelocity, idleVelocity = 900;
    public double holdBall = 0.38, passBall = 0.7;

    // --- Heading drift correction ---
    private double headingBiasDeg = 0;
    private double lastHeadingDeg = 0;
    private ElapsedTime headingTimer = new ElapsedTime();

    public static double m1 = 0;
    private double  m2 = 1-m1;


    // --- PID Constants for 145 TPR turret ---
    public static double turretP = 0.02;
    public static double turretI = 0.01;
    public static double turretD = 0.00003;
    private double lastError = 0;
    private double integralSum = 0;

    private final double kS = 0.07; // static friction compensation

    public double P = 20;
    public double F = 15;

    // --- Gearing Logic (384.5 TPR Motor | 190/45 Ratio) ---
    private final double TICKS_PER_DEGREE = (384.5 * (190.0 / 45.0)) / 360.0;           // 145.1 for 1150 rpm

    public double turretOffsetDeg = 0;   // offset in degrees

    private boolean trackingEnabled = false;
    private boolean isResetting = false;
    private boolean isLaunching = false;
    private boolean resetTimer = true;

    public static final Pose blueGoalPose = new Pose(0, 141);
    public static final Pose redGoalPose = new Pose(142, 141);
    private Pose goalPose = blueGoalPose;

    public LaunchSystem(Configuration config, Pose pose) {
        this.lm1 = config.launchMotor1;
        this.lm2 = config.launchMotor2;
        this.im = config.intakeMotor;
        this.turret = config.turretMotor;
        this.stopper = config.stopper;
        this.marco = config.marco;
        this.goalPose = pose;

        lm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Turret init
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);  // prevents drift when idle

        lm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        updatePIDF();
    }

    public void updatePIDF() {
        lm1.setVelocityPIDFCoefficients(P, 0, 0, F);
        lm2.setVelocityPIDFCoefficients(P, 0, 0, F);
    }

    public void updateTurret(Pose robotPose) {
        m2=1-m1;

        double currentDeg = getCurrentDeg();
        double targetDeg;
        boolean activeControl = trackingEnabled || isResetting;

        // --- Determine target ---
        if (trackingEnabled) {
            double dx = goalPose.getX() - robotPose.getX();
            double dy = goalPose.getY() - robotPose.getY();
            double fieldAngle = Math.toDegrees(Math.atan2(dy, dx));

// --- Raw heading ---
            double rawHeading = Math.toDegrees(robotPose.getHeading());

// --- Compute heading rate ---
            double dtHeading = headingTimer.seconds();
            if (dtHeading <= 0) dtHeading = 0.001;
            headingTimer.reset();

            double headingRate = (rawHeading - lastHeadingDeg) / dtHeading;
            lastHeadingDeg = rawHeading;

// --- If robot is not rotating, slowly correct drift ---
            if (Math.abs(headingRate) < 5) { // deg/sec threshold
                headingBiasDeg = headingBiasDeg * m2 + rawHeading * m1;
            }

// --- Corrected heading ---
            double robotHeading = rawHeading - headingBiasDeg;

            targetDeg = normalizeAngle(fieldAngle - robotHeading + turretOffsetDeg);

            targetDeg = Range.clip(targetDeg, -100, 100); // hard limits
        } else {
            // HOLD current position when idle
            targetDeg = currentDeg;
        }

        // --- PID calculation ---
        double error = normalizeAngle(targetDeg - currentDeg);

        // Timing
        double dt = turretTimer.seconds();
        if (dt <= 0) dt = 0.001;
        turretTimer.reset();

        // Integral accumulation only when actively controlling
        if (activeControl) {
            integralSum += error * dt;
        }

        double derivative = (error - lastError) / dt;
        lastError = error;

        double power = (error * turretP) + (integralSum * turretI) + (derivative * turretD);

        // --- Static friction compensation ---
        if (Math.abs(error) > 1.0) {
            power += Math.signum(error) * kS;
        }

        // --- Stop jitter near target ---
        if (Math.abs(error) < 0.5) {
            power = 0;
        }

        // --- Apply power with limits ---
        turret.setPower(Range.clip(power, -1, 1));
    }

    private double normalizeAngle(double angle) {
        angle = Range.clip(angle, -140, 140);
        return angle;
    }

    public double getTargetDeg(Pose robotPose) {
        double dx = goalPose.getX() - robotPose.getX();
        double dy = goalPose.getY() - robotPose.getY();
        double angleToGoalDeg = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeadingDeg = Math.toDegrees(robotPose.getHeading());
        return (angleToGoalDeg - robotHeadingDeg) + turretOffsetDeg;
    }

    public double getCurrentDeg() {
        return turret.getCurrentPosition() / TICKS_PER_DEGREE;
    }


    public boolean isTracking() { return trackingEnabled; }
    public void toggleTracking() { trackingEnabled = !trackingEnabled; isResetting = false; }
    public void startReset() { isResetting = true; trackingEnabled = false; }

    public boolean update(double distance) {
        updatePIDF();

        if (!isLaunching) return true;

        double threshold = currentTargetVelocity;

        if (getVelocity() >= threshold) {
            if(resetTimer) {
                launchTimer.reset();
                resetTimer = false;
            }
            stopper.setPosition(passBall);
            if(distance <90)
                im.setPower(1);
            else
                im.setPower(0.8);

            if (launchTimer.milliseconds() > 500) {
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
    public void setGoal(Pose pose) { goalPose = pose; }

    public double returnDistance(Pose robotPose){
        double dx = goalPose.getX() - robotPose.getX();
        double dy = goalPose.getY() - robotPose.getY();
        return Math.hypot(dx, dy);
    }
    public void adjustOffset(double deltaDeg) {
        turretOffsetDeg += deltaDeg;
    }

    public double getOffsetDeg() {
        return turretOffsetDeg;
    }

    public boolean isLaunching(){
        return isLaunching;
    }

    public void manualZeroTurret() {
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
