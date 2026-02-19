package org.firstinspires.ftc.teamcode.configs;

import static org.firstinspires.ftc.teamcode.comp.tele.TeleRed.speed;

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

    public static double currentTargetVelocity, idleVelocity = 1300;

    // PERSISTENCE: Saved from Auto.stop()
    public static double lastSavedPosition = 0;

    public double holdBall = 0.38, passBall = 0.7;

    // --- PID Constants ---
    public static double turretP = 0.017;
    public static double turretI = 0.005;
    public static double turretD = 0.00003;
    private double lastError = 0;
    private double integralSum = 0;
    private final double kS = 0.07;

    public static double P = 25;
    public static double F = 14.5;

    // --- Gearing Logic (REV Through-Bore 8192 TPR | 190/30 Ratio) ---
    private final double TICKS_PER_DEGREE = (8192.0 * (190.0 / 35.0)) / 360.0;

    public double turretOffsetDeg = 0;
    private boolean trackingEnabled = false;
    private boolean isResetting = false;
    private boolean isLaunching = false;
    private boolean resetTimer = true;
    private boolean go = false;

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

            // 1. Calculăm unde ar fi coșul în mod ideal (-180 la 180)
            double rawTarget = betterNormalize(fieldAngle - robotHeading);

            // 2. LIMITARE STRICTĂ (-100 la 100)
            // Dacă coșul este în afara range-ului, tureta "îngheață" la marginea cea mai apropiată.
            if (rawTarget > 100) {
                targetDeg = 100;
            } else if (rawTarget < -100) {
                targetDeg = -100;
            } else {
                targetDeg = rawTarget;
            }
        } else {
            targetDeg = isResetting ? 0 : currentDeg;
        }

        // 3. CALCULUL ERORII LINIAR (FĂRĂ betterNormalize)
        // Dacă tureta e la 100 și ținta devine -100, eroarea va fi -200.
        // Motorul va roti tureta invers pe tot parcursul celor 200 de grade.
        double error = targetDeg - currentDeg;

        // PID Math
        double dt = turretTimer.seconds();
        if (dt <= 0) dt = 0.001;
        turretTimer.reset();

        if (trackingEnabled || isResetting) {
            integralSum += error * dt;
            // Anti-windup pentru a proteja motorul la margini
            integralSum = Range.clip(integralSum, -20, 20);
        } else {
            integralSum = 0;
        }

        double derivative = (error - lastError) / dt;
        lastError = error;

        double power = (error * turretP) + (integralSum * turretI) + (derivative * turretD);

        // Static friction compensation (kS)
        if (Math.abs(error) > 1.0) power += Math.signum(error) * kS;

        // Deadzone pentru stabilitate
        if (Math.abs(error) < 0.5) power = 0;

        // Limităm puterea pentru a nu brusca mecanismul la capete
        turretMotor.setPower(Range.clip(power, -1, 1));
    }

    private double betterNormalize(double degrees) {
        while (degrees > 180) degrees -= 360;
        while (degrees < -180) degrees += 360;
        return degrees;
    }

    public double getCurrentDeg() {
        return -(turretEncoderPort.getCurrentPosition() / TICKS_PER_DEGREE) + turretOffsetDeg;
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

    public void start(double target) {
//        this.currentTargetVelocity = target;
        this.isLaunching = true;
        this.resetTimer = true;
//        lm1.setVelocity(target);
//        lm2.setVelocity(target);
    }

    public void idle() {
        if (isLaunching) return;
        isLaunching = false;
        stopper.setPosition(holdBall);
        im.setPower(0);

        lm1.setVelocity(idleVelocity);
        lm2.setVelocity(idleVelocity);
    }


    private boolean speedReached = false;

    public boolean update(double distance, double currentDynamicSpeed) {
        updatePIDF();
        if (!isLaunching) return true;

        lm1.setVelocity(currentDynamicSpeed);
        lm2.setVelocity(currentDynamicSpeed);
        this.currentTargetVelocity = currentDynamicSpeed;

        // Check speed only once
        if (!speedReached && getVelocity() >= currentTargetVelocity - 10) {
            speedReached = true;
            launchTimer.reset();
        }

        if (speedReached) {
            if (launchTimer.milliseconds() > 100) {
                stopper.setPosition(passBall);
                if (distance <= 90) im.setPower(0.8);
                else if (distance < 110) im.setPower(0.75);
                else im.setPower(0.7);
            }

            if (launchTimer.milliseconds() > 900) {
                isLaunching = false;
                speedReached = false; // reset for next shot
                resetTimer = true;
                stopper.setPosition(holdBall);
                im.setPower(0);
                idle();
                return true;
            }
        }

        return false;
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