package org.firstinspires.ftc.teamcode.noncomp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Simple Servo", group = "Tuning")
public class ServoControl extends LinearOpMode {

    private Servo servo;
    private DcMotor motor;
    private double position =0.5;

    @Override
    public void runOpMode() {

        servo = hardwareMap.get(Servo.class, "marco");
        motor = hardwareMap.get(DcMotor.class, "im");

        waitForStart();

        while (opModeIsActive()) {

            if(-gamepad1.left_stick_y > 0)
                position +=0.002;
            if(-gamepad1.left_stick_y<0)
                position -= 0.002;

            position = Range.clip(position,0,0.85);

            if(gamepad1.right_trigger>0.1) motor.setPower(gamepad1.right_trigger);
            else motor.setPower(0);

            servo.setPosition(position);
            telemetry.addData("position", position);
            telemetry.update();
        }
    }
}
