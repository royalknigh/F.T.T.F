package org.firstinspires.ftc.teamcode.noncomp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Simple Servo", group = "Tuning")
public class ServoControl extends LinearOpMode {

    private Servo servo, servo1;
    private DcMotor motor;
    private double position =0.5, position1 = 0.5;

    @Override
    public void runOpMode() {

        servo = hardwareMap.get(Servo.class, "stopper");
        servo1 = hardwareMap.get(Servo.class, "marco");

        waitForStart();

        while (opModeIsActive()) {

            if(-gamepad1.left_stick_y > 0)
                position +=0.002;
            if(-gamepad1.left_stick_y<0)
                position -= 0.002;

            if(-gamepad1.right_stick_y > 0)
                position1 +=0.002;
            if(-gamepad1.right_stick_y<0)
                position1 -= 0.002;


            position = Range.clip(position,0,0.85);
            position1= Range.clip(position1,0,0.85);



            servo.setPosition(position);
            servo1.setPosition(position1);
            telemetry.addData("position", position);
            telemetry.addData("position1", position1);
            telemetry.update();
        }
    }
}
