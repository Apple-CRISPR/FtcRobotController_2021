package org.firstinspires.ftc.teamcode.Testcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorDigitalTouch;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.AcRobot;
import org.firstinspires.ftc.teamcode.HardwareClass;

@TeleOp (name = "test1", group = "TeleOp")
public class test1 extends OpMode {

    private AcRobot robot = new AcRobot();



    @Override
    public void init() {

        robot.initHardware(hardwareMap);



        telemetry.addData("Status: ", "Initialized");
        telemetry.update();


    }

    @Override
    public void loop() {

        move();

        if (gamepad1.x) {
            robot.armJoint.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            telemetry.addData("Position: ", "reset");
            telemetry.addData("armJoint position: ", robot.armJoint.getCurrentPosition());
            telemetry.update();
        }

        if (gamepad1.y) {
            int currentPosition = robot.armJoint.getCurrentPosition();

            robot.armJoint.setTargetPosition(currentPosition + 1);

        } else if (gamepad1.a) {

            int currentPosition = robot.armJoint.getCurrentPosition();

            robot.armJoint.setTargetPosition(currentPosition - 1);
        }


        telemetry.addData("armJoint position: ", robot.armJoint.getCurrentPosition());
        telemetry.update();




        robot.carousel.setPower(gamepad2.right_stick_x);

        boolean limitFrontState = robot.limitFront.getState();
        boolean limitRearState = robot.limitRear.getState();
        boolean grabberTouchState = !robot.grabberTouch.getState();



        telemetry.addData("LimitFront: ", limitFrontState);
        telemetry.addData("LimitRear: ", limitRearState);
        telemetry.addData("grabberTouch:", grabberTouchState);
        telemetry.update();




    }

    public void move() {


        robot.leftRear.setPower(-gamepad1.left_stick_y);
        robot.leftFront.setPower(-gamepad1.left_stick_y);
        robot.rightFront.setPower(-gamepad1.left_stick_y);
        robot.rightRear.setPower(-gamepad1.left_stick_y);

    }
}