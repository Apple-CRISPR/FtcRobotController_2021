package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name = "Grabber Test", group = "Linear Opmode")
public class GrabberTest extends LinearOpMode {

    /** This program does nothing
     * parks in the warehouse **/

    DigitalChannel grabberTouch;

    private CRServo grabberLeft;
    private CRServo grabberRight;

    @Override
    public void runOpMode() throws InterruptedException {
        grabberTouch = hardwareMap.get(DigitalChannel.class, "grabberTouch");
        grabberTouch.setMode(DigitalChannel.Mode.INPUT);

        grabberLeft = hardwareMap.get (CRServo.class, "left");
        grabberRight = hardwareMap.get (CRServo.class, "right");

        grabberLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {

            if(gamepad1.a) {
                ServoGrab();
            }

            if(gamepad1.b) {
                ServoUngrab();
            }

            telemetry.update();
        }
    }

    public void ServoGrab () {
        if(!grabberTouch.getState()) return;
        grabberLeft.setPower(1);
        grabberRight.setPower(1);
        while(true) {
            if(!grabberTouch.getState()) {
                break;
            }
        }
        grabberLeft.setPower(0);
        grabberRight.setPower(0);
    }

    public void ServoUngrab() {
        grabberLeft.setPower(-1);
        grabberRight.setPower(-1);
        sleep(1000);
        grabberLeft.setPower(0);
        grabberRight.setPower(0);
    }
}
