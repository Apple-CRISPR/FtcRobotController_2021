package org.firstinspires.ftc.teamcode.Testcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "ServoTest", group = "Test")
public class ServoTest extends OpMode {

    private CRServo grabberLeft;
    private CRServo grabberRight;

    static double GRAB = 0.35;
    static double UNGRAB = -0.35;



    @Override
    public void init() {

        grabberLeft = hardwareMap.get (CRServo.class, "left");
        grabberRight = hardwareMap.get (CRServo.class, "right");

        grabberLeft.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void loop() {

        if (gamepad2.dpad_down) {
            ServoGrab();
        } else if(gamepad2.dpad_up) {
            ServoUngrab();
        } else {
            grabberRight.setPower(0);
            grabberLeft.setPower(0);
        }

        telemetry.addData("Right servo power: ", grabberRight.getPower());
        telemetry.addData("Left Servo power: ", grabberLeft.getPower());
    }

    public void ServoGrab () {

        grabberLeft.setPower(GRAB);
        grabberRight.setPower(GRAB);


    }

    public void ServoUngrab() {

        grabberRight.setPower(UNGRAB);
        grabberLeft.setPower(UNGRAB);

    }
}
