package org.firstinspires.ftc.teamcode.Autonomous.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AcRobot;

@Autonomous (name = "redStorageParkDoNothing", group = "Autonomous")
@Disabled
public class redStorageParkDoNothing extends LinearOpMode {


    private double MOTOR_PWR = AcRobot.autoMotorPower;

    @Override
    public void runOpMode() throws InterruptedException {

        AcRobot robot = new AcRobot();
        robot.initHardware(hardwareMap);


        waitForStart();
        if(opModeIsActive()) {


        }
    }
}
