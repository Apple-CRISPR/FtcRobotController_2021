package org.firstinspires.ftc.teamcode.Autonomous.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AcRobot;

@Autonomous (name = "blueStorageParkMain", group = "Autonomous")
public class blueStorageParkMain extends LinearOpMode {

    /** This program detects the position of the duck
     * places the pre-loaded box on the corresponding shipping hub level
     * Spins the carousel
     * Parks in the storage unit */



    @Override
    public void runOpMode() throws InterruptedException {

        AcRobot robot = new AcRobot();
        robot.initHardware(hardwareMap);


        waitForStart();
        while(opModeIsActive()) {


        }
    }
}
