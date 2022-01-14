package org.firstinspires.ftc.teamcode.Autonomous.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AcRobot;

@Autonomous (name = "blueStorageParkCarouselOnly", group = "Autonomous")
public class blueStorageParkCarouselOnly extends LinearOpMode {

    /** This program goes and spins the carousel
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

