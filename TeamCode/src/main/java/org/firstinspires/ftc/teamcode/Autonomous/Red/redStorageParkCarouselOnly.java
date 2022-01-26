package org.firstinspires.ftc.teamcode.Autonomous.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.AcRobot;

@Autonomous (name = "redStorageParkCarouselOnly", group = "Autonomous")
public class redStorageParkCarouselOnly extends LinearOpMode {

    /** STARTS ON THE LEFT EDGE OF THE TILE
     *  This program goes and spins the carousel
     * Parks in the storage unit */

    private final double MOTOR_PWR = AcRobot.autoMotorPower;

    @Override
    public void runOpMode() throws InterruptedException {


        AcRobot robot = new AcRobot();
        robot.initHardware(hardwareMap);

        waitForStart();

        if(opModeIsActive()) {
        robot.drive(DistanceUnit.CM.fromInches(3), MOTOR_PWR);
        robot.rotate(90, MOTOR_PWR);
        robot.drive(DistanceUnit.CM.fromInches(-17.5), MOTOR_PWR);
        //robot.drive(DistanceUnit.CM.fromInches(-2), 0.25);

            robot.carousel.setPower(-0.75);
            sleep(2200);
            robot.carousel.setPower(-1);
            sleep(1000);
            robot.carousel.setPower(0);

        robot.strafe(DistanceUnit.CM.fromInches(-23.2), MOTOR_PWR);
        robot.drive(DistanceUnit.CM.fromInches(-6), MOTOR_PWR);
    }


    }
}

