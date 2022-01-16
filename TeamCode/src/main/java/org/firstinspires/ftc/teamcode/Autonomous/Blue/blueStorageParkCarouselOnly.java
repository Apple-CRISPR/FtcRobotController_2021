package org.firstinspires.ftc.teamcode.Autonomous.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.AcRobot;

@Autonomous (name = "blueStorageParkCarouselOnly", group = "Autonomous")
public class blueStorageParkCarouselOnly extends LinearOpMode {

    /** This program goes and spins the carousel
     * Parks in the storage unit */

    private final double MOTOR_PWR = AcRobot.autoMotorPower;

    @Override
    public void runOpMode() throws InterruptedException {


        AcRobot robot = new AcRobot();
        robot.initHardware(hardwareMap);

        waitForStart();

        robot.drive(DistanceUnit.CM.fromInches(5), MOTOR_PWR);
        robot.strafe(DistanceUnit.CM.fromInches(20), MOTOR_PWR);

        robot.carousel.setPower(0.75);
        sleep(2200);
        robot.carousel.setPower(1);
        sleep(1000);
        robot.carousel.setPower(0);

        robot.drive(DistanceUnit.CM.fromInches(22.2), MOTOR_PWR);
        robot.strafe(DistanceUnit.CM.fromInches(4), MOTOR_PWR);
    }
}

