package org.firstinspires.ftc.teamcode.Autonomous.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.AcRobot;

@Autonomous (name = "blueWarehouseParkMain", group = "Autonomous")
public class blueWarehouseParkMain extends LinearOpMode {

    /** This program goes and spins the carousel
     * Parks in the storage unit */

    private final double MOTOR_PWR = 0.4;

    @Override
    public void runOpMode() throws InterruptedException {


        AcRobot robot = new AcRobot();
        robot.initHardware(hardwareMap);

        waitForStart();

        // Locate duck position
        sleep(10000);

        robot.strafe(DistanceUnit.CM.fromInches(22), MOTOR_PWR);
        robot.drive(DistanceUnit.CM.fromInches(12), MOTOR_PWR);

        // place item on shipping hub
        sleep(6000);

        robot.rotate(-90, MOTOR_PWR);
        robot.strafe(DistanceUnit.CM.fromInches(5), MOTOR_PWR);

        robot.drive(DistanceUnit.CM.fromInches(64), 1);
    }
}