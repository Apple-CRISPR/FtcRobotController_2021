package org.firstinspires.ftc.teamcode.Autonomous.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AcRobot;

@Autonomous (name = "redWarehouseParkDoNothing", group = "Autonomous")
@Disabled
public class redWarehouseParkDoNothing extends LinearOpMode {

    /** STARTS ON LEFT EDGE OF TILE
     * This program does nothing
     * parks in the warehouse **/

    private double MOTOR_PWR = AcRobot.autoMotorPower;

    @Override
    public void runOpMode() throws InterruptedException {

    }
}
