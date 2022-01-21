package org.firstinspires.ftc.teamcode;
import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.arm.Arm;
import org.firstinspires.ftc.teamcode.math.Constants;
import org.firstinspires.ftc.teamcode.math.Vector;

public class AcRobot {
    // movement
    public DcMotorEx leftFront = null;
    public DcMotorEx rightFront = null;
    public DcMotorEx leftRear = null;
    public DcMotorEx rightRear = null;

    // DcMotor
    public DcMotor carousel;

    public DigitalChannel grabberTouch = null;
    public DigitalChannel limitFront = null;
    public DigitalChannel limitRear = null;
    public DigitalChannel upperArmFront = null;
    public DigitalChannel upperArmRear = null;

    /* servos */
    public CRServo grabberRight = null;
    public CRServo grabberLeft = null;

    public DcMotorEx armBase = null;
    public DcMotorEx armJoint = null;

    public Arm arm = null;

    //offsets
    public double baseOffset = 130;
    public double jointOffset = 150;

    public AcRobot(){
    }

    //run this before anything else
    public void initHardware(HardwareMap hardwareMap){

        //initialize drive motors
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");

        //set two of the motors to be reversed
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        //Servos
        grabberLeft = hardwareMap.get(CRServo.class, "left");
        grabberRight = hardwareMap.get(CRServo.class, "right");

        //Sensors
        limitFront = hardwareMap.get(DigitalChannel.class, "armLimitFront");
        limitFront.setMode(DigitalChannel.Mode.INPUT);
        limitRear = hardwareMap.get(DigitalChannel.class, "armLimitRear");
        limitRear.setMode(DigitalChannel.Mode.INPUT);
        upperArmFront = hardwareMap.get(DigitalChannel.class, "upperArmFront");
        upperArmFront.setMode(DigitalChannel.Mode.INPUT);
        upperArmRear = hardwareMap.get(DigitalChannel.class, "upperArmRear");
        upperArmRear.setMode(DigitalChannel.Mode.INPUT);

        grabberTouch = hardwareMap.get(DigitalChannel.class, "grabberTouch");
        grabberTouch.setMode(DigitalChannel.Mode.INPUT);

        leftFront.setMode(  DcMotor.RunMode.RUN_USING_ENCODER );
        rightFront.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
        leftRear.setMode(   DcMotor.RunMode.RUN_USING_ENCODER );
        rightRear.setMode(  DcMotor.RunMode.RUN_USING_ENCODER );

        armBase = hardwareMap.get(DcMotorEx.class, "armBase");
        armJoint = hardwareMap.get(DcMotorEx.class, "armJoint");

        // set modes for arm motors
        armBase.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armJoint.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //initialize arm
        DcMotorEx armMotors[] = {armBase, armJoint};
        arm = new Arm(armMotors, 336); //in milimeters

        // set arm segment starting positions
    }

    public void testJoint(double ticks){
        armJoint.setTargetPosition((int)ticks);
        armJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armJoint.setPower(1);
    }

    // converts angle to ticks for motor
    public double ToTicks(double angle, double motorTPR){
        return (angle/360)*motorTPR;
    }
    public void setArmPosition(Vector position){
        arm.setTarget(position);
    }
    public void update(){
        arm.update();

        setArmAngle(radToDeg(arm.segments[0].showAngle), radToDeg(arm.segments[1].showAngle) );
        System.out.println("base: "+radToDeg(arm.segments[0].showAngle)+" joint: "+radToDeg(arm.segments[1].showAngle));
    }
    public void setArmAngle(double lowerAngle, double upperAngle){
        double basePos = ToTicks(lowerAngle, Constants.motor5202TPR);
        double jointPos = ToTicks(upperAngle, Constants.motor5202TPR);

        double baseOffset = ToTicks(130, Constants.motor5202TPR);
        double jointOffset = ToTicks(150, Constants.motor5202TPR);

        basePos-=baseOffset;
        jointPos-=jointOffset;

        //joint limit switch code
        double currentJointPos = armJoint.getCurrentPosition()/(-1);
        double jointVel = jointPos-currentJointPos;
        if(upperArmFront.getState() && jointVel > 0){
            jointPos = currentJointPos;
        }
        if(upperArmRear.getState() && jointVel <= 0) {
            jointPos = currentJointPos;
        }
        double currentPos = armBase.getCurrentPosition()/(-1.5);
        double vel = basePos-currentPos;

        if(limitFront.getState() && vel <= 0){
            basePos = currentPos;
        }
        if(limitRear.getState() && vel > 0){
            basePos = currentPos;
        }

        armBase.setTargetPosition((int)(basePos*(-1.5)));
        armJoint.setTargetPosition((int)jointPos*(-1));

        armBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armBase.setPower(0.25);
        armJoint.setPower(1);
    }
    public double radToDeg(double radians)
    {
        double pi = Math.PI;
        return radians * (180/pi);
    }

    public void DriveTo(Vector position, double duration){
        DriveWithVelocity(Vector.normalize(position), 0, (int)(position.magnitude()/duration));
        sleep((long)duration*1000);
        stop();
    }

    public void DriveWithVelocity(Vector direction, double rot, int speed){
        double TPR = Constants.motor5203TPR;
        double r = Math.hypot(direction.x, direction.y);
        double robotAngle = Math.atan2(direction.y, -direction.x) - Math.PI / 4;
        double rightX = -rot;
        final double lf = r * Math.cos(robotAngle) + rightX;
        final double rl = r * Math.sin(robotAngle) - rightX;
        final double lr = r * Math.sin(robotAngle) + rightX;
        final double rr = r * Math.cos(robotAngle) - rightX;

        leftFront.setVelocity(lf*((double)speed/360)*TPR);
        rightFront.setVelocity(rl*((double)speed/360)*TPR);
        leftRear.setVelocity(lr*((double)speed/360)*TPR);
        rightRear.setVelocity(rr*((double)speed/360)*TPR);
    }
    public void stop(){
        leftFront.setVelocity(0);
        rightFront.setVelocity(0);
        leftRear.setVelocity(0);
        rightRear.setVelocity(0);
    }

    // x and y are a vector direction
    public void DRIVE(double x, double y, double rot){
        double r = Math.hypot(x, y);
        double robotAngle = Math.atan2(y, -x) - Math.PI / 4;
        double rightX = -rot;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        leftFront.setPower(v1);
        rightFront.setPower(v2);
        leftRear.setPower(v3);
        rightRear.setPower(v4);

    }
}
