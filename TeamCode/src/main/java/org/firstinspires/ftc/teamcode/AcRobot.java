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
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.math.Constants;
import org.firstinspires.ftc.teamcode.math.Vector;

import java.util.List;

public class AcRobot {

    /** autonomous movement constants **/
    public static final double encoderResolution = 537.7; // Ticks per revolution
    public static final double wheelDiameter = 96; // mm
    public static final double rotationDistance = 238; // cm
    public static final double strafeModifier = 1.125;
    public static final double mmPerTick = (Math.PI*wheelDiameter)/encoderResolution;
    public static final double autoMotorPower = 0.4;

    //Possible states of the grabber: Grabbing, Releasing, Idle
    public enum grabberStates {
        GRABBING, IDLE, RELEASING;
    }

    //Grabber variables
    public grabberStates grabberMode = grabberStates.IDLE;
    public int grabTime;

    // movement
    public DcMotorEx leftFront = null;
    public DcMotorEx rightFront = null;
    public DcMotorEx leftRear = null;
    public DcMotorEx rightRear = null;

    //arm motors
    public DcMotorEx armJoint = null;
    public DcMotorEx armBase = null;

    // DcMotor (Carousel spinner)
    public DcMotor carousel = null;

    //Touch Sensor for the grabber
    public DigitalChannel grabberTouch = null;

    //Limit switches (For the lower segment)
    public DigitalChannel limitFront = null;
    public DigitalChannel limitRear = null;

    //Limit switches (For the upper segment
    public DigitalChannel upperArmFront = null;
    public DigitalChannel upperArmRear = null;

    // servos
    public CRServo grabberRight = null;
    public CRServo grabberLeft = null;

    public Arm arm = null;

    // shiping hub levels
    private double levels[] = {217, 327.65, 482.75};

    //offsets
    public double baseOffset = 130;
    public double jointOffset = 150;

    public boolean slowMode = false;


    public static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    public static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };


    public static final String VUFORIA_KEY = "ASqpJr7/////AAABmUVwKJGyqUYXglgkc+gVFKaMDJvVe1kCbfQEOluUHsrX0uw34sWuJRkDlw6hPRpC4eu08HxrIDCThmAlBj8A"
            + "/Mjvlve5ieeCVQ6yPoz01voa9FUrsR4pfYrM9n6CtqC2a8DXN0nFfFR0maREQO0csOige5xAxVWPpg3RUEUt9Ncs/7EQ8FG"
            + "50IFy7GqykqK2C3r73em1a2w9rsCwYHghJN5/dR44OEd6GWVQIRErDeXvuSuhVLJFjnvaHJhm3QG6rOH+uE+8/YI+imlImad21HyBdTb53q6E0IWpv"
            + "OVfC1AtX9MFgJIn6diRyp1Q0ULp7K6KHyzRlD/5b+bgKesqw1yFyb/jdSkJMiojDEWt4a8F";


    public AcRobot(){
    }

    //run this before anything else
    public void initHardware(HardwareMap hardwareMap){

        //initialize drive motors
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");

        //Arm motors
        armJoint = hardwareMap.get(DcMotorEx.class, "armJoint");
        armBase = hardwareMap.get(DcMotorEx.class, "armBase");

        carousel = hardwareMap.get(DcMotor.class, "carousel");

        //set two of the motors to be reversed
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        //Servos
        grabberLeft = hardwareMap.get(CRServo.class, "left");
        grabberRight = hardwareMap.get(CRServo.class, "right");

        grabberLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //Sensors
        limitFront = hardwareMap.get(DigitalChannel.class, "armLimitFront");
        limitFront.setMode(DigitalChannel.Mode.INPUT);
        limitRear = hardwareMap.get(DigitalChannel.class, "armLimitRear");
        limitRear.setMode(DigitalChannel.Mode.INPUT);
        upperArmFront = hardwareMap.get(DigitalChannel.class, "upperArmFront");
        upperArmFront.setMode(DigitalChannel.Mode.INPUT);
        upperArmRear = hardwareMap.get(DigitalChannel.class, "upperArmRear");
        upperArmRear.setMode(DigitalChannel.Mode.INPUT);

        carousel = hardwareMap.get(DcMotor.class, "carousel");

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
        arm = new Arm(armMotors, 336); //336 in milimeters
        arm.segments[0].length = 312;

        // set arm segment starting positions
        initAngleSoItNoBreak();
    }

    public void moveArmToLevel(int level){
        setArmPosition(new Vector(300, levels[level-1]));
    }
    public void moveToPickUpBlock(){
        setArmPosition(new Vector(450, 225));
    }
    public void initAngleSoItNoBreak(){
        double x = -660;
        double y = 364;
        setArmPosition(new Vector(x,y));
        arm.update();
        x = 50;
        y = 336;
        setArmPosition(new Vector(x,y));
        for(int i = 0; i < 100; i++){
            arm.update();
        }
    }
    public void recalibrateArm(){
        armBase.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armJoint.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armJoint.setPower(-0.25);
        armBase.setPower(-0.25);
        System.out.println(limitRear.getState());
        while(!(limitRear.getState()&&upperArmFront.getState())){
            armBase.setPower(-0.25);
            armJoint.setPower(-0.25);
            if(limitRear.getState()) {
                armBase.setPower(0);
            }
            if(upperArmFront.getState()) {
                armJoint.setPower(0);
            }
        }
        armJoint.setPower(0);
        armBase.setPower(0);
        armBase.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armJoint.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        moveArmToLevel(1);
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
        // update arm position
        arm.update();

        // update grabber
        updateGrabber();

        // move arm motors
        setArmAngle(arm.segments[0].setAngle, -arm.segments[1].setAngle);
    }

    public void updateGrabber(){
        if(grabberMode.equals(grabberStates.GRABBING)) {
            if (grabberTouch.getState()) {
                grabberLeft.setPower(-1);
                grabberRight.setPower(-1);
            }else{
                grabberLeft.setPower(0);
                grabberRight.setPower(0);
                grabberMode = grabberStates.IDLE;
            }
        }else if(grabberMode.equals(grabberStates.RELEASING)){
            grabberLeft.setPower(1);
            grabberRight.setPower(1);
            grabTime =+ 4;
        }else{
            grabberLeft.setPower(0);
            grabberRight.setPower(0);
        }
        if(grabTime>100){
            grabTime = 0;
            grabberMode = grabberStates.IDLE;
        }
    }

    public void grab(){
        grabberMode = grabberStates.GRABBING;
    }
    public void release(){
        grabberMode = grabberStates.RELEASING;
    }

    public void setArmAngle(double lowerAngle, double upperAngle){
        double basePos = ToTicks(lowerAngle, Constants.motor5202TPR);
        double jointPos = ToTicks(upperAngle, Constants.motor5202TPR);

        double baseOffset = ToTicks(122, Constants.motor5202TPR);
        double jointOffset = ToTicks(148, Constants.motor5202TPR);

        basePos-=baseOffset;
        jointPos-=jointOffset;

        //joint limit switch code
        double currentJointPos = armJoint.getCurrentPosition()/(-1);
        double jointVel = (jointPos)-(currentJointPos);
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
        System.out.println(vel);

        armBase.setTargetPosition((int)(basePos*(-1.5)));
        armJoint.setTargetPosition((int)jointPos*(-1));

        armBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armBase.setPower(0.7);
        armJoint.setPower(1);
    }
    public double radToDeg(double radians)
    {
        double pi = Math.PI;
        return radians * (180/pi);
    }

    /*
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
    */

    // x and y are a vector direction
    // rot will rotate the robot
    public void Drive(double x, double y, double rot){
        double r = Math.hypot(x, y);
        double robotAngle = Math.atan2(y, -x) - Math.PI / 4;
        double rightX = -rot;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        if(slowMode){
            leftFront.setPower(v1*0.5);
            rightFront.setPower(v2*0.5);
            leftRear.setPower(v3*0.5);
            rightRear.setPower(v4*0.5);
        }else{
            leftFront.setPower(v1);
            rightFront.setPower(v2);
            leftRear.setPower(v3);
            rightRear.setPower(v4);
        }


    }

    /** Drive method
     * @param cm    Distance the robot will travel in cm
     * positive values - Move forward
     * negative values - Move backward
     * @param power    Power given to motors (Robot will move the same distance no matter the motor power) */
    public void drive(double cm, double power) {

        double ticks = cmToTick(cm);

        moveMotor(leftFront, (int)ticks, power);
        moveMotor(rightFront, (int)ticks, power);
        moveMotor(leftRear, (int)ticks, power);
        moveMotor(rightRear, (int)ticks, power);
        while(leftFront.isBusy() ||  rightFront.isBusy() || leftRear.isBusy() || rightRear.isBusy()) {}
    }

    /** strafe method
     * @param cm    Distance the robot will strafe
     * Positive values - Strafe right
     * Negative values - Strafe left
     * @param power    power given to the motors (robot will strafe the same distance no matter the power) **/
    public void strafe(double cm, double power) {

        double ticks = -cmToTick(cm)*strafeModifier;

        moveMotor(leftFront, (int)-ticks, power);
        moveMotor(rightFront, (int)ticks, power);
        moveMotor(leftRear, (int)ticks, power);
        moveMotor(rightRear, (int)-ticks, power);
        while(leftFront.isBusy() ||  rightFront.isBusy() || leftRear.isBusy() || rightRear.isBusy()) {}
    }


    /** rotate method
     * @param deg   The number of degrees the robot will rotate
     * Positive values - rotate clockwise
     * Negative vlues - rotate counterclockwise
     * @param power    power given to motors(the robot will always rotate the same amount, no matter what the power) */
    public void rotate(double deg, double power) {

        double ticks = -cmToTick(rotationDistance/360*deg);

        moveMotor(leftFront, (int)-ticks, power);
        moveMotor(rightFront, (int)ticks, power);
        moveMotor(leftRear, (int)-ticks, power);
        moveMotor(rightRear, (int)ticks, power);
        while(leftFront.isBusy() ||  rightFront.isBusy() || leftRear.isBusy() || rightRear.isBusy()) {}
    }

    // Used inside of autonomous drive methods
    // This calculates the target position for the encoder
    private void moveMotor(DcMotor motor, int ticks, double power) {

        int postion = motor.getCurrentPosition();

        motor.setTargetPosition(postion - ticks);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);
    }

    // unit conversions
    public double inToCm(double in) { return in*2.54; }
    public double ftToCm(double ft) { return ft*12*2.54; }
    double cmToTick(double cm) { return cm*10/mmPerTick; }
}
