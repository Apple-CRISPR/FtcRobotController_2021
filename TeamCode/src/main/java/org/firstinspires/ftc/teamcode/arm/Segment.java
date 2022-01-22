package org.firstinspires.ftc.teamcode.arm;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.math.Vector;

public class Segment {
    public Vector a = new Vector();
    public Vector b = new Vector();
    public double length;
    public double angle = 0;
    public double showAngle = 0;
    public Segment child;
    public DcMotorEx motor;
    public double setAngle = 0;

    public Segment(double length, DcMotorEx outputMotor){

        //set length
        this.length = length;

        motor = outputMotor;

    }
    public void follow(Vector target){
        if(Math.round(target.x) == 0||Math.round(target.y) == 0){
            return;
        }
        Vector dir = Vector.sub(target, a);
        angle = dir.heading();

        dir.setMagnitude(length);
        dir.mult(-1);
        a = Vector.add(target, dir);
    }
    public void follow(){
        Vector target = child.a;
        follow(target);
    }

    public void calculateB() {
        double dx = length * Math.cos(angle);
        double dy = length * Math.sin(angle);
        b.set(a.x + dx, a.y + dy);
    }

    public void setA(Vector vector){  
        a = vector;
        calculateB();
    }
    public void update(){
        calculateB();

        if(child != null){
            child.showAngle = child.angle-angle;
        }

        //double angle = Math.round(radToDeg(showAngle));
        setAngle = radToDeg(showAngle);
        //System.out.println(setAngle);
        showAngle = angle;
    }
    public double radToDeg(double radians)
    {
        double pi = Math.PI;
        return radians * (180/pi);
    }
}
