package org.firstinspires.ftc.teamcode.arm;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.math.Vector;

import java.sql.Array;

public class Arm {

    //segments
    public Segment[] segments;
    //base of arm
    public Vector base = new Vector();

    public Vector target = new Vector();

    public Arm(DcMotorEx[] motors, int segmentLength){

        //initialize arm segments
        initSegments(motors, segmentLength);
    }
    public void initSegments(DcMotorEx[] motors, int length){
        int num = motors.length;
        segments = new Segment[num];
        for(int i = 0; i < num; i++){
            segments[i] = new Segment(length, motors[i]);
            if(i>0){
                segments[i-1].child = segments[i];
            }
        }
    }

    public void setTarget(Vector position){
        target = position;
    }

    public void update(){

        segments[segments.length-1].follow(target);
        for(int i = segments.length-2; i >= 0; i--){
            segments[i].follow();
        }

        segments[0].setA(base);
        for(int i = 1; i < segments.length; i++){
            segments[i].setA(segments[i-1].b);
        }

        for(int i = 0; i < segments.length; i++){
            segments[i].update();
        }
    }
}

