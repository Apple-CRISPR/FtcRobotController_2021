package org.firstinspires.ftc.teamcode.experimental;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.*;

public class Recorder {
    public Recorder(){

    }
    public void Play(){
        System.out.println("test");
        try{
            // File path is passed as parameter
            File file = new File("\\res\\raw\\test.con");

            // Creating an object of BufferedReader class
            BufferedReader br = new BufferedReader(new FileReader(file));

            String st;

            while ((st = br.readLine()) != null){
                // Print the string
                System.out.println(st);
            }
        }catch (Exception e){
            System.out.println(e);
        }
    }
}
