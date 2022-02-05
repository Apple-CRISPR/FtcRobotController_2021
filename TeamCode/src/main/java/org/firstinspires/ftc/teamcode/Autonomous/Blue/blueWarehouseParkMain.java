package org.firstinspires.ftc.teamcode.Autonomous.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.AcRobot;
import org.tensorflow.lite.task.vision.detector.Detection;

import java.util.List;

@Autonomous (name = "blueWarehouseParkMain", group = "Autonomous")
public class blueWarehouseParkMain extends LinearOpMode {

    /** STARTS ON THE RIGHT EDGE OF THE TILE
     * This program detects the duck
     * Places freight on shipping hub level
     * Parks in warehouse */


    private final double MOTOR_PWR = AcRobot.autoMotorPower;

    /** {@link #vuforia} is the variable we will use to store our instance of the Vuforia localization engine. */
    private VuforiaLocalizer vuforia;

    /** {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object Detection engine. */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() throws InterruptedException {

        AcRobot robot = new AcRobot();

        //Initialize
        robot.initHardware(hardwareMap);
        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.0, 16.0 / 9.0);
        }

        //When initialization of Tfod is finished
        telemetry.addData("Status: ", "initialized");
        telemetry.update();

        waitForStart();
        if (opModeIsActive()) {

            sleep(300);
            /** Locate duck position **/
            int hubLevel = locateDuck();

            sleep(200);
            telemetry.addData("Hub Level: ", hubLevel);
            telemetry.update();


            /** Move arm to correct level **/
            robot.moveArmToLevel(hubLevel);
            int time = 0;
            boolean done = false;

            while(!done){
                robot.update();
                time++;
                if(time > 150){
                    done = true;
                    time = 0;
                }
            }

            /** Drive to shipping hub **/
            robot.strafe(DistanceUnit.CM.fromInches(22), MOTOR_PWR);
            robot.drive(DistanceUnit.CM.fromInches(15), MOTOR_PWR);

            /** place item on shipping hub **/
            time = 0;
            done = false;
            while(!done) {

                robot.release();
                robot.update();
                time++;

                if(time>100){
                    done = true;
                }
            }

            //turn off grabber
            robot.grabberMode = AcRobot.grabberStates.IDLE;
            robot.moveArmToLevel(1);
            robot.update();

            //recalibrate arm
            //robot.recalibrateArm();

            /** Drive into warehouse **/
            robot.rotate(-90, MOTOR_PWR);
            robot.drive(DistanceUnit.CM.fromInches(64), 1);
        }

    }

    //Initialize the Vuforia localization engine.
    private void initVuforia() {
        //Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = AcRobot.VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "webcam");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    // Initialize TensorFlow Object Detection engine.
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.4f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(AcRobot.TFOD_MODEL_ASSET, AcRobot.LABELS);
    }


    /* Method for locating duck, returns the hub level
    * Blue 2 center: 70-90
    * Blue 3 center: 370-390 */
    private int locateDuck() {

        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        int theHubLevel = 1;

        // step through the list of recognitions and get boundary info.
        for (Recognition recognition : updatedRecognitions) {
            if (recognition.getLabel().equals("Duck")) {
                double RECOGNITION_CENTER = (recognition.getLeft() + recognition.getRight()) / 2;

                if (RECOGNITION_CENTER < 220) {
                    theHubLevel = 2;
                } else if (RECOGNITION_CENTER > 220) {
                    theHubLevel = 3;
                }

                telemetry.addLine("Duck Detected.");
                telemetry.update();
            }
        }

        return theHubLevel;
    }
}