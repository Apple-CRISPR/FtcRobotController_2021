package org.firstinspires.ftc.teamcode.Autonomous.Blue;

import android.view.Display;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.AcRobot;
import org.tensorflow.lite.task.vision.detector.Detection;

import java.util.List;

@Autonomous (name = "blueStorageParkMain", group = "Autonomous")
public class blueStorageParkMain extends LinearOpMode {

    /** STARTS ON RIGHT EDGE OF TILE
     * This program detects the position of the duck
     * places the pre-loaded box on the corresponding shipping hub level
     * Spins the carousel
     * Parks in the storage unit */

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
        if(opModeIsActive()) {

            sleep(300);
            /** Locate duck position **/
            int hubLevel = locateDuck();

            sleep(200);
            telemetry.addData("Hub Level: ", hubLevel);
            telemetry.update();


            /** Move arm to position */
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

            /** Move to shipping hub, */
            robot.strafe(DistanceUnit.CM.fromInches(-18.5), MOTOR_PWR);
            robot.drive(DistanceUnit.CM.fromInches(15), MOTOR_PWR);

             /** place block on shipping hub */
            time = 0;
            done = false;
            while(!done) {
                robot.update();
                time++;
                robot.release();
                if(time>100){
                    done = true;
                }
            }

            /** drive to and spin carousel */
            robot.drive(DistanceUnit.CM.fromInches(-15), MOTOR_PWR);
            robot.strafe(DistanceUnit.CM.fromInches(39), MOTOR_PWR);

            robot.carousel.setPower(0.75);
            sleep(2200);
            robot.carousel.setPower(1);
            sleep(1000);
            robot.carousel.setPower(0);

            /** Park */
            robot.drive(DistanceUnit.CM.fromInches(22), MOTOR_PWR);
            robot.strafe(DistanceUnit.CM.fromInches(4), MOTOR_PWR);

        }
    }

    // Initialize the Vuforia localization engine.
    private void initVuforia() {

        //Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = AcRobot.VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "webcam");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    // Initialize the TensorFlow Object Detection engine.
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

        // step through the list of recognitions and find center position info.
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
