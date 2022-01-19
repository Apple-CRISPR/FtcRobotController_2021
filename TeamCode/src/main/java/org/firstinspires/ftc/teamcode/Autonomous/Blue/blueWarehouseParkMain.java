package org.firstinspires.ftc.teamcode.Autonomous.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.AcRobot;

import java.util.List;

@Autonomous (name = "blueWarehouseParkMain", group = "Autonomous")
public class blueWarehouseParkMain extends LinearOpMode {

    /** STARTS ON THE RIGHT EDGE OF THE TILE
     * This program detects the duck
     * Places freight on shipping hub level
     * Parks in warehouse
     */


    private final double MOTOR_PWR = AcRobot.autoMotorPower;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;


    @Override
    public void runOpMode() throws InterruptedException {


        AcRobot robot = new AcRobot();
        robot.initHardware(hardwareMap);

        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0 / 9.0);
        }
        waitForStart();

        if (opModeIsActive()) {

            /** Locate duck position **/
            int hubLevel = locateDuck();



            robot.strafe(DistanceUnit.CM.fromInches(22), MOTOR_PWR);
            robot.drive(DistanceUnit.CM.fromInches(12), MOTOR_PWR);

            /** place item on shipping hub **/

            if (hubLevel == 1) {
                /** Place block on shipping hub level 1 **/

            } else if (hubLevel == 2) {
                /** place block on shipping hub level 2 **/

            } else if (hubLevel == 3) {
                /** Place block on shipping hub level 3 **/

            }

            robot.rotate(-90, MOTOR_PWR);
            robot.strafe(DistanceUnit.CM.fromInches(5), MOTOR_PWR);

            robot.drive(DistanceUnit.CM.fromInches(64), 1);
        }

    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = AcRobot.VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "webcam");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.6f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(AcRobot.TFOD_MODEL_ASSET, AcRobot.LABELS);
    }


    /* Method for locating duck, returns the hub level
    * Blue 2 center: 70-90
    * Blue 3 center: 370-390
     */
    private int locateDuck() {

        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

        int theHubLevel = 1;

        // step through the list of recognitions and display boundary info.
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