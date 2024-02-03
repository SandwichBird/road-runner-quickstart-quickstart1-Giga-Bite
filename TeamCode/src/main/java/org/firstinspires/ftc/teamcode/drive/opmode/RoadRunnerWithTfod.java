//import libraries
package org.firstinspires.ftc.teamcode.drive.opmode;

//import roadrunner library
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

//import vision library
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;


/*
 This OpMode detects the position of a the Blue Team Prop
 Then moves to drop a pixel on the corresponding spot
 Then Parks in the Side
 Positions are set for blue side
 */
@Autonomous(name = "RoadRunnerWithTfod", group = "Linear OpMode")
//@Disabled
public class RoadRunnerWithTfod extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "BlueTeamProp.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    // private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/myCustomModel.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "Blue Team Prop",
    };

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    boolean isProp = false;

    // position of prop in frame
    double propLocation = 0;

    // prop is left, right, or center
    double propPose = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d startPose = new Pose2d(-60, 13, 0);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Servo pixelDropServo = hardwareMap.get(Servo.class, "pixelDropServo");

        drive.setPoseEstimate(startPose);

        Trajectory scoringPositionTraj = drive.trajectoryBuilder(startPose)
                .forward(27)
                .build();

        Trajectory spikeRightTraj = drive.trajectoryBuilder(scoringPositionTraj.end())
                .splineToConstantHeading(new Vector2d(-35, 0.5), 0)
                .build();

        Trajectory spikeLeftTraj = drive.trajectoryBuilder(scoringPositionTraj.end())
                .splineToConstantHeading(new Vector2d(-35, 22.6), 0)
                .build();

        Trajectory spikeCenterTraj = drive.trajectoryBuilder(scoringPositionTraj.end())
                .splineToConstantHeading(new Vector2d(-25, 12), 0)
                .build();

        Trajectory preparkFromCenterTraj = drive.trajectoryBuilder(spikeCenterTraj.end())
                .splineToConstantHeading(new Vector2d(-55, 40), 0)
                .build();

        Trajectory preparkFromLeftTraj = drive.trajectoryBuilder(spikeLeftTraj.end())
                .splineToConstantHeading(new Vector2d(-55, 40), 0)
                .build();

        Trajectory preparkFromRightTraj = drive.trajectoryBuilder(spikeRightTraj.end())
                .splineToConstantHeading(new Vector2d(-55, 40), 0)
                .build();

        Trajectory finalPark = drive.trajectoryBuilder(new Pose2d(-55, 40))
                .splineToConstantHeading(new Vector2d(-55, 57), 0)
                .build();

        initTfod();

        waitForStart();
        if (isStopRequested())
            return;

        resetRuntime();
        while(getRuntime() < 5 && !isProp){
            telemetryTfod();

        }

        drive.followTrajectory(scoringPositionTraj);

        tfod.shutdown();

        if (isProp) {

            //sets propPose depending on position of prop in camera
            // 1 = left, 2 = center, 3 = right
            if (propLocation <= 213) {
                propPose = 1;

            } else if (propLocation <= 426) {
                propPose = 2;

            } else if (propLocation > 426) {
                propPose = 3;

            }

            if (propPose == 1) {
                drive.followTrajectory(spikeLeftTraj);
                sleep(500);
                pixelDropServo.setPosition(0);
                sleep(500);
                drive.followTrajectory(preparkFromLeftTraj);

            } else if (propPose == 2) {
                drive.followTrajectory(spikeCenterTraj);
                sleep(500);
                pixelDropServo.setPosition(0);
                sleep(500);
                drive.followTrajectory(preparkFromCenterTraj);

            } else if (propPose == 3) {
                drive.followTrajectory(spikeRightTraj);
                sleep(500);
                pixelDropServo.setPosition(0);
                sleep(500);
                drive.followTrajectory(preparkFromRightTraj);
            }
        }

        drive.followTrajectory(finalPark);

        requestOpModeStop();

    } // end runOpMode()

    /**
     * Initialize the TensorFlow Object Detection processor.
     */

    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }


        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();


    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;

            telemetry.addData("", " ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());

            if (recognition.getLabel().equals("Blue Team Prop")) {
                isProp = true;
                propLocation = (recognition.getLeft() + recognition.getRight()) / 2;
            } // end if(recognitions = "Blue Team Prop")
        }   // end for() loop

    }   // end method telemetryTfod()

} // end linear opMode