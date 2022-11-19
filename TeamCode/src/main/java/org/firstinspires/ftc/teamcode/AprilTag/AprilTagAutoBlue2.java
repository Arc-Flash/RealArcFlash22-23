package org.firstinspires.ftc.teamcode.AprilTag;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class AprilTagAutoBlue2 extends LinearOpMode {
    static final double FEET_PER_METER = 3.28084;
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // Need to Calibrate for the Logitech C270
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int ID_TAG_OF_INTEREST = 18; // Tag ID 18 from the 36h11 family
    int ID_TAG_OF_INTEREST_2 = 19;
    int ID_TAG_OF_INTEREST_3 = 20;
    AprilTagDetection tagOfInterest = null;
    SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);


    TrajectorySequence Blue2Signal1 = drivetrain.trajectorySequenceBuilder(new Pose2d(-38, 60, Math.toRadians(-90))).strafeRight(26)
            .strafeRight(-26)
            .forward(48)
            .strafeRight(43)
            .turn(Math.toRadians(-90))
            .waitSeconds(.5)

            .turn(Math.toRadians(60))
            .back(5)
            .waitSeconds(.5)
            .forward(5)
            .turn(Math.toRadians(-60))
            .waitSeconds(.5)

            .turn(Math.toRadians(60))
            .back(5)
            .waitSeconds(.5)
            .forward(5)
            .turn(Math.toRadians(-60))
            .waitSeconds(.5)

            .turn(Math.toRadians(60))
            .back(5)
            .waitSeconds(.5)
            .forward(5)
            .turn(Math.toRadians(-60))
            .waitSeconds(.5)

            .turn(Math.toRadians(60))
            .back(5)
            .waitSeconds(.5)
            .forward(5)
            .turn(Math.toRadians(-60))
            .waitSeconds(.5)

            .turn(Math.toRadians(60))
            .back(5)
            .waitSeconds(.5)
            .forward(5)
            .turn(Math.toRadians(-60))
            .waitSeconds(.5)


            .build();

    Trajectory Park2 = drivetrain.trajectoryBuilder(Blue2Signal1.end()).back(20).build();
    Trajectory Park3 = drivetrain.trajectoryBuilder(Blue2Signal1.end()).back(42).build();

    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tag1Found = false;
                boolean tag2Found = false;
                boolean tag3Found = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == ID_TAG_OF_INTEREST) {
                        tagOfInterest = tag;
                        tag1Found = true;
                        break;
                    } else if (tag.id == ID_TAG_OF_INTEREST_2) {
                        tagOfInterest = tag;
                        tag2Found = true;
                        break;
                    } else if (tag.id == ID_TAG_OF_INTEREST_3) {
                        tagOfInterest = tag;
                        tag3Found = true;
                        break;
                    }
                }

                if (tag1Found) {
                    waitForStart();
                    drivetrain.followTrajectorySequence(Blue2Signal1);

                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else if (tag2Found) {
                    waitForStart();
                    drivetrain.followTrajectorySequence(Blue2Signal1);
                    drivetrain.followTrajectory(Park2);
                } else if (tag3Found) {
                    waitForStart();
                    drivetrain.followTrajectorySequence(Blue2Signal1);
                    drivetrain.followTrajectory(Park3);
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        if (tagOfInterest == null) {
            /*
             * Insert your autonomous code here, presumably running some default configuration
             * since the tag was never sighted during INIT
             */
        } else {
            /*
             * Insert your autonomous code here, probably using the tag pose to decide your configuration.
             */

            // e.g.
            if (tagOfInterest.pose.x <= 20) {
                // do something
            } else if (tagOfInterest.pose.x >= 20 && tagOfInterest.pose.x <= 50) {
                // do something else
            } else if (tagOfInterest.pose.x >= 50) {
                // do something else
            }
        }


        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {
            sleep(20);
        }
    }

    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}
