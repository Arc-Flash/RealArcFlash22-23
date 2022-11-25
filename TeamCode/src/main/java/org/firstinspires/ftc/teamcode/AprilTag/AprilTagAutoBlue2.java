package org.firstinspires.ftc.teamcode.AprilTag;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import static org.firstinspires.ftc.teamcode.FieldRelativeTeleop.d;
import static org.firstinspires.ftc.teamcode.FieldRelativeTeleop.f;
import static org.firstinspires.ftc.teamcode.FieldRelativeTeleop.i;
import static org.firstinspires.ftc.teamcode.FieldRelativeTeleop.p;
import static org.firstinspires.ftc.teamcode.FieldRelativeTeleop.target;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous (name = "Blue+Red2Auto")
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

    int ID_TAG_OF_INTEREST = 17; // Tag ID 17 from the 36h11 family
    int ID_TAG_OF_INTEREST_2 = 18;
    int ID_TAG_OF_INTEREST_3 = 19;
    AprilTagDetection tagOfInterest = null;
    SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);
    private DcMotorEx liftmotor1;
    private DcMotorEx liftmotor2;
    private PIDController controller;
    public static int target = 0;
    private Servo Drew;
    private Servo Claw;

    private final double ticks_in_degree = 700/180.0;


    int liftPos = liftmotor1.getCurrentPosition();
    double pid = controller.calculate(liftPos, target);
    double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

    double power = pid + ff;


    TrajectorySequence Blue2Signal1 = drivetrain.trajectorySequenceBuilder(new Pose2d(-38, 60, Math.toRadians(-90))).strafeLeft(26)
            .forward(28)
            .turn(Math.toRadians(-45))
            .forward(2)
            .back(2)
            .turn(Math.toRadians(45))
            .forward(20)
            .strafeRight(43)
            .turn(Math.toRadians(-90))
            .waitSeconds(.5)

            //space
            .addDisplacementMarker(() -> {
                Claw.setPosition(0);
            })
            .turn(Math.toRadians(-120))
            .forward(5)
            .addDisplacementMarker(1, () -> {
                target = 1000;
            })
            .addDisplacementMarker(() -> {
                Claw.setPosition(75);
            })
            .back(5)
            .addDisplacementMarker(1, () -> {
                target = 0;
            })
            .turn(Math.toRadians(120))
            .addDisplacementMarker(() -> {
                Claw.setPosition(0);
            })
            .waitSeconds(.5)


            //space
            .addDisplacementMarker(() -> {
                Claw.setPosition(0);
            })
            .turn(Math.toRadians(-120))
            .forward(5)
            .addDisplacementMarker(1, () -> {
                target = 1000;
            })
            .addDisplacementMarker(() -> {
                Claw.setPosition(75);
            })
            .back(5)
            .addDisplacementMarker(1, () -> {
                target = 0;
            })
            .turn(Math.toRadians(120))
            .addDisplacementMarker(() -> {
                Claw.setPosition(0);
            })
            .waitSeconds(.5)


            //space
            .addDisplacementMarker(() -> {
                Claw.setPosition(0);
            })
            .turn(Math.toRadians(-120))
            .forward(5)
            .addDisplacementMarker(1, () -> {
                target = 1000;
            })
            .addDisplacementMarker(() -> {
                Claw.setPosition(75);
            })
            .back(5)
            .addDisplacementMarker(1, () -> {
                target = 0;
            })
            .turn(Math.toRadians(120))
            .addDisplacementMarker(() -> {
                Claw.setPosition(0);
            })
            .waitSeconds(.5)


            //space
            .addDisplacementMarker(() -> {
                Claw.setPosition(0);
            })
            .turn(Math.toRadians(-120))
            .forward(5)
            .addDisplacementMarker(1, () -> {
                target = 1000;
            })
            .addDisplacementMarker(() -> {
                Claw.setPosition(75);
            })
            .back(5)
            .addDisplacementMarker(1, () -> {
                target = 0;
            })
            .turn(Math.toRadians(120))
            .addDisplacementMarker(() -> {
                Claw.setPosition(0);
            })
            .waitSeconds(.5)

            .build();

    Trajectory Park2 = drivetrain.trajectoryBuilder(Blue2Signal1.end()).back(20).build();
    Trajectory Park3 = drivetrain.trajectoryBuilder(Blue2Signal1.end()).back(42).build();

    @Override
    public void runOpMode() {

        controller = new PIDController(0.01, 0.01, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        liftmotor1 = hardwareMap.get(DcMotorEx.class, "lift_motor");
        liftmotor2 = hardwareMap.get(DcMotorEx.class, "liftmotor2");
        Drew = hardwareMap.get(Servo.class, "ClawAim");
        Claw = hardwareMap.get(Servo.class, "Claw");





        telemetry.addData("pos ", liftPos);
        telemetry.addData("target ", target);
        telemetry.addData("DrewPos", Drew.getPosition());
        telemetry.update();

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

            liftPos = liftmotor1.getCurrentPosition();
            pid = controller.calculate(liftPos, target);
            ff = Math.cos(Math.toRadians(target + 100 / ticks_in_degree)) * f;
            liftmotor1.setPower(pid + ff);

            liftPos = liftmotor2.getCurrentPosition();
            pid = controller.calculate(liftPos, target);
            ff = Math.cos(Math.toRadians(target + 100 / ticks_in_degree)) * f;
            liftmotor2.setPower(pid + ff);

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
                    Drew.setPosition(90);
                    drivetrain.followTrajectorySequence(Blue2Signal1);
                    drivetrain.followTrajectory(Park3);

                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    telemetry.addLine("Anikets code is cracked and this is Davi's pat on back");
                    tagToTelemetry(tagOfInterest);
                } else if (tag2Found) {
                    waitForStart();
                    Drew.setPosition(90);
                    drivetrain.followTrajectorySequence(Blue2Signal1);
                    drivetrain.followTrajectory(Park2);
                } else if (tag3Found) {
                    waitForStart();
                    Drew.setPosition(90);
                    drivetrain.followTrajectorySequence(Blue2Signal1);

                }

                telemetry.addData("pos ", liftPos);
                telemetry.addData("target ", target);
                telemetry.addData("DrewPos", Drew.getPosition());
                telemetry.update();

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
        //while (opModeIsActive()) {
            //sleep(20);
        //}
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
