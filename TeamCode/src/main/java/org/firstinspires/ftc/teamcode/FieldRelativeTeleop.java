package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp(name = "Field Relative")
public class FieldRelativeTeleop extends LinearOpMode {

    public static double p = 0.005, i = 0.01, d = 0;
    public static double f = 0;
    public static int target;
    private final double ticks_in_degree = 700 / 180.0;
    double speedModifier = 0.8; //@TODO If your drivers complain that the robot is too fast fix this :)
    double robotAngle = 0; //For Field Relative
    double angleZeroValue = org.firstinspires.ftc.teamcode.StaticField.autonHeading;  //gets value from auton. if auton fails for some reason the
    private DcMotorEx frontLeft;
    private DcMotorEx backLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backRight;

    private DcMotorEx liftmotor1;
    private DcMotorEx liftmotor2;
    private PIDController controller;
    private BNO055IMU imu;
    private Servo Drew;
    private Servo Claw;
    //import static org.firstinspires.ftc.teamcode.FieldRelativeTeleop.TIP_AUTHORITY;
    //import static org.firstinspires.ftc.teamcode.FieldRelativeTeleop.TIP_TOLERANCE;

    //default angle is 0 degrees. Remember that your drivers should recalibrate if this happens by facing
    //the front of the robot away from them (i.e. the front of the robot is facing your opponents) and then press x.
    @Override


    //public static double TIP_TOLERANCE = Math.toRadians(10);

    //public static double TIP_AUTHORITY = 0.9;
    public void runOpMode() {
        telemetry.addLine("Davi's code suprisingly worked");
        telemetry.update();
        PhotonCore.enable();
        //@TODO Check hardware mappings
        frontLeft = hardwareMap.get(DcMotorEx.class, "leftFront");
        backLeft = hardwareMap.get(DcMotorEx.class, "leftRear");
        frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");
        backRight = hardwareMap.get(DcMotorEx.class, "rightRear");
        Drew = hardwareMap.get(Servo.class, "ClawAim");
        Claw = hardwareMap.get(Servo.class, "Claw");
//

        //PID stuff follows
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        liftmotor1 = hardwareMap.get(DcMotorEx.class, "lift_motor");
        liftmotor2 = hardwareMap.get(DcMotorEx.class, "liftmotor2");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        initIMU();
        //since this is mecanum
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        liftmotor1.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            drivetrain();
        }


    }


    public void initIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);


    }

    //public void setSafeDrivePower(Pose2d raw){
    //Orientation i = imu.getAngularOrientation();
    //float x = 0, y = 0, adjX = xOffset-i.secondAngle, adjY = i.thirdAngle-yOffset;
    //if(Math.abs(adjY) > TIP_TOLERANCE) y = adjY;
    //if(Math.abs(adjX) > TIP_TOLERANCE) x = adjX;
//        //setWeightedDrivePower(raw.plus(new Pose2d(x>0 ? Math.max(x-TIP_TOLERANCE, 0) : Math.min(x+TIP_TOLERANCE, 0), y>0 ? Math.max(y-TIP_TOLERANCE, 0) : Math.min(y+TIP_TOLERANCE, 0), 0)));
    //setWeightedDrivePower(raw.plus(new Pose2d(Range.clip(x*2, -TIP_AUTHORITY, TIP_AUTHORITY), Range.clip(y*2, -TIP_AUTHORITY, TIP_AUTHORITY), 0)));
    //}


    public void drivetrain() {

        controller.setPID(p, i, d);
        int liftPos = liftmotor1.getCurrentPosition();
        double pid = controller.calculate(liftPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double power = pid + ff;


        telemetry.addData("pos ", liftPos);
        telemetry.addData("target ", target);


        //x will calibrate field relative
        if (gamepad1.x) {
            //the calibration angle
            angleZeroValue = this.getRawExternalHeading();

        }
        //getting the current angle of the robot and subtracting the calibration angle lets us know delta_theta
        //or the robot's angle relative to its calibration angle
        robotAngle = this.getRawExternalHeading() - angleZeroValue; //angle of robot

        double speed = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y); //get speed
        double LeftStickAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4; //get angle
        double rightX = -gamepad1.right_stick_x; //rotation
        rightX *= 0.8; //optionally reduce rotation value for better turning
        //linear the angle by the angle of the robot to make it field relative
        double leftFrontPower = speed * Math.cos(LeftStickAngle - robotAngle) + rightX;
        double rightFrontPower = speed * Math.sin(LeftStickAngle - robotAngle) - rightX;
        double leftBackPower = speed * Math.sin(LeftStickAngle - robotAngle) + rightX;
        double rightBackPower = speed * Math.cos(LeftStickAngle - robotAngle) - rightX;


//        speedModifier = .8 + (.8 * gamepad1.right_trigger) - (.4 * gamepad1.left_trigger);
//        Our drivers are video game players so this is why we added this ^
//        No Davi, We are NOT Having a Speed Boost!!!!

        if (gamepad2.dpad_up) {
            target = 500;
            liftmotor1.setTargetPosition(500);
            liftmotor2.setTargetPosition(500);
            liftmotor1.setPower(power*0.5);
            liftmotor2.setPower(power*0.5);
        } else if (gamepad2.dpad_left) {
            target = 400;
            liftmotor1.setTargetPosition(400);
            liftmotor2.setTargetPosition(400);
            liftmotor1.setPower(power * 0.5);
            liftmotor2.setPower(power * 0.5);
        } else if (gamepad2.dpad_right) {
            target = 325;
            liftmotor1.setTargetPosition(325);
            liftmotor2.setTargetPosition(325);
            liftmotor1.setPower(power * 0.5);
            liftmotor2.setPower(power * 0.5);
        } else if (gamepad2.dpad_down) {
            //new PIDController(p = 0.001, i =0.01, d = 0);
            target = 100;
            //power = 0.3;
            liftmotor1.setTargetPosition(180);
            liftmotor2.setTargetPosition(180);
            liftmotor1.setPower(power * 0.08); //changed
            liftmotor2.setPower(power * 0.08);
        }
        if (gamepad1.a) {
            Claw.setPosition(1);
        }
        if (gamepad1.b) {
            Claw.setPosition(0.1);
        }
        if (gamepad2.y) {
            Drew.setPosition(0.7);
        }

        if (gamepad2.x) {
            Drew.setPosition(0.5);
        }
        if (gamepad2.a) {
            Drew.setPosition(0);
        }
//        Drew.setPosition(gamepad2.right_trigger);

        liftmotor1.setPower(gamepad2.left_stick_y * -1);
        liftmotor2.setPower(gamepad2.left_stick_y * -1);
//
//        Drew.setPosition(-gamepad2.right_stick_y);


//

        //setting powers correctly
        frontLeft.setPower(leftFrontPower * 0.5);
        frontRight.setPower(rightFrontPower * 0.5);
        backLeft.setPower(leftBackPower * 0.5);
        backRight.setPower(rightBackPower * 0.5);


        telemetry.addData("Robot Angle: ", robotAngle); //this is all telemetry stuff
        telemetry.addData("Front Left Power: ", leftFrontPower);
        telemetry.addData("Front Right Power: ", rightFrontPower);
        telemetry.addData("Rear Left Power: ", leftBackPower);
        telemetry.addData("Rear Right Power: ", rightBackPower);
        telemetry.addData("Claw", Claw.getPosition());
        telemetry.addData("Claw Aim", Drew.getPosition());
        telemetry.addData("liftmotor1position: ", liftmotor1.getTargetPosition());
        telemetry.addData("liftmotor2position, Davi Hates Code: ", liftmotor2.getTargetPosition());
        telemetry.addData("CurrentLift", liftmotor1.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("CurrentLift2", liftmotor2.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Front Left Current: ", frontLeft.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Front Right Current: ", frontRight.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Rear Left Current: ", backLeft.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Rear Right Current: ",  backRight.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Total Current: ",  backLeft.getCurrent(CurrentUnit.AMPS) + backRight.getCurrent(CurrentUnit.AMPS) +  frontRight.getCurrent(CurrentUnit.AMPS) + frontLeft.getCurrent(CurrentUnit.AMPS) +  liftmotor2.getCurrent(CurrentUnit.AMPS) +liftmotor1.getCurrent(CurrentUnit.AMPS) );
        telemetry.addData("Front Left Current: ",  leftFrontPower + rightFrontPower + leftBackPower +  rightBackPower + liftmotor1.getPower() + liftmotor2.getPower());
        telemetry.update();



    }

    public double getRawExternalHeading() {
        //gives us the robot angle
        return imu.getAngularOrientation().firstAngle;
    }


}