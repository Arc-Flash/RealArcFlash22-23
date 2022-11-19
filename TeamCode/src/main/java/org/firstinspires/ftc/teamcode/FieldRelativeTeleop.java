package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;


@TeleOp(name = "Field Relative")
public class FieldRelativeTeleop extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
//    private CRServo intake1;
//    private CRServo intake2;
    private Servo V4bServo1;
    private Servo V4bServo2;
    private DcMotorEx liftmotor1;
    private DcMotorEx liftmotor2;

    private PIDController controller;

    public static double p = 0.01, i = 0.01, d = 0;
    public static double f = 0;

    public static int target = 10000;

    private final double ticks_in_degree = 700/180.0;

    //double clawOffset = 0;
    //double clawSpeed = 0.2;
    //double clawStartPosition = 0.5;

    private BNO055IMU imu;

    double speedModifier = 0.8; //@TODO If your drivers complain that the robot is too fast fix this :)
    double robotAngle = 0; //For Field Relative
    double angleZeroValue = org.firstinspires.ftc.teamcode.StaticField.autonHeading;  //gets value from auton. if auton fails for some reason the
    //default angle is 0 degrees. Remember that your drivers should recalibrate if this happens by facing
    //the front of the robot away from them (i.e. the front of the robot is facing your opponents) and then press x.
    @Override
    public void runOpMode(){
        telemetry.addLine("Davi's code suprisingly worked");
        telemetry.update();
        PhotonCore.enable();
        //@TODO Check hardware mappings
        frontLeft = hardwareMap.get(DcMotor.class,"leftFront");
        backLeft = hardwareMap.get(DcMotor.class,"leftRear");
        frontRight = hardwareMap.get(DcMotor.class,"rightFront");
        backRight = hardwareMap.get(DcMotor.class,"rightRear");
//        intake1 = hardwareMap.get(CRServo.class,"intake1");
//        intake2 = hardwareMap.get(CRServo.class,"intake1");
        V4bServo1 = hardwareMap.get(Servo.class,"V4bServo1");
        V4bServo2 = hardwareMap.get(Servo.class,"V4bServo2");

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

        while(opModeIsActive()){
            drivetrain();
        }


    }


    public void initIMU(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        //V4bServo1.setPosition(clawStartPosition);
        //V4bServo2.setPosition(clawStartPosition);
    }
    public void drivetrain() {

        controller.setPID(p, i, d);
        int liftPos = liftmotor1.getCurrentPosition();
        double pid = controller.calculate(liftPos, target);
        double ff = Math.cos(Math.toRadians(target+100 / ticks_in_degree)) * f;

        double power = pid + ff;

//        liftmotor1.setPower(power);
//        liftmotor2.setPower(power);

        telemetry.addData("pos ", liftPos);
        telemetry.addData("target ", target);
        telemetry.update();

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
//        double testServo1Power = speed;
//        double testServo2Power = speed;


//        speedModifier = .8 + (.8 * gamepad1.right_trigger) - (.4 * gamepad1.left_trigger);
//        Our drivers are video game players so this is why we added this ^
//        No Davi, We are NOT Having a Speed Boost!!!!
        if(gamepad2.y){ //v4b servo stuf and lift maybe?
            V4bServo1.setPosition(0.75);
            V4bServo2.setPosition(0.75);
        } else if(gamepad2.a) {
            V4bServo1.setPosition(0.25);
            V4bServo2.setPosition(0.25);
        } else if(gamepad2.b){
            V4bServo1.setPosition(0);
            V4bServo2.setPosition(0);
        }
        if(gamepad1.dpad_up){
            liftmotor1.setTargetPosition(target);
            liftmotor2.setTargetPosition(target);
            liftmotor1.setPower(power);
            liftmotor2.setPower(power);
        }
        if(gamepad1.dpad_down){
            liftmotor1.setTargetPosition(0);
            liftmotor2.setTargetPosition(0);
            liftmotor1.setPower(power);
            liftmotor2.setPower(power);
        }
//        if (gamepad1.a){
//            intake2.setPower(0.5);
//            intake1.setPower(0.5);
//        } else if (gamepad1.b){
//            intake2.setPower(0);
//            intake1.setPower(0);
//        }

        //setting powers correctly
        frontLeft.setPower(leftFrontPower * speedModifier);
        frontRight.setPower(rightFrontPower * speedModifier);
        backLeft.setPower(leftBackPower * speedModifier);
        backRight.setPower(rightBackPower * speedModifier);



        telemetry.addData("Robot Angle: ", robotAngle); //this is all telemetry stuff
        telemetry.addData("Front Left Power: ", leftFrontPower);
        telemetry.addData("Front Right Power: ", rightFrontPower);
        telemetry.addData("Rear Left Power: ", leftBackPower);
        telemetry.addData("Rear Right Power: ", rightBackPower);
        telemetry.addData("V4bServo 1 Position: ", V4bServo1.getPosition());
        telemetry.addData("V4bServo 2 Position: ", V4bServo2.getPosition());
        telemetry.addData("liftmotor1position: ", liftmotor1.getTargetPosition());
        telemetry.addData("liftmotor2position, Davi Hates Code: ", liftmotor2.getTargetPosition());
//        telemetry.addData("Intake Tings: ", intake1.getDirection());
        telemetry.update();



    }
    public double getRawExternalHeading() {
        //gives us the robot angle
        return imu.getAngularOrientation().firstAngle;
    }



}