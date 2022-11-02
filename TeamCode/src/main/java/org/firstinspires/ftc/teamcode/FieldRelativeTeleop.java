package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
//import com.outoftheboxrobotics.photoncore.PhotonCore;


@TeleOp(name = "Field Relative")
public class FieldRelativeTeleop extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
      private DcMotor backRight;
//    private CRServo testServo1;
//    private CRServo testServo2;
    private Servo V4bServo1;
    private Servo V4bServo2;

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
        //@TODO Check hardware mappings
        frontLeft = hardwareMap.get(DcMotor.class,"leftFront");
        backLeft = hardwareMap.get(DcMotor.class,"leftRear");
        frontRight = hardwareMap.get(DcMotor.class,"rightFront");
        backRight = hardwareMap.get(DcMotor.class,"rightRear");
//        testServo1 = hardwareMap.get(CRServo.class,"testServo1");
//        testServo2 = hardwareMap.get(CRServo.class,"testServo2");
        V4bServo1 = hardwareMap.get(Servo.class,"V4bServo1");
        V4bServo2 = hardwareMap.get(Servo.class,"V4bServo2");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        initIMU();
        //since this is mecanum
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

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
        if(gamepad1.y){
            V4bServo1.setPosition(0.75);
            V4bServo2.setPosition(0.75);
        } else if(gamepad1.a) {
            V4bServo1.setPosition(0.25);
            V4bServo2.setPosition(0.25);
        }
        if(gamepad1.b){
            V4bServo1.setPosition(0);
            V4bServo2.setPosition(0);
        }


//        clawOffset = Range.clip(clawOffset,-0.5,0.5);
//        V4bServo1.setPosition(clawStartPosition + clawOffset);
//        V4bServo2.setPosition(clawStartPosition - clawOffset);

        //setting powers correctly
        frontLeft.setPower(leftFrontPower * speedModifier);
        frontRight.setPower(rightFrontPower * speedModifier);
        backLeft.setPower(leftBackPower * speedModifier);
        backRight.setPower(rightBackPower * speedModifier);
//        testServo1.setPower(testServo1Power * speedModifier);
//        testServo2.setPower(testServo2Power * speedModifier);



        telemetry.addData("Robot Angle: ", robotAngle);
        telemetry.addData("Front Left Power: ", leftFrontPower);
        telemetry.addData("Front Right Power: ", rightFrontPower);
        telemetry.addData("Rear Left Power: ", leftBackPower);
        telemetry.addData("Rear Right Power: ", rightBackPower);
        telemetry.addData("V4bServo 1 Position: ", V4bServo1.getPosition());
        telemetry.addData("V4bServo 2 Position: ", V4bServo2.getPosition());
        telemetry.update();
//        telemetry.addData("Servo 1 Power: ",testServo1Power);
//        telemetry.addData("Servo 2 Power: ",testServo2Power);



    }
    public double getRawExternalHeading() {
        //gives us the robot angle
        return imu.getAngularOrientation().firstAngle;
    }



}