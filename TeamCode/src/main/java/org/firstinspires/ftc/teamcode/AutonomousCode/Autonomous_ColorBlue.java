package org.firstinspires.ftc.teamcode.AutonomousCode;

import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import static java.lang.Thread.sleep;


@Autonomous(name = "Red", group = "Autonomous Version:")

public class Autonomous_ColorBlue extends LinearOpMode {
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;
    NormalizedColorSensor colorSensor;
    View relativeLayout;
    private double start_time;
    private int TICKS_PER_REVOLUTION = 1120;
    ModernRoboticsI2cGyro gyro;
    ModernRoboticsI2cGyro modernRoboticsI2cGyro;
    ElapsedTime timer = new ElapsedTime();

    CRServo servo1 = null;
    CRServo servo2 = null;
    double servo_position = 0;


    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position

    // Define class members
    Servo servo;
    double  position = 0; // Start at halfway position
    boolean rampUp = true;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");

        leftMotor = hardwareMap.dcMotor.get("left_drive"); //we would configure this in FTC Robot Controller app
        rightMotor = hardwareMap.dcMotor.get("right_drive");
        servo = hardwareMap.get(Servo.class, "servo_jewel");
        gyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "sensor_gyro");
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.getCurrentPosition(); //gets current pos
        leftMotor.getTargetPosition(); //use with runToPosition (set where u want ot go to)
        leftMotor.isBusy(); //tells you if it is still running to the position that u set
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        gyro.calibrate();


        //initialize the serovs for the claw
        servo1 =  hardwareMap.crservo.get("servo_left");
        servo2 =  hardwareMap.crservo.get("servo_right");


        while (gyro.isCalibrating())  {
            telemetry.addData("calibrating", "%s", Math.round(timer.seconds())%2==0 ? "|.." : "..|");
            telemetry.update();
            sleep(50);
        }


        telemetry.addData("Servo position: " + servo.getPosition()+"", "");
        telemetry.update();
        servo.setPosition(0.25);
        Thread.sleep(2500);


        while(true) {
            telemetry.addData("In while loop","while");
            telemetry.update();
            try {
                String color = getColor();
                if(color.equals("Blue")) {
                    //red is on the right
                    hitBall("Blue");
                    break;
                } else if(color.equals("Red")) {
                    //blue is on the left
                    hitBall("Red");
                    break;
                } else {
                    telemetry.addData("Neither", "neither");
                    break;
                    //recalibrate
                }
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

        }
        servo.setPosition(0);
        Thread.sleep(25000);
        //move towards glyph
//        driveForward(0.5, convert_to_REV_distance(6,1));
//        turnTo(90);
//        driveForward(0.25, convert_to_REV_distance(6,0));
//        openClaw();
//        driveForward(0.25, convert_to_REV_distance(6,0));
        telemetry.addData("Done with autonomous Blue test", "");
        telemetry.update();


    }

    private void openClaw() {

        while(servo_position < 0.5) {

        }

        servo_position -= (INCREMENT);
        servo2.setDirection(CRServo.Direction.REVERSE);
        servo2.setPower(1);
        telemetry.addData("R Trigger", "");
        telemetry.update();
        servo_position += (INCREMENT);
        servo2.setDirection(CRServo.Direction.FORWARD);
        servo2.setPower(1);

    }

    public void driveForward(double power, int distance){
        leftMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);

        leftMotor.setTargetPosition(distance);
        rightMotor.setTargetPosition(distance);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DriveForward(power);
        while(leftMotor.isBusy() && rightMotor.isBusy()){

        }
        StopDriving();

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void StopDriving(){
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
    public void DriveForward(double power){
        //For now, we set leftMotor power to negative because our summer training robot has the left motor facing backwards. TODO: Change this after when we switch robots
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }
    public void turnTo(double degrees){
        int turnBy = -1;                 //turns clockwise

//        if(degrees < gyro.getHeading()){
//            turnBy *= -1;
//        }

        telemetry.addData("In the turnTo Method", gyro.getHeading()+"");
        telemetry.update();

        while((degrees - 4.6) > gyro.getHeading() && opModeIsActive()){
            leftMotor.setPower(-0.25);
            rightMotor.setPower(0.25);
        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
    public void TurnLeft(double power){
        leftMotor.setPower(-power);
        rightMotor.setPower(power);
    }
    public void TurnRight(double power){
        leftMotor.setPower(power);
        rightMotor.setPower(-power);
    }
    //Pass in right for right, left for left
    public void hitBall(String direction){

        //move the servo the correct amount of degress.
        if(direction.equals("Blue")){

            driveForward(0.25, convert_to_REV_distance(25,0));
        } else if(direction.equals("Red")){
            leftMotor.setDirection(DcMotor.Direction.REVERSE);
            rightMotor.setDirection(DcMotor.Direction.FORWARD);
            driveForward(0.25, convert_to_REV_distance(20,0));
            servo.setPosition(0);
            leftMotor.setDirection(DcMotor.Direction.FORWARD);
            rightMotor.setDirection(DcMotor.Direction.REVERSE);
            driveForward(0.25, convert_to_REV_distance(25,0));
            //leftMotor.setPower(-1);
            //rightMotor.setPower(-1);
        }

    }

    public void calibrate() {
        //turn through an angle of 120 until we find the color
        double dir = Double.parseDouble(formatFloat(modernRoboticsI2cGyro.getAngularVelocity(AngleUnit.DEGREES).zRotationRate));
        double MAX_ANGLE = dir + 90;

        while (dir < MAX_ANGLE) {
            TurnRight(0.1);
        }

    }


    public int convert_to_REV_distance(int inches, int feet) {
        double conversation_1_foot = 1120;
        return (int) ((inches/12) * conversation_1_foot + feet*conversation_1_foot);
    }

    protected String getColor() throws InterruptedException {

        // values is a reference to the hsvValues array.
        float[] hsvValues = new float[3];
        final float values[] = hsvValues;

        boolean bPrevState = false;
        boolean bCurrState = false;

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
        }

        bCurrState = gamepad1.x;

        bPrevState = bCurrState;
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        Color.colorToHSV(colors.toColor(), hsvValues);
        int color = colors.toColor();

        float max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);

        double ratio = colors.red / colors.blue;
        if(ratio >= 0.15 && ratio <= 1.3) {
            telemetry.addLine("Blue");
            telemetry.update();
            return "Blue";

        } else if(ratio > 1.7 && ratio <= 3.5) {
            telemetry.addLine("Red");
            telemetry.update();
            return "Red";

        } else {
            telemetry.addLine("Neither");
            telemetry.update();
            return "Neither";
        }


    }
    String formatRaw(int rawValue) {
        return String.format("%d", rawValue);
    }

    String formatRate(float rate) {
        return String.format("%.3f", rate);
    }

    String formatFloat(float rate) {
        return String.format("%.3f", rate);
    }

}

