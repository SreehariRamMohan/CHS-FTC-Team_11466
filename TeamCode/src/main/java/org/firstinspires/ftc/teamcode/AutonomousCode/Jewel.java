package org.firstinspires.ftc.teamcode.AutonomousCode;

import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static java.lang.Thread.sleep;

@Autonomous(name="Jewel Blue", group="Iterative Opmode")  // @Autonomous(...) is the other common choice

public class Jewel extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;
    private NormalizedColorSensor colorSensor = null;
    private ModernRoboticsI2cGyro gyro = null;
    private Servo servo = null;
    private double start_time = 0;
    private final int TICKS_PER_REVOLUTION = 1120;
    //double natural_zero;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");

        leftMotor = hardwareMap.dcMotor.get("left_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("sensor_gyro");
        servo = hardwareMap.get(Servo.class, "servo_jewel");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

        // right motor is reverse because Praneeth put right motor on backwards :/
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData(">", "calibrating gyro");
        telemetry.update();
        gyro.calibrate();

        //natural_zero = gyro.getHeading();

        while (gyro.isCalibrating()) {
            telemetry.addData("calibrating", "DO NOT TOUCH or the grim Ashish Rao will find you");
            telemetry.update();
        }

        telemetry.addData("calibration is done", "hella ");
        telemetry.update();

        waitForStart();

        start_time = System.currentTimeMillis();
        telemetry.addData("robot starting", "please work");
        telemetry.update();

//        forward = 0.65
//        folded = 0.3 (right)
//        left = 0.825
//        right = 0.475

//        servo.setPosition(1);
//        Thread.sleep(2000);
//        servo.setPosition(0.65);
//        Thread.sleep(2000);
//        servo.setPosition(0.25);

        servo.setPosition(0.65);
        // driveForward(-0.33,convert_to_REV_distance(9,0));

        // color sensor is on the right
        while(true) {
            try {
                String color = getColor();
                if(color.equals("Blue")) {
                    servo.setPosition(0.825);
                    Thread.sleep(2000);
                    servo.setPosition(1);
                    break;
                } else if(color.equals("Red")) {
                    servo.setPosition(0.475);
                    Thread.sleep(2000);
                    servo.setPosition(0.3);
                    break;
                } else {
                    break;
                    // recalibrate
                }
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        //driveForward(0.33,convert_to_REV_distance(10,0));
        turnTo(-90);
        //driveForward(0.33,convert_to_REV_distance(0,2));

        telemetry.addLine("YAY");
        telemetry.update();

        while (opModeIsActive()) {
            telemetry.addData("servo position", servo.getPosition());
            telemetry.update();
        }
    }

    public void driveForward(double power, int distance){
        leftMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);

        leftMotor.setTargetPosition(distance);
        rightMotor.setTargetPosition(distance);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //TODO: MAKE SURE ROBOT DRIVES STRAIGHT

        double power2 = 0.01;

        leftMotor.setPower(power2);
        rightMotor.setPower(power2);

        while(leftMotor.isBusy() && rightMotor.isBusy()){
            if (power2 < power) {
                power2 *= 1.2;
            }
            leftMotor.setPower(power2);
            rightMotor.setPower(power2);
        }
        stopDriving();

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public String getColor() throws InterruptedException {

        // values is a reference to the hsvValues array.
        float[] hsvValues = new float[3];
        final float values[] = hsvValues;

        boolean bPrevState = false;
        boolean bCurrState = gamepad1.x;

        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
        }

        bPrevState = bCurrState;
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        Color.colorToHSV(colors.toColor(), hsvValues);
        int color = colors.toColor();

        float max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);

        double ratio = colors.red / colors.blue;
        if(ratio >= 0.15 && ratio <= 1.3) {
            telemetry.addLine("Blue");
            return "Blue";

        } else if(ratio > 1.7 && ratio <= 3.5) {
            telemetry.addLine("Red");
            return "Red";

        } else {
            telemetry.addLine("Neither");
            return "Neither";
        }
    }

    public void stopDriving(){
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    public int convert_to_REV_distance(int inches, int feet) {
        return (int)((inches/12) * TICKS_PER_REVOLUTION + feet*TICKS_PER_REVOLUTION);
    }

    public void turnTo(double degrees){
        int turnBy = -1;                 //turns clockwise
        telemetry.addData("in the turnTo method", gyro.getHeading()+"");
        telemetry.update();

        while((degrees - 4.6) > gyro.getHeading() && opModeIsActive()){
            leftMotor.setPower(-0.25);
            rightMotor.setPower(0.25);
        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
}
