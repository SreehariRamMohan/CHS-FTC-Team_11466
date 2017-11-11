package org.firstinspires.ftc.teamcode.AutonomousCode;

import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import static java.lang.Thread.sleep;


@TeleOp(name = "Red", group = "Autonomous Version:") // @Autonomous(...) is the other common choice

public class Autonomous_ColorRed extends OpMode{
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;
    NormalizedColorSensor colorSensor;
    View relativeLayout;
    private double start_time;
    private int TICKS_PER_REVOLUTION = 1120;
    IntegratingGyroscope gyro;
    ModernRoboticsI2cGyro modernRoboticsI2cGyro;

    // A timer helps provide feedback while calibration is taking place
    ElapsedTime timer = new ElapsedTime();

    /*
         * Code to run ONCE when the driver hits INIT
         */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        leftMotor = hardwareMap.dcMotor.get("left_drive"); //we would configure this in FTC Robot Controller app
        rightMotor = hardwareMap.dcMotor.get("right_drive");

        modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "sensor_gyro");
        gyro = (IntegratingGyroscope)modernRoboticsI2cGyro;

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotor.getCurrentPosition(); //gets current pos
        leftMotor.getTargetPosition(); //use with runToPosition (set where u want ot go to)
        leftMotor.isBusy(); //tells you if it is still running to the position that u set

        leftMotor.setDirection(DcMotor.Direction.FORWARD);

        //Right motor is reverse because Praneeth put right motor on backwards :/
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        //colorSensor = hardwareMap.colorSensor.get("name_of_color_sensor"); //we would configure the name of the color sensor later in the
        //ftc robot controller

        while (modernRoboticsI2cGyro.isCalibrating())  {
            telemetry.addData("calibrating", "%s", Math.round(timer.seconds())%2==0 ? "|.." : "..|");
            telemetry.update();
            try {
                sleep(50);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }




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
    public void TurnLeftDistance(double power, int distance){
        leftMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);

        leftMotor.setTargetPosition(distance);
        rightMotor.setTargetPosition(-distance);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        TurnLeft(power);
        while(leftMotor.isBusy() && rightMotor.isBusy()){

        }
        StopDriving();

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void TurnRightDistance(double power, int distance){
        leftMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);

        leftMotor.setTargetPosition(-distance);
        rightMotor.setTargetPosition(distance);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        TurnRight(power);
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
        leftMotor.setPower(1);
        rightMotor.setPower(1);
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
    }

    public void callibrate() {
        //turn through an angle of 120 until we find the color
        double dir = Double.parseDouble(formatFloat(modernRoboticsI2cGyro.getAngularVelocity(AngleUnit.DEGREES).zRotationRate));
        double MAX_ANGLE = dir + 90;

        while (dir < MAX_ANGLE) {
            TurnRight(0.1);
            dir = Double.parseDouble(formatFloat(modernRoboticsI2cGyro.getAngularVelocity(AngleUnit.DEGREES).zRotationRate));

        }


    }


    @Override
    public void init_loop() {


    }
    @Override
    public void start() {
        //this is a way to print to the screen of the iphone app, useful for debugging.
        start_time = System.currentTimeMillis();
        telemetry.addData("Autonomous Color Red Starting", "");

        //driveForward(to the ball);

        //

        TurnRightDistance(0.25, convert_to_REV_distance(0, 1));
        while(true) {
            try {
                String color = getColor();
                if(color.equals("Red")) {
                    //red is on the right
                    hitBall("Right");
                    break;
                } else if(color.equals("Blue")) {
                    //blue is on the left
                    hitBall("Left");
                    break;
                } else {

                    //recalibrate
                    callibrate();

                }
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

        }

    }




    @Override
    public void loop() {
        telemetry.addData("Robot starting Will this work?", "");

    }
    @Override
    public void stop() {

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
                return "Blue";

            } else if(ratio > 1.7 && ratio <= 3.5) {
                telemetry.addLine("Red");
                return "Red";

            } else {
                telemetry.addLine("Neither");
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
