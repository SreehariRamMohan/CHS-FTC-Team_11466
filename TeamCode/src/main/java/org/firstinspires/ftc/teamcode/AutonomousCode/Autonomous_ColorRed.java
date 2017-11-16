package org.firstinspires.ftc.teamcode.AutonomousCode;

import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static java.lang.Thread.sleep;


@TeleOp(name = "Red", group = "Autonomous Version:")

public class Autonomous_ColorRed extends LinearOpMode {
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
    ElapsedTime timer = new ElapsedTime();

    //attributes for vuforia
    public static final String TAG = "Vuforia VuMark Sample";
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;

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
//        servo.setPosition(90);

        //colorSensor = hardwareMap.colorSensor.get("name_of_color_sensor"); //we would configure the name of the color sensor later in the
        //ftc robot controller

        while (modernRoboticsI2cGyro.isCalibrating())  {
            telemetry.addData("calibrating", "%s", Math.round(timer.seconds())%2==0 ? "|.." : "..|");
            telemetry.update();
            sleep(50);
        }

        telemetry.addData("Servo position: " + servo.getPosition()+"", "");
        telemetry.update();
        servo.setPosition(0.25);
        Thread.sleep(2500);

        while(true) {
            try {
                String color = getColor();
                if(color.equals("Blue")) {
                    //red is on the right
                    hitBall("Red");
                    servo.setPosition(1);
                    break;
                } else if(color.equals("Red")) {
                    //blue is on the left
                    hitBall("Blue");
                    servo.setPosition(1);
                    break;
                } else {
                    break;
                    //recalibrate
                }
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

        }


        telemetry.addData("Done with autonomous test", "");
        telemetry.update();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "Aeaj8tD/////AAAAGW6Wh+xeOELSl05fggXCvs+JRckfIhAAgllP8tM1hJ7PMV7jHtRtTppBITGm51+X50wNJNHDXpZs7/qdm40pOq3jC/3Bgz2ikwHANjDyKdT/GMyPOKCiOYmFfbwzhRfMDC6zrh3xubGspJOJY6GGhRX2sk1q/NEmlMgLnZ/i5FHlkhIe8d12BRzSKUTolwxMzDucm21O4iruVbA/6ojfW0xLN8xzu5OX8EVclVAC5ZbTdVKe8cUysBdJAUgbATu0L42HXsGqG+McRwnhhg+A5ESeLwb7Oy23gmfq1Pkfd+5sbVrZWJD5+c8Gg1B6BIKuwHvNkZl3OgngJH5EtWgTaUV2z4OZZFtBYOnY/HGxK5jr";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        relicTrackables.activate();

        while (opModeIsActive()) {

            /**
             * See if any of the instances of {@link relicTemplate} are currently visible.
             * {@link RelicRecoveryVuMark} is an enum which can have the following values:
             * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
             * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
             */
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                if(vuMark == RelicRecoveryVuMark.LEFT){
                    //Pass the data "LEFT" to the glyph autonomous somehow
                }else if(vuMark == RelicRecoveryVuMark.CENTER){
                    //Pass the data "CENTER" to the glyph autonomous somehow
                } else if(vuMark == RelicRecoveryVuMark.RIGHT){
                    //Pass the data "RIGHT" to the glyph autonomous somehow
                }
                //This should print out which one was visible
                telemetry.addData("VuMark", "%s visible", vuMark);

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
                //TODO WE CAN PROLLY REMOVE THIS POSE STUFF
                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                telemetry.addData("VuMark", "%s visible", vuMark);

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
                telemetry.addData("Pose", format(pose));

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
                if (pose != null) {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
                    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);

                    // Extract the rotational components of the target relative to the robot
                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;
                }
            }
            else {
                telemetry.addData("VuMark", "not visible");
            }

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
            leftMotor.setPower(1);
            rightMotor.setPower(1);
            driveForward(0.25, convert_to_REV_distance(35,0));
        } else if(direction.equals("Red")){
            leftMotor.setDirection(DcMotor.Direction.REVERSE);
            rightMotor.setDirection(DcMotor.Direction.FORWARD);
            driveForward(0.25, convert_to_REV_distance(35,0));
            leftMotor.setDirection(DcMotor.Direction.FORWARD);
            rightMotor.setDirection(DcMotor.Direction.REVERSE);
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

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

}
