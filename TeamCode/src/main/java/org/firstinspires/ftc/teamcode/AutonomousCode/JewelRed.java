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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
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

@Autonomous(name="Autonomous Red", group="Iterative Opmode")  // @Autonomous(...) is the other common choice

public class JewelRed extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;
    private NormalizedColorSensor colorSensor = null;
    private ModernRoboticsI2cGyro gyro = null;
    private Servo servo = null;
    private double start_time = 0;
    private final int TICKS_PER_REVOLUTION = 1120;
    private final double SPEED = 0.33;
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

        // right motor is reverse because Praneeth put right motor on backwards :/ um no
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

//        telemetry.addData(">", "calibrating gyro");
//        telemetry.update();
//        gyro.calibrate();

        //natural_zero = gyro.getHeading();

//        while (gyro.isCalibrating()) {
//            telemetry.addData("calibrating", "DO NOT TOUCH or the grim Ashish Rao will find you");
//            telemetry.update();
//        }

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
        Thread.sleep(2000);

        // color sensor is facing forward
        OUTER: while(true) {
            try {
                String color = getColor();
                switch(color) {
                    case "Blue": telemetry.addData("blue", "");
                        telemetry.update();
//                        servo.setPosition(0.825);
//                        Thread.sleep(2000);
//                        servo.setPosition(1);
                        rightMotor.setDirection(DcMotor.Direction.FORWARD);
                        leftMotor.setDirection(DcMotor.Direction.REVERSE);
                        driveForward(SPEED,convert_to_REV_distance(6,0));
                        servo.setPosition(1);
                        leftMotor.setDirection(DcMotor.Direction.FORWARD);
                        rightMotor.setDirection(DcMotor.Direction.REVERSE);
                        driveForward(SPEED,convert_to_REV_distance(14.5,0));
                        break OUTER;
                    case "Red":  telemetry.addData("red", "");
                        telemetry.update();
//                        servo.setPosition(0.475);
//                        Thread.sleep(2000);
//                        servo.setPosition(0.3);
                        driveForward(SPEED,convert_to_REV_distance(8.5,0));
                        servo.setPosition(1);
                        driveForward(SPEED,convert_to_REV_distance(0, 1));
                        break OUTER;
                    default:     //recalibrate
                        break OUTER;
                }
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        telemetry.addLine("YAY");
        telemetry.update();

//        String column = scanVuMark();
//
//        switch(column) {
//            case "LEFT"
//        }






        while (opModeIsActive()) {
            telemetry.addData("servo position", servo.getPosition());
            telemetry.update();
        }
    }

    public void driveForward(double power, int distance){
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setTargetPosition(distance);
        rightMotor.setTargetPosition(distance);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        go(power);
        while(leftMotor.isBusy() && rightMotor.isBusy()){

        }
        StopDriving();

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void go(double power){
        //For now, we set leftMotor power to negative because our summer training robot has the left motor facing backwards. TODO: Change this after when we switch robots
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }
    public void StopDriving(){
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

//    public void driveForward(double power, int distance){
//        leftMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
//        rightMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
//
//        leftMotor.setTargetPosition(distance);
//        rightMotor.setTargetPosition(distance);
//
//        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        double power2 = 0.01;
//
//        leftMotor.setPower(power2);
//        rightMotor.setPower(power2);
//
//        while(leftMotor.isBusy() && rightMotor.isBusy()){
//            if (power2 < power) {
//                power2 *= 1.2;
//            }
//            leftMotor.setPower(power2);
//            rightMotor.setPower(power2);
//        }
//        stopDriving();
//
//        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }

    protected String getColor() throws InterruptedException {
//    telemetry.addData("getColor","");
//        telemetry.update();
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
        telemetry.addData(colors.toString(), "");
        telemetry.update();
        int color = colors.toColor();

        float max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);

        double ratio = colors.red / colors.blue;

        if(ratio >= 0.15 && ratio <= 1.3) {
            telemetry.addLine("Blue");
            //telemetry.update();
            return "Blue";

        } else if(ratio > 1.7 && ratio <= 3.5) {
            telemetry.addLine("Red");
            //telemetry.update();
            return "Red";

        } else {
            telemetry.addLine("Neither");
            //telemetry.update();
            return "Neither";
        }


    }
    public void stopDriving(){
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    public int convert_to_REV_distance(double inches, int feet) {
        return (int)((inches/12.0) * TICKS_PER_REVOLUTION + feet*TICKS_PER_REVOLUTION);
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

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

    String scanVuMark() {
        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code onthe next line, between the double quotes.
         */
        parameters.vuforiaLicenseKey = "Aeaj8tD/////AAAAGW6Wh+xeOELSl05fggXCvs+JRckfIhAAgllP8tM1hJ7PMV7jHtRtTppBITGm51+X50wNJNHDXpZs7/qdm40pOq3jC/3Bgz2ikwHANjDyKdT/GMyPOKCiOYmFfbwzhRfMDC6zrh3xubGspJOJY6GGhRX2sk1q/NEmlMgLnZ/i5FHlkhIe8d12BRzSKUTolwxMzDucm21O4iruVbA/6ojfW0xLN8xzu5OX8EVclVAC5ZbTdVKe8cUysBdJAUgbATu0L42HXsGqG+McRwnhhg+A5ESeLwb7Oy23gmfq1Pkfd+5sbVrZWJD5+c8Gg1B6BIKuwHvNkZl3OgngJH5EtWgTaUV2z4OZZFtBYOnY/HGxK5jr";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        relicTrackables.activate();

        while (opModeIsActive()) {

            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                if (vuMark == RelicRecoveryVuMark.LEFT) {
                    return "LEFT";
                } else if (vuMark == RelicRecoveryVuMark.CENTER) {
                    return "CENTER";
                } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                    return "RIGHT";
                } else {
                    return "NONE";
                }
            }

            return "NONE";
        }

        return "NONE";
    }
}
