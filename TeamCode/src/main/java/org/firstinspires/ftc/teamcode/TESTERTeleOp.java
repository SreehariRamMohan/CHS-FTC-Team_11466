package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

/**
 * Created by sreeharirammohan on 8/31/17.
 */


@TeleOp(name="Template: Linear OpMode v1", group="Linear Opmode")  // @Autonomous(...) is the other common choice
public class TESTERTeleOp extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    ModernRoboticsI2cGyro modernRoboticsI2cGyro;
    ElapsedTime timer = new ElapsedTime();


    //variables for gyro turn
    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 0.75;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.5 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double     DRIVE_SPEED             = 0.7;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.3;     // Nominal half speed for better accuracy.



    ModernRoboticsI2cGyro gyro = null;                    // Additional Gyro device
    DcMotor leftMotor   = null;
    DcMotor rightMotor  = null;


    @Override
    public void runOpMode() throws InterruptedException {
        /*
         * Initialize the standard drive system variables.
         * The init() method of the hardware class does most of the work here
         */
        //robot.init(hardwareMap);
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("sensor_gyro");
        leftMotor = (DcMotor) hardwareMap.dcMotor.get("left_drive");
        rightMotor = (DcMotor) hardwareMap.dcMotor.get("right_drive");

        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();

        gyro.calibrate();

        // make sure the gyro is calibrated before continuing
        while (gyro.isCalibrating())  {
            Thread.sleep(50);
            idle();
        }

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
            telemetry.addData(">", "Robot Heading = %d", gyro.getIntegratedZValue());
            telemetry.update();
            idle();
        }
        gyro.resetZAxisIntegrator();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // Put a hold after each turn

        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);


        //CODE HERE



        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftMotor = hardwareMap.dcMotor.get("left_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");

        // Start calibrating the gyro. This takes a few seconds and is worth performing
        // during the initialization phase at the start of each opMode.
        telemetry.log().add("Gyro Calibrating. Do Not Move!");

        // Wait until the gyro calibration is complete
        timer.reset();

        telemetry.log().clear(); telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.clear(); telemetry.update();

        // Wait for the start button to be pressed
        waitForStart();
        runtime.reset();



    }

    public void TurnLeftAngle(double power, double angle) {
        // The raw() methods report the angular rate of change about each of the
        // three axes directly as reported by the underlying sensor IC.
        int rawX = modernRoboticsI2cGyro.rawX();
        int rawY = modernRoboticsI2cGyro.rawY();
        int rawZ = modernRoboticsI2cGyro.rawZ();
        int heading = modernRoboticsI2cGyro.getHeading();
        int integratedZ = modernRoboticsI2cGyro.getIntegratedZValue();

        // Read dimensionalized data from the gyro. This gyro can report angular velocities
        // about all three axes. Additionally, it internally integrates the Z axis to
        // be able to report an absolute angular Z orientation.
        AngularVelocity rates = gyro.getAngularVelocity(AngleUnit.DEGREES);
        float zAngle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        // Read administrative information from the gyro
        int zAxisOffset = modernRoboticsI2cGyro.getZAxisOffset();
        int zAxisScalingCoefficient = modernRoboticsI2cGyro.getZAxisScalingCoefficient();

//        telemetry.addLine()
//                .addData("dx", formatRate(rates.xRotationRate))
//                .addData("dy", formatRate(rates.yRotationRate))
//                .addData("dz", "%s deg/s", formatRate(rates.zRotationRate));
//        telemetry.addData("angle", "%s deg", formatFloat(zAngle));
//        telemetry.addData("heading", "%3d deg", heading);
//        telemetry.addData("integrated Z", "%3d", integratedZ);
//        telemetry.addLine()
//                .addData("rawX", formatRaw(rawX))
//                .addData("rawY", formatRaw(rawY))
//                .addData("rawZ", formatRaw(rawZ));
//        telemetry.addLine().addData("z offset", zAxisOffset).addData("z coeff", zAxisScalingCoefficient);
//        telemetry.update();
        double angleToAim = zAngle + 90;
        while(gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle <= angleToAim) {
            leftMotor.setPower(-0.25);
            rightMotor.setPower(0.25);
        }




    }

    public void TurnLeftDistance(double power, int distance){

        leftMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);

        leftMotor.setTargetPosition(distance);
        rightMotor.setTargetPosition(distance);

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
        rightMotor.setTargetPosition(-distance);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        TurnRight(power);
        while(leftMotor.isBusy() && rightMotor.isBusy()){

        }
        StopDriving();

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void TurnLeft(double power){
        leftMotor.setPower(-power);
        rightMotor.setPower(power);
    }
    public void TurnRight(double power){
        leftMotor.setPower(power);
        rightMotor.setPower(-power);
    }
    public void StopDriving(){
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
    public int convert_to_REV_distance(int inches, int feet) {
        double conversation_1_foot = 1120;
        return (int) ((inches/12) * conversation_1_foot + feet*conversation_1_foot);
    }

    public void turnTo(double degrees){
        int turnBy = -1;                 //turns clockwise

        if(degrees < gyro.getHeading()){
            turnBy *= -1;
        }

        while(degrees != gyro.getHeading()){
            leftMotor.setPower(-turnBy);
            rightMotor.setPower(turnBy);
        }
    }


}


