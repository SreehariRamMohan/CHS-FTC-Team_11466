package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by anjanbharadwaj on 11/13/17.
 */


@TeleOp(name="Glyph TeleOp", group="Linear Opmode")  // @Autonomous(...) is the other common choice
public class GlyphTeleOp extends LinearOpMode {
    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  0.31;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position

    private ElapsedTime runtime = new ElapsedTime();

    ModernRoboticsI2cGyro modernRoboticsI2cGyro;
    ElapsedTime timer = new ElapsedTime();

    ModernRoboticsI2cGyro gyro = null;                    // Additional Gyro device
    DcMotor leftMotor   = null;
    DcMotor rightMotor  = null;
    Servo servo1 = null;
    Servo servo2 = null;


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
        servo1 = (Servo) hardwareMap.servo.get("servo_left");
        servo2 = (Servo) hardwareMap.servo.get("servo_right");

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

        // Wait until the gyro calibration is complete
        timer.reset();

        telemetry.log().clear(); telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.clear(); telemetry.update();

        // Wait for the start button to be pressed
        waitForStart();
        runtime.reset();

        double servo_position = 0;


        double position = 0;

        while(opModeIsActive()) {

            if(gamepad1.a) {
                telemetry.addData("A Pressed", "Hella");
                telemetry.update();

//                servo_position += INCREMENT;
//                servo1.setPosition(servo_position);
//                servo2.setPosition(1 - servo_position);
                while (opModeIsActive()) {
                    // slew the servo, according to the rampUp (direction) variable.
                    // Keep stepping up until we hit the max value.
                    position += INCREMENT;
                    if (position >= MAX_POS) {
                        position = MAX_POS;
                        break;  // Switch ramp direction
                    }
                    // Display the current value
                    servo1.setPosition(position);
                    servo2.setPosition(1-position);
                    telemetry.addData("Servo Position", "%5.2f", position);
                    telemetry.addData(">", "Press Stop to end test.");
                    telemetry.update();
                    waitForNextHardwareCycle();
                    // Set the servo to the new position and pause;

                    sleep(CYCLE_MS);
                    idle();
                }
            } else if(gamepad1.b) {
                telemetry.addData("B Pressed", "Hella");
                telemetry.update();
//                servo_position -= INCREMENT;
//                servo1.setPosition(servo_position);
//                servo2.setPosition(1 - servo_position);
                while (opModeIsActive()) {
                    // slew the servo, according to the rampUp (direction) variable.
                    // Keep stepping up until we hit the max value.
                    position -= INCREMENT;
                    if (position <= MIN_POS) {
                        position = MAX_POS;
                        break;  // Switch ramp direction
                    }
                    // Display the current value
                    servo1.setPosition(1-position);
                    servo2.setPosition(position);
                    telemetry.addData("Servo Position", "%5.2f", position);
                    telemetry.addData(">", "Press Stop to end test.");
                    telemetry.update();
                    waitForNextHardwareCycle();
                    // Set the servo to the new position and pause;

                    sleep(CYCLE_MS);
                    idle();
                }
            }


//            sleep(CYCLE_MS);
//            idle();
        }



    }
    public void driveForward(double power, int distance){
        leftMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);

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
    public int convert_to_REV_distance(int inches, int feet) {
        double conversation_1_foot = 1120;
        return (int) ((inches/12) * conversation_1_foot + feet*conversation_1_foot);
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


}


