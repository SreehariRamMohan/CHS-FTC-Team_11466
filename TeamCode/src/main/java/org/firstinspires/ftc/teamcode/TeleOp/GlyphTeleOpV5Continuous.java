package org.firstinspires.ftc.teamcode.TeleOp;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by michelle on 1/9/2018.
 */
@TeleOp(name="Glyph TeleOp Version 5", group="Linear Opmode")  // @Autonomous(...) is the other common choice
public class GlyphTeleOpV5Continuous extends LinearOpMode {
    static final double INCREMENT   = 0.001;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  0.31;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position
    private ElapsedTime runtime = new ElapsedTime();
    ModernRoboticsI2cGyro modernRoboticsI2cGyro;
    ElapsedTime timer = new ElapsedTime();
    ModernRoboticsI2cGyro gyro = null;                    // Additional Gyro device
    DcMotor leftMotor   = null;
    DcMotor pulleyMotor   = null;
    DcMotor rightMotor  = null;
    CRServo servo1 = null;
    CRServo servo2 = null;
    private double y1 = 0.;
    private double y2 = 0.;
    private double speed = .3;
    @Override
    public void runOpMode() throws InterruptedException {
        /*
         * Initialize the standard drive system variables.
         * The init() method of the hardware class does most of the work here
         */
        //robot.init(hardwareMap);
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("sensor_gyro");
        leftMotor = (DcMotor) hardwareMap.dcMotor.get("left_drive");
        pulleyMotor = (DcMotor) hardwareMap.dcMotor.get("pulley");
        rightMotor = (DcMotor) hardwareMap.dcMotor.get("right_drive");
        servo1 =  hardwareMap.crservo.get("servo_left");
        servo2 =  hardwareMap.crservo.get("servo_right");
        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();
        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        timer.reset();
        telemetry.log().clear(); telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.clear(); telemetry.update();
        waitForStart();
        runtime.reset();
        double servo_position = 0;

        //double start1 = servo1.getPosition();
        //double start2 = servo2.getPosition();

        while(opModeIsActive()) {
            if(gamepad1.left_bumper) {
                telemetry.addData("L Bumper", "");
                telemetry.update();
                openLeft();
            } else if(gamepad1.left_trigger > 0.5) {
                telemetry.addData("L Trigger", "");
                telemetry.update();
                openRight();
                openLeft();
            } else if(gamepad1.right_bumper) {
                telemetry.addData("R Bumper", "");
                telemetry.update();
                openRight();
            } else if(gamepad1.right_trigger>0.5) {
                telemetry.addData("R Trigger", "");
                telemetry.update();
                closeRight();
                closeLeft();
            } else if(gamepad1.dpad_left) {
                closeLeft();
            } else if(gamepad1.dpad_right) {
                closeRight();
            }else {
                servo1.setPower(0);
                servo2.setPower(0);
            }


            if(gamepad1.dpad_down) {
                pulleyMotor.setPower(0.5);
            } else if(gamepad1.dpad_up) {
                pulleyMotor.setPower(-0.5);
            } else {
                pulleyMotor.setPower(0);

            }

            sleep(CYCLE_MS);
            idle();
            y1 = gamepad2.left_stick_y;
            y2 = gamepad2.right_stick_y;

            telemetry.update();

            leftMotor.setPower(-y1 * speed);
            rightMotor.setPower(-y2 * speed);
            if(gamepad2.b){
                speed +=0.01;
            }
            if(gamepad2.a){
                speed -= 0.01;
            }


            if(gamepad2.x) {
                Servo servo = hardwareMap.get(Servo.class, "servo_jewel");
                servo.setPosition(0);
            }


        }
    }

    public void openLeft() {
        servo1.setDirection(CRServo.Direction.FORWARD);
        servo1.setPower(1);
    }

    public void openRight() {
        servo2.setDirection(CRServo.Direction.REVERSE);
        servo2.setPower(1);
    }

    public void closeLeft() {
        servo1.setDirection(CRServo.Direction.REVERSE);
        servo1.setPower(1);
    }

    public void closeRight() {
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
        telemetry.addData("In the turnTo Method", gyro.getHeading()+"");
        telemetry.update();
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
    public void wait(double seconds){
        double time = this.time;
        while(this.time - time < seconds){
        }
    }
}