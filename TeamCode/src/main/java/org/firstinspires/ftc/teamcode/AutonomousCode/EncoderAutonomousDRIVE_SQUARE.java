package org.firstinspires.ftc.teamcode.AutonomousCode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name="Encoder Test", group="Iterative Opmode")  // @Autonomous(...) is the other common choice

public class EncoderAutonomousDRIVE_SQUARE extends LinearOpMode {
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;
    private ColorSensor colorSensor = null;
    private double start_time;
    private int TICKS_PER_REVOLUTION = 1120;
    ModernRoboticsI2cGyro gyro = null;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");

        leftMotor = hardwareMap.dcMotor.get("left_drive"); //we would configure this in FTC Robot Controller app
        rightMotor = hardwareMap.dcMotor.get("right_drive");
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("sensor_gyro");
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

        start_time = System.currentTimeMillis();
        telemetry.addData("Robot starting Will this work?", "");
        driveForward(0.25, convert_to_REV_distance(35,0));
        turnTo(90);
        driveForward(0.25, convert_to_REV_distance(35, 0));
        turnTo(90);
        driveForward(0.25, convert_to_REV_distance(35, 0));
        turnTo(90);
        driveForward(0.25, convert_to_REV_distance(35, 0));
    }

    public void driveForward(double power, int distance){
        leftMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);

        leftMotor.setTargetPosition(distance);
        rightMotor.setTargetPosition(distance);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor.setPower(1);
        rightMotor.setPower(1);

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
    public void TurnLeft(double power){
        leftMotor.setPower(-power);
        rightMotor.setPower(power);
    }
    public void TurnRight(double power){
        leftMotor.setPower(power);
        rightMotor.setPower(-power);
    }
    public int convert_to_REV_distance(int inches, int feet) {
        double conversation_1_foot = 1120;
        return (int) ((inches/12) * conversation_1_foot + feet*conversation_1_foot);
    }
    public void turnTo(double degrees){

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
