package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
/*
Created by anjanbharadwaj on 11/7/2017
 */

@TeleOp(name="Basic Tank Drive Tele-Op", group="Linear Opmode")  // @Autonomous(...) is the other common choice
public class BasicTankDriveDONOTCHANGE extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;
    private double y1 = 0.;
    private double y2 = 0.;
    private double speed = .5;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        leftMotor = hardwareMap.dcMotor.get("left_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");

        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        //leftMotor.setDirection(DcMotor.Direction.FORWARD);  // Set to REVERSE if using AndyMark motors FORWARD for right motor
        rightMotor.setDirection(DcMotor.Direction.REVERSE); // REVERSE because this motor is mounted reverse

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            y1 = gamepad1.left_stick_y;
            y2 = gamepad1.right_stick_y;
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("rightMotor: " + y2, "leftMotor: " + y1);
            telemetry.addData("speed: ", speed);
            telemetry.update();

            // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)

            //forward movement
            leftMotor.setPower(-y1 * speed);
            rightMotor.setPower(-y2 * speed);

            if(gamepad1.b){
                leftMotor.setPower(0);
                rightMotor.setPower(0);
            }

            //set speed
            if(gamepad1.left_bumper){
                speed -= .1;
                wait(.5);
                if(speed < .1){
                    speed = .1;
                }
            }else if(gamepad1.right_bumper) {
                speed += .1;
                wait(.5);
                if (speed > 1) {
                    speed = 1;
                }
            }
        }
    }
    public void wait(double seconds){
        double time = this.time;
        while(this.time - time < seconds){

            //does nothing, purpose is to wait a certain amount of seconds
        }
    }
}