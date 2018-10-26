package org.firstinspires.ftc.teamcode.MostCurrentProgram;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="Oppreborn TeleOp", group="TeleOP")
public class OpprebornTeleOp extends LinearOpMode {
    private HardwarePushbot oppreborn = new HardwarePushbot();
    private BNO055IMU imu;
    private Orientation angles;
    private ElapsedTime runtime = new ElapsedTime();

    private final double DRIVE_SPEED = 0.3;                                                         // Nominal speed for better accuracy.
    private final double DROP_SPEED = 0.4;                                                          //This was created as a precaution to ensure that the robot didn't drop down too quickly
    private final double TURN_SPEED = 0.3;                                                          // Nominal half speed for better accuracy.
    private double timeoutmilli = 30000;
    @Override
    public synchronized void runOpMode() {
        oppreborn.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        oppreborn.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        oppreborn.liftnLower.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        oppreborn.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()){
            updateDrive();
            updateHanger();
            updateStrafe();

            idle();
        }
    }
private synchronized void updateDrive(){
        oppreborn.leftDrive.setPower(gamepad1.left_stick_y*.7);
        oppreborn.rightDrive.setPower(gamepad1.right_stick_y*.7);
}
private synchronized void updateHanger(){
        if (gamepad1.left_trigger>0 || gamepad1.right_trigger>0){
            oppreborn.liftnLower.setPower((-((gamepad1.left_trigger/2) + (gamepad1.right_trigger/2)))*.7);
        }
        else if (gamepad1.left_bumper && gamepad1.right_bumper){
            oppreborn.liftnLower.setPower(DROP_SPEED);
        }
        else {
            oppreborn.liftnLower.setPower(0);
        }
}
private synchronized void updateStrafe(){
        oppreborn.midDrive.setPower(((gamepad1.right_stick_x/2)+(gamepad1.left_stick_x/2))*.7);
}
    }
