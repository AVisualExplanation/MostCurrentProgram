package org.firstinspires.ftc.teamcode.MostCurrentProgram;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class OpprebornTeleOp extends LinearOpMode {
    private HardwarePushbot oppreborn = new HardwarePushbot();
    private BNO055IMU imu;
    private Orientation angles;
    private ElapsedTime runtime = new ElapsedTime();

    private final double DRIVE_SPEED = 0.3;                                                         // Nominal speed for better accuracy.
    private final double DROP_SPEED = 0.4;                                                          //This was created as a precaution to ensure that the robot didn't drop down too quickly
    private final double TURN_SPEED = 0.3;                                                          // Nominal half speed for better accuracy.
    private double timeoutmilli = 30000;
}
