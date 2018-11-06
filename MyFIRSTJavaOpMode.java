package org.firstinspires.ftc.teamcode.MostCurrentProgram; //This declares that this class is located within the team code folder of the ftc folder

import com.qualcomm.hardware.bosch.BNO055IMU; //This imports the required information for running the IMU and allowing it to recognize it
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;//This imports the autonomous reference to allow the driver station to recognize it as such
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode; //This imports the methods found within LineaOpMode so that they can be used here
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;


/*
TODO:
-Create method for dismounting from the lander (created by "Samuel Tukua", "24/9/2018")
-Create method for putting down the marker in the depot.(created by "Samuel Tukua", "24/9/2018")
-Create a method to identify the color of the minerals in the sampling field and move the correct one
 (created by "Samuel Tukua" , "28/9/2018")
-It would be useful that if the rover goes too far off course that it would stop moving entirely and then
 reorient itself rather than continue forward. It could do this by making use of the accelerometer and the
 integration formula in order to find the distance that it has unexpectedly moved in any combination of x
 and y directions based upon how hard and how long it was pushed in that direction. This would allow for
 backtracking into its correct position and orientation. (created by "Samuel Tukua", "25/9/2018")
-The ability of the robot to notice other robots in its direct way. My best idea for this would be a distance
 sensor that would constantly be sending the rover a change in distance to an object over time. If another
 rover were to get in the way of our robot then the distance over time would experience a spike relative to
 the typically unchanging rover speed. If the distance change that caused this spike lasts for greater than
 two seconds, then the robot will enter a while function, stop, and wait until the distance increases. Because
 the robot is not moving at the time, the only cause to an increase in distance would be another robot moving
 out of the way. (created by "Samuel Tukua", "25/9/2018")
-Check that no more "NEEDEDIT" fields still exist (created by "Samuel Tukua", "24/9/2018")
-Make sure that all codes relay that they properly accomplished a task when finished (created by "Samuel Tukua", "24/9/2018")
 */

/*Possible Sources of Errors
 -Intrinsic vs. extrinsic
 -Old Encoder wheel issues
 -Mixing up lefts and rights in whether the change in power should be positive or negative
 -Trial and Error the gyroscopes values with the gyroscope demonstration tool*/

/**
 ***IMPORTANT READ***
 * It is important to note that only the final version of each section of code has been left in here. A full history of the code can be accessed
 * within the "11857 2018-2019 Programmer Journal". The reason for this is in order to make the code easier to understand while still ensuring that all edits to the code are addressed.
 *
 * The citation of each individual edit to the code has a specific formula:
 * "(Created by "name", "date (DD/MM/YYYY)", "edit number", "what was changed and why", and "results")"
 */

/*IMU Explanation
The Rev Tech Expansion Hub has a built in IMU which has various important uses and capabilities. IMU is short for Inertial Measurement Unit
and the built in version is called the "BNO055" and is capable of measuring absolute orientation, Angular Velocity Vector, Acceleration Vector,
Linear Acceleration Vector, and Gravity as specified by "https://www.adafruit.com/product/2472".
*/


/* The following code starts with the statement of "@Autonomous". This sends a message from the Robot Control ,where this code is executed, to the
Driver Station that tells the driver station that this class is Autonomous. The parameters for the "@Autonomous" are name and group. The name
parameter is what this class will show up as to the phone. The group parameter is going to be autonomous.
 */

// (Created by "Samuel Tukua","24/09/2018", "#1", "This is just the establishing code or the first run of the robot", "NEEDEDIT for the counts per motor rev and the like variables")
@Autonomous(name="AutoOPGyroFacingDepot", group="Autonomous")
public class MyFIRSTJavaOpMode extends LinearOpMode {
    private HardwarePushbot oppreborn = new HardwarePushbot();
    private BNO055IMU imu;
    private Orientation angles;
    private ElapsedTime runtime = new ElapsedTime();

    /*This was taken from the sample of SensorBNO055IMU.java for simplicity. However, the loggingEnabled was changed to false
    and all accompanying data was removed in order to use the imu as an input for movements rather than a console output. The purpose
    of this code is to simply set the initial parameters of the IMU and to calibrate it. Parts of this were adapted from
    " http://stemrobotics.cs.pdx.edu/node/7265 "
     */
    private BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();                           //This creates a new instance of the parameters
    private final double COUNTS_PER_MOTOR_REV_WHEELS = 2240;                                               //Different motors will spin at different rates even when the same amount of power is applied. These are identified as counts. For example, Tetrix motors have 1440 counts for every single rotation/revolution.
    private final double DRIVE_GEAR_REDUCTION = 1.0;                                                // This is < 1.0 if geared UP. This has to do with how the motors are connected to the wheels. If it is a direct connection then there is no gear up. But, if there are gears in between then the wheel will likely not rotate at the same rate as the motor. This accounts for that.
    private final double WHEEL_DIAMETER_INCHES = 3.54331;                                               // For figuring circumference
    private final double COUNTS_PER_INCH_WHEELS = (COUNTS_PER_MOTOR_REV_WHEELS * DRIVE_GEAR_REDUCTION) /          //Because the motor measures its rotations in "counts", this translates those counts by answering "how many counts should the motor go in order to move the wheel by one inch".
            (WHEEL_DIAMETER_INCHES * 3.1415);
    private final double COUNTS_PER_MOTOR_REV_LIFTNLOWER = 288;                                     // This is needed because the linear slide has a different motor count from the wheels.
    private final double LIFTNLOWER_DIAMETER = 1.0 ;
    private final double COUNTS_PER_INCH_LIFTNLOWER = (COUNTS_PER_MOTOR_REV_LIFTNLOWER * DRIVE_GEAR_REDUCTION) /
            (LIFTNLOWER_DIAMETER * 3.1415);

    private final double DRIVE_SPEED = 0.3;                                                         // Nominal speed for better accuracy.
    private final double DROP_SPEED = 0.4;                                                          //This was created as a precaution to ensure that the robot didn't drop down too quickly
    private final double TURN_SPEED = 0.3;                                                          // Nominal half speed for better accuracy.
    @Override
    public synchronized void runOpMode() throws InterruptedException {                                                          //runOpMode() is where all of the information for actually running the op mode goes. This is what is called when the big white button that says "init" on it is pressed.
        oppreborn.init(hardwareMap);
        telemetry.addData("hardwareMap", "Initilized");
        telemetry.update();
        //sleep(1000);
        // (Created by "Samuel Tukua","26/09/2018", "edit #2", "This changes the Zero Power Mode to resist being pushed", "NEEDEDIT for results of this change")
        oppreborn.leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);                  //Naturally when the robot is pushed while its wheels are set to zero power, the robot
        oppreborn.rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);                 //wheels will spin. This means that another robot would be able to push the robot out
        oppreborn.liftnLower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);                 //of the way. Setting the ZeroPowerBehavior() to "BRAKE" means that when the wheels
                                                                                                    //are given a power value of "0" then they will both stop and actively resist movement.
        //Encoder Wheels
        oppreborn.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);                        //This stops the left drive if it was moving and then resets the encoder
        oppreborn.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);                       //This stops the right drive if it was moving and then resets the encoder
        oppreborn.liftnLower.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);                       //This stops the liftnLower drive if it was moving and then resets the encoder
        telemetry.addData("Path0", "Starting at %7d :%7d",                           //This just updates the telemetry data saying that the rover is on path zero starting at the current position of the left and right wheels which should be zero
                oppreborn.leftDrive.getCurrentPosition(),
                oppreborn.rightDrive.getCurrentPosition());
        telemetry.update();

        //IMU Gyro
        // (Created by "Samuel Tukua","26/09/2018", "edit #3", "This puts the IMU into sensor mode rather than keeping it in config mode", "NEEDEDIT for results of this change")
        parameters.mode = BNO055IMU.SensorMode.IMU;                                                 //This puts the IMU into sensor mode as opposed to config mode
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;                                         //This sets the unit for the IMU's angle to be in degrees
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;                            //This sets the unit for the IMU's acceleration to be meters per second squared
        parameters.loggingEnabled = false;                                                          //This turns off logging and allows for the use of the IMU as an input to guide the robots movements
        parameters.accelerationIntegrationAlgorithm = null;                                         //This is an algorithm that can use acceleration in order to find velocity and position using integral calculus
        imu = hardwareMap.get(BNO055IMU.class, "imu");                                   //This establishes the use of the imu under the hardware map to just be referenced as "imu". Most importantly, it means that the robot configuration in the expansion hub will refer to the port where the imu is located as "imu".
        imu.initialize(parameters);                                                                 //This initializes the parameters (moves the parameters specified to be associated with the imu)


        //Dismount();
        //Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //This establishes that when I ask for angles, I am wanting the Extrinsic angles listed in degrees in the format of ZYX.

        IMUDrive(0.3,25,0);
        IMUDrive(0.3, 50, 90);
        IMUDrive(0.3,47,135);
        PlaceMarker();
        IMUDrive(0.3,78,-45);

        // imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);                  //This starts the integration (integral calculus) processes for the acceleration.
        telemetry.addData("Path", "Complete");                                        //This sends the driver station phone the message that the robot has completed all of its necessary paths
        telemetry.update();
    }


    /*This is the method that dismounts the rover from the lander. It does this method in a three step process
     of lowering, detaching, and preparing for the next command.
     */
    //private synchronized void Dismount(double inches) {
     //   int newdismount = oppreborn.liftnLower.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH_LIFTNLOWER); //This establishes the number of counts that are needed to go the desired distance.
       // oppreborn.liftnLower.setTargetPosition(newdismount);                                        //This sets the target position to be the ground
      //  oppreborn.liftnLower.setMode(DcMotor.RunMode.RUN_TO_POSITION);                              //This causes the motor to run until it gets to its destination
     //   oppreborn.liftnLower.setPower(Math.abs(DROP_SPEED));                                        //This tells the robot to run at the given speed for dropping which is lower th
        //part 2: release latching mechanism
        //Part 3: put robot into needed position and form
        //oppreborn.liftnLower.setPower(0);
    //}


    /******************************************************************
     public void SampleArea() {
     //This method is just being created in order to server as a reminder of future work.
     }
     *********************************************************************/



    /*This is the method that drives the rover forward at any desired angle relative to the rover's current angle.
      it does this by first finding its current orientation and then turns either clockwise
      or counter clockwise until it matches the angle depending on whether the degrees input was 0 to +180 (counter clockwise)
      or 0 to -180 (clockwise). Finally, it uses the encoders within the wheel motors
      to go a desired distance forward while continually checking that it is in a straight line.*/
    private synchronized void IMUDrive(double speed, double inches, double angle) {
        Rotate(angle);
        int newLeftTarget;
        int newRightTarget;
        if (opModeIsActive()) {

            oppreborn.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            oppreborn.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Determine new target position, and pass to motor controller
            newLeftTarget = oppreborn.leftDrive.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH_WHEELS); //This gets the wheels current position, and adds the number of inches forward desired. But, remember that the rover moves in terms of counts, so it translates the number of inches into the number of counts.
            newRightTarget = oppreborn.rightDrive.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH_WHEELS);
            oppreborn.leftDrive.setTargetPosition(newLeftTarget);                                   //This sets the new target equal to the distance defined above in terms of counts
            oppreborn.rightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            oppreborn.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);                           //This makes the motors both begin to drive to their desired position as defined below.
            oppreborn.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            oppreborn.leftDrive.setPower(Math.abs(speed));                                          //This sets the wheel speed equal to the speed defined before
            oppreborn.rightDrive.setPower(Math.abs(speed));
            oppreborn.midDrive.setPower(0);

            while (opModeIsActive() &&
                    (oppreborn.leftDrive.isBusy() && oppreborn.rightDrive.isBusy())){

                double adjustment = Math.abs(AdjustOrientation(angle));                                       //This calls upon the AdjustOrientation() function defined below.
                if (angles.firstAngle > angle) {                                                    //This checks to see if the current angle is greater than the desired angle in turns of euclidean angles if this is true, then the speed of the left wheel will increase causing the robot to speed up by the increment of adjustment
                    oppreborn.leftDrive.setPower(Math.abs(speed + adjustment));
                    oppreborn.rightDrive.setPower(Math.abs(speed - adjustment));
                }
                else if (angles.firstAngle < angle) {                                               //This does the same as the previous if statement, but for turning to the right
                    oppreborn.rightDrive.setPower(Math.abs(speed + adjustment));
                    oppreborn.leftDrive.setPower(Math.abs(speed - adjustment));
                }
                else {
                    oppreborn.leftDrive.setPower(speed);
                    oppreborn.rightDrive.setPower(speed);
                }

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        oppreborn.leftDrive.getCurrentPosition()/COUNTS_PER_INCH_WHEELS,
                        oppreborn.rightDrive.getCurrentPosition()/COUNTS_PER_INCH_WHEELS);
                telemetry.update();
                idle();
            }

            // Stop all motion;
            oppreborn.leftDrive.setPower(0);
            oppreborn.rightDrive.setPower(0);
            oppreborn.midDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            oppreborn.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            oppreborn.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }

        //part 2: turn that angle and double check that you are at the correct angle when finished.
        //part 3: drive the required distance while continually adjusting so that on correct angle
    }


    /*This just commands the motor/servo that has the sole purpose of putting down the marker*/
    private void PlaceMarker() {
while((oppreborn.placeMarker.getPosition()!= .4) && (opModeIsActive())) {
    oppreborn.placeMarker.setPosition(.4);
    idle();
}
    }


    //(Created by "Samuel Tukua","26/09/2018", "edit #5", "Finished the rotate method, but still need to check it", "NEEDEDIT checking that the thirdangle is the one that I want to use")
    private synchronized void Rotate(double dsrangle) {                                             //This method will be called upon in order to rotate the rover using the gyro
        if (opModeIsActive()) {
            oppreborn.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            oppreborn.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES);
            if (dsrangle-1<angles.firstAngle && dsrangle+1>angles.firstAngle){
                return;
            }
            if (angles.firstAngle > dsrangle) {                                                          //This checks to see if the current angle is greater than the desired angle, "dsrangle", and if so, it will tell the robot that it needs to Rotate until the angles are equal
                while (angles.firstAngle > dsrangle && opModeIsActive()){                                                   //This stops when the current angle equals the desired angle or if the current time exceeds the 30 seconds that the match is allowed to take.
                    oppreborn.leftDrive.setPower(TURN_SPEED);                                       //This rotation results in the rover turning in a clockwise fashion which in euclidean angles means that it's rotation is approaching -180 degrees.
                    oppreborn.rightDrive.setPower(-TURN_SPEED);
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES);
                    telemetry.addData("Rotation at","%7d of %7d", Math.round(angles.firstAngle) , Math.round(dsrangle));
                    telemetry.update();
                    idle();
                }
                oppreborn.leftDrive.setPower(0);                                                    //This makes sure that the wheels have stopped spinning once the robot has finished its rotation
                oppreborn.rightDrive.setPower(0);
            } else if (angles.firstAngle < dsrangle) {                                                   //This checks to see if the current angle is less than the desired angle, "dsrangle", and if so, it will tell the robot that it needs to Rotate until the angles are equal.
                while (angles.firstAngle < dsrangle && opModeIsActive()) {                                                   //This setup does the  same as the previous setup but instead does it in a counter clockwise fashion in order to rotate the robot towards the positive 180 degrees section
                    oppreborn.leftDrive.setPower(-TURN_SPEED);
                    oppreborn.rightDrive.setPower(TURN_SPEED);
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    telemetry.addData("Rotation at","%7d of %7d", Math.round(angles.firstAngle) , Math.round(dsrangle));
                    telemetry.update();
                    idle();
                }
                oppreborn.leftDrive.setPower(0);
                oppreborn.rightDrive.setPower(0);
            } else if (angles.firstAngle == dsrangle) {                                                  //This checks to see if the current angle is equal to the desired angle, "dsrangle", and if so, it will just move on to the next method
                oppreborn.leftDrive.setPower(0);
                oppreborn.rightDrive.setPower(0);
                telemetry.addData("Robot Orientation", "Correctly at " + dsrangle);
                telemetry.update();
                return;                                                                             //This essentially just ends the loop early
            } else {
                telemetry.addData("The following just happened: ", "The IMPOSSIBLE"); //This is a bit of joke code that will likely never get executed because it is impossible for two real numbers to neither equal, be less than, or be greater than each other at the same time. I just thought it'd be a funny fail safe.
                telemetry.update();
            }
            oppreborn.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            oppreborn.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            telemetry.addData("Robot Orientation", "Correctly at " + dsrangle);       //This displays on the driver station that the robot is correctly oriented at the given angle
            telemetry.update();
        }
    }


    private synchronized double AdjustOrientation(double angle) {                                   //This is the method that keeps the robot in a straight line while it is driving. This method has an input for the correct angle that the robot should be going and bases its data off of that.
        double adjustment, calibration = 0.002;                                                       //This establishes two variables that are both doubles. One is the adjustment variable that the method will be returning and the other is a calibration variable that will be experimentally tweaked until it fits the robot the best.
        adjustment = (angles.firstAngle - angle) * calibration;                                     //This defines the adjustment as the difference between the desired angle and the current angle and multiplies that by the calibration.
        return adjustment;                                                                          //This return statement means that the method "AdjustOrientation(angle)" will return the required amount of adjustment to be added to the wheels.
    }


    /**
     * private synchronized double[] RoverFromHit(){
     * while (opModeIsActive() && duration<timeoutmilli){
     * "get current acceleration";
     * "get expected acceleration"//you could create getter and setter threads that always keep track of
     * if (( "current_acceleration>expected_acceleration+0.1")
     * (|| "current acceleration>wheel acceleration: You will need to know whether the robot ")
     * while "";}
     * double[] recoverinstruct =new double[5];
     * recoverinstruct[0]=1.0;
     * return recoverinstruct;
     * }
     */
/*
    private final Thread TimeKeeper = new Thread(new Runnable() {                                   //This creates a new thread called TimeKeeper. Java is multithreaded and so this program can run seperately from the main program while still making alterations. This one in particular turns off the robot once the time surpasses the allowed time.
        public void run() {
            try {                                                                                   //Because the TimeKeeper.join() statement throws InterruptedException, the try/catch statement is used to handle that error.
                final long startTime = System.currentTimeMillis();                                  //This establishes the start time as whenever the thread is initialized. In this cause, the thread is started right after the the waitforstart() in the runopmode() method.
                double duration = (int) (System.currentTimeMillis() - startTime);                   //This establishes what it means when the code references duration.
                while (timeoutmilli < duration && opModeIsActive()) {                               //This checks that the duration is less than the timeout of 30 seconds and if so then it displays the current time in milliseconds to the driver using the telemetry.addData() method.
                    duration = (int) (System.currentTimeMillis() - startTime);
                    telemetry.addData("Running Time: ", "%7d milliseconds", duration);
                    telemetry.update();
            }
                if (timeoutmilli > duration && opModeIsActive()) {                                  //This checks if the current time exceeds the timeout period
                    telemetry.addData("Running Status: ", "Finished");                //If the current time is outside of the time allowed then the driver station will state that it is finished
                    telemetry.update();f
                    requestOpModeStop();                                                            //This stops the opmode because otherwise the robot would continue to run after 30 seconds and be disqualified
                    oppreborn.leftDrive.setPower(0);                                                //This stops both of the wheels in case turning off the opmode somehow didn't
                    oppreborn.rightDrive.setPower(0);
                    TimeKeeper.join();                                                              //This command kills the current thread which obviously stops it from running.
                } else {
                    TimeKeeper.join();
                }
            } catch (InterruptedException e) {
                telemetry.addData("TimeKeeper", "Had an error");                      //This tells the driver that the TimeKeeper had an error if it is given an InterruptedException.
            }

        }

    });
    */
    private synchronized void Dismount() {
        oppreborn.liftnLower.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        oppreborn.liftnLower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        runtime.reset();
        while (opModeIsActive() && runtime.milliseconds()<3250) {
            oppreborn.liftnLower.setPower(0.6);
            idle();
        }
        oppreborn.liftnLower.setPower(0);
        runtime.reset();
        while (opModeIsActive() && runtime.milliseconds()<700) {
            oppreborn.midDrive.setPower(.7);
            idle();
        }
        oppreborn.midDrive.setPower(0);
    }

}