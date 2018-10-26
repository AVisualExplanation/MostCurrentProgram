package org.firstinspires.ftc.teamcode.MostCurrentProgram;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@Autonomous(name="AutoOpModeFacing Crater Prototype", group="Autonomous")
public class MyFIRST_Prototype extends OpMode {
    public HardwarePushbot oppreborn = new HardwarePushbot();
    public BNO055IMU imu;
    public Orientation angles;
    @Override
    public void init() {

        final double DRIVE_SPEED = 0.3;                                                         // Nominal speed for better accuracy.
        final double DROP_SPEED = 0.4;                                                          //This was created as a precaution to ensure that the robot didn't drop down too quickly
        final double TURN_SPEED = 0.3;                                                          // Nominal half speed for better accuracy.
    }
    @Override
    public void loop(){
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
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;                                                 //This puts the IMU into sensor mode as opposed to config mode
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;                                         //This sets the unit for the IMU's angle to be in degrees
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;                            //This sets the unit for the IMU's acceleration to be meters per second squared
        parameters.loggingEnabled = false;                                                          //This turns off logging and allows for the use of the IMU as an input to guide the robots movements
        parameters.accelerationIntegrationAlgorithm = null;                                         //This is an algorithm that can use acceleration in order to find velocity and position using integral calculus
        imu = hardwareMap.get(BNO055IMU.class, "imu");                                   //This establishes the use of the imu under the hardware map to just be referenced as "imu". Most importantly, it means that the robot configuration in the expansion hub will refer to the port where the imu is located as "imu".
        imu.initialize(parameters);                                                                 //This initializes the parameters (moves the parameters specified to be associated with the imu)


        try {
            Dismount();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //This establishes that when I ask for angles, I am wanting the Extrinsic angles listed in degrees in the format of ZYX.

        telemetry.addData("Path", "Complete");                                        //This sends the driver station phone the message that the robot has completed all of its necessary paths
        telemetry.update();
    }
    private synchronized void IMUDrive(double speed, double inches, double angle) {
        Rotate(angle);
        int newLeftTarget;
        int newRightTarget;
        if (true) {

            oppreborn.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            oppreborn.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Determine new target position, and pass to motor controller
            newLeftTarget = oppreborn.leftDrive.getCurrentPosition() + (int) (inches * 201.2283); //This gets the wheels current position, and adds the number of inches forward desired. But, remember that the rover moves in terms of counts, so it translates the number of inches into the number of counts.
            newRightTarget = oppreborn.rightDrive.getCurrentPosition() + (int) (inches * 201.2283);
            oppreborn.leftDrive.setTargetPosition(newLeftTarget);                                   //This sets the new target equal to the distance defined above in terms of counts
            oppreborn.rightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            oppreborn.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);                           //This makes the motors both begin to drive to their desired position as defined below.
            oppreborn.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            oppreborn.leftDrive.setPower(Math.abs(speed));                                          //This sets the wheel speed equal to the speed defined before
            oppreborn.rightDrive.setPower(Math.abs(speed));
            oppreborn.midDrive.setPower(0);

            while ((oppreborn.leftDrive.isBusy() && oppreborn.rightDrive.isBusy())){

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
                        oppreborn.leftDrive.getCurrentPosition()/201.2283,
                        oppreborn.rightDrive.getCurrentPosition()/201.2283);
                telemetry.update();
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
        oppreborn.placeMarker.setPosition(1);
    }


    //(Created by "Samuel Tukua","26/09/2018", "edit #5", "Finished the rotate method, but still need to check it", "NEEDEDIT checking that the thirdangle is the one that I want to use")
    private synchronized void Rotate(double dsrangle) {                                             //This method will be called upon in order to rotate the rover using the gyro
        if (true) {
            oppreborn.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            oppreborn.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES);
            if (angles.firstAngle > dsrangle) {                                                          //This checks to see if the current angle is greater than the desired angle, "dsrangle", and if so, it will tell the robot that it needs to Rotate until the angles are equal
                while (angles.firstAngle > dsrangle){                                                   //This stops when the current angle equals the desired angle or if the current time exceeds the 30 seconds that the match is allowed to take.
                    oppreborn.leftDrive.setPower(0.3);                                       //This rotation results in the rover turning in a clockwise fashion which in euclidean angles means that it's rotation is approaching -180 degrees.
                    oppreborn.rightDrive.setPower(-0.3);
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES);
                    telemetry.addData("Rotation at","%7d of %7d", Math.round(angles.firstAngle) , Math.round(dsrangle));
                    telemetry.update();
                }
                oppreborn.leftDrive.setPower(0);                                                    //This makes sure that the wheels have stopped spinning once the robot has finished its rotation
                oppreborn.rightDrive.setPower(0);
            } else if (angles.firstAngle < dsrangle) {                                                   //This checks to see if the current angle is less than the desired angle, "dsrangle", and if so, it will tell the robot that it needs to Rotate until the angles are equal.
                while (angles.firstAngle < dsrangle) {                                                   //This setup does the  same as the previous setup but instead does it in a counter clockwise fashion in order to rotate the robot towards the positive 180 degrees section
                    oppreborn.leftDrive.setPower(-0.3);
                    oppreborn.rightDrive.setPower(0.3);
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    telemetry.addData("Rotation at","%7d of %7d", Math.round(angles.firstAngle) , Math.round(dsrangle));
                    telemetry.update();
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
    private synchronized void Dismount() throws InterruptedException {
        oppreborn.liftnLower.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        oppreborn.liftnLower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        oppreborn.liftnLower.setPower(0.4);
        wait(4500);
        oppreborn.liftnLower.setPower(0);
        oppreborn.midDrive.setPower(.7);
        wait(1000);
        oppreborn.midDrive.setPower(0);
    }


}

