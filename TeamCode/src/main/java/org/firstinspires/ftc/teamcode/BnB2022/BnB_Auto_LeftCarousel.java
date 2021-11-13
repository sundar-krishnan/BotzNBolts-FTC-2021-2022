/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.BnB2022;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.teamcode.HardwareMap.HardwareMap_Example;

//import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
// List where other files are located that are used in this OpMode

/**
 * In this example:
 * This file illustrates the concept of driving OUR robot in HardwareMap_Example
 *
 */
// CHAWKS: Name it something useful!
@Autonomous(name="BnB_Auto_LeftCarousel", group="RedTest")
// CHAWKS: What does @Disabled mean? what happens if we remove it?
//@Disabled
public class BnB_Auto_LeftCarousel extends LinearOpMode {

    /* CHAWKS: Call and declare the robot here */
//    HardwareMap_Example     robot   = new HardwareMap_Example();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor throwerDrive = null;
    private DcMotor ringrollerDrive = null;
    private Servo leftGrabberServo = null;
    private Servo rightGrabberServo = null;
    private Servo grabberServo = null;
    private Servo collectorServo = null;
    private DcMotor armLifter = null;


    /*
        CHAWKS: All the values can be moved to HardwareMap? Are these common values?
     */
//    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder 60:1
//    static final double     COUNTS_PER_MOTOR_REV    = 960 ;    // eg: TETRIX Motor Encoder 40:1
    static final double     COUNTS_PER_MOTOR_REV    = 480 ;    // eg: TETRIX Motor Encoder 20:1
    static final double     DRIVE_GEAR_REDUCTION    = 0.05 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.85 ;     // For figuring circumference
//    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
//                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     QTR_TURN         = (COUNTS_PER_MOTOR_REV) / 4;

    static final double     DRIVE_SPEED             = 0.03;
    static final double     TURN_SPEED              = 0.03;
    static final String     LEFT                    = "LEFT";
    static final String     RIGHT                   = "RIGHT";
    static final String     FORWARD                 = "FORWARD";
    static final String     BACKWARD                = "BACKWARD";
    static final String     LEFTSLIDE               = "LEFTSLIDE";
    static final String     RIGHTSLIDE              = "RIGHTSLIDE";
    private double servoPosition = 0.0;
    static final double INCREMENT = 0.05;     // amount to slew servo each CYCLE_MS cycle
    static final double INCREMENTWRIST = 0.1;
    static final int    CYCLE_MS  =  100;     // period of each cycle
    static final double MAX_POS   =  1.0;     // Maximum rotational position
    static final double MIN_POS   =  0.35;     // Minimum rotational position
    static final double MAX_POS_WRIST   =  0.3;     // Maximum rotational position
    static final double MIN_POS_WRIST    =  0.8;     // Minimum rotational position
    double speedAdjust =7.0;
    double  position = 0.0; //(MAX_POS - MIN_POS) / 2; // Start at halfway position
    double  positionWrist = 5.0;
    double armMotorPower=1.0;
    int targetPosition = 0;
    double drivePower = 0.5;


    /*
        CHAWKS: It has begun!!! Run the OpMode!!! Make the robot execute all our code!!!
    */

    // MUST HAVE
    @Override
    public void runOpMode() {
         // Initialize the drive system variables.
         // The init() method of the hardware class does all the work here
        /*
            CHAWKS: On Driver Station, telemetry will be display!
                    Why is this good for the Drivers?
        */
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status: ", "Hit [Init] to Initialize ze bot");    //
        telemetry.update();

        /*
            CHAWKS: Step 0. Initialize OUR ROBOT
        */
        // MUST HAVE THIS LINE BELOW
        //robot.init(hardwareMap);
        initializeDriveMotor();
        initializeGrabberServoMotor();
        initializeRingRollerMotor();
        // Send telemetry message to "Driver Station" signify robot waiting;
        telemetry.addData("Status: ", "Hit [PLAY] to start!");    //
        telemetry.update();

        /*
            CHAWKS: Step 1. Hit Play to run through the code!
        */
        // MUST HAVE THIS LINE BELOW
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
//        encoderDrive(FORWARD, DRIVE_SPEED,  12,  12, 1.5);  // S1: Forward 47 Inches with 5 Sec timeout
//        encoderDrive(LEFT, TURN_SPEED, 12, 12, 5.0);  // S2: Turn Right 12 Inches with 4 Sec timeout

        encoderDrive(BACKWARD, DRIVE_SPEED, 2, 2, 1);  // S3: Reverse 24 Inches with 4 Sec timeout
        sleep(200);     // pause
        encoderDrive(LEFT, TURN_SPEED,  5,  5, 1.5);  // S1: Forward 47 Inches with 5 Sec timeout
        sleep(100);     // pause
        encoderDrive(BACKWARD, DRIVE_SPEED, 12, 12, 2.5);  // S3: Reverse 24 Inches with 4 Sec timeout
        sleep(20);     // pause
        RingRollerDrive(5);
        sleep(400);     // pause
//        encoderDrive(LEFT, TURN_SPEED,  12,  12, 1);  // S1: Forward 47 Inches with 5 Sec timeout
//        sleep(1000);     // pause
////        throwDrive(5);
//        encoderDrive(RIGHT, TURN_SPEED,  12,  12, 1);  // S1: Forward 47 Inches with 5 Sec timeout
//        sleep(500);     // pause
////        throwDrive(5);
//        encoderDrive(RIGHT, TURN_SPEED,  12,  12, 1);  // S1: Forward 47 Inches with 5 Sec timeout
//        sleep(500);     // pause
//        throwDrive(5);
 //       encoderDrive(BACKWARD, TURN_SPEED,  12,  12, 1.5);  // S1: Forward 47 Inches with 5 Sec timeout

        telemetry.addData("Path", "Complete!");
        telemetry.update();
    }

    /*
     *  CHAWKS: It's a METHOD!!!
     *
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive( String direction, double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftFrontTarget;
        int newLeftBackTarget;
        int newRightFrontTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        /*
            CHAWKS: opModeIsActive is a very awesome & important
                    Autonomous mode is 30 seconds...
         */
        if (opModeIsActive()) {

           //reset all encoder values
            leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Determine new target position, and pass to motor controller
            newLeftBackTarget = leftBackDrive.getCurrentPosition() + (int)(leftInches * Math.abs(COUNTS_PER_INCH));
            newLeftFrontTarget = leftFrontDrive.getCurrentPosition() + (int)(leftInches * Math.abs(COUNTS_PER_INCH));
            newRightBackTarget = rightBackDrive.getCurrentPosition() + (int)(rightInches * Math.abs(COUNTS_PER_INCH));
            newRightFrontTarget = rightFrontDrive.getCurrentPosition() + (int)(rightInches * Math.abs(COUNTS_PER_INCH));

//            newLeftBackTarget = leftBackDrive.getCurrentPosition() + (int)(QTR_TURN);
//            newLeftFrontTarget = leftFrontDrive.getCurrentPosition() + (int)(QTR_TURN);
//            newRightBackTarget = rightBackDrive.getCurrentPosition() + (int)(QTR_TURN);
//            newRightFrontTarget = rightFrontDrive.getCurrentPosition() + (int)(QTR_TURN);

//            newLeftBackTarget =  (int)(QTR_TURN);
//            newLeftFrontTarget = (int)(QTR_TURN);
//            newRightBackTarget = (int)(QTR_TURN);
//            newRightFrontTarget = (int)(QTR_TURN);

//
            // reset the timeout time and start motion.
            runtime.reset();
            switch (direction)
            {
                case LEFT:
                    leftBackDrive.setTargetPosition(newLeftBackTarget * -1);
                    rightBackDrive.setTargetPosition(newRightBackTarget);
                    leftFrontDrive.setTargetPosition(newLeftFrontTarget * -1);
                    rightFrontDrive.setTargetPosition(newRightFrontTarget);

//                    leftBackDrive.setPower(Math.abs(speed));
//                    rightBackDrive.setPower(Math.abs(speed));
//                    leftFrontDrive.setPower(Math.abs(speed));
//                    rightFrontDrive.setPower(Math.abs(speed));

                    break;
                case RIGHT:
                    leftBackDrive.setTargetPosition(newLeftBackTarget);
                    rightBackDrive.setTargetPosition(newRightBackTarget * -1);
                    leftFrontDrive.setTargetPosition(newLeftFrontTarget);
                    rightFrontDrive.setTargetPosition(newRightFrontTarget * -1);

//                    leftBackDrive.setPower(Math.abs(speed));
//                    rightBackDrive.setPower(Math.abs(speed));
//                    leftFrontDrive.setPower(Math.abs(speed));
//                    rightFrontDrive.setPower(Math.abs(speed));

                    break;
                case FORWARD:
                    leftBackDrive.setTargetPosition(newLeftBackTarget);
                    rightBackDrive.setTargetPosition(newRightBackTarget);
                    leftFrontDrive.setTargetPosition(newLeftFrontTarget);
                    rightFrontDrive.setTargetPosition(newRightFrontTarget);

//                    leftBackDrive.setPower(Math.abs(speed));
//                    rightBackDrive.setPower(Math.abs(speed));
//                    leftFrontDrive.setPower(Math.abs(speed));
//                    rightFrontDrive.setPower(Math.abs(speed));

                    break;

                case BACKWARD:
                    leftBackDrive.setTargetPosition(newLeftBackTarget * -1);
                    rightBackDrive.setTargetPosition(newRightBackTarget * -1);
                    leftFrontDrive.setTargetPosition(newLeftFrontTarget * -1);
                    rightFrontDrive.setTargetPosition(newRightFrontTarget * -1);

//                    leftBackDrive.setPower(Math.abs(speed));
//                    rightBackDrive.setPower(Math.abs(speed));
//                    leftFrontDrive.setPower(Math.abs(speed));
//                    rightFrontDrive.setPower(Math.abs(speed));

                    break;
                case RIGHTSLIDE:
                    leftBackDrive.setTargetPosition(newLeftBackTarget);
                    rightBackDrive.setTargetPosition(newRightBackTarget * -1);
                    leftFrontDrive.setTargetPosition(newLeftFrontTarget * -1);
                    rightFrontDrive.setTargetPosition(newRightFrontTarget);

//                    leftBackDrive.setPower(Math.abs(speed));
//                    rightBackDrive.setPower(Math.abs(speed));
//                    leftFrontDrive.setPower(Math.abs(speed));
//                    rightFrontDrive.setPower(Math.abs(speed));

                    break;
                case LEFTSLIDE:
                    leftBackDrive.setTargetPosition(newLeftBackTarget * -1);
                    rightBackDrive.setTargetPosition(newRightBackTarget);
                    leftFrontDrive.setTargetPosition(newLeftFrontTarget);
                    rightFrontDrive.setTargetPosition(newRightFrontTarget * -1);

//                    leftBackDrive.setPower(Math.abs(speed));
//                    rightBackDrive.setPower(Math.abs(speed));
//                    leftFrontDrive.setPower(Math.abs(speed));
//                    rightFrontDrive.setPower(Math.abs(speed));

                    break;
            }


            // Turn On RUN_TO_POSITION
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftBackDrive.setPower(Math.abs(speed));
            rightBackDrive.setPower(Math.abs(speed));
            leftFrontDrive.setPower(Math.abs(speed));
            rightFrontDrive.setPower(Math.abs(speed));

//              sleep(5000);   // optional pause after each move
//
                // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (leftBackDrive.isBusy() && rightBackDrive.isBusy() && leftFrontDrive.isBusy() && rightFrontDrive.isBusy()))
//            while (leftBackDrive.isBusy() &&
//                       (runtime.seconds() < timeoutS))
            {
                // Display it for the driver.
                telemetry.addData("isbusy",  "right back and left back" + rightBackDrive.isBusy() + " " +  leftBackDrive.isBusy());
                telemetry.addData("Target",  "Back Running to %7d :%7d", newLeftBackTarget,  newRightBackTarget);
                telemetry.addData("Back Current",  "Back Running at %7d :%7d",
                        leftBackDrive.getCurrentPosition(),
                        rightBackDrive.getCurrentPosition());
                telemetry.addData("Back Target",  "Back Running at %7d :%7d",
                        leftBackDrive.getTargetPosition(),
                        rightBackDrive.getTargetPosition());
                telemetry.addData("Target",  "Front Running to %7d :%7d", newLeftFrontTarget,  newRightFrontTarget);
                telemetry.addData("Front Current",  "Front Running at %7d :%7d",
                        leftFrontDrive.getCurrentPosition(),
                        rightFrontDrive.getCurrentPosition());
                telemetry.addData("Front Target",  "Back Running at %7d :%7d",
                        leftFrontDrive.getTargetPosition(),
                        rightFrontDrive.getTargetPosition());
                telemetry.addData("Speed",  "right back and left back " + rightBackDrive.getPower() + " " +  leftBackDrive.getPower());
                telemetry.addData("Speed",  "right back and left back " + rightFrontDrive.getPower() + " " +  rightBackDrive.getPower());
                telemetry.addData("Speed",  "Direction " + direction);
                telemetry.update();
            }

            // Stop all motion;
            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //  sleep(250);   // optional pause after each move
        }
    }



    private void initializeDriveMotor()
    {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "LeftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RightFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "LeftBackDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "RightBackDrive");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Turn off RUN_TO_POSITION
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    private void initializeGrabberServoMotor()
    {
        leftGrabberServo = hardwareMap.get(Servo.class, "LeftGrabber");
        leftGrabberServo.setDirection(Servo.Direction.FORWARD);
        leftGrabberServo.setPosition(MIN_POS);

        rightGrabberServo = hardwareMap.get(Servo.class, "RightGrabber");
        rightGrabberServo.setDirection(Servo.Direction.REVERSE);
        rightGrabberServo.setPosition(MIN_POS);

    }


    private void initializeRingRollerMotor()
    {
        ringrollerDrive = hardwareMap.get(DcMotor.class, "RingRoller");
        ringrollerDrive.setDirection(DcMotor.Direction.FORWARD);
        ringrollerDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ringrollerDrive.setPower(0.0);
    }

    private void RingRollerDrive(double timeoutS)
    {
        ringrollerDrive.setPower(1.0);
        if (opModeIsActive())
        {
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < timeoutS))
            {
                telemetry.addData("ringrollerDrive", "ringrollerDIR: " + ringrollerDrive.getDirection() + " ringrollerDrive " + ringrollerDrive.getDirection());
                telemetry.update();
            }
            ringrollerDrive.setPower(0);
        }

    }

}
