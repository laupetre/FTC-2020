package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Commands extends HardwareMapping{

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    private ElapsedTime runtime = new ElapsedTime();


    public void moveForward(float distance, double power, int timeout){

        int newTarget;

        //reset the current time
        runtime.reset();

        newTarget =  (int)(distance * COUNTS_PER_INCH);

        leftFront.setTargetPosition(newTarget);
        leftRear.setTargetPosition(newTarget);
        rightFront.setTargetPosition(newTarget);
        rightRear.setTargetPosition(newTarget);

        // Turn On RUN_TO_POSITION
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(power);
        leftRear.setPower(power);
        rightFront.setPower(power);
        rightRear.setPower(power);

        while ( ( runtime.seconds() < timeout ) &&
                (leftFront.isBusy() || rightFront.isBusy() || leftRear.isBusy() || rightRear.isBusy())
            ) {

        }


        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);

        //reset encoders
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void moveBackwards(float distance, double power, int timeout){

        int newTarget;

        //reset the current time
        runtime.reset();

        newTarget = (int)(distance * COUNTS_PER_INCH);

        leftFront.setPower(power);
        leftRear.setPower(power);
        rightFront.setPower(power);
        rightRear.setPower(power);

        leftFront.setTargetPosition(-newTarget);
        leftRear.setTargetPosition(-newTarget);
        rightFront.setTargetPosition(-newTarget);
        rightRear.setTargetPosition(-newTarget);

        // Turn On RUN_TO_POSITION
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while ( ( runtime.seconds() < timeout ) &&
                (leftFront.isBusy() || rightFront.isBusy() || leftRear.isBusy() || rightRear.isBusy())
        ) {

        }


        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);

        //reset encoders
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void slideLeft(float distance, double power, int timeout){

        int newTarget;

        //reset the time
        runtime.reset();

        newTarget = (int)(distance * COUNTS_PER_INCH);

        leftFront.setPower(power);
        leftRear.setPower(power);
        rightFront.setPower(power);
        rightRear.setPower(power);

        leftFront.setTargetPosition(-newTarget);
        leftRear.setTargetPosition(newTarget);
        rightFront.setTargetPosition(newTarget);
        rightRear.setTargetPosition(-newTarget);

        // Turn On RUN_TO_POSITION
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while ( ( runtime.seconds() < timeout ) &&
                (leftFront.isBusy() || rightFront.isBusy() || leftRear.isBusy() || rightRear.isBusy())
        ) {

        }


        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);

        //reset encoders
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void slideRight(float distance, double power, int timeout){

        int newTarget;

        //reset the time
        runtime.reset();

        newTarget = (int)(distance * COUNTS_PER_INCH);

        leftFront.setPower(power);
        leftRear.setPower(power);
        rightFront.setPower(power);
        rightRear.setPower(power);

        leftFront.setTargetPosition(newTarget);
        leftRear.setTargetPosition(-newTarget);
        rightFront.setTargetPosition(-newTarget);
        rightRear.setTargetPosition(newTarget);

        // Turn On RUN_TO_POSITION
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while ( ( runtime.seconds() < timeout ) &&
                (leftFront.isBusy() || rightFront.isBusy() || leftRear.isBusy() || rightRear.isBusy())
        ) {

        }


        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);

        //reset encoders
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }

    public void rotateClockwise(float distance, double power, int timeout){

        int newTarget;

        //reset the time
        runtime.reset();

        newTarget = (int)(distance * COUNTS_PER_INCH);

        leftFront.setPower(power);
        leftRear.setPower(power);
        rightFront.setPower(power);
        rightRear.setPower(power);

        leftFront.setTargetPosition(newTarget);
        leftRear.setTargetPosition(newTarget);
        rightFront.setTargetPosition(-newTarget);
        rightRear.setTargetPosition(-newTarget);

        // Turn On RUN_TO_POSITION
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while ( ( runtime.seconds() < timeout ) &&
                (leftFront.isBusy() || rightFront.isBusy() || leftRear.isBusy() || rightRear.isBusy())
        ) {

        }


        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);

        //reset encoders
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }

    public void rotateCounterClockwise(float distance, double power, int timeout){

        int newTarget;

        //reset the time
        runtime.reset();

        newTarget = (int)(distance * COUNTS_PER_INCH);

        leftFront.setPower(power);
        leftRear.setPower(power);
        rightFront.setPower(power);
        rightRear.setPower(power);

        leftFront.setTargetPosition(-newTarget);
        leftRear.setTargetPosition(-newTarget);
        rightFront.setTargetPosition(newTarget);
        rightRear.setTargetPosition(newTarget);

        // Turn On RUN_TO_POSITION
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while ( ( runtime.seconds() < timeout ) &&
                (leftFront.isBusy() || rightFront.isBusy() || leftRear.isBusy() || rightRear.isBusy())
        ) {

        }


        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);

        //reset encoders
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



    }

    /**
     *
     * @param distance positive distance is CCW
     * @param power
     * @param timeout
     */
    public void leftRearPivot(float distance, double power, int timeout){

        int newTarget;

        //reset the time
        runtime.reset();

        newTarget = (int)(distance * COUNTS_PER_INCH);

        rightFront.setPower(power);
        rightRear.setPower(power);

        rightFront.setTargetPosition(newTarget);
        rightRear.setTargetPosition(newTarget);

        // Turn On RUN_TO_POSITION
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while ( ( runtime.seconds() < timeout ) &&
                (rightFront.isBusy() || rightRear.isBusy())
        ) {

        }

        rightFront.setPower(0);
        rightRear.setPower(0);

        //reset encoders
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     *
     * @param distance positive distance is CW
     * @param power
     * @param timeout
     */
    public void rightRearPivot(float distance, double power, int timeout){

        int newTarget;

        //reset the time
        runtime.reset();

        newTarget = (int)(distance * COUNTS_PER_INCH);

        leftFront.setPower(power);
        leftRear.setPower(power);

        leftFront.setTargetPosition(newTarget);
        leftRear.setTargetPosition(newTarget);

        // Turn On RUN_TO_POSITION
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while ( ( runtime.seconds() < timeout ) &&
                (leftFront.isBusy() || leftRear.isBusy())
        ) {

        }

        leftFront.setPower(0);
        leftRear.setPower(0);

        //reset encoders
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
