package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import static java.lang.Math.abs;

@TeleOp ( name = "Moving" , group = "" )
public class Moving extends LinearOpMode {


    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");


        // Put initialization blocks here.
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

        double threshold = 0.99;
        double RightStickY;
        double RightStickX;
        double LeftStickX;

        //waitForStart();

        while (opModeIsActive()) {
            {
                RightStickY = 0.75 * gamepad1.right_stick_y;
                RightStickX = 0.75 * gamepad1.right_stick_x;
                LeftStickX = 0.75 * gamepad1.left_stick_x;


                if (abs(gamepad1.right_stick_y) > threshold || abs(gamepad1.right_stick_x) > threshold) {
                    //[rightFront] = (gamepad1.right_stick_y - gamepad1.right_stick_x);
                    rightFront.setPower(RightStickY - RightStickX);
                    //[leftFront] = (-gamepad1.right_stick_y - gamepad1.right_stick_x);
                    leftFront.setPower(-RightStickY - RightStickX);
                    //[rightRear] = (-gamepad1.right_stick_y - gamepad1.right_stick_x);
                    rightRear.setPower(-RightStickY - RightStickX);
                    //DcMotor[leftRear] = (gamepad1.right_stick_y - gamepad1.right_stick_x);
                    leftRear.setPower(RightStickY - RightStickX);
                }
                if (abs(gamepad1.left_stick_x) > threshold) {
                    /*
                    rotate
                    [rightFront] = (-gamepad1.left_stick_x);
                    */
                    rightFront.setPower(-LeftStickX);
                    //[leftFront] = (-gamepad1.left_stick_x);
                    leftFront.setPower(-LeftStickX);
                    //[rightRear] =(gamepad1.left_stick_x);
                    rightRear.setPower(LeftStickX);
                    //[leftRear] = (gamepad1.left_stick_x);
                    leftRear.setPower(LeftStickX);
                }
            }
        }
    }
}