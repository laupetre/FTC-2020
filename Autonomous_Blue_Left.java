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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Blue-Left", group="Pushbot")
//@Disabled
public class Autonomous_Blue_Left extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    Commands commands = new Commands();
    RingDetector ringDetector = new RingDetector();

    int rings = -1;

    @Override
    public void runOpMode() throws InterruptedException {

        commands.init(hardwareMap);
        ringDetector.init(hardwareMap);

        //detect the rings
        rings = ringDetector.ringDectorLoop();

        telemetry.addData("rings : ", rings);
        telemetry.update();

        waitForStart();
        runtime.reset();

        while(opModeIsActive()) {

            commands.wobbleGoalClaw(0);
            sleep(1000);
            commands.wobbleGoalClaw(0.5);
            sleep(1000);
            commands.woableGoalElevator();
            sleep(30000);

            //face forward
            commands.rightRearPivot(-10, 0.5, 3);
            sleep(100);

            //move to A B or C - 0 1 4
            if (rings == 0){
                //move forward shoot turn CW move back, drop wobble slide right forward slide felt park
                commands.moveForward(36,0.5,5);
                sleep(100);
                commands.rotateClockwise(9,0.5,3);
                sleep(100);
                commands.moveBackwards(20,0.5,4);
                sleep(100);
                //drop wobble

                commands.slideRight(24,0.5,5);
                sleep(100);
                commands.moveForward(48,0.5,6);
                sleep(100);
                //park
                commands.slideLeft(30,0.5,5);

                sleep(30000);

            }
            else if (rings == 1){
                //move forward, shoot rings, turn clockwise, move back, drop off wobble, slide right, move forward, slide left and park
                sleep(100);
                commands.moveForward(36,0.5,5);
                //shoot rings
                sleep(100);
                commands.rotateClockwise(9,0.5,3);
                sleep(100);
                commands.moveBackwards(20,0.5,4);
                sleep(100);
                //drop off wobble
                commands.slideRight(24,0.5,5);
                sleep(100);
                commands.moveForward(48,0.5,6);
                sleep(100);
                commands.slideLeft(30,0.5,5);
                //park on line

                sleep(30000);

            }
            else {
                //move forward shoot forward slide left drop wobble slide right move backward
                commands.moveForward(40,0.5,6);
                sleep(100);
                //shoot

                commands.moveForward(32,0.5,5);
                sleep(100);
                commands.slideLeft(24,0.5,4);
                sleep(100);
                //drop wobble

                commands.slideRight(20,0.5,5);
                sleep(100);
                //park
                commands.moveBackwards(62,0.5,6);
                sleep(30000);
            }


            telemetry.addData("rings : ", rings);
            telemetry.update();

            telemetry.addData("leftFront : ", commands.leftFront.getCurrentPosition());
            telemetry.addData("leftRear : ", commands.leftRear.getCurrentPosition());
            telemetry.addData("rightFront : ", commands.rightFront.getCurrentPosition());
            telemetry.addData("rightRear : ", commands.rightRear.getCurrentPosition());
            telemetry.addData("rings : ", ringDetector.ringNumber());
            telemetry.update();

        }


    }


}
