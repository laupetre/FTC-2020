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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Red-Left", group="Iterative Opmode")
//@Disabled
public class Autonomous_Red_Left extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    Commands commands = new Commands();
    RingDetector ringDetector = new RingDetector();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        //initialize hardware
        commands.init(hardwareMap);
        ringDetector.init(hardwareMap);

        //grab the wobble


 //       telemetry.addData("leftFront : ", commands.leftFront.getCurrentPosition());
  //      telemetry.addData("leftRear : ", commands.leftRear.getCurrentPosition());
  //      telemetry.addData("rightFront : ", commands.rightFront.getCurrentPosition());
  //      telemetry.addData("rightRear : ", commands.rightRear.getCurrentPosition());


    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        int rings = 0;

        //detect rings
        //move the robot to launch place
        //move the robot to landing place
        //move the robot to launch line
        rings = ringDetector.ringNumber(); // -1 0 1 4

        "Quad" "quad"

        //check if we detected the rings
        if ( rings == 0 || rings == 1 || rings == 4 ){

            //we can continue

        }
        else{
            //do something
        }

        //commands.moveForward(12,0.5,10);

        sleep(1000);

  //      commands.moveBackwards(12,0.5,10);
       // sleep(1000);

      //  commands.leftFront.setPower(0.5);
      //  commands.rightFront.setPower(0.5);

        // Show the elapsed game time and wheel power.
      //  telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("leftFront : ", commands.leftFront.getCurrentPosition());
        telemetry.addData("leftRear : ", commands.leftRear.getCurrentPosition());
        telemetry.addData("rightFront : ", commands.rightFront.getCurrentPosition());
        telemetry.addData("rightRear : ", commands.rightRear.getCurrentPosition());
        telemetry.addData("rings : ", rings);
        telemetry.update();

        //  telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    public void sleep(int milis){
        try {
            Thread.sleep(milis);
        } catch (Exception e){}
    }

}
