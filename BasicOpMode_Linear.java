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

package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Tank_Drive", group="GameCode")

public class BasicOpMode_Linear extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor Left_Drive_Motor= null;
    private DcMotor Right_Drive_Motor = null;
    private DcMotor Right_Lift_Motor = null;
    private DcMotor Left_Lift_Motor = null;
    private DcMotor Lift_Angle_Motor = null;
    private DcMotor Flywheel_Motor = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        Left_Drive_Motor  = hardwareMap.get(DcMotor.class, "Left_Drive_Motor");
        Right_Drive_Motor = hardwareMap.get(DcMotor.class, "Right_Drive_Motor");
        Right_Lift_Motor = hardwareMap.get(DcMotor.class, "Right_Lift_Motor" );
        Left_Lift_Motor = hardwareMap.get(DcMotor.class, "Left_Lift_Motor");
        Lift_Angle_Motor = hardwareMap.get(DcMotor.class, "Lift_Angle_Motor");
        Flywheel_Motor = hardwareMap.get(DcMotor.class, "Flywheel_Motor");


        // Reverse the motor that runs backwards when connected directly to the battery
        Right_Drive_Motor.setDirection(DcMotor.Direction.FORWARD);
        Left_Drive_Motor.setDirection(DcMotor.Direction.REVERSE);

        //Set Start power to motors
        Left_Lift_Motor.setPower(0);
        Left_Drive_Motor.setPower(0);
        Right_Drive_Motor.setPower(0);
        Right_Lift_Motor.setPower(0);
        Lift_Angle_Motor.setPower(0);
        Flywheel_Motor.setPower(0);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double DriveLeftPower;
            double DriveRightPower;
            double LiftLeftPower;
            double LiftRightPower;

            // set power for each motor
            DriveLeftPower  = -gamepad1.left_stick_y - .2;
            DriveRightPower = -gamepad1.right_stick_y - .2;
            LiftLeftPower = -gamepad2.left_stick_y - .2;
            LiftRightPower = -gamepad2.right_stick_y - .2;


            // Send calculated power to wheels
            Left_Drive_Motor.setPower(DriveLeftPower);
            Right_Drive_Motor.setPower(DriveRightPower);

            //Send Calculated power to lifts
            Left_Lift_Motor.setPower(LiftLeftPower - .2);
            Right_Lift_Motor.setPower(LiftRightPower - .2);

            //Lift Angle Motor power bond to triggers
            if (gamepad1.right_trigger == 1){
                Lift_Angle_Motor.setPower(.5);
            }
            else{
                Lift_Angle_Motor.setPower(0);
            }
            if (gamepad1.left_trigger == 1){
                Lift_Angle_Motor.setPower(-.5);
            }
            else{
                Lift_Angle_Motor.setPower(0);
            }

            //A and B button toggle for fly wheel moors.
            double lifton;
            double reversefly;
            lifton = 0;
            reversefly = 0;
            if (gamepad2.a){
                lifton = 1;
            }
            if (gamepad2.b){
                lifton = 0;
            }
            if (gamepad2.x){
                reversefly = 1;
            }
            if (lifton == 1){
                Flywheel_Motor.setPower(.8);
            }
            if (lifton == 0){
                Flywheel_Motor.setPower(0);
            }
            if (lifton == 0 && reversefly ==1 ){
                Flywheel_Motor.setPower(-.8);
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", DriveLeftPower, DriveRightPower);
            telemetry.update();
        }
    }
}