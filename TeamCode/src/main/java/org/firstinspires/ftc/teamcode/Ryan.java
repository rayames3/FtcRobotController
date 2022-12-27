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
import com.qualcomm.robotcore.robot.Robot;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Gyroscope;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import java.io.*;
import java.lang.Thread;
/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Ryan", group="Iterative Opmode")

public class Ryan extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime period  = new ElapsedTime();
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private DcMotor intake_1 = null;
    private DcMotor intake_2 = null;
    private DcMotor lift = null;
    //private Gyroscope imu = null;
    int lifter = 0;
    private BNO055IMU   imu;
    Orientation angle = new Orientation();
    Orientation lastAngles = new Orientation();
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack  = hardwareMap.get(DcMotor.class, "leftRear");
        rightBack = hardwareMap.get(DcMotor.class, "rightRear");
        intake_1 = hardwareMap.get(DcMotor.class, "intake_1");
        intake_2 = hardwareMap.get(DcMotor.class, "intake_2");
        lift = hardwareMap.get(DcMotor.class, "spinner");
        //imu = hardwareMap.get(Gyroscope.class, "imu");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        //rightBack.setDirection(DcMotor.Direction.REVERSE);
        //leftFront.setDirection(DcMotor.Direction.REVERSE);
        //Arm.setDirection(DcMotor.Direction.FORWARD);
        //arm_1.setDirection(Servo.Direction.REVERSE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //spinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        intake_1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

        double intakePower;

        double y =gamepad1.left_stick_y;
        double x =-gamepad1.left_stick_x;
        double angleX =-gamepad1.right_stick_x;
        double outtakePower;
        boolean intake = gamepad1.right_bumper;
        boolean outtake = gamepad1.left_bumper;
        double spinner_f = gamepad1.left_trigger;
        double spinner_r = gamepad1.right_trigger;
        boolean a = gamepad1.a;
        boolean b = gamepad1.b;
        boolean arm = gamepad1.x;
        boolean arm2 = gamepad1.y;
        boolean top = gamepad1.dpad_up;
        boolean mid = gamepad1.dpad_right;
        boolean bottom = gamepad1.dpad_down;
        boolean reset = gamepad1.dpad_left;
        angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double i= angle.firstAngle;

        drive(x, y, angleX);

        if (intake == true) {
            intake_1.setPower(-1);
            intake_2.setPower(-1);
        } else if (outtake == true){
            intake_1.setPower(1);
            intake_2.setPower(1);
        }else {
            intake_1.setPower(0);
            intake_2.setPower(0);
        }
        if(spinner_f> 1.0){
            lift.setPower(1.0);
        }
        if(spinner_r>1.0){
            lift.setPower(-1.0);
        }

        if(arm == true){
            lift.setTargetPosition(185);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(1.0);
        } else {
            lift.setTargetPosition(0);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(1.0);
        }


        telemetry.addData("Path1",  "Running at %d", lift.getCurrentPosition());
        //telemetry.addData("Path1","Running at %7f,%7f,%7f,%7f,%7f",
        //                i, v1, v2, v3, v4);
        telemetry.update();

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    public void drive(double x, double y, double z){
        double r = Math.hypot(x, y);
        double robotAngle = Math.atan2(y, x) - Math.PI / 4;
        double rightX = z;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;
        leftFront.setPower(-v1);
        rightFront.setPower(v2);
        leftBack.setPower(-v3);
        rightBack.setPower(v4);
    }


}