package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.apache.commons.math3.analysis.function.Power;
import org.firstinspires.ftc.teamcode.DefineRobot.PowerPlayBot;
import static org.firstinspires.ftc.teamcode.DefineRobot.PowerPlayBot.INCREMENT;
import static org.firstinspires.ftc.teamcode.DefineRobot.PowerPlayBot.CYCLE_MS;
import static org.firstinspires.ftc.teamcode.DefineRobot.PowerPlayBot.AMAX_POS;
import static org.firstinspires.ftc.teamcode.DefineRobot.PowerPlayBot.AMIN_POS;
import static org.firstinspires.ftc.teamcode.DefineRobot.PowerPlayBot.BMAX_POS;
import static org.firstinspires.ftc.teamcode.DefineRobot.PowerPlayBot.BMIN_POS;



/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class LocalizationTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        PowerPlayBot ppb = new PowerPlayBot(this, hardwareMap);

        try {
            ppb.init();
        } catch (Exception e) {
            e.printStackTrace();
        }


        ppb.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested()) {
            ppb.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.right_stick_x,
                            -gamepad1.left_stick_x
                    )
            );

            ppb.clawPosition();

            ppb.update();

            Pose2d poseEstimate = ppb.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}
