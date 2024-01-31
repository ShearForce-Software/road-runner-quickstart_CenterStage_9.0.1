package org.firstinspires.ftc.teamcode;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Blue Far Stack")
public class BlueFarStackAuto extends LinearOpMode {
    UniversalControlClass control = new UniversalControlClass(true, false, this);
    MecanumDrive drive;
    Pose2d startPose;
    Pose2d deliverToFloorPose;
    Pose2d deliverToBoardPose;
    Vector2d stackVec = new Vector2d(-56, 12);
    boolean aidanParallelTestEnabled = false;

    public void runOpMode() {
        startPose = new Pose2d(-35.5, 62.5, Math.toRadians(270));
        drive = new MecanumDrive(hardwareMap, startPose);
        control.Init(hardwareMap);
        control.HuskyLensInit();
        control.AutoStartPos();

        telemetry.update();

        while (!isStarted()) {
            control.DetectTeamArtBlue();
            telemetry.update();
        }
        waitForStart();

        // lock the pixels
        control.GrabPixels();
        control.ReleaseLeft();

        // Drives to left, center, or right positions based on team art location.
        BlueRightTeamArtPixelDelivery();

        // Drop the LEFT pixel (put PURPLE on LEFT, YELLOW on RIGHT) on the line
        control.DropOnLine();
        // put the arm back in a safe to travel position
        control.ResetArmAuto();
        //control.SpecialSleep(10000);
        control.SpecialSleep(6000);
        control.ServoIntake();
        if(control.autoPosition == 1) {
            // drive to stack
            Actions.runBlocking(
                    drive.actionBuilder(deliverToFloorPose)
                            .strafeTo(new Vector2d(-36, 12))
                            .splineToConstantHeading(stackVec, Math.toRadians(180))
                            .build()
            );
        }
        else{
            Actions.runBlocking(
                    drive.actionBuilder(deliverToFloorPose)
                            .lineToY(10)
                            .splineToLinearHeading(new Pose2d(-38.5, 12, Math.toRadians(180)), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(-50, 12, Math.toRadians(180)), Math.toRadians(0))
                            .splineToConstantHeading(stackVec, Math.toRadians(180))
                            .build()
            );
        }
        control.AutoPickupRoutine();

        // drive to the backboard area
        drive.updatePoseEstimate();

        if (aidanParallelTestEnabled) {
            Actions.runBlocking(new ParallelAction(
                    stopSpinners(),
                    driveAcrossField()
                    ));
        }
        else {
            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(drive.pose.position.x, drive.pose.position.y, Math.toRadians(180)))
                            .setTangent(0)
                            .splineToLinearHeading(new Pose2d(-30, 9, Math.toRadians(180)), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(30, 9, Math.toRadians(180)), Math.toRadians(0))
                            .build());

        }



        // drive to the correct backboard spot based on the team art
        BlueBoardDelivery();

        // Return Arm to Ready position
        control.SlidesToAuto();
        sleep(150);
        control.DeliverPixelToBoardPos();
        control.StopNearBoardAuto(true);
        sleep(200);

        drive.updatePoseEstimate();
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(drive.pose.position.x, drive.pose.position.y, Math.toRadians(180)))
                        .lineToX(47)
                        .build());

        // Return Arm to Ready position
        control.ResetArm();
        sleep(400);

        // Move the robot to the parking position
        drive.updatePoseEstimate();
        Actions.runBlocking(
                //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
                drive.actionBuilder(new Pose2d(drive.pose.position.x, drive.pose.position.y, Math.toRadians(180)))
                        .splineToLinearHeading(new Pose2d(48, 15, Math.toRadians(270)), Math.toRadians(270))
                        .build());
        control.ServoStop();
        sleep(100);
        telemetry.update();
    }
    public void BlueBoardDelivery() {
        // Look for potential errors
        //***POSITION 1***
        if (control.autoPosition == 1) {
            deliverToBoardPose = new Pose2d(50,34.5,Math.toRadians(180));
            Actions.runBlocking(
                    //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
                    drive.actionBuilder(new Pose2d(30, 9, Math.toRadians(180)))
                            .setTangent(0)
                            .splineToLinearHeading(deliverToBoardPose, Math.toRadians(0))
                            .build());
        }
        //***POSITION 3***
        else if (control.autoPosition == 3) {
            deliverToBoardPose = new Pose2d(50,24.5,Math.toRadians(180));
            Actions.runBlocking(
                    //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.
                    // yawError, Math.toRadians(180)))
                    drive.actionBuilder(new Pose2d(30, 9, Math.toRadians(180)))
                            .setTangent(0)
                            .splineToLinearHeading(deliverToBoardPose, Math.toRadians(0))
                            .build());
        }
        //***POSITION 2***
        else {
            deliverToBoardPose = new Pose2d(50,29,Math.toRadians(180));
            Actions.runBlocking(
                    //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
                    drive.actionBuilder(new Pose2d(30, 9, Math.toRadians(180)))
                            .setTangent(0)
                            .splineToLinearHeading(deliverToBoardPose, Math.toRadians(0))
                            .build());
        }
    }

    public void BlueRightTeamArtPixelDelivery() {

        Pose2d aTempPose = new Pose2d(-38.5, 33, Math.toRadians(270));

        Actions.runBlocking(
                //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
                drive.actionBuilder(startPose)
                        .splineToLinearHeading(aTempPose, Math.toRadians(270))
                        .build());
        //***POSITION 1***
        if (control.autoPosition == 1) {
            deliverToFloorPose = new Pose2d(-34.5, 32, Math.toRadians(180));
            Actions.runBlocking(
                    //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
                    drive.actionBuilder(aTempPose)
                            .splineToLinearHeading(new Pose2d(-27, 33, Math.toRadians(180)), Math.toRadians(180))
                            .splineToLinearHeading(deliverToFloorPose, Math.toRadians(180))
                            .build());
        }
        //***POSITION 3***
        else if (control.autoPosition == 3) {
            deliverToFloorPose = new Pose2d(-37, 20.5, Math.toRadians(315));
            Actions.runBlocking(
                    //drive.actionBuilder(drive.  newa Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
                    drive.actionBuilder(aTempPose)
                            .splineToLinearHeading (deliverToFloorPose, Math.toRadians(315))
                            .build());
        }
        //***POSITION 2***
        else {
            deliverToFloorPose = new Pose2d(-38.5, 12.5, Math.toRadians(270));
            Actions.runBlocking(
                    //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
                    drive.actionBuilder(aTempPose)
                            .splineToLinearHeading(deliverToFloorPose, Math.toRadians(270))
                            .build());
        }
    }
    public Action stopSpinners() {
        return new StopSpinners();
    }
    public class StopSpinners implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.ServoStop();
                initialized = true;
            }


            packet.put("disable Intakes", 0);
            return false;
        }

    }
    public Action driveAcrossField()
    {
        return new DriveAcrossField();
    }
    public class DriveAcrossField implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                initialized = true;
                drive.updatePoseEstimate();
                drive.actionBuilder(new Pose2d(drive.pose.position.x, drive.pose.position.y, Math.toRadians(180)))
                        .setTangent(0)
                        .splineToLinearHeading(new Pose2d(-30, 9, Math.toRadians(180)), Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(30, 9, Math.toRadians(180)), Math.toRadians(0))
                        .build();
            }
            packet.put("drive across blue field", 0);
            boolean returnValue = true;

            drive.updatePoseEstimate();
            if ((drive.pose.position.x == 30) && (drive.pose.position.y == 9))
            {
                returnValue = false;
            }
            return returnValue;
        }

    }
}
