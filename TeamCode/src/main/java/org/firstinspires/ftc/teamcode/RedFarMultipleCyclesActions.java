package org.firstinspires.ftc.teamcode;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//@Disabled
@Autonomous(name="Red Far Multiple Cycles Actions", preselectTeleOp = "1 Manual Control")
public class RedFarMultipleCyclesActions extends LinearOpMode {
    UniversalControlClass control = new UniversalControlClass(true, false,this);
    MecanumDrive drive;
    Pose2d startPose;
    Pose2d deliverToFloorPose;
    Pose2d deliverToBoardPose;
    Pose2d stackPose = new Pose2d(-56, -12, Math.toRadians(180));
    Action FloorTraj;
    Action DriveToStack;
    Action BoardTraj2;
    Action BoardTraj1;
    Action BackUp;
    Action Park;
    Action DriveBackToStack;
    public void runOpMode(){
        startPose = new Pose2d(-35.5,-62.5,Math.toRadians(90));
        drive = new MecanumDrive(hardwareMap, startPose);

        control.Init(hardwareMap);
        control.HuskyLensInit();
        control.AutoStartPos();

        BoardTraj1 = drive.actionBuilder(stackPose)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(-30, -7, Math.toRadians(180)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(20, -7, Math.toRadians(180)), Math.toRadians(0))
                .build();

        telemetry.update();

        while(!isStarted()){
            control.DetectTeamArtRed();
            telemetry.update();
        }
        waitForStart();

        //make decisions
        RedLeftPurplePixelDecision();
        DriveToStack = drive.actionBuilder(deliverToFloorPose)
                .splineToLinearHeading(stackPose, Math.toRadians(180))
                .build();
        RedBoardDecision();
        BackUp = drive.actionBuilder(deliverToBoardPose)
                .lineToX(47)
                .build();
        DriveBackToStack = drive.actionBuilder(new Pose2d(47, deliverToBoardPose.position.y, Math.toRadians(180)))
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(30,9, Math.toRadians(180)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-50,12, Math.toRadians(180)), Math.toRadians(180))
                .build();
        Park = drive.actionBuilder(new Pose2d(47, deliverToBoardPose.position.y, Math.toRadians(180)))
                .splineToLinearHeading(new Pose2d(48, -9, Math.toRadians(90)), Math.toRadians(90))
                .build();


        Actions.runBlocking(
                new SequentialAction(
                        /* Drive to Floor Position */
                        new ParallelAction(
                                lockPixels(),
                                FloorTraj),
                        /* Deliver the Purple Pixel */
                        dropOnLine(),
                        resetArm(),
                        /* Drive to the Stack */
                        new ParallelAction(
                                servoIntake(),
                                DriveToStack),
                        /* Pick up a White Pixel */
                        autoPickup(),
                        /* Drive to the Board */
                        /*BoardTraj1,*/
                        new ParallelAction(
                                BoardTraj2,
                                halfwayTrigger()),
                        /* Use AprilTags to Align Perfectly to the Board */
                        /* Deliver to the Board */
                        stopNearBoardAuto()
                        /* Reset Arm */
                )
        );
        telemetry.update();
/*
        // lock the pixels
        control.GrabPixels();
        control.ReleaseLeft();

        //Drives to LCR drop on line
        RedLeftTeamArtPixelDelivery();

        // Drop the LEFT pixel (put PURPLE on LEFT, YELLOW on RIGHT) on the line
        control.DropOnLine();
        // put the arm back in a safe to travel position
        control.ResetArmAuto();
       // control.SpecialSleep(8000);

        if(control.autoPosition == 3) {
            Actions.runBlocking(
                    drive.actionBuilder(deliverToFloorPose)
                            .strafeTo(new Vector2d(-36, -12))
                            .splineToLinearHeading(stackPose, Math.toRadians(180))
                            .build());
        }
        else{
            //drive to stack
            Actions.runBlocking(
                    drive.actionBuilder(deliverToFloorPose)
                            .lineToY(-10)
                            .splineToLinearHeading(new Pose2d(-38.5, -12, Math.toRadians(180)), Math.toRadians(0))
                            //.splineToLinearHeading(new Pose2d(-50,-12, Math.toRadians(180)), Math.toRadians(0))
                            .splineToLinearHeading(stackPose, Math.toRadians(180))
                            .build());
        }
        control.AutoPickupRoutine();

        // drive to the backboard area
        drive.updatePoseEstimate();
        Actions.runBlocking(
            drive.actionBuilder(new Pose2d(drive.pose.position.x, drive.pose.position.y, Math.toRadians(180)))
                    .setTangent(0)
                    .splineToLinearHeading(new Pose2d(-30,-9, Math.toRadians(180)), Math.toRadians(0))
                    .splineToLinearHeading(new Pose2d(30,-9, Math.toRadians(180)), Math.toRadians(0))
            .build());

        // drive to the correct backboard spot based on the team art
        RedBoardDelivery();

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

        control.ResetArmAuto();
        sleep(200);

        //drive back to stack
        drive.updatePoseEstimate();
        Actions.runBlocking(
                //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
                drive.actionBuilder(new Pose2d(drive.pose.position.x, drive.pose.position.y, Math.toRadians(180)))
                        .setReversed(true)
                        .setTangent(180)
                        .splineToLinearHeading(new Pose2d(30,-9, Math.toRadians(180)), Math.toRadians(180))
                        //.setTangent(180)
                        //.splineToLinearHeading(new Pose2d(-50,-12, Math.toRadians(180)), Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(-50,-12, Math.toRadians(180)), Math.toRadians(180))
                        .build());
        control.AutoPickupRoutine();

        //drive back to board
        drive.updatePoseEstimate();
        Actions.runBlocking(
                //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
                drive.actionBuilder(new Pose2d(drive.pose.position.x, drive.pose.position.y, Math.toRadians(180)))
                        .setTangent(0)
                        .splineToLinearHeading(new Pose2d(-30,-9, Math.toRadians(180)), Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(30,-9, Math.toRadians(180)), Math.toRadians(0))
                        .build());
        RedBoardDelivery();
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

        control.ResetArmAuto();
        sleep(200);

        // Move the robot to the parking position
        Actions.runBlocking(
                //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
                drive.actionBuilder(new Pose2d(50, -28, Math.toRadians(180)))
                        .splineToLinearHeading(new Pose2d(48,-15, Math.toRadians(90)), Math.toRadians(90))
                        .build());
        control.ServoStop();
        sleep(100);
        telemetry.update();*/
    }
    public void RedBoardDecision() {
        // Look for potential errors
        //***POSITION 1***
        if (control.autoPosition == 1) {
            deliverToBoardPose = new Pose2d(50,-28,Math.toRadians(180));
            BoardTraj2 = drive.actionBuilder(stackPose)
                    .setTangent(0)
                    .splineToLinearHeading(new Pose2d(-30, -11.5, Math.toRadians(180)), Math.toRadians(0))
                    .splineToLinearHeading(new Pose2d(49, -11.5, Math.toRadians(180)), Math.toRadians(0))
                    .setTangent(Math.toRadians(270))
                    .splineToLinearHeading(deliverToBoardPose, Math.toRadians(270))
                    .build();
                    //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))

        }
        //***POSITION 3***
        else if (control.autoPosition == 3) {
            deliverToBoardPose = new Pose2d(50,-38,Math.toRadians(180));
            BoardTraj2 = drive.actionBuilder(stackPose)
                    .setTangent(0)
                    .splineToLinearHeading(new Pose2d(-30, -11.5, Math.toRadians(180)), Math.toRadians(0))
                    .splineToLinearHeading(new Pose2d(49, -11.5, Math.toRadians(180)), Math.toRadians(0))
                    .setTangent(Math.toRadians(270))
                    .splineToLinearHeading(deliverToBoardPose, Math.toRadians(270))
                    .build();
                    //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
        }
        //***POSITION 2***
        else {
            deliverToBoardPose = new Pose2d(50,-33,Math.toRadians(180));
            BoardTraj2 = drive.actionBuilder(stackPose)
                    .setTangent(0)
                    .splineToLinearHeading(new Pose2d(-30, -11.5, Math.toRadians(180)), Math.toRadians(0))
                    .splineToLinearHeading(new Pose2d(49, -11.5, Math.toRadians(180)), Math.toRadians(0))
                    .setTangent(Math.toRadians(270))
                    .splineToLinearHeading(deliverToBoardPose, Math.toRadians(270))
                    .build();
                    //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
        }
    }
    public void RedLeftPurplePixelDecision() {
        //***POSITION 1***
        if (control.autoPosition == 1) {
            deliverToFloorPose = new Pose2d(-39.5, -20.5, Math.toRadians(45));
            FloorTraj = drive.actionBuilder(startPose)
                    .splineToLinearHeading(new Pose2d(-38.5, -33, Math.toRadians(90)), Math.toRadians(90))
                    .splineToLinearHeading (deliverToFloorPose, Math.toRadians(45))
                    .build();
        }
        //***POSITION 3***
        else if (control.autoPosition == 3) {
            deliverToFloorPose = new Pose2d(-34.5, -32, Math.toRadians(180));
            FloorTraj = drive.actionBuilder(startPose)
                    .splineToLinearHeading(new Pose2d(-38.5, -33, Math.toRadians(90)), Math.toRadians(90))
                    .splineToLinearHeading(new Pose2d(-27, -33, Math.toRadians(180)), Math.toRadians(180))
                    .splineToLinearHeading(deliverToFloorPose, Math.toRadians(180))
                    .build();
        }
        //***POSITION 2***
        else {
            deliverToFloorPose = new Pose2d(-38.5, -12.5, Math.toRadians(90));
            FloorTraj = drive.actionBuilder(startPose)
                    .splineToLinearHeading(new Pose2d(-38.5, -33, Math.toRadians(90)), Math.toRadians(90))
                    .splineToLinearHeading(deliverToFloorPose, Math.toRadians(90))
                    .build();
        }
    }
    public Action lockPixels(){return new LockPixels();}
    public class LockPixels implements Action{
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.GrabPixels();
                control.ReleaseLeft();
                initialized = true;
            }
            packet.put("lock purple pixel", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action dropOnLine(){return new DropOnLine();}
    public class DropOnLine implements Action{
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.DropOnLine();
                initialized = true;
            }
            packet.put("drop purple pixel on line", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action resetArm(){return new ResetArm();}
    public class ResetArm implements Action{
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.ResetArmAuto();
                initialized = true;
            }
            packet.put("ResetArm", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action servoIntake(){return new ServoIntake();}
    public class ServoIntake implements Action{
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.ServoIntake();
                initialized = true;
            }
            packet.put("servoIntake", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action autoPickup(){return new AutoPickup();}
    public class AutoPickup implements Action{
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.AutoPickupRoutine();
                initialized = true;
            }
            packet.put("AutoPickup", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action slidesToAuto(){return new SlidesToAuto();}
    public class SlidesToAuto implements Action{
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.SlidesToAuto();
                initialized = true;
            }
            packet.put("SlidesToAuto", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action deliverPixelToBoardPos(){return new DeliverPixelToBoardPos();}
    public class DeliverPixelToBoardPos implements Action{
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.DeliverPixelToBoardPos();
                initialized = true;
            }
            packet.put("DeliverPixelToBoardPos", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action stopNearBoardAuto(){return new StopNearBoardAuto();}
    public class StopNearBoardAuto implements Action{
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.StopNearBoardAuto(true);
                initialized = true;
            }
            packet.put("StopNearBoardAuto", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action halfwayTrigger(){return new HalfwayTrigger();}
    public class HalfwayTrigger implements Action{
        public boolean run(@NonNull TelemetryPacket packet) {
            boolean moveArm = false;
            drive.updatePoseEstimate();
            if (drive.pose.position.x >= 20) {
                moveArm = true;
                control.SlidesToAuto();
                sleep(150);
                control.DeliverPixelToBoardPos();
            }
            packet.put("move arm trigger", 0);
            return !moveArm;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action servoStop(){return new ServoStop();}
    public class ServoStop implements Action{
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.ServoStop();
                initialized = true;
            }
            packet.put("drop purple pixel on line", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
            packet.put("halfwayTrigger", 0);
            return !moveArm;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
}

