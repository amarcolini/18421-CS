package org.firstinspires.ftc.teamcode.opmode.auto

import com.acmerobotics.dashboard.FtcDashboard
import com.amarcolini.joos.command.Command
import com.amarcolini.joos.command.CommandOpMode
import com.amarcolini.joos.command.SequentialCommand
import com.amarcolini.joos.command.WaitCommand
import com.amarcolini.joos.dashboard.JoosConfig
import com.amarcolini.joos.geometry.Pose2d
import com.amarcolini.joos.util.deg
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.CSRobot
import org.firstinspires.ftc.teamcode.Intake
import org.firstinspires.ftc.teamcode.tile
import org.firstinspires.ftc.teamcode.vision.PropPipeline
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraRotation

@Autonomous
@JoosConfig
class BlueCycleAuto2 : CommandOpMode() {
    private val robot by robot<CSRobot>()
    private val pipeline = PropPipeline(false)

    companion object {
        var startPose = Pose2d(-2 * tile + 8.0, 3 * tile - 9.0, (-90).deg)
        var rightPlopPose = Pose2d(-39.5, 32.0, (-180).deg)
        var centerPlopPose = Pose2d(-49.0, 28.0, (0).deg)
        var leftPlopPose = Pose2d(-34.0, 35.0, (0).deg)
        var rightPlacePose = Pose2d(47.5, 30.0, 5.deg)
        var centerPlacePose = Pose2d(47.5, 35.0, 0.deg)
        var leftPlacePose = Pose2d(47.5, 43.0, 0.deg)
        var regularExitPose = Pose2d(-40.0, 13.0, 0.deg)
        var centerExitPose = Pose2d(-50.0, 13.0, 0.deg)
        var stackPose = Pose2d(-61.5, 17.0, 0.deg)
        var crossPose = Pose2d(30.0, 13.0, 0.deg)

        var stackHigh = 0.47
    }

    override fun preInit() {
        robot.outtake.init()
        robot.pixelPlopper.prime()
        robot.webcam.openCameraDeviceAsync(object : OpenCvCamera.AsyncCameraOpenListener {
            override fun onOpened() {
                robot.webcam.startStreaming(
                    640,
                    480,
                    OpenCvCameraRotation.UPRIGHT
                )
                robot.webcam.setPipeline(pipeline)
                FtcDashboard.getInstance().startCameraStream(robot.webcam, 0.0)

                schedule(true) {
                    telem.addData("prop location", pipeline.lastKnownLocation)
                }
            }

            override fun onError(errorCode: Int) {
                telem.addLine("Camera failed to open!!").setRetained(true)
                telem.update()
                requestOpModeStop()
            }

        })
    }

    override fun preStart() {
        cancelAll()
        robot.drive.poseEstimate = startPose
        val location = pipeline.lastKnownLocation

        val (plopPose, placePose) = when (location) {
            PropPipeline.PropLocation.Left -> leftPlopPose to leftPlacePose
            PropPipeline.PropLocation.Center -> centerPlopPose to centerPlacePose
            PropPipeline.PropLocation.Right -> rightPlopPose to rightPlacePose
        }

        val exitPose = if (location == PropPipeline.PropLocation.Center) {
            centerExitPose
        } else regularExitPose
        val purplePlaceCommand = robot.drive.pathBuilder(startPose)
            .lineToSplineHeading(plopPose)
            .then(robot.pixelPlopper.plop())
            .back(3.0)
            .lineToSplineHeading(exitPose)
            .build()

        val intakeAndCrossCommand = robot.drive.pathBuilder(exitPose)
            .lineToSplineHeading(stackPose)
            .and(
                robot.intake.waitForServoPosition(stackHigh)
                    .then {
                        robot.intake.servoState = Intake.ServoState.STACK
                        robot.intake.motorState = Intake.MotorState.ACTIVE
                    }
            )
            .then(Command.emptyCommand().waitUntil {
                robot.intake.numPixels == 2
            }.withTimeout(3.0))
            .splineTo(crossPose.vec(), crossPose.heading)
            .splineTo(placePose.vec(), placePose.heading)
            .and(WaitCommand(0.5).then(robot.transfer()))
            .build()

        val yellowPlaceCommand = robot.drive.pathBuilder(placePose)
            .then(
                robot.outtake.extend().and(
                    robot.verticalExtension.goToPosition(200.0)
                )
            )
            .forward(3.0).withTimeout(0.5)
            .wait(1.0)
            .then {
                robot.outtake.releaseLeft()
                robot.outtake.releaseRight()
            }
            .wait(1.0)
            .then(robot.outtake.reset())
            .then(robot.verticalExtension.goToPosition(0.0))
            .build()

        SequentialCommand(
            true,
            robot.outtake.ready(),
            Command.of {
                robot.drive.setDrivePower(Pose2d(0.1))
            },
            purplePlaceCommand,
            intakeAndCrossCommand,
            yellowPlaceCommand
        ).thenStopOpMode().schedule()
    }
}