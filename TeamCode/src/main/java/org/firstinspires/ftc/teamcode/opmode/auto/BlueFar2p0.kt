package org.firstinspires.ftc.teamcode.opmode.auto

import com.amarcolini.joos.command.Command
import com.amarcolini.joos.command.CommandOpMode
import com.amarcolini.joos.command.SequentialCommand
import com.amarcolini.joos.command.WaitCommand
import com.amarcolini.joos.dashboard.JoosConfig
import com.amarcolini.joos.geometry.Pose2d
import com.amarcolini.joos.path.PathBuilder
import com.amarcolini.joos.util.deg
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.CSRobot
import org.firstinspires.ftc.teamcode.tile
import org.firstinspires.ftc.teamcode.vision.PropPipeline

@Autonomous
@JoosConfig
class BlueFar2p0 : CommandOpMode() {
    private val robot by robot<CSRobot>()
    private val pipeline = PropPipeline(false)

    companion object {
        var startPose = Pose2d(-16.0, 3 * tile - 9.0, (-90).deg)
        var rightPlopPose = Pose2d(-15.5, 32.0, (-180).deg)
        var centerPlopPose = Pose2d(-25.0, 28.0, (0).deg)
        var leftPlopPose = Pose2d(-10.0, 35.0, (0).deg)
        var rightPlacePose = Pose2d(71.5, 34.0, 5.deg)
        var centerPlacePose = Pose2d(71.5, 37.0, 0.deg)
        var leftPlacePose = Pose2d(71.5, 43.0, 0.deg)
        var regularExitPose = Pose2d(-16.0, 13.0, 0.deg)
        var centerExitPose = Pose2d(-26.0, 13.0, 0.deg)
        var crossPose = Pose2d(62.0, 17.0, 0.deg)
    }

    override fun preInit() {
        robot.outtake.init()
        robot.pixelPlopper.prime()
        robot.openFrontCameraAsync {
            robot.frontCamera.setPipeline(pipeline)
            schedule(true) {
                telem.addData("prop location", pipeline.lastKnownLocation)
            }
        }
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
        //Placing purple
        val purplePlopCommand = robot.drive.pathCommandBuilder(startPose)
            .lineToSplineHeading(plopPose)
            .then(robot.pixelPlopper.plop())
            .back(3.0)
            .lineToSplineHeading(exitPose)
            .build()

        //Crossing stage door and placing yellow (no slide extension)
        val yellowPlaceCommand = robot.drive.pathCommandBuilder(purplePlopCommand.endPose)
            .lineToSplineHeading(crossPose)
            .lineToSplineHeading(placePose)
            .forward(3.0).withTimeout(1.5)
            .then(robot.outtake.extend())
            .wait(2.0)
            .then(robot.outtake::releaseLeft)
            .wait(2.0)
            .then(robot.outtake.reset())
            .wait(2.0)
            .build()

        SequentialCommand(
            true,
            robot.outtake.ready(),
            purplePlopCommand,
            WaitCommand(3.0),
            yellowPlaceCommand
        ).thenStopOpMode().schedule()
    }
}