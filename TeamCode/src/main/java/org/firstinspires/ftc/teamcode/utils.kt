package org.firstinspires.ftc.teamcode

import com.amarcolini.joos.command.Command
import com.amarcolini.joos.command.Component
import com.amarcolini.joos.command.SequentialCommand
import com.amarcolini.joos.command.WaitCommand
import com.amarcolini.joos.geometry.Angle
import com.amarcolini.joos.geometry.Pose2d
import com.amarcolini.joos.geometry.Vector2d
import com.amarcolini.joos.hardware.drive.DrivePathFollower
import com.amarcolini.joos.path.Path
import com.amarcolini.joos.path.PathBuilder
import com.amarcolini.joos.path.PathContinuityViolationException
import com.amarcolini.joos.path.heading.*

const val tile = 24.0

class PathCommandBuilder(
    private val drive: DrivePathFollower,
    private val startPose: Pose2d,
    startTangent: Angle = startPose.heading,
) {
    private var builder = PathBuilder(startPose, startTangent)
    private var currentCommand: Command = SequentialCommand()
    private var currentPose = startPose

    private fun tryAdd(segment: PathBuilder.() -> Unit): PathCommandBuilder {
        try {
            builder.segment()
        } catch (e: PathContinuityViolationException) {
            pushAndAddPath()
            builder.segment()
        }
        return this
    }

    private fun pushPath(newTangent: (Angle) -> Angle = { it }): Path? {
        val path = builder.build().let {
            if (it.segments.isNotEmpty()) it else null
        }
        val newPose = path?.end() ?: currentPose
        builder = PathBuilder(
            newPose,
            Pose2d(newTangent(newPose.heading).vec()),
            path?.endSecondDeriv() ?: Pose2d()
        )
        currentPose = path?.end() ?: currentPose
        return path
    }

    private fun pushPathCommand(newTangent: (Angle) -> Angle = { it }) = pushPath(newTangent)?.let {
        drive.followerPath(it)
    }

    private fun pushAndAddPath(newTangent: (Angle) -> Angle = { it }) {
        pushPathCommand(newTangent)?.let {
            currentCommand = currentCommand then it
        }
    }

    fun setTangent(angle: (Angle) -> Angle): PathCommandBuilder {
        pushAndAddPath(angle)
        return this
    }

    fun setTangent(angle: Angle) = setTangent { angle }

    fun reverseTangent(): PathCommandBuilder {
        pushAndAddPath { -it }
        return this
    }

    fun then(command: Command): PathCommandBuilder {
        pushAndAddPath()
        currentCommand = currentCommand then command
        return this
    }

    fun then(runnable: Runnable) = then(Command.of(runnable))

    fun combine(command: Command, with: Command.(Command) -> Command): PathCommandBuilder {
        val original = pushPathCommand()
        currentCommand = currentCommand then (original?.with(command) ?: command)
        return this
    }

    fun and(command: Command): PathCommandBuilder = combine(command) {
        this and it
    }

    fun wait(duration: Double) = then(WaitCommand(duration))

    fun race(command: Command): PathCommandBuilder = combine(command) {
        this race it
    }

    fun withTimeout(duration: Double) = race(WaitCommand(duration))

    fun build(): PathCommand {
        pushAndAddPath()
        return PathCommand(currentCommand, startPose, currentPose)
    }

    /**
     * Adds a line segment with the specified heading interpolation.
     *
     * @param endPosition end position
     * @param headingInterpolation desired heading interpolation
     * @see HeadingInterpolation
     */
    fun addLine(endPosition: Vector2d, headingInterpolation: HeadingInterpolation = TangentHeading) =
        tryAdd {
            this.addLine(endPosition, headingInterpolation)
        }

    fun lineTo(endPosition: Vector2d) = addLine(endPosition, TangentHeading)
    fun lineToConstantHeading(endPosition: Vector2d) = addLine(endPosition, ConstantHeading)
    fun lineToLinearHeading(endPose: Pose2d) = addLine(endPose.vec(), LinearHeading(endPose.heading))
    fun lineToSplineHeading(endPose: Pose2d) = addLine(endPose.vec(), SplineHeading(endPose.heading))
    fun forward(distance: Double) = tryAdd { forward(distance) }
    fun back(distance: Double) = forward(-distance)
    fun strafeLeft(distance: Double) = tryAdd { strafeLeft(distance) }
    fun strafeRight(distance: Double) = strafeLeft(-distance)

    /**
     * Adds a spline segment with the specified heading interpolation.
     *
     * @param endPosition end position
     * @param endTangent end tangent
     * @param startTangentMag the magnitude of the start tangent (negative = default magnitude)
     * @param endTangentMag the magnitude of the end tangent (negative = default magnitude)
     * @param headingInterpolation desired heading interpolation
     * @see HeadingInterpolation
     */
    @JvmOverloads
    fun addSpline(
        endPosition: Vector2d,
        endTangent: Angle,
        headingInterpolation: HeadingInterpolation = TangentHeading,
        startTangentMag: Double = -1.0,
        endTangentMag: Double = -1.0,
    ) = tryAdd {
        this.addSpline(endPosition, endTangent, startTangentMag, endTangentMag, headingInterpolation)
    }

    fun splineTo(endPosition: Vector2d, endTangent: Angle) = addSpline(endPosition, endTangent)
    fun splineToConstantHeading(endPosition: Vector2d, endTangent: Angle) =
        addSpline(endPosition, endTangent, ConstantHeading)

    fun splineToLinearHeading(endPose: Pose2d, endTangent: Angle) =
        addSpline(endPose.vec(), endTangent, LinearHeading(endPose.heading))

    fun splineToSplineHeading(endPose: Pose2d, endTangent: Angle) =
        addSpline(endPose.vec(), endTangent, SplineHeading(endPose.heading))
}

data class PathCommand(val command: Command, val startPose: Pose2d, val endPose: Pose2d) : Command() {
    override val isInterruptable: Boolean
        get() = command.isInterruptable

    override val requirements: Set<Component>
        get() = command.requirements

    override fun execute() {
        command.execute()
    }

    override fun init() {
        command.init()
    }

    override fun isFinished(): Boolean {
        return command.isFinished()
    }

    override fun end(interrupted: Boolean) {
        command.end(interrupted)
    }
}