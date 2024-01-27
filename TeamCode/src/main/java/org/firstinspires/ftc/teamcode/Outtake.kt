package org.firstinspires.ftc.teamcode

import com.amarcolini.joos.command.AbstractComponent
import com.amarcolini.joos.command.Command
import com.amarcolini.joos.command.InstantCommand
import com.amarcolini.joos.dashboard.JoosConfig
import com.amarcolini.joos.hardware.Servo

@JoosConfig
class Outtake(
    val armServo: Servo, val leftServo: Servo,
    val rightServo: Servo
) : AbstractComponent() {
    companion object {
        private val initPosition = 0.7
        private val outtakePosition = 0.35
        private val armNeutral = 0.93
        var armTransfer = 0.94
        private val rightPositions = arrayOf(0.35, 0.6)
        private val leftPositions = arrayOf(0.97, 0.75)
        private val climbPosition = 0.6
        private val armSpeed = 0.5
    }

    var isLeftOpen = true
        private set
    var isRightOpen = true
    val isOpen get() = isLeftOpen && isRightOpen
    var isExtended = false
        private set

    init {
        subcomponents += listOf(armServo, leftServo, rightServo)
    }

    fun releaseLeft() {
        isLeftOpen = true
        leftServo.position = leftPositions[1]
    }

    fun releaseRight() {
        isRightOpen = true
        rightServo.position = rightPositions[1]
    }

    fun open() {
        releaseLeft()
        releaseRight()
    }

    private fun close() {
        isLeftOpen = false
        isRightOpen = false
        leftServo.position = leftPositions[0]
        rightServo.position = rightPositions[0]
    }

    fun stopPixels() {
        leftServo.position = leftPositions[0]
        rightServo.position = rightPositions[0]
    }

    fun prepareTransfer() = Command.select {
        stopPixels()
        armServo.waitForPosition(armTransfer, armSpeed)
    }

    fun prepareClimb() = armServo.waitForPosition(climbPosition, armSpeed)

    fun resetArm() = Command.select {
        isExtended = false
        armServo.waitForPosition(armNeutral, armSpeed)
    }

    fun extend() = Command.select {
        isExtended = true
        armServo.waitForPosition(outtakePosition, armSpeed)
    }

    fun reset() = Command.select {
        stopPixels()
        resetArm()
    }

    fun ready() = InstantCommand(::close).wait(0.5) then resetArm()

    fun init() {
        armServo.position = initPosition
        close()
    }
}