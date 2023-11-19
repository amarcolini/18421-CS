package org.firstinspires.ftc.teamcode

import com.amarcolini.joos.command.AbstractComponent
import com.amarcolini.joos.dashboard.JoosConfig
import com.amarcolini.joos.hardware.Servo

class Outtake(
    val armServo: Servo, val leftServo: Servo,
    val rightServo: Servo
) : AbstractComponent() {
    companion object {
        private val resetPosition = 0.93
        private val primePosition = 0.93
        private val initPosition = 0.7
        private val outtakePosition = 0.3
        private val leftOpen = 1.0
        private val rightOpen = 0.0
        private val leftClose = 0.5
        private val rightClose = 0.5
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
        leftServo.position = leftOpen
    }

    fun releaseRight() {
        isRightOpen = true
        rightServo.position = rightOpen
    }

    fun open() {
        releaseLeft()
        releaseRight()
    }

    private fun close() {
        isLeftOpen = false
        isRightOpen = false
        leftServo.position = leftClose
        rightServo.position = rightClose
    }

    fun resetArm() {
        isExtended = false
        armServo.position = resetPosition
    }

    fun extend() {
        isExtended = true
        armServo.position = outtakePosition
    }

    private fun primeArm() {
        armServo.position = primePosition
    }

    fun reset() {
        resetArm()
        open()
    }

    fun ready() {
        primeArm()
        close()
    }

    fun init() {
        armServo.position = initPosition
        close()
    }
}