using UnityEngine;
using UnityEngine.UI;
using TMPro;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class CarControl : MonoBehaviour
{
    public float motorTorque = 20f;
    private float breakTorque = 30f;
    public float maxSpeed = 2f;
    public float wheelbase = 1.0f; // Distance between left and right wheels

    public WheelCollider frontLeftWheel;
    public WheelCollider frontRightWheel;
    public WheelCollider rearLeftWheel;
    public WheelCollider rearRightWheel;

    public TextMeshProUGUI statusText;

    private bool Left;
    private bool Right;
    private bool Forward;
    private bool Backward;

    private float linearVelocity;
    private float angularVelocity;
    private float torqueLeft;
    private float torqueRight;
    private float cmdVelLinear = 0f;
    private float cmdVelAngular = 0f;
    private bool useCmdVel = false;

    private ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<TwistMsg>("/cmd_vel", CmdVelCallback);
    }

    private void CmdVelCallback(TwistMsg msg)
    {
        cmdVelLinear = (float)msg.linear.x;
        cmdVelAngular = (float)msg.angular.z;
        useCmdVel = true;
    }

    private void Update()
    {
        GetInput();
        if (useCmdVel)
            HandleCmdVel();
        else
            HandleMotor();
        UpdateStatusText();
    }

    private void GetInput()
    {
        Left = Input.GetKey("a");
        Right = Input.GetKey("d");
        Forward = Input.GetKey("w");
        Backward = Input.GetKey("s");

        // If any key is pressed, disable cmd_vel control
        if (Left || Right || Forward || Backward)
            useCmdVel = false;
    }

    private void HandleMotor()
    {
        float leftVel = (
            frontLeftWheel.rpm * frontLeftWheel.radius * 2f * Mathf.PI / 60f +
            rearLeftWheel.rpm * rearLeftWheel.radius * 2f * Mathf.PI / 60f
        ) / 2f;

        float rightVel = (
            frontRightWheel.rpm * frontRightWheel.radius * 2f * Mathf.PI / 60f +
            rearRightWheel.rpm * rearRightWheel.radius * 2f * Mathf.PI / 60f
        ) / 2f;

        // Differential drive kinematics
        linearVelocity = (leftVel + rightVel) / 2f;
        angularVelocity = (rightVel - leftVel) / wheelbase;

        if (linearVelocity < maxSpeed && linearVelocity > -maxSpeed && angularVelocity < maxSpeed/wheelbase*2f && angularVelocity > -maxSpeed/wheelbase*2f)
        {
            if (Forward)
            {
                AllWheels(motorTorque);
            }
            else if (Backward)
            {
                AllWheels(-motorTorque);
            }
            else if (Right)
            {
                LeftWheels(motorTorque);
                RightWheels(-motorTorque);
            }
            else if (Left)
            {
                LeftWheels(-motorTorque);
                RightWheels(motorTorque);
            }
            else
            {
                AllWheels(0f);
                AllWheelsBrake(breakTorque);
            }
        }
        else
        {
            AllWheels(0f);
        }
    }

    private void HandleCmdVel()
    {
        float leftVel = (
            frontLeftWheel.rpm * frontLeftWheel.radius * 2f * Mathf.PI / 60f +
            rearLeftWheel.rpm * rearLeftWheel.radius * 2f * Mathf.PI / 60f
        ) / 2f;

        float rightVel = (
            frontRightWheel.rpm * frontRightWheel.radius * 2f * Mathf.PI / 60f +
            rearRightWheel.rpm * rearRightWheel.radius * 2f * Mathf.PI / 60f
        ) / 2f;

        linearVelocity = (leftVel + rightVel) / 2f;
        angularVelocity = (rightVel - leftVel) / wheelbase;

        float v = cmdVelLinear;
        float w = 6f * cmdVelAngular;
        float v_left = v - (w * wheelbase / 2f);
        float v_right = v + (w * wheelbase / 2f);
        float current_left = frontLeftWheel.rpm * frontLeftWheel.radius * 2f * Mathf.PI / 60f;
        float current_right = frontRightWheel.rpm * frontRightWheel.radius * 2f * Mathf.PI / 60f;

        torqueLeft = 0;
        torqueRight = 0;
        if (angularVelocity < w && torqueLeft > -motorTorque && torqueRight < motorTorque)
        {
            torqueLeft += 5f;
            torqueRight -= 5f;
        }
        else if (angularVelocity > w && torqueLeft < motorTorque && torqueRight > -motorTorque)
        {
            torqueLeft -= 5f;
            torqueRight += 5f;
        }
        else
        {
            torqueLeft = 0;
            torqueRight = 0;
        }
        if (linearVelocity < v && torqueLeft < motorTorque && torqueRight < motorTorque)
        {
            torqueLeft += 5f;
            torqueRight += 5f;
        }
        else if (linearVelocity > v && torqueLeft > -motorTorque && torqueRight > -motorTorque)
        {
            torqueLeft -= 5f;
            torqueRight -= 5f;
        }
        else
        {
            torqueLeft = 0;
            torqueRight = 0;
        }
        LeftWheels(torqueLeft);
        RightWheels(torqueRight);


    }

    private void AllWheels(float torque)
    {   
        AllWheelsBrake(0f);
        frontLeftWheel.motorTorque = torque;
        frontRightWheel.motorTorque = torque;
        rearLeftWheel.motorTorque = torque;
        rearRightWheel.motorTorque = torque;
    }
    private void LeftWheels(float torque)
    {   
        AllWheelsBrake(0f);
        frontLeftWheel.motorTorque = torque;
        rearLeftWheel.motorTorque = torque;
    }
    private void RightWheels(float torque)
    {   
        AllWheelsBrake(0f);
        frontRightWheel.motorTorque = torque;
        rearRightWheel.motorTorque = torque;
    }
    private void AllWheelsBrake(float torque)
    {
        frontLeftWheel.brakeTorque = torque;
        frontRightWheel.brakeTorque = torque;
        rearLeftWheel.brakeTorque = torque;
        rearRightWheel.brakeTorque = torque;
    }

    private void UpdateStatusText()
    {
        if (statusText != null)
        {
            statusText.text =
                $"Left: {Left}\n" +
                $"Right: {Right}\n" +
                $"Forward: {Forward}\n" +
                $"Backward: {Backward}\n" +
                $"Linear Velocity: {linearVelocity:F2} m/s\n" +
                $"Angular Velocity: {angularVelocity:F2} rad/s\n" +
                $"cmd_vel: {(useCmdVel ? "ON" : "OFF")}\n" +
                $"cmd_vel linear: {cmdVelLinear:F2} m/s\n" +
                $"cmd_vel angular: {cmdVelAngular:F2} rad/s\n" +
                $"Left Torque: {torqueLeft:F2} N·m\n" +
                $"Right Torque: {torqueRight:F2} N·m";
        }
    }
}