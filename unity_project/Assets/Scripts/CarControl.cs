using UnityEngine;
using UnityEngine.UI;
using TMPro;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using System.Collections.Generic;

// Simple PID Controller class
public class PIDController
{
    private float kp, ki, kd;
    private float integral = 0f;
    private float previousError = 0f;
    private float lastTime = 0f;

    public PIDController(float kp, float ki, float kd)
    {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        lastTime = Time.time;
    }

    public float Calculate(float setpoint, float processValue)
    {
        float currentTime = Time.time;
        float deltaTime = currentTime - lastTime;
        
        if (deltaTime <= 0f) deltaTime = 0.02f; // Prevent division by zero
        
        float error = setpoint - processValue;
        
        // Proportional term
        float proportional = kp * error;
        
        // Integral term
        integral += error * deltaTime;
        float integralTerm = ki * integral;
        
        // Derivative term
        float derivative = (error - previousError) / deltaTime;
        float derivativeTerm = kd * derivative;
        
        // Calculate output
        float output = proportional + integralTerm + derivativeTerm;
        
        // Update for next iteration
        previousError = error;
        lastTime = currentTime;
        
        return output;
    }

    public void Reset()
    {
        integral = 0f;
        previousError = 0f;
        lastTime = Time.time;
    }

    public void UpdateParameters(float kp, float ki, float kd)
    {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }
}

public class CarControl : MonoBehaviour
{
    [Header("Vehicle Settings")]
    public float maxSpeed = 2f;
    public float maxAngularSpeed = 2f;
    public float wheelbase = 0.6972f;
    public float maxMotorTorque = 100f;

    [Header("PID Parameters - Linear Velocity")]
    private float linearKp = 20f;
    private float linearKi = 2f;
    private float linearKd = 1f;

    [Header("PID Parameters - Angular Velocity")]
    private float angularKp = 15f;
    private float angularKi = 1f;
    private float angularKd = 0.5f;

    [Header("Manual Control Settings")]
    public float manualLinearSpeed = 1f;
    public float manualAngularSpeed = 1f;

    public WheelCollider frontLeftWheel;
    public WheelCollider frontRightWheel;
    public WheelCollider rearLeftWheel;
    public WheelCollider rearRightWheel;

    public TextMeshProUGUI statusText;

    private bool Left, Right, Forward, Backward;
    private float linearVelocity, angularVelocity;
    private float targetLinearVel, targetAngularVel;
    private float cmdVelLinear = 0f, cmdVelAngular = 0f;
    private bool useCmdVel = false;
    private float lastCmdVelTime = 0f;
    private float cmdVelTimeout = 1f; // 1 second timeout

    // Velocity averaging
    private Queue<float> linearVelHistory = new Queue<float>();
    private Queue<float> angularVelHistory = new Queue<float>();
    private Queue<float> timeHistory = new Queue<float>();
    private float averagingPeriod = 0.1f;

    // PID Controllers
    private PIDController linearPID;
    private PIDController angularPID;

    private Rigidbody carRigidbody;
    private ROSConnection ros;

    void Start()
    {
        carRigidbody = GetComponent<Rigidbody>();
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<TwistMsg>("/cmd_vel", CmdVelCallback);
        
        // Initialize PID controllers
        linearPID = new PIDController(linearKp, linearKi, linearKd);
        angularPID = new PIDController(angularKp, angularKi, angularKd);
    }

    private void CmdVelCallback(TwistMsg msg)
    {
        cmdVelLinear = (float)msg.linear.x;
        cmdVelAngular = (float)msg.angular.z;
        useCmdVel = true;
        lastCmdVelTime = Time.time; // Update timestamp when cmd_vel is received
    }

    private void Update()
    {
        // Update PID parameters if changed in inspector
        linearPID.UpdateParameters(linearKp, linearKi, linearKd);
        angularPID.UpdateParameters(angularKp, angularKi, angularKd);
        
        // Check cmd_vel timeout
        if (useCmdVel && Time.time - lastCmdVelTime > cmdVelTimeout)
        {
            useCmdVel = false;
            cmdVelLinear = 0f;
            cmdVelAngular = 0f;
        }
        
        GetInput();
        CalculateAverageVelocities();
        
        if (useCmdVel)
            HandleCmdVel();
        else
            HandleManualInput();
            
        ApplyPIDControl();
        UpdateStatusText();
    }

    private void CalculateAverageVelocities()
    {
        // Get instantaneous velocities from rigidbody
        Vector3 velocity = carRigidbody.linearVelocity;
        Vector3 angularVel = -carRigidbody.angularVelocity;
        
        // Calculate forward velocity (assuming forward is along Z-axis)
        float instantLinearVel = Vector3.Dot(velocity, transform.forward);
        
        // Use Y-axis angular velocity for turning (assuming Y is up)
        float instantAngularVel = angularVel.y;

        // Add current values to history
        linearVelHistory.Enqueue(instantLinearVel);
        angularVelHistory.Enqueue(instantAngularVel);
        timeHistory.Enqueue(Time.time);

        // Remove old values outside the averaging period
        while (timeHistory.Count > 0 && Time.time - timeHistory.Peek() > averagingPeriod)
        {
            linearVelHistory.Dequeue();
            angularVelHistory.Dequeue();
            timeHistory.Dequeue();
        }

        // Calculate averages
        if (linearVelHistory.Count > 0)
        {
            float linearSum = 0f;
            float angularSum = 0f;
            
            foreach (float vel in linearVelHistory)
                linearSum += vel;
            
            foreach (float vel in angularVelHistory)
                angularSum += vel;

            linearVelocity = linearSum / linearVelHistory.Count;
            angularVelocity = angularSum / angularVelHistory.Count;
        }
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

    private void HandleManualInput()
    {
        // Set target velocities based on input with simultaneous control
        targetLinearVel = 0f;
        targetAngularVel = 0f;

        // Handle forward/backward movement
        if (Forward)
            targetLinearVel += manualLinearSpeed;
        if (Backward)
            targetLinearVel -= manualLinearSpeed;

        // Handle turning (can be combined with forward/backward)
        if (Right)
            if(Backward) targetAngularVel += manualAngularSpeed;
            else targetAngularVel -= manualAngularSpeed;
        if (Left)
            if(Backward) targetAngularVel -= manualAngularSpeed;
            else targetAngularVel += manualAngularSpeed;

        // Clamp to max speeds
        targetLinearVel = Mathf.Clamp(targetLinearVel, -maxSpeed, maxSpeed);
        targetAngularVel = Mathf.Clamp(targetAngularVel, -maxAngularSpeed, maxAngularSpeed);
    }

    private void HandleCmdVel()
    {
        // Set target velocities from ROS command
        targetLinearVel = Mathf.Clamp(cmdVelLinear, -maxSpeed, maxSpeed);
        targetAngularVel = Mathf.Clamp(cmdVelAngular, -maxAngularSpeed, maxAngularSpeed);
    }

    private void ApplyPIDControl()
    {
        // Calculate PID outputs
        float linearOutput = linearPID.Calculate(targetLinearVel, linearVelocity);
        
        // Only apply angular PID when there's an angular target or significant angular velocity
        float angularOutput = 0f;
        if (Mathf.Abs(targetAngularVel) > 0.01f || Mathf.Abs(angularVelocity) > 0.1f)
        {
            angularOutput = angularPID.Calculate(targetAngularVel, angularVelocity);
        }
        else
        {
            // Reset angular PID when not in use to prevent integral windup
            angularPID.Reset();
        }

        // Calculate torques for each side
        float leftTorque = linearOutput - angularOutput;
        float rightTorque = linearOutput + angularOutput;

        // Clamp torques to maximum motor torque
        leftTorque = Mathf.Clamp(leftTorque, -maxMotorTorque, maxMotorTorque);
        rightTorque = Mathf.Clamp(rightTorque, -maxMotorTorque, maxMotorTorque);

        // Apply torques to wheels
        ApplyWheelTorques(leftTorque, rightTorque);
    }

    private void ApplyWheelTorques(float leftTorque, float rightTorque)
    {
        // Remove all brake torques first
        frontLeftWheel.brakeTorque = 0f;
        frontRightWheel.brakeTorque = 0f;
        rearLeftWheel.brakeTorque = 0f;
        rearRightWheel.brakeTorque = 0f;

        // Apply motor torques
        frontLeftWheel.motorTorque = leftTorque;
        rearLeftWheel.motorTorque = leftTorque;
        frontRightWheel.motorTorque = rightTorque;
        rearRightWheel.motorTorque = rightTorque;

        // Apply light braking when no input is given
        if (Mathf.Abs(targetLinearVel) < 0.1f && Mathf.Abs(targetAngularVel) < 0.1f)
        {
            float brakeTorque = 10f;
            frontLeftWheel.brakeTorque = brakeTorque;
            frontRightWheel.brakeTorque = brakeTorque;
            rearLeftWheel.brakeTorque = brakeTorque;
            rearRightWheel.brakeTorque = brakeTorque;
        }
    }

    private void UpdateStatusText()
    {
        if (statusText != null)
        {
            statusText.text =
                $"Left: {Left} | Right: {Right} | Forward: {Forward} | Backward: {Backward}\n" +
                $"Linear Velocity: {linearVelocity:F2} m/s (Target: {targetLinearVel:F2})\n" +
                $"Angular Velocity: {angularVelocity:F2} rad/s (Target: {targetAngularVel:F2})\n" +
                $"cmd_vel: {(useCmdVel ? "ON" : "OFF")}\n" +
                $"cmd_vel linear: {cmdVelLinear:F2} m/s\n" +
                $"cmd_vel angular: {cmdVelAngular:F2} rad/s\n" +
                $"PID Linear: Kp={linearKp:F1} Ki={linearKi:F1} Kd={linearKd:F1}\n" +
                $"PID Angular: Kp={angularKp:F1} Ki={angularKi:F1} Kd={angularKd:F1}";
        }
    }
}
