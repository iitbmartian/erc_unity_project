using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Tf2;
using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Rosgraph;
using System.Collections.Generic;

public class TFPublisher : MonoBehaviour
{
    [Header("TF Configuration")]
    public float publishFrequency = 30f;

    [Header("Transform References")]
    public Transform baseLinkTransform;
    public Transform bodyLinkTransform;
    public Transform cameraLinkTransform;
    public Transform imuLinkTransform;
    public Transform leftCameraTransform;

    [Header("Frame IDs")]
    public string baseLinkFrameId = "base_link";
    public string bodyLinkFrameId = "body_link";
    public string cameraLinkFrameId = "camera_link";
    public string cameraOpticalFrameId = "left_camera_link_optical";
    public string imuLinkFrameId = "imu_link";
    public string leftCameraFrameId = "left_camera_link";
    public string odomFrameId = "odom";

    [Header("Odometry")]
    public bool publishOdometry = true;
    public bool publishOdomTF = true;
    public string odomTopic = "/odom";

    private ROSConnection ros;
    private double lastPublishTime = 0;
    private float publishInterval;
    private string clockTopic = "/clock";

    // Track initial pose to define odom origin
    private Vector3 initialPosition;
    private Quaternion initialRotation;
    private bool isInitialized = false;

    // Track previous pose/time to compute velocity
    private Vector3 previousPosition;
    private Quaternion previousRotation;
    private float previousTime;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<TFMessageMsg>("/tf");
        ros.RegisterPublisher<ClockMsg>(clockTopic);

        if (publishOdometry)
        {
            ros.RegisterPublisher<OdometryMsg>(odomTopic);
        }
        
        publishInterval = 1.0f / publishFrequency;

        // Store base_link's initial pose as odom origin
        if (baseLinkTransform != null)
        {
            initialPosition = baseLinkTransform.position;
            initialRotation = baseLinkTransform.rotation;
            previousPosition = initialPosition;
            previousRotation = initialRotation;
            previousTime = Time.time;
            isInitialized = true;
        }
    }

    void Update()
    {
        double currentTime = Time.timeAsDouble;
        if (currentTime - lastPublishTime >= publishInterval)
        {
            PublishTF();
            if (publishOdometry && isInitialized)
            {
                PublishOdometry();
            }
            lastPublishTime = currentTime;
        }

        // Publish ROS clock
        var clockMsg = new ClockMsg
        {
            clock = new TimeMsg
            {
                sec = (int)Time.time,
                nanosec = (uint)((Time.time - Mathf.Floor(Time.time)) * 1e9f)
            }
        };
        ros.Publish(clockTopic, clockMsg);
    }

    void PublishOdometry()
    {
        // Calculate relative pose from initial pose
        Vector3 currentPosition = baseLinkTransform.position;
        Quaternion currentRotation = baseLinkTransform.rotation;

        Vector3 relativePosition = currentPosition - initialPosition;
        Quaternion relativeRotation = Quaternion.Inverse(initialRotation) * currentRotation;
        relativeRotation *= Quaternion.Euler(0f, 180f, 0f);

        // Compute velocity
        float dt = Time.time - previousTime;
        Vector3 linearVelocity = Vector3.zero;
        Vector3 angularVelocity = Vector3.zero;

        if (dt > 0)
        {
            // Unity forward = +Z, ROS forward = +X
            Vector3 deltaPos = (currentPosition - previousPosition) / dt;
            linearVelocity = new Vector3(deltaPos.z, -deltaPos.x, deltaPos.y);

            Quaternion deltaRot = currentRotation * Quaternion.Inverse(previousRotation);
            deltaRot.ToAngleAxis(out float angle, out Vector3 axis);
            angularVelocity = axis.normalized * (angle * Mathf.Deg2Rad / dt);
        }

        // Convert position to ROS
        var rosPosition = new Vector3Msg
        {
            x = relativePosition.z,
            y = -relativePosition.x,
            z = relativePosition.y
        };

        // Convert orientation to ROS (handles the rotation transform)
        var rosRotation = relativeRotation.To<FLU>();

        // Convert velocities to ROS
        var rosLinearVel = new Vector3Msg { x = linearVelocity.x, y = linearVelocity.y, z = linearVelocity.z };
        var rosAngularVel = angularVelocity.To<FLU>();

        // Create odometry message
        var currentTime = new TimeMsg 
        { 
            sec = (int)Time.time, 
            nanosec = (uint)((Time.time - (int)Time.time) * 1e9f)
        };

        var odomMsg = new OdometryMsg
        {
            header = new RosMessageTypes.Std.HeaderMsg
            {
                stamp = currentTime,
                frame_id = odomFrameId
            },
            child_frame_id = baseLinkFrameId,
            pose = new PoseWithCovarianceMsg
            {
                pose = new PoseMsg
                {
                    position = new PointMsg { x = rosPosition.x, y = rosPosition.y, z = rosPosition.z },
                    orientation = new QuaternionMsg { x = rosRotation.x, y = rosRotation.y, z = rosRotation.z, w = rosRotation.w }
                },
                covariance = new double[36] // Zero by default
            },
            twist = new TwistWithCovarianceMsg
            {
                twist = new TwistMsg
                {
                    linear = rosLinearVel,
                    angular = rosAngularVel
                },
                covariance = new double[36] // Zero by default
            }
        };

        // Publish odometry
        ros.Publish(odomTopic, odomMsg);

        // Update previous
        previousPosition = currentPosition;
        previousRotation = currentRotation;
        previousTime = Time.time;
    }

    void PublishTF()
    {
        var tfArray = new TFMessageMsg();
        var transforms = new List<TransformStampedMsg>();
        var currentTime = new TimeMsg 
        {
            sec = (int)Time.time,
            nanosec = (uint)((Time.time - (int)Time.time) * 1e9)
        };

        // Odometry TF: odom -> base_link
        if (publishOdomTF && isInitialized && baseLinkTransform != null)
        {
            Vector3 currentPosition = baseLinkTransform.position;
            Quaternion currentRotation = baseLinkTransform.rotation;

            Vector3 relativePosition = currentPosition - initialPosition;
            Quaternion relativeRotation = Quaternion.Inverse(initialRotation) * currentRotation;

            // Convert to ROS
            var rosPosition = new Vector3Msg 
            {
                x = relativePosition.z,
                y = -relativePosition.x,
                z = relativePosition.y
            };
            var rosRotation = relativeRotation.To<FLU>();

            transforms.Add(new TransformStampedMsg
            {
                header = new RosMessageTypes.Std.HeaderMsg
                {
                    stamp = currentTime,
                    frame_id = odomFrameId
                },
                child_frame_id = baseLinkFrameId,
                transform = new TransformMsg
                {
                    translation = rosPosition,
                    rotation = rosRotation
                }
            });
        }

        // base_link -> body_link
        if (baseLinkTransform != null && bodyLinkTransform != null)
        {
            transforms.Add(CreateTransformStamped(
                baseLinkFrameId,
                bodyLinkFrameId,
                baseLinkTransform,
                bodyLinkTransform,
                currentTime
            ));
        }

        // base_link -> camera_link
        if (baseLinkTransform != null && cameraLinkTransform != null)
        {
            transforms.Add(CreateTransformStamped(
                baseLinkFrameId,
                cameraLinkFrameId,
                baseLinkTransform,
                cameraLinkTransform,
                currentTime
            ));
        }

        // camera_link -> left_camera_link_optical
        if (cameraLinkTransform != null)
        {
            transforms.Add(CreateOpticalFrameTransform(
                leftCameraFrameId,
                cameraOpticalFrameId,
                currentTime
            ));
        }

        // base_link -> imu_link
        if (baseLinkTransform != null && imuLinkTransform != null)
        {
            transforms.Add(CreateTransformStamped(
                baseLinkFrameId,
                imuLinkFrameId,
                baseLinkTransform,
                imuLinkTransform,
                currentTime
            ));
        }

        // camera_link -> left_camera_link
        if (cameraLinkTransform != null && leftCameraTransform != null)
        {
            transforms.Add(CreateTransformStamped(
                cameraLinkFrameId,
                leftCameraFrameId,
                cameraLinkTransform,
                leftCameraTransform,
                currentTime
            ));
        }

        if (transforms.Count > 0)
        {
            tfArray.transforms = transforms.ToArray();
            ros.Publish("/tf", tfArray);
        }
    }

    private TransformStampedMsg CreateOpticalFrameTransform(
        string parentFrame,
        string opticalFrame,
        TimeMsg timestamp)
    {
        var transformStamped = new TransformStampedMsg
        {
            header = new RosMessageTypes.Std.HeaderMsg
            {
                stamp = timestamp,
                frame_id = parentFrame
            },
            child_frame_id = opticalFrame,
            transform = new TransformMsg
            {
                translation = new Vector3Msg { x = 0, y = 0, z = 0 },
                rotation = new QuaternionMsg { x = -0.5f, y = 0.5f, z = -0.5f, w = 0.5f }
            }
        };
        return transformStamped;
    }

    private TransformStampedMsg CreateTransformStamped(
        string parentFrame, 
        string childFrame, 
        Transform parentTransform, 
        Transform childTransform,
        TimeMsg timestamp)
    {
        var transformStamped = new TransformStampedMsg
        {
            header = new RosMessageTypes.Std.HeaderMsg
            {
                stamp = timestamp,
                frame_id = parentFrame
            },
            child_frame_id = childFrame
        };

        // Basic relative transform
        Vector3 relativePosition = parentTransform.InverseTransformPoint(childTransform.position);
        Quaternion relativeRotation = Quaternion.Inverse(parentTransform.rotation) * childTransform.rotation;

        transformStamped.transform.translation = relativePosition.To<FLU>();
        transformStamped.transform.rotation = relativeRotation.To<FLU>();
        return transformStamped;
    }
}