// using UnityEngine;
// using System;

// public class BlueRovVeloControl : MonoBehaviour {
//     public float lvx = 0.0f; 
//     public float lvy = 0.0f;
//     public float lvz = 0.0f;
//     public float avz = 0.0f;
//     public float avy = 0.0f;
//     public float avx = 0.0f;
//     public Rigidbody rb;
    
//     // Timeout to stop robot when no new commands received
//     private float commandTimeout = 0.1f; // Stop after 0.1 seconds of no input
//     private float lastCommandTime;
    
//     void Start() {
//         this.rb = GetComponent<Rigidbody>();
        
//         // Collision detection
//         rb.collisionDetectionMode = CollisionDetectionMode.Continuous;
//         rb.interpolation = RigidbodyInterpolation.Interpolate;
        
//         // Underwater physics
//         rb.useGravity = false;
//         rb.linearDamping = 1f;
//         rb.angularDamping = 2f;
        
//         lastCommandTime = Time.time;
//     }
    
//     private void moveVelocityRigidbody() {
        
//         if (Time.time - lastCommandTime > commandTimeout) {
//             // Stop the robot
//             rb.linearVelocity = Vector3.zero;
//             rb.angularVelocity = Vector3.zero;
//             return;
//         }
        
//         // Apply commanded velocity
//         rb.linearVelocity = transform.TransformDirection(new Vector3(-lvx, lvz, -lvy));
//         rb.angularVelocity = transform.TransformDirection(new Vector3(avx, -avz, avy));
//     }
    
//     public void moveVelocity(RosMessageTypes.Geometry.TwistMsg velocityMessage) {
//         this.lvx = (float)velocityMessage.linear.x;
//         this.lvy = (float)velocityMessage.linear.y;
//         this.lvz = (float)velocityMessage.linear.z;
//         this.avz = (float)velocityMessage.angular.z;
//         this.avy = (float)velocityMessage.angular.y;
//         this.avx = (float)velocityMessage.angular.x;
        
//         // Update last command time
//         lastCommandTime = Time.time;
//     }

//     void FixedUpdate() {
//         moveVelocityRigidbody();
//     }
    
//     void OnCollisionEnter(Collision collision) {
//         Debug.Log("Hit " + collision.gameObject.name);
//     }
// }

// using UnityEngine;
// using System;

// public class BlueRovVeloControl : MonoBehaviour 
// {
//     public Rigidbody rb;
    
//     // Initial positions
//     private Vector3 initialStonefishPos;
//     private Quaternion initialStonefishRot;
//     private Vector3 initialUnityPos;
//     private Quaternion initialUnityRot;
//     private bool initialized = false;
    
//     // Smoothing
//     public float positionSmoothTime = 0.05f;
//     private Vector3 velocitySmooth = Vector3.zero;
    
//     // Debug
//     public bool showDebugInfo = true;
    
//     void Start() 
//     {
//         this.rb = GetComponent<Rigidbody>();
        
//         if (rb != null)
//         {
//             rb.collisionDetectionMode = CollisionDetectionMode.Continuous;
//             rb.interpolation = RigidbodyInterpolation.Interpolate;
//             rb.useGravity = false;
//             rb.isKinematic = true;
//         }
        
//         // Store initial Unity position
//         initialUnityPos = transform.position;
//         initialUnityRot = transform.rotation;
        
//         Debug.Log($"Initial Unity Position: {initialUnityPos}, Rotation: {initialUnityRot.eulerAngles}");
//     }
    
//     public void UpdateOdometry(RosMessageTypes.Nav.OdometryMsg odomMessage)
//     {
//         // Get current Stonefish position (NED frame)
//         float ned_x = (float)odomMessage.pose.pose.position.x;
//         float ned_y = (float)odomMessage.pose.pose.position.y;
//         float ned_z = (float)odomMessage.pose.pose.position.z;
        
//         Vector3 currentStonefishPos = new Vector3(ned_x, ned_y, ned_z);
        
//         // Get current Stonefish orientation
//         float qx = (float)odomMessage.pose.pose.orientation.x;
//         float qy = (float)odomMessage.pose.pose.orientation.y;
//         float qz = (float)odomMessage.pose.pose.orientation.z;
//         float qw = (float)odomMessage.pose.pose.orientation.w;
        
//         Quaternion currentStonefishRot = new Quaternion(qx, qy, qz, qw);
        
//         // Initialize on first message
//         if (!initialized)
//         {
//             initialStonefishPos = currentStonefishPos;
//             initialStonefishRot = currentStonefishRot;
//             initialized = true;
//             Debug.Log($"Initial Stonefish Position: {initialStonefishPos}, Rotation: {currentStonefishRot.eulerAngles}");
//             return;
//         }
        
//         // Calculate delta position in Stonefish
//         Vector3 deltaPos = currentStonefishPos - initialStonefishPos;
        
//         // Convert NED delta to Unity delta
//         // NED: X=North, Y=East, Z=Down
//         // Unity: X=East, Y=Up, Z=North
//         Vector3 unityDelta = new Vector3(
//             -deltaPos.x,   // North -> X (forward in Stone = sway in Unity, so we swap)
//             -deltaPos.z,  // Up (negate Down) -> Y
//             deltaPos.y    // East -> Z (sway in Stone = forward in Unity, so we swap)
//         );
        
//         // Apply delta to initial Unity position
//         Vector3 targetPosition = initialUnityPos + unityDelta;
        
//         // Smooth movement
//         if (positionSmoothTime > 0)
//         {
//             transform.position = Vector3.SmoothDamp(
//                 transform.position,
//                 targetPosition,
//                 ref velocitySmooth,
//                 positionSmoothTime
//             );
//         }
//         else
//         {
//             transform.position = targetPosition;
//         }
        
//         // Calculate delta rotation in Stonefish
//         Quaternion deltaRot = currentStonefishRot * Quaternion.Inverse(initialStonefishRot);
        
//         // Convert NED quaternion delta to Unity
//         Quaternion unityDeltaRot = new Quaternion(
//             -deltaRot.y,
//             deltaRot.z,
//             -deltaRot.x,
//             deltaRot.w
//         );
        
//         // Apply delta rotation to initial Unity rotation
//         transform.rotation = initialUnityRot * unityDeltaRot;
        
//         if (showDebugInfo)
//         {
//             Debug.Log($"Delta: Stone({deltaPos.x:F3}, {deltaPos.y:F3}, {deltaPos.z:F3}) | Unity({unityDelta.x:F3}, {unityDelta.y:F3}, {unityDelta.z:F3}) | Pos: {transform.position}");
//         }
//     }
    
//     void OnCollisionEnter(Collision collision) 
//     {
//         Debug.Log("Hit " + collision.gameObject.name);
//     }
    
//     void OnDrawGizmos()
//     {
//         if (showDebugInfo && Application.isPlaying)
//         {
//             Gizmos.color = Color.green;
//             Gizmos.DrawWireSphere(transform.position, 0.3f);
            
//             Gizmos.color = Color.blue;
//             Gizmos.DrawRay(transform.position, transform.forward * 1f);
            
//             // Draw initial position
//             if (initialized)
//             {
//                 Gizmos.color = Color.yellow;
//                 Gizmos.DrawWireSphere(initialUnityPos, 0.2f);
//             }
//         }
//     }
    
//     // Optional: Reset calibration with a key press
//     void Update()
//     {
//         if (Input.GetKeyDown(KeyCode.R))
//         {
//             initialized = false;
//             Debug.Log("Reset calibration - will recalibrate on next odometry message");
//         }
//     }
// }




using UnityEngine;
using System;

public class BlueRovVeloControl : MonoBehaviour 
{
    public Rigidbody rb;
    
    // Coordinate system selection
    [Header("Coordinate System")]
    [Tooltip("True = Unity coordinates (no conversion), False = Stonefish NED coordinates (with conversion)")]
    public bool useUnityCoordinates = true;
    
    // Initial positions
    private Vector3 initialOdomPos;
    private Quaternion initialOdomRot;
    private Vector3 initialUnityPos;
    private Quaternion initialUnityRot;
    private bool initialized = false;
    
    // Smoothing
    [Header("Smoothing")]
    public float positionSmoothTime = 0.05f;
    private Vector3 velocitySmooth = Vector3.zero;
    
    // Debug
    [Header("Debug")]
    public bool showDebugInfo = true;
    
    void Start() 
    {
        this.rb = GetComponent<Rigidbody>();
        
        if (rb != null)
        {
            rb.collisionDetectionMode = CollisionDetectionMode.Continuous;
            rb.interpolation = RigidbodyInterpolation.Interpolate;
            rb.useGravity = false;
            rb.isKinematic = true;
        }
        
        // Store initial Unity position
        initialUnityPos = transform.position;
        initialUnityRot = transform.rotation;
        
        string coordSystem = useUnityCoordinates ? "Unity" : "Stonefish NED";
        Debug.Log($"BlueRovVeloControl: Using {coordSystem} coordinate system");
        Debug.Log($"Initial Unity Position: {initialUnityPos}, Rotation: {initialUnityRot.eulerAngles}");
    }
    
    public void UpdateOdometry(RosMessageTypes.Nav.OdometryMsg odomMessage)
    {
        // Get current odometry position
        float x = (float)odomMessage.pose.pose.position.x;
        float y = (float)odomMessage.pose.pose.position.y;
        float z = (float)odomMessage.pose.pose.position.z;
        
        Vector3 currentOdomPos = new Vector3(x, y, z);
        
        // Get current odometry orientation
        float qx = (float)odomMessage.pose.pose.orientation.x;
        float qy = (float)odomMessage.pose.pose.orientation.y;
        float qz = (float)odomMessage.pose.pose.orientation.z;
        float qw = (float)odomMessage.pose.pose.orientation.w;
        
        Quaternion currentOdomRot = new Quaternion(qx, qy, qz, qw);
        
        // Initialize on first message
        if (!initialized)
        {
            initialOdomPos = currentOdomPos;
            initialOdomRot = currentOdomRot;
            initialized = true;
            string coordSystem = useUnityCoordinates ? "Unity" : "Stonefish";
            Debug.Log($"Initial {coordSystem} Position: {initialOdomPos}, Rotation: {currentOdomRot.eulerAngles}");
            return;
        }
        
        // Calculate delta position in odometry frame
        Vector3 deltaPos = currentOdomPos - initialOdomPos;
        
        Vector3 unityDelta;
        
        if (useUnityCoordinates)
        {
            // Unity-to-Unity: No conversion needed
            unityDelta = deltaPos;
        }
        else
        {
            // Stonefish NED to Unity conversion
            // NED: X=North, Y=East, Z=Down
            // Unity: X=East, Y=Up, Z=North
            unityDelta = new Vector3(
                -deltaPos.x,   // North -> -X (swap to match sway)
                -deltaPos.z,   // Up (negate Down) -> Y
                deltaPos.y     // East -> Z (swap to match forward)
            );
        }
        
        // Apply delta to initial Unity position
        Vector3 targetPosition = initialUnityPos + unityDelta;
        
        // Smooth movement
        if (positionSmoothTime > 0)
        {
            transform.position = Vector3.SmoothDamp(
                transform.position,
                targetPosition,
                ref velocitySmooth,
                positionSmoothTime
            );
        }
        else
        {
            transform.position = targetPosition;
        }
        
        // Calculate delta rotation in odometry frame
        Quaternion deltaRot = currentOdomRot * Quaternion.Inverse(initialOdomRot);
        
        Quaternion unityDeltaRot;
        
        if (useUnityCoordinates)
        {
            // Unity-to-Unity: No conversion needed
            unityDeltaRot = deltaRot;
        }
        else
        {
            // Stonefish NED quaternion to Unity conversion
            unityDeltaRot = new Quaternion(
                -deltaRot.y,
                deltaRot.z,
                -deltaRot.x,
                deltaRot.w
            );
        }
        
        // Apply delta rotation to initial Unity rotation
        transform.rotation = initialUnityRot * unityDeltaRot;
        
        if (showDebugInfo)
        {
            if (useUnityCoordinates)
            {
                Debug.Log($"[Unity] Delta: ({deltaPos.x:F3}, {deltaPos.y:F3}, {deltaPos.z:F3}) | Pos: {transform.position}");
            }
            else
            {
                Debug.Log($"[Stonefish] Delta: Stone({deltaPos.x:F3}, {deltaPos.y:F3}, {deltaPos.z:F3}) | Unity({unityDelta.x:F3}, {unityDelta.y:F3}, {unityDelta.z:F3}) | Pos: {transform.position}");
            }
        }
    }
    
    void OnCollisionEnter(Collision collision) 
    {
        Debug.Log("Hit " + collision.gameObject.name);
    }
    
    void OnDrawGizmos()
    {
        if (showDebugInfo && Application.isPlaying)
        {
            // Current position
            Gizmos.color = Color.green;
            Gizmos.DrawWireSphere(transform.position, 0.3f);
            
            // Forward direction
            Gizmos.color = Color.blue;
            Gizmos.DrawRay(transform.position, transform.forward * 1f);
            
            // Initial position
            if (initialized)
            {
                Gizmos.color = Color.yellow;
                Gizmos.DrawWireSphere(initialUnityPos, 0.2f);
                
                // Line from initial to current
                Gizmos.color = Color.cyan;
                Gizmos.DrawLine(initialUnityPos, transform.position);
            }
        }
    }
    
    // Optional: Reset calibration with a key press
    void Update()
    {
        if (Input.GetKeyDown(KeyCode.R))
        {
            initialized = false;
            Debug.Log("Reset calibration - will recalibrate on next odometry message");
        }
        
        // Toggle coordinate system with 'C' key (for testing)
        if (Input.GetKeyDown(KeyCode.C))
        {
            useUnityCoordinates = !useUnityCoordinates;
            string coordSystem = useUnityCoordinates ? "Unity" : "Stonefish NED";
            Debug.Log($"Coordinate system changed to: {coordSystem}");
            initialized = false; // Reset on mode change
        }
    }
}