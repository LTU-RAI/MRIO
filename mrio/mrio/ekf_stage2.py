import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from tf_transformations import quaternion_matrix, quaternion_multiply, euler_from_quaternion, quaternion_from_euler

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Vector3, TwistStamped, TwistWithCovarianceStamped, TransformStamped
from sensor_msgs.msg import Imu
from rio_msgs.msg import StateWithCovarianceStamped

from example_interfaces.msg import Float64

from ekf import EKF

qosProfile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)

class UGVModel2D_noBias():
    def __init__(self):
        self.estimatedYawRate = 0.0

    def systemDynamics(self, x, u, dt):
        p_x = x[0]
        p_y = x[1]
        v_x = x[2]
        yaw = x[3]
        
        acc_x = u[0]
        omega_z = u[1]

        forwardShift = v_x * dt
        
        p_x_out = p_x + np.cos(yaw) * forwardShift
        p_y_out = p_y + np.sin(yaw) * forwardShift
        v_x_out = v_x + acc_x * dt
        self.estimatedYawRate = omega_z
        yaw_out = yaw + self.estimatedYawRate * dt

        return np.array([p_x_out, p_y_out, v_x_out, yaw_out])

    def getStateTransitionMatrix(self, x, u, dt):
        cy = np.cos(x[3])
        sy = np.sin(x[3])

        return np.array([[1, 0, cy*dt, -sy*x[2]*dt],
                         [0, 1, sy*dt,  cy*x[2]*dt],
                         [0, 0, 1, 0],
                         [0, 0, 0, 1]
                        ])

    def observationFunction(self, x):
        return np.array([x[2], x[3]])

    def getObservationMatrix(self, x):
        return np.array([[0, 0, 1, 0],
                         [0, 0, 0, 1]])          

class EKFSensorFusion(Node):
    def __init__(self):
        super().__init__('ekf_sensor_fusion')
        self.logger = self.get_logger()

        self.declare_parameter('baseLinkFrame', 'base_link')
        self.declare_parameter('egoVelocityWithCovarianceTopic', 'rio/ego_velocity_with_cov')
        self.declare_parameter('imuTopic', rclpy.Parameter.Type.STRING)
        self.declare_parameter('imuFrame', rclpy.Parameter.Type.STRING)
        self.declare_parameter('outputPoseTopic', 'rio/odometry/pose')
        self.declare_parameter('outputTwistTopic', 'rio/odometry/twist')
        self.declare_parameter('odomFrame', 'odom')
        self.declare_parameter('stage1_outputTopic', 'rio/stage1/acc_x')
        self.declare_parameter('useEgoVelCovarianceFromMessage', True)
        self.declare_parameter('stage2_processNoiseCovariance', rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('stage2_initialStateCovariance', rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('stage2_egoVelCovariance', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('stage2_minimumEgoVelCovariance', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('stage2_egoVelCovarianceMultiplier', 1.0)
        self.declare_parameter('stage2_yawAngleCovariance', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('useJosephForm', False)
        self.declare_parameter('stage2_stateWithCovarianceTopic', 'rio/stage2/state_with_cov')

        self.baseLinkFrame = self.get_parameter('baseLinkFrame').value
        self.imuFrame = self.get_parameter('imuFrame').value
        self.odomFrame = self.get_parameter('odomFrame').value

        self.useJosephForm = self.get_parameter('useJosephForm').value
        self.useEgoVelCovarianceFromMessage = self.get_parameter('useEgoVelCovarianceFromMessage').value
        self.yawAngleCovariance = self.get_parameter('stage2_yawAngleCovariance').value
        
        if self.useEgoVelCovarianceFromMessage:
            self.egoVelCovarianceMultiplier = self.get_parameter('stage2_egoVelCovarianceMultiplier').value
            self.minimumEgoVelCovariance = self.get_parameter('stage2_minimumEgoVelCovariance').value
        else:
            egoVelXCovariance = self.get_parameter('stage2_egoVelCovariance').value
            self.measurementNoiseCovariance = np.diag([egoVelXCovariance, self.yawAngleCovariance])

        # Get TF to transform IMU readings to base link frame
        tfBuffer = Buffer()
        tfListener = TransformListener(tfBuffer, self)        
        transformReady = tfBuffer.wait_for_transform_async(self.baseLinkFrame, self.imuFrame, rclpy.time.Time())
        
        self.logger.info(f"Waiting for TF for IMU")
        rclpy.spin_until_future_complete(self, transformReady)
        trans = tfBuffer.lookup_transform(self.baseLinkFrame, self.imuFrame, rclpy.time.Time())
        self.logger.info(f"Received TF for IMU")

        q = trans.transform.rotation
        qArray = np.array([q.x, q.y, q.z, q.w])
        self.imuRotationMatrix = quaternion_matrix(qArray)
        self.imuRotationMatrix = np.array(self.imuRotationMatrix[0:3, 0:3], dtype=np.float32)    
        self.imuQuart = qArray

        self.model = UGVModel2D_noBias()
        self.processNoiseCovariance = np.diag(self.get_parameter('stage2_processNoiseCovariance').value)
        self.P0 = np.diag(self.get_parameter('stage2_initialStateCovariance').value)

        self.create_subscription(TwistWithCovarianceStamped, self.get_parameter('egoVelocityWithCovarianceTopic').value, self.egoVelocityCallback, 10)
        self.create_subscription(Imu, self.get_parameter('imuTopic').value, self.imuCallback, qosProfile)
        self.create_subscription(Float64, self.get_parameter('stage1_outputTopic').value, self.stage1OutputCallback, 10)        

        self.outputPosePublisher = self.create_publisher(PoseStamped, self.get_parameter('outputPoseTopic').value, 10)
        self.outputTwistPublisher = self.create_publisher(TwistStamped, self.get_parameter('outputTwistTopic').value, 10)
        self.tfBroadcaster = TransformBroadcaster(self)

        self.stateWithCovariancePublisher = self.create_publisher(StateWithCovarianceStamped, self.get_parameter('stage2_stateWithCovarianceTopic').value, 10)

        self.previousImuMsg = None
        self.ekf = None
        self.stage1Output = None

    def imuCallback(self, msg):
        self.logger.debug(f"IMU message received with timestamp [{msg.header.stamp.sec}.{msg.header.stamp.nanosec}]")
        
        if self.previousImuMsg is None:
            self.logger.info("First IMU message. Establishing initial yaw angle and initializing EKF")
            q = msg.orientation
            qBaseLinkFrame = quaternion_multiply(self.imuQuart, [q.x, q.y, q.z, q.w])
            _,_,self.initialYaw = euler_from_quaternion(qBaseLinkFrame)
            self.ekf = EKF(self.model, np.array([0.0, 0.0, 0.0, 0.0]), self.P0, self.useJosephForm)
        else:
            deltaSec = msg.header.stamp.sec - self.previousImuMsg.header.stamp.sec
            deltaNanoSec = msg.header.stamp.nanosec - self.previousImuMsg.header.stamp.nanosec            
            dt = deltaSec + 1e-9 * deltaNanoSec

            if dt <= 0:
                self.logger.debug("IMU delta time zero or negative.")
                self.previousImuMsg = msg
                return
            if self.stage1Output is not None:
                # Get filtered acceleration output from stage 1
                acc_x = self.stage1Output
            else:
                # Transform IMU readings to base link frame
                acc = msg.linear_acceleration
                acc_x,_,_ = self.imuRotationMatrix @ [acc.x, acc.y, acc.z]

            angVel = msg.angular_velocity
            _,_,omega_z = self.imuRotationMatrix @ [angVel.x, angVel.y, angVel.z]
            self.ekf.predict(np.array([acc_x, omega_z]), self.processNoiseCovariance, dt)
            self.logger.debug(f"State vector after prediction step: {self.ekf.x}")
            self.logger.debug(f"State covariance after prediction step: {self.ekf.P}")
            self.publishOdomAndTransform()
        
        self.previousImuMsg = msg

    def egoVelocityCallback(self, msg):
        self.logger.debug(f"Egovelocity message received with timestamp [{msg.header.stamp.sec}.{msg.header.stamp.nanosec}]")
        if self.previousImuMsg is None:
            self.logger.debug("Egovelocity received before EKF initialization, returning.")
            return
        
        q = self.previousImuMsg.orientation
        qBaseLinkFrame = quaternion_multiply(self.imuQuart, [q.x, q.y, q.z, q.w])
        _,_,currentYaw = euler_from_quaternion(qBaseLinkFrame)
        currentYawOdomFrame = currentYaw - self.initialYaw
        
        # Code to avoid singularity problems in orientation when calculating the innovation, y - h(x)
        angleDifference = currentYawOdomFrame - self.ekf.x[3]
        if abs(angleDifference) > np.pi:
            numberOfRevolutions = (abs(angleDifference) + np.pi) // (2*np.pi)
            currentYawOdomFrame -= np.sign(angleDifference)* 2*np.pi * numberOfRevolutions

        if self.useEgoVelCovarianceFromMessage:
            egoVelXCovariance = msg.twist.covariance[0] * self.egoVelCovarianceMultiplier
            if egoVelXCovariance < self.minimumEgoVelCovariance:
                egoVelXCovariance = self.minimumEgoVelCovariance
            self.measurementNoiseCovariance = np.diag([egoVelXCovariance, self.yawAngleCovariance])
            
        egoVelX = msg.twist.twist.linear.x
        self.ekf.update(np.array([egoVelX, currentYawOdomFrame]), self.measurementNoiseCovariance)
        self.logger.debug(f"State vector after update step: {self.ekf.x}")
        self.logger.debug(f"State covariance after update step: {self.ekf.P}")
        self.publishOdomAndTransform()

    def stage1OutputCallback(self, msg):
        self.stage1Output = msg.data

    def publishOdomAndTransform(self):
        timeStamp = self.get_clock().now().to_msg()
        position = Point(x=self.ekf.x[0], y=self.ekf.x[1], z=0.0)
        translation = Vector3(x=self.ekf.x[0], y=self.ekf.x[1], z=0.0)
        q = quaternion_from_euler(0.0, 0.0, self.ekf.x[3])
        orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        linearVelocity = Vector3(x=self.ekf.x[2], y=0.0, z=0.0)
        angularVelocity = Vector3(x=0.0, y=0.0, z=self.model.estimatedYawRate)
        
        poseMsg = PoseStamped()
        poseMsg.header = Header(stamp = timeStamp, frame_id = self.odomFrame)
        poseMsg.pose.position = position
        poseMsg.pose.orientation = orientation
        self.outputPosePublisher.publish(poseMsg)
        
        twistMsg = TwistStamped()
        twistMsg.header = Header(stamp = timeStamp, frame_id = self.baseLinkFrame)
        twistMsg.twist.linear = linearVelocity
        twistMsg.twist.angular = angularVelocity
        self.outputTwistPublisher.publish(twistMsg)

        transformMsg = TransformStamped()
        transformMsg.header = poseMsg.header
        transformMsg.child_frame_id = self.baseLinkFrame
        transformMsg.transform.translation = translation
        transformMsg.transform.rotation = orientation
        self.tfBroadcaster.sendTransform(transformMsg)

        stateWithCovMsg = StateWithCovarianceStamped()
        stateWithCovMsg.header.stamp = timeStamp
        stateWithCovMsg.state = self.ekf.x.tolist()
        stateWithCovMsg.covariance = self.ekf.P.flatten().tolist()
        self.stateWithCovariancePublisher.publish(stateWithCovMsg)         

def main():
    rclpy.init()
    ekfSensorFusion = EKFSensorFusion()
    try:
        rclpy.spin(ekfSensorFusion)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()    

if __name__ == "__main__":
    main()