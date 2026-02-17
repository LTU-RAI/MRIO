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

class velocityModel1D():
    def systemDynamics(self, x, u, dt):
        v_x = x[0]
        acc_x = u[0]

        v_x_out = v_x + acc_x * dt

        return np.array([v_x_out])

    def getStateTransitionMatrix(self, x, u, dt):
        return np.array([[1.0]])

    def observationFunction(self, x):
        return x

    def getObservationMatrix(self, x):
        return np.array([[1.0]])

class EKFStage1(Node):
    def __init__(self):
        super().__init__('ekf_stage1')
        self.logger = self.get_logger()

        self.declare_parameter('baseLinkFrame', 'base_link')
        self.declare_parameter('egoVelocityWithCovarianceTopic', 'rio/ego_velocity_with_cov')
        self.declare_parameter('imuTopic', rclpy.Parameter.Type.STRING)
        self.declare_parameter('imuFrame', rclpy.Parameter.Type.STRING)
        self.declare_parameter('stage1_outputTopic', 'rio/stage1/acc_x')
        self.declare_parameter('useJosephForm', False)
        self.declare_parameter('useEgoVelCovarianceFromMessage', True)
        self.declare_parameter('stage1_processNoiseCovariance', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('stage1_egoVelCovariance', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('stage1_minimumEgoVelCovariance', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('stage1_egoVelCovarianceMultiplier', 1.0)
        self.declare_parameter('stage1_movingAverageSize', 50)
        self.declare_parameter('stage1_stateWithCovarianceTopic', 'rio/stage1/state_with_cov')

        self.baseLinkFrame = self.get_parameter('baseLinkFrame').value
        self.imuFrame = self.get_parameter('imuFrame').value

        self.useJosephForm = self.get_parameter('useJosephForm').value
        self.useEgoVelCovarianceFromMessage = self.get_parameter('useEgoVelCovarianceFromMessage').value
        
        if self.useEgoVelCovarianceFromMessage:
            self.egoVelCovarianceMultiplier = self.get_parameter('stage1_egoVelCovarianceMultiplier').value
            self.minimumEgoVelCovariance = self.get_parameter('stage1_minimumEgoVelCovariance').value
        else:
            self.measurementNoiseCovariance = np.array([[self.get_parameter('stage1_egoVelCovariance').value]])

        self.movingAvgSize = self.get_parameter('stage1_movingAverageSize').value
        self.movingAvgBuffer = np.zeros(self.movingAvgSize)
        self.movingAvgIndex = 0
        self.movingAvgBufferFilled = False

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

        self.model = velocityModel1D()
        self.P0 = np.array([[0.0]])
        self.processNoiseCovariance = np.array([[self.get_parameter('stage1_processNoiseCovariance').value]])

        self.create_subscription(TwistWithCovarianceStamped, self.get_parameter('egoVelocityWithCovarianceTopic').value, self.egoVelocityCallback, 10)
        self.create_subscription(Imu, self.get_parameter('imuTopic').value, self.imuCallback, qosProfile)        

        self.outputPublisher = self.create_publisher(Float64, self.get_parameter('stage1_outputTopic').value, 10)
        self.stateWithCovariancePublisher = self.create_publisher(StateWithCovarianceStamped, self.get_parameter('stage1_stateWithCovarianceTopic').value, 10)

        self.previousImuMsg = None
        self.previousVelX = None
        self.ekf = None

    def imuCallback(self, msg):
        self.logger.debug(f"IMU message received with timestamp [{msg.header.stamp.sec}.{msg.header.stamp.nanosec}]")
        
        if self.previousImuMsg is None:
            self.logger.info("First IMU message. Initializing EKF")
            self.ekf = EKF(self.model, np.array([0.0]), self.P0, self.useJosephForm)
            self.previousVelX = 0.0
        else:
            deltaSec = msg.header.stamp.sec - self.previousImuMsg.header.stamp.sec
            deltaNanoSec = msg.header.stamp.nanosec - self.previousImuMsg.header.stamp.nanosec            
            dt = deltaSec + 1e-9 * deltaNanoSec

            if dt <= 0:
                self.logger.debug("IMU delta time zero or negative.")
                self.previousImuMsg = msg
                return
            # Transform IMU readings to base link frame
            acc = msg.linear_acceleration
            acc_x,_,_ = self.imuRotationMatrix @ [acc.x, acc.y, acc.z]
            self.ekf.predict(np.array([acc_x]), self.processNoiseCovariance, dt)
            self.logger.debug(f"State vector after prediction step: {self.ekf.x}")
            self.logger.debug(f"State covariance after prediction step: {self.ekf.P}")
            acc_x_noisy = (self.ekf.x[0] - self.previousVelX) / dt
            self.previousVelX = self.ekf.x[0]

            self.movingAvgBuffer[self.movingAvgIndex] = acc_x_noisy
            if self.movingAvgIndex < (self.movingAvgSize - 1):
                self.movingAvgIndex += 1
            else:
                self.movingAvgIndex = 0
                self.movingAvgBufferFilled = True
            
            if self.movingAvgBufferFilled:
                acc_x_smoothed = np.sum(self.movingAvgBuffer) / self.movingAvgSize
            else:
                acc_x_smoothed = np.sum(self.movingAvgBuffer) / self.movingAvgIndex
            
            outputMsg = Float64(data = acc_x_smoothed)
            self.logger.info(f"Publishing filtered acceleration value: {acc_x_smoothed}")
            self.outputPublisher.publish(outputMsg)

            stateWithCovMsg = StateWithCovarianceStamped()
            stateWithCovMsg.header.stamp = msg.header.stamp
            stateWithCovMsg.state = self.ekf.x.tolist()
            stateWithCovMsg.covariance = self.ekf.P.flatten().tolist()
            self.stateWithCovariancePublisher.publish(stateWithCovMsg)
        
        self.previousImuMsg = msg

    def egoVelocityCallback(self, msg):
        self.logger.debug(f"Egovelocity message received with timestamp [{msg.header.stamp.sec}.{msg.header.stamp.nanosec}]")
        if self.previousImuMsg is None:
            self.logger.debug("Egovelocity received before EKF initialization, returning.")
            return

        if self.useEgoVelCovarianceFromMessage:
            egoVelXCovariance = msg.twist.covariance[0] * self.egoVelCovarianceMultiplier
            if egoVelXCovariance < self.minimumEgoVelCovariance:
                egoVelXCovariance = self.minimumEgoVelCovariance
            self.measurementNoiseCovariance = np.array([[egoVelXCovariance]])
            
        egoVelX = msg.twist.twist.linear.x
        self.ekf.update(np.array([egoVelX]), self.measurementNoiseCovariance)
        self.logger.debug(f"State vector after update step: {self.ekf.x}")
        self.logger.debug(f"State covariance after update step: {self.ekf.P}")

def main():
    rclpy.init()
    ekfStage1 = EKFStage1()
    try:
        rclpy.spin(ekfStage1)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()    

if __name__ == "__main__":
    main()