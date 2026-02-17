import numpy as np
import rclpy
from rclpy.node import Node

import yaml

from std_msgs.msg import Header
from geometry_msgs.msg import Vector3, Twist, TwistStamped, TwistWithCovariance, TwistWithCovarianceStamped
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pcl2

from sklearn import linear_model

class EgoVelocityEstimator(Node):
    def __init__(self):
        super().__init__('ego_velocity_estimator')
        self.logger = self.get_logger()

        self.declare_parameter('baseLinkFrame', 'base_link')
        self.declare_parameter('mergedPCLTopic', 'rio/merged_pcl')
        self.declare_parameter('egoVelocityTopic', 'rio/ego_velocity')
        self.declare_parameter('egoVelocityWithCovarianceTopic', 'rio/ego_velocity_with_cov')
        self.declare_parameter('useRANSAC', True)
        self.declare_parameter('ransacedPCLTopic', 'rio/ransaced_pcl')
        self.declare_parameter('ransacUseRelativeTolerance', True)
        self.declare_parameter('ransacAbsoluteTolerance', 0.01)
        self.declare_parameter('ransacRelativeTolerance', 0.05)
        self.declare_parameter('ransacMaxTrials', 100)
        self.declare_parameter('ransacStopProbability', 0.99)

        self.baseLinkFrame = self.get_parameter('baseLinkFrame').value
        self.useRANSAC = self.get_parameter('useRANSAC').value
        self.ransacUseRelativeTolerance = self.get_parameter('ransacUseRelativeTolerance').value

        if self.useRANSAC:
            self.outFields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                              PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                              PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                              PointField(name='velocity', offset=12, datatype=PointField.FLOAT32, count=1),
                             ]
                        
            ransacedPCLTopic = self.useRANSAC = self.get_parameter('ransacedPCLTopic').value
            self.ransacedPCLPublisher = self.create_publisher(PointCloud2, ransacedPCLTopic, 10)
            self.estimator = linear_model.LinearRegression(fit_intercept=False)
            maxTrials = self.get_parameter('ransacMaxTrials').value
            stopProbability = self.get_parameter('ransacStopProbability').value            
            if self.ransacUseRelativeTolerance:
                self.ransac = linear_model.RANSACRegressor(self.estimator, residual_threshold=1.0, loss=self.ransacLossFunction, 
                                                           max_trials=maxTrials, stop_probability=stopProbability)
            else:
                tolerance = self.get_parameter('ransacAbsoluteTolerance').value
                self.ransac = linear_model.RANSACRegressor(self.estimator, residual_threshold=tolerance,
                                                           max_trials=maxTrials, stop_probability=stopProbability)


        self.egoVelocityPublisher = self.create_publisher(TwistStamped, self.get_parameter('egoVelocityTopic').value, 10)
        self.egoVelocityWithCovariancePublisher = self.create_publisher(TwistWithCovarianceStamped, self.get_parameter('egoVelocityWithCovarianceTopic').value, 10)
        self.create_subscription(PointCloud2, self.get_parameter('mergedPCLTopic').value, self.mergedPCLCallback, 10)

    def mergedPCLCallback(self, msg):
        self.logger.debug("Merged pointcloud received.")

        mergedPCL = pcl2.read_points_numpy(msg, skip_nans=True, field_names = ('x', 'y', 'z', 'velocity'))
        N = len(mergedPCL)
        if N < 4:
            self.logger.info(f"Ego velocity estimate impossible, too few points ({N})")
            return
        
        pointPositions = mergedPCL[:, 0:3]
        dopplerVelocities = mergedPCL[:, 3]

        distances = np.linalg.norm(pointPositions, axis=1, keepdims=True)
        unitVectors = pointPositions / distances

        if self.useRANSAC:
            if self.ransacUseRelativeTolerance:
                self.ransacAbsoluteTolerance = self.get_parameter('ransacAbsoluteTolerance').get_parameter_value().double_value
                self.ransacRelativeTolerance = self.get_parameter('ransacRelativeTolerance').get_parameter_value().double_value

            self.ransac.fit(unitVectors, -dopplerVelocities)
            self.logger.debug(f"Number of RANSAC trials: {self.ransac.n_trials_}")
            inliers = self.ransac.inlier_mask_
            self.ransacedPCL = mergedPCL[inliers]
            N = len(self.ransacedPCL)
            self.publishRansacedPCL(N, N / len(mergedPCL))
            egoVel = self.ransac.estimator_.coef_.astype(np.float64)
            inlierUnitVectors = unitVectors[inliers]
            inlierDopplerVelocities = dopplerVelocities[inliers]
            residuals = (-inlierDopplerVelocities - self.ransac.estimator_.predict(inlierUnitVectors))
            SSE = np.array([np.dot(residuals, residuals)])
        else:
            leastSquareSolution = np.linalg.lstsq(unitVectors, -dopplerVelocities, rcond=None)
            egoVel = leastSquareSolution[0].astype(np.float64)
            SSE = leastSquareSolution[1]

        covariance = np.zeros((6,6))
        if len(SSE) == 1 and N >= 4:
            estimatedOutputVariance = SSE / (N - 3)
            estimatedVelocityCovariance = estimatedOutputVariance * np.linalg.inv(unitVectors.T @ unitVectors)
            covariance[0:3, 0:3] = estimatedVelocityCovariance
        else:
            if len(SSE) != 1:
                self.logger.warning("Singular matrix in least square ego velocity estimation. Can't estimate covariance matrix.")
            if N < 4:
                self.logger.warning("Too few points left after RANSAC. Can't estimate covariance matrix.")
            covariance[0:3, 0:3] = 1e6
        
        self.logger.info("Publishing ego velocity estimate.")
        # header = Header(stamp = self.get_clock().now().to_msg(), frame_id = self.baseLinkFrame)
        header = msg.header # Publish with the same timestamp as input pointcloud instead of current time
        twist = Twist(linear = Vector3(x=egoVel[0], y=egoVel[1], z=egoVel[2]))
        twistMsg = TwistStamped(header = header, twist = twist)     
        self.egoVelocityPublisher.publish(twistMsg)
        twistWithCovMsg = TwistWithCovarianceStamped(header = header, twist = TwistWithCovariance(covariance = covariance.reshape(36), twist = twist))
        self.egoVelocityWithCovariancePublisher.publish(twistWithCovMsg)

    def ransacLossFunction(self, true, predicted):
        totalTolerance = self.ransacAbsoluteTolerance + self.ransacRelativeTolerance * abs(predicted)
        return abs(true - predicted) / totalTolerance

    def publishRansacedPCL(self, totalPoints, ratio):
        self.logger.info(f"Publishing RANSACed pointcloud with a total of {totalPoints} points ({ratio:.0%} of original).")
        header = Header(stamp = self.get_clock().now().to_msg(), frame_id = self.baseLinkFrame)            
        pcl = PointCloud2(
            header=header,
            height=1, 
            width=totalPoints,
            is_dense=True,
            is_bigendian=False,
            fields=self.outFields,
            point_step=16,
            row_step=totalPoints*16,
            data=self.ransacedPCL.tobytes()
        )
        self.ransacedPCLPublisher.publish(pcl)

def main():
    rclpy.init()
    egoVelocityEstimator = EgoVelocityEstimator()
    try:
        rclpy.spin(egoVelocityEstimator)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()    

if __name__ == "__main__":
    main()