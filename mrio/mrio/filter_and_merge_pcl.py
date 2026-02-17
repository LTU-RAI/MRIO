from dataclasses import dataclass

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import quaternion_matrix

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

import yaml

from std_msgs.msg import Header
from sensor_msgs.msg import PointField, PointCloud2
from sensor_msgs_py import point_cloud2 as pcl2

from ament_index_python.packages import get_package_share_directory
import os

qosProfile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)

@dataclass
class Radar:
    topic: str
    dopplerVelocityFieldName: str
    rotationMatrix: np.ndarray
    translation: np.ndarray
    innerRadius: float
    outerRadius: float
    pclData: np.ndarray = None
    receivedNewData: bool = False

class RadarPointcloudFiltererAndMerger(Node):
    def __init__(self):
        super().__init__('radar_pointcloud_filterer_and_merger')
        self.logger = self.get_logger()

        self.declare_parameter('radarSetupFile', 'radar_setup.yaml')
        self.declare_parameter('baseLinkFrame', 'base_link')
        self.declare_parameter('mergedPCLTopic', 'rio/merged_pcl')
        self.declare_parameter('useIndividualRadii', False)
        self.declare_parameter('globalInnerRadius', 0.0)
        self.declare_parameter('globalOuterRadius', np.inf)
        self.declare_parameter('discardPointsBelowZValue', -np.inf)
        
        pkg_path = get_package_share_directory("mrio")
        radarSetupFileName = os.path.join(pkg_path, "config", self.get_parameter('radarSetupFile').value)
        
        with open(radarSetupFileName, 'r') as file:
            radarSetup = yaml.safe_load(file)

        self.radarPublishPeriod = radarSetup['radarPublishPeriod']
        self.baseLinkFrame = self.get_parameter('baseLinkFrame').value

        self.mergedPointcloudPublisher = self.create_publisher(PointCloud2, self.get_parameter('mergedPCLTopic').value, 10)
        self.outFields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                          PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                          PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                          PointField(name='velocity', offset=12, datatype=PointField.FLOAT32, count=1),
                         ]

        tfBuffer = Buffer()
        tfListener = TransformListener(tfBuffer, self)
        self.radars = []
        for radarIndex, radar in enumerate(radarSetup['radars']):
            topic = radar['topic']
            dopplerVelocityFieldName = radar['dopplerVelocityFieldName']

            innerRadius = radar['innerRadius']
            outerRadius = radar['outerRadius']

            # Get TF to transform cloud from each radar to target frame
            transformReady = tfBuffer.wait_for_transform_async(self.baseLinkFrame, radar['frame'], rclpy.time.Time())
            self.logger.info(f"Waiting for TF for radar {radarIndex}")
            rclpy.spin_until_future_complete(self, transformReady)
            self.logger.info(f"Received TF for radar {radarIndex}")
            trans = tfBuffer.lookup_transform(self.baseLinkFrame, radar['frame'], rclpy.time.Time())
            t = trans.transform.translation
            translation = np.array([t.x, t.y, t.z], dtype=np.float32)
            q = trans.transform.rotation
            rotationMatrix = quaternion_matrix([q.x, q.y, q.z, q.w])
            rotationMatrix = np.array(rotationMatrix[0:3, 0:3], dtype=np.float32)
            
            self.radars.append(Radar(topic, dopplerVelocityFieldName, rotationMatrix, translation, innerRadius, outerRadius))
            self.create_subscription(PointCloud2, topic, lambda msg, idx = radarIndex: self.radarIndataCallback(msg, idx), qosProfile)

        if len(self.radars) > 1:
            self.create_timer(self.radarPublishPeriod, self.mergeAndPublish, clock=self.get_clock())

    def radarIndataCallback(self, msg, radarIndex):
        radar = self.radars[radarIndex]
        self.logger.debug(f"Data received from radar {radarIndex}.")

        inData = pcl2.read_points_numpy(msg, skip_nans=True, field_names = ('x', 'y', 'z', radar.dopplerVelocityFieldName))
        pointPositions = inData[:, 0:3]
        
        # Transform pointcloud to target frame
        pointPositions[:] = pointPositions @ radar.rotationMatrix.T + radar.translation
        radar.pclData = inData
        self.filterPointsByGeometry(radar)

        radar.receivedNewData = True
        if len(self.radars) == 1:
            self.mergeAndPublish()

    def filterPointsByGeometry(self, radar):
        discardPointsBelowZValue = self.get_parameter('discardPointsBelowZValue').get_parameter_value().double_value
        useIndividualRadii = self.get_parameter('useIndividualRadii').get_parameter_value().bool_value
        if useIndividualRadii:
            innerRadius = radar.innerRadius
            outerRadius = radar.outerRadius
        else:
            innerRadius = self.get_parameter('globalInnerRadius').get_parameter_value().double_value
            outerRadius = self.get_parameter('globalOuterRadius').get_parameter_value().double_value

        distances = np.linalg.norm(radar.pclData[:, 0:3], axis=1)
        radar.pclData = radar.pclData[(distances > innerRadius) & (distances < outerRadius) & (radar.pclData[:, 2] > discardPointsBelowZValue)]

    def mergeAndPublish(self):
        mergedData = np.empty((0, 4), dtype=np.float32)
        for radarIndex, radar in enumerate(self.radars):
            if radar.receivedNewData:
                mergedData = np.concatenate((mergedData, radar.pclData), axis=0)
                radar.receivedNewData = False
                self.logger.debug(f"Merge: Got new data from radar {radarIndex}.")
            else:
                self.logger.debug(f"Merge: NO new data from radar {radarIndex}.")
                
        totalPoints = len(mergedData)
        if totalPoints > 0:
            self.logger.info(f"Publishing merged pointcloud with a total of {totalPoints} points.")
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
                data=mergedData.tobytes()
            )
            self.mergedPointcloudPublisher.publish(pcl)
        else:
            self.logger.info("No new data since last publish.")

def main():
    rclpy.init()
    radarPointcloudFiltererAndMerger = RadarPointcloudFiltererAndMerger()
    try:
        rclpy.spin(radarPointcloudFiltererAndMerger)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()    

if __name__ == "__main__":
    main()