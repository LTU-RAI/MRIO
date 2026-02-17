import numpy as np
import rclpy
from rclpy.node import Node

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException

from tf_transformations import quaternion_matrix

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pcl2

class Mapping(Node):
    def __init__(self):
        super().__init__('mapping')
        self.logger = self.get_logger()

        self.declare_parameter('baseLinkFrame', 'base_link')
        self.declare_parameter('odomFrame', 'odom')
        self.declare_parameter('mergedPCLTopic', 'rio/merged_pcl')
        self.declare_parameter('useRANSAC', True)
        self.declare_parameter('ransacedPCLTopic', 'rio/ransaced_pcl')
        self.declare_parameter('mapTopic', 'rio/mapping/map')

        self.baseLinkFrame = self.get_parameter('baseLinkFrame').value
        self.odomFrame = self.get_parameter('odomFrame').value
        useRANSAC = self.get_parameter('useRANSAC').value

        if useRANSAC:
            self.create_subscription(PointCloud2, self.get_parameter('ransacedPCLTopic').value, self.PCLCallback, 10)
        else:
            self.create_subscription(PointCloud2, self.get_parameter('mergedPCLTopic').value, self.PCLCallback, 10)

        self.tfTimeout = 0.1
        tfNode = Node('tf_node')       
        self.tfBuffer = Buffer()
        self.tfListener = TransformListener(self.tfBuffer, tfNode, spin_thread=True)

        self.mapPublisher = self.create_publisher(PointCloud2, self.get_parameter('mapTopic').value, 10)

        self.outFields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                          PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                          PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                         ]
        self.map = np.zeros((0, 3), dtype=np.float32)

        self.publishEvery = 30
        self.pclIndex = 0
                
    def PCLCallback(self, msg):
        try:
            tf = self.tfBuffer.lookup_transform(
                self.odomFrame,
                self.baseLinkFrame,
                rclpy.time.Time.from_msg(msg.header.stamp),
                # rclpy.time.Time(),
                rclpy.duration.Duration(seconds=self.tfTimeout),
            )
        except TransformException as ex:
            self.logger.warning(f"Could not get transform for pointcloud: {ex}")
            return

        t = tf.transform.translation
        translation = np.array([t.x, t.y, t.z], dtype=np.float32)
        q = tf.transform.rotation
        rotationMatrix = quaternion_matrix([q.x, q.y, q.z, q.w])
        rotationMatrix = np.array(rotationMatrix[0:3, 0:3], dtype=np.float32)

        pcl = pcl2.read_points_numpy(msg, skip_nans=True, field_names = ('x', 'y', 'z'))
        # Transform pointcloud to odom frame
        transformedPCL = pcl @ rotationMatrix.T + translation

        # Add pointcloud to map
        self.map = np.vstack((self.map, transformedPCL))

        if self.pclIndex % self.publishEvery == 0:
            totalPoints = len(self.map)
            self.logger.info(f"Publishing map with a total of {totalPoints} points.")
            header = Header(stamp = self.get_clock().now().to_msg(), frame_id = self.odomFrame)
            pcl = PointCloud2(
                header=header,
                height=1, 
                width=totalPoints,
                is_dense=True,
                is_bigendian=False,
                fields=self.outFields,
                point_step=12,
                row_step=totalPoints*12,
                data=self.map.tobytes()
            )
            self.mapPublisher.publish(pcl)  
        
        self.pclIndex += 1

def main():
    rclpy.init()
    mapping = Mapping()
    try:
        rclpy.spin(mapping)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()    

if __name__ == "__main__":
    main()