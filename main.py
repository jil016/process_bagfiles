import rosbag
import yaml
import cv2
from cv_bridge import CvBridge

'''
Write
    def write(self, topic, msg, t=None, raw=False, connection_header=None):
        """
        Write a message to the bag.
        @param topic: name of topic
        @type  topic: str
        @param msg: message to add to bag, or tuple (if raw)
        @type  msg: Message or tuple of raw message data
        @param t: ROS time of message publication, if None specifed, use current time [optional]
        @type  t: U{genpy.Time}
        @param raw: if True, msg is in raw format, i.e. (msg_type, serialized_bytes, md5sum, pytype)
        @type  raw: bool
        @raise ValueError: if arguments are invalid or bag is closed
        """
'''

def readBagInfo(bag):
    return yaml.load(bag._get_yaml_info())

def readBagTopicList(bag):
    topics = bag.get_type_and_topic_info()[1].keys()
    types = []
    for i in range(0,len(bag.get_type_and_topic_info()[1].values())):
        types.append(bag.get_type_and_topic_info()[1].values()[i][0])

    return topics

def modifyMITMedfield(inpath='/home/erl/rosbag/mit_medfield.bag', outpath='/home/erl/rosbag/mit_medfield_modified.bag'):
    '''
    Topic list:
    /flea3/camera_info
    /flea3/image_raw
    /mavros/distance_sensor/lidarlite_pub
    /mavros/imu/atm_pressure
    /mavros/imu/data
    /mavros/imu/data_raw
    /mavros/imu/mag
    /mavros/local_position/pose
    /mavros/local_position/velocity
    /mavros/rc/in
    /mavros/state
    '''
    bag = rosbag.Bag(inpath, 'r')
    topic_list = readBagTopicList(bag)

    with rosbag.Bag(outpath, 'w') as outbag:
        for idx, topic_name in enumerate(topic_list):
            for topic, msg, t in bag.read_messages(topics=[topic_name]):
            # This also replaces tf timestamps under the assumption 
            # that all transforms in the message share the same timestamp
                if topic == "/flea3/camera_info": # add K, D
                    msg.K = (484.0, 0.0, 320.0, 0.0, 484.0, 262.5, 0.0, 0.0, 1.0)
                    msg.D = (0.0, 0.0, 0.0, 0.0, 0.0)
                    outbag.write(topic, msg, t)
                elif topic == "/flea3/image_raw": # rotate 180 deg
                    bridge = CvBridge()
                    
                    orig_img = bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
                    rot_img = cv2.rotate(orig_img, cv2.ROTATE_180)
                    rot_msg = bridge.cv2_to_imgmsg(rot_img, encoding="mono8")
                    rot_msg.header = msg.header
                    outbag.write(topic, rot_msg, t)
                else:
                    outbag.write(topic, msg, t)
            
            print("Finished processing topic: {}; Progress: {}/{}".format(topic_name, idx, len(topic_list)))
            
    print("DONE!")
    return

def testModifiedBag(mdf_path="/home/erl/rosbag/mit_medfield_modified.bag"):
    '''
    Compare timestamps:
    '''    
    bag = rosbag.Bag(mdf_path, 'r')
    topic_list = readBagTopicList(bag)

    for topic_name in topic_list:
        for topic, msg, t in bag.read_messages(topics=[topic_name]):
            # timestamps should be in the same range but slightly different
            print("Topic name: {}; Sync timestamp: {}; Header timestamp: {}".format(topic, msg.header.stamp, t))
            break
    return

def testPhoenixTimestamps(inpath="/home/erl/rosbag/phoenix/outdoor_joy_2020-10-18-13-03-47.bag"):
    bag = rosbag.Bag(inpath, 'r')
    info_dict = readBagInfo(bag)
    start_time = info_dict['start']
    end_time = info_dict['end']
    duration = info_dict['duration']
    imu_topic = "/acl_jackal/forward/imu"
    image_topic = "/acl_jackal/forward/color/image_rect"

    imu_frames = []
    image_frames = []

    # IMU data
    for topic, msg, t in bag.read_messages(topics=[imu_topic]):
        a = 1
    


    return

if __name__ == '__main__':
    # modifyMITMedfield()
    # testModifiedBag()
    testPhoenixTimestamps()
    
    