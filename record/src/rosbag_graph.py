#!/usr/bin/env python3
import argparse
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import matplotlib.pyplot as plt

def read_ros2_bag(bag_path: str):
    reader = rosbag2_py.SequentialReader()
    
    storage_options = rosbag2_py.StorageOptions(
        uri=bag_path,
        storage_id='mcap'
    )
    
    converter_options = rosbag2_py.ConverterOptions('', '')
    
    reader.open(storage_options, converter_options)
    
    topics_and_types = reader.get_all_topics_and_types()
    
    # ex.) {'/some_topic': 'std_msgs/msg/String', '/another_topic': 'sensor_msgs/msg/Image', ...}
    type_dict = { t.name: t.type for t in topics_and_types }

    topic_messages = {t.name: [] for t in topics_and_types}
    
    print("== topics ==")
    for t_name, t_type in type_dict.items():
        print(f"{t_name}: {t_type}")
    print("================\n")
    
    while reader.has_next():
        topic_name, data, t_stamp = reader.read_next()
        
        msg_type_str = type_dict[topic_name]
        msg_type = get_message(msg_type_str)
        
        msg = deserialize_message(data, msg_type)
        topic_messages[topic_name].append((t_stamp, msg))
    
    print("== topic name ==")
    for t_name, msgs in topic_messages.items():
        print(f"{t_name}: {len(msgs)} messages")
    
    x_c = []
    y_c = []
    p_c = []
    q_c = []
    B_c = []
    x_c_des = []
    y_c_des = []
    p_c_des = []
    q_c_des = []
    B_c_des = []
    ts = []
    for _ts, msg in topic_messages["/sim/cluster_info"]:
        ts.append(_ts)
        x_c.append(msg.cluster.data[0])
        y_c.append(msg.cluster.data[1])
        p_c.append(msg.cluster.data[6])
        q_c.append(msg.cluster.data[7])
        B_c.append(msg.cluster.data[8])
        x_c_des.append(msg.cluster_desired.data[0])
        y_c_des.append(msg.cluster_desired.data[1])
        p_c_des.append(msg.cluster_desired.data[6])
        q_c_des.append(msg.cluster_desired.data[7])
        B_c_des.append(msg.cluster_desired.data[8])
    ts = [(t - ts[0])/1000000000 for t in ts]
    print(x_c)
    print(y_c)
    plt.plot(x_c_des, y_c_des, label="Cluster Desired")
    plt.plot(x_c, y_c, label="Cluster")

    plt.title("Sample Line Graph")
    plt.xlabel("X-axis")
    plt.ylabel("Y-axis")
    plt.legend()
    plt.show()

    plt.figure()
    plt.plot(ts, p_c_des, label="desired p")
    plt.plot(ts, p_c, label="actual p")

    plt.title("Sample Line Graph")
    plt.xlabel("t")
    plt.ylabel("p")
    plt.legend()
    plt.show()

    plt.figure()
    plt.plot(ts, q_c_des, label="desired q")
    plt.plot(ts, q_c, label="actual q")

    plt.title("Sample Line Graph")
    plt.xlabel("t")
    plt.ylabel("q")
    plt.legend()
    plt.show()

    plt.figure()
    plt.plot(ts, B_c_des, label="desired B")
    plt.plot(ts, B_c, label="actual B")

    plt.title("Sample Line Graph")
    plt.xlabel("t")
    plt.ylabel("B")
    plt.legend()
    plt.show()


def main():
    parser = argparse.ArgumentParser(
        description="ROS 2 bag reader"
    )
    
    parser.add_argument(
        "bag_path", 
        help="ROS 2 bag path"
    )
    args = parser.parse_args()
    
    read_ros2_bag(args.bag_path)

if __name__ == "__main__":
    main()