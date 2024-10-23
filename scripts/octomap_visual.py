import rospy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from octomap_msgs.msg import Octomap
import octomap

def octomap_callback(msg):
    # Chuyển đổi thông điệp Octomap thành OcTree
    tree = octomap.OcTree()
    tree.readBinary(msg.data)

    # Khởi tạo matplotlib 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Duyệt qua các nút trong Octree và hiển thị các ô bị chiếm
    for node in tree:
        if tree.isNodeOccupied(node):
            x, y, z = node.getX(), node.getY(), node.getZ()
            ax.scatter(x, y, z, c='red', marker='s')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    
    plt.show()

def main():
    rospy.init_node('octomap_visualization_node')
    rospy.Subscriber("/octomap_binary", Octomap, octomap_callback)
    rospy.spin()

if __name__ == "__main__":
    main()
