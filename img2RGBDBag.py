import time, sys, os
from ros import rosbag
import roslib
import rospy
import numpy
import cv2
roslib.load_manifest('sensor_msgs')
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
# import ImageFile
from PIL import ImageFile
from PIL import Image as ImagePIL

# get image from dir
def GetFilesFromDir(dir):
    # Generates a list of files from the directory
    print( "Searching directory %s" % dir )
    rgb_files = []
    depth_files = []
    if os.path.exists(dir):
        for path, names, files in os.walk(dir + '/rgb'):
            for f in sorted(files):
                rgb_files.append( os.path.join( path, f ) )
        for path, names, files in os.walk(dir + '/depth'):
            for f in sorted(files):
                depth_files.append( os.path.join( path, f ) )
    return rgb_files, depth_files

def CreateRGBDBag(rgb_files, depth_files, bagname):
    # create a rosbag from files
    cb = CvBridge()
    bag = rosbag.Bag(bagname, 'w')
    width = 640
    height = 480

    try:

        for i in range(len(depth_files)):
            img = cv2.imread(depth_files[i], cv2.IMREAD_UNCHANGED)# reads the image as is from the source
            print("Adding %s" % depth_files[i])
            
            (time,extension) = os.path.splitext(os.path.basename(depth_files[i]))
            floattime = float(time)

            image = cb.cv2_to_imgmsg(img, encoding='16UC1')# 16-bit grayscale
            image.header.stamp = rospy.rostime.Time.from_sec(floattime)
            image.header.frame_id = "camera/aligned_depth_to_color"

            bag.write('camera/aligned_depth_to_color/image_raw', image, rospy.rostime.Time.from_sec(floattime))

        for i in range(len(rgb_files)):
            img = cv2.imread(rgb_files[i], cv2.IMREAD_UNCHANGED)
            print("Adding %s" % rgb_files[i])

            (time,extension) = os.path.splitext(os.path.basename(rgb_files[i]))
            floattime = float(time)

            image = cb.cv2_to_imgmsg(img, encoding='bgr8')
            image.header.stamp = rospy.rostime.Time.from_sec(floattime)
            image.header.frame_id = "camera/color"

            bag.write('camera/color/image_raw', image, rospy.rostime.Time.from_sec(floattime))

    finally:
        bag.close()


def CreateBag(args):
    '''Creates the actual bag file by successively adding images'''
    rgb_files, depth_files = GetFilesFromDir(args[0])
    CreateRGBDBag(rgb_files, depth_files, args[1])

if __name__ == "__main__":
    if len( sys.argv ) == 3:
        CreateBag(sys.argv[1:])
    else:
        print( "Usage: img2RGBDBag.py rootdir bagfilename")
        print( "Example: python img2RGBDBag.py /media/c/F/dataset/re-edit-bag test.bag")
