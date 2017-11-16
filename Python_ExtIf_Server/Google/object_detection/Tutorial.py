import numpy as np
import os
import six.moves.urllib as urllib
import sys
import tarfile
import tensorflow as tf
import zipfile
import io

from collections import defaultdict
from io import StringIO
from matplotlib import pyplot as plt
from PIL import Image

# This is needed to display the images.
# %matplotlib inline

# This is needed since the notebook is stored in the object_detection folder.
from lib.ExtIf.ttypes import TDataResult

sys.path.append("..")

from utils import label_map_util

from utils import visualization_utils as vis_util

tf_session = None
# detection_graph = None
# What model to download.
MODEL_NAME = 'ssd_mobilenet_v1_coco_11_06_2017'
MODEL_FILE = MODEL_NAME + '.tar.gz'
DOWNLOAD_BASE = 'http://download.tensorflow.org/models/object_detection/'

#todo: anpasen
# Path to frozen detection graph. This is the actual model that is used for the object detection.
PATH_TO_CKPT = os.path.join('/home/aadc/ADTF/extern/Python_ExtIf_Server/Google/object_detection/ssd_mobilenet_v1_coco_11_06_2017', 'my_graph.pb')  # MODEL_NAME + '/my_graph.pb'  # '/frozen_inference_graph.pb' #  '/home/aadc/ADTF/extern/Python_ExtIf_Server/Google/object_detection/VOCdevkit/Eval/model.ckpt-0.graph'  #

# List of the strings that is used to add correct label for each box.
PATH_TO_LABELS = '/home/aadc/ADTF/extern/Python_ExtIf_Server/Google/object_detection/data/my_label_map.pbtxt'  # os.path.join('/home/aadc/ADTF/extern/Python_ExtIf_Server/Google/object_detection/ssd_mobilenet_v1_coco_11_06_2017', 'my_graph.pbtxt')
    #'/home/aadc/ADTF/extern/Python_ExtIf_Server/Google/object_detection/VOCdevkit/Eval', 'graph.pbtxt')
# ('/home/aadc/ADTF/extern/Python_ExtIf_Server/Google/object_detection/ssd_mobilenet_v1_coco_11_06_2017', 'mscoco_label_map.pbtxt')

NUM_CLASSES = 90

# opener = urllib.request.URLopener()
# opener.retrieve(DOWNLOAD_BASE + MODEL_FILE, MODEL_FILE)
# tar_file = tarfile.open(MODEL_FILE)
# for file in tar_file.getmembers():
#     file_name = os.path.basename(file.name)
#     if 'frozen_inference_graph.pb' in file_name:
#        tar_file.extract(file, os.getcwd())


def init_tf_session():
    # initialize TensorFlow session.
    global tf_session
    # tf_session = tf.Session(config=tf.ConfigProto(log_device_placement=True))
    tf_session = tf.Session()


def create_graph():
    global detection_graph
    global sess
    detection_graph = tf.Graph()
    with detection_graph.as_default():
        od_graph_def = tf.GraphDef()
        with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
            serialized_graph = fid.read()
            od_graph_def.ParseFromString(serialized_graph)
            tf.import_graph_def(od_graph_def, name='')

    detection_graph.as_default()
    sess = tf.Session(graph=detection_graph)

def classify_image(image):
    """Creates a graph from saved GraphDef file and returns a saver."""
    # introduce global config dictionary
    global tf_session
    global sess
    label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
    categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES,
                                                                use_display_name=True)
    category_index = label_map_util.create_category_index(categories)
    # create empty result list
    result_list = []

    # Todo: Don't create a new session for every new image
    # with detection_graph.as_default():
    # with tf.Session(graph=detection_graph) as sess:
        # the array based representation of the image will be used later in order to prepare the
        # result image with boxes and labels on it.

    image_np = load_image_into_numpy_array(Image.open(io.BytesIO(image)))
    # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
    image_np_expanded = np.expand_dims(image_np, axis=0)
    image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
    # Each box represents a part of the image where a particular object was detected.
    boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
    # Each score represent how level of confidence for each of the objects.
    # Score is shown on the result image, together with the class label.
    scores = detection_graph.get_tensor_by_name('detection_scores:0')
    classes = detection_graph.get_tensor_by_name('detection_classes:0')
    num_detections = detection_graph.get_tensor_by_name('num_detections:0')
    # Actual detection.

    (boxes, scores, classes, num_detections) = sess.run(
        [boxes, scores, classes, num_detections],
        feed_dict={image_tensor: image_np_expanded})
    # Visualization of the results of a detection.
    # vis_util.visualize_boxes_and_labels_on_image_array(
    #    image_np,
    #    np.squeeze(boxes),
    #    np.squeeze(classes).astype(np.int32),
    #    np.squeeze(scores),
    #    category_index,
    #    use_normalized_coordinates=True,
    #    line_thickness=8)
    # plt.figure(figsize=IMAGE_SIZE)
    # plt.imshow(image_np)
    # result_list.append("IBIMS")  # (category_index[classes[0]]['name'])  # what does this?

    for i in range(0, 5):
        class_name = category_index[classes[0][i]]['name']
        probability = scores[0][i]
        box = boxes[0][i]
        result_list.append(TDataResult(class_name, probability, box))
    return result_list

# label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
# categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
# category_index = label_map_util.create_category_index(categories)


def load_image_into_numpy_array(image):
    (im_width, im_height) = image.size
    return np.array(image).reshape((im_height, im_width, 3)).astype(np.uint8)

# For the sake of simplicity we will use only 2 images:
# image1.jpg
# image2.jpg
# If you want to test the code with your images, just add path to the images to the TEST_IMAGE_PATHS.
PATH_TO_TEST_IMAGES_DIR = 'test_images'
TEST_IMAGE_PATHS = [os.path.join(PATH_TO_TEST_IMAGES_DIR, 'image{}.jpg'.format(i)) for i in range(1, 3)]

# Size, in inches, of the output images.
IMAGE_SIZE = (12, 8)

# label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
# categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES,
#                                                            use_display_name=True)
# category_index = label_map_util.create_category_index(categories)

# init_tf_session()
# create_graph()

# with detection_graph.as_default():
#  with tf.Session(graph=detection_graph) as sess:
#    for image_path in TEST_IMAGE_PATHS:
#      image = Image.open(image_path)
#      # the array based representation of the image will be used later in order to prepare the
#      # result image with boxes and labels on it.
#      image_np = load_image_into_numpy_array(image)
#      # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
#      image_np_expanded = np.expand_dims(image_np, axis=0)
#      image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
#      # Each box represents a part of the image where a particular object was detected.
#      boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
#      # Each score represent how level of confidence for each of the objects.
#      # Score is shown on the result image, together with the class label.
#      scores = detection_graph.get_tensor_by_name('detection_scores:0')
#      classes = detection_graph.get_tensor_by_name('detection_classes:0')
#      num_detections = detection_graph.get_tensor_by_name('num_detections:0')
#      # Actual detection.
#      (boxes, scores, classes, num_detections) = sess.run(
#          [boxes, scores, classes, num_detections],
#          feed_dict={image_tensor: image_np_expanded})
#      # Visualization of the results of a detection.
#      vis_util.visualize_boxes_and_labels_on_image_array(
 #         image_np,
 ##         np.squeeze(boxes),
 #         np.squeeze(classes).astype(np.int32),
 #         np.squeeze(scores),
 #         category_index,
  #        use_normalized_coordinates=True,
 #         line_thickness=8)
 #     plt.figure(figsize=IMAGE_SIZE)
 #     plt.imshow(image_np)


#      class_name = category_index[classes[0][0]]['name']
#      probability = scores[0][0]
#      print(class_name)
#      print(probability)


