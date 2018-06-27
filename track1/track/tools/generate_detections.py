# vim: expandtab:ts=4:sw=4
import os
import errno
import argparse
import numpy as np
import cv2
import tensorflow as tf
#from google.protobuf import text_format
'''
python tools/generate_detections.py --model=resources/networks/detrac.pb --mot_dir=./Nvidia --output_dir=./resources/detections/Nvidia
'''
#n_pic = 0
def _run_in_batches(f, data_dict, out, batch_size):
    data_len = len(out)
    num_batches = int(data_len / batch_size)

    s, e = 0, 0
    #print('-------------------------------',num_batches,batch_size,data_len,data_dict.keys())
    for i in range(num_batches):
        s, e = i * batch_size, (i + 1) * batch_size
        batch_data_dict = {k: v[s:e] for k, v in data_dict.items()}
        out[s:e] = f(batch_data_dict)
    if e < len(out):
        batch_data_dict = {k: v[e:] for k, v in data_dict.items()}
        #print('===========================',[v[e:].shape for k, v in data_dict.items()])
        out[e:] = f(batch_data_dict)

def crop_minAreaRect(img, rect, box):
    W = rect[1][0]
    H = rect[1][1]

    Xs = [i[0] for i in box]
    Ys = [i[1] for i in box]
    x1 = min(Xs)
    x2 = max(Xs)
    y1 = min(Ys)
    y2 = max(Ys)

    rotated = False
    angle = rect[2]

    if angle < -45:
        angle += 90
        rotated = True

    center = (int((x1 + x2) / 2), int((y1 + y2) / 2))
    size = (int((x2 - x1)), int((y2 - y1)))

    M = cv2.getRotationMatrix2D((size[0] / 2, size[1] / 2), angle, 1.0)

    cropped = cv2.getRectSubPix(img, size, center)
    cropped = cv2.warpAffine(cropped, M, size)

    croppedW = W if not rotated else H
    croppedH = H if not rotated else W

    croppedRotated = cv2.getRectSubPix(cropped, (int(croppedW), int(croppedH)),
                                       (size[0] / 2, size[1] / 2))
    #global n_pic
    #cv2.imwrite("../data/cropped_img" + str(n_pic + 1)+ ".jpg", croppedRotated)
    #n_pic += 1
    return croppedRotated

def extract_image_patch(image, bbox, contour, patch_shape):
    """Extract image patch from bounding box.

    Parameters
    ----------
    image : ndarray
        The full image.
    bbox : array_like
        The bounding box in format (x, y, width, height).
    patch_shape : Optional[array_like]
        This parameter can be used to enforce a desired patch shape
        (height, width). First, the `bbox` is adapted to the aspect ratio
        of the patch shape, then it is clipped at the image boundaries.
        If None, the shape is computed from :arg:`bbox`.
    contour:
        The contour of the object in the image
    Returns
    -------
    ndarray | NoneType
        An image patch showing the :arg:`bbox`, optionally reshaped to
        :arg:`patch_shape`.
        Returns None if the bounding box is empty or fully outside of the image
        boundaries.

    """
    bbox = np.array(bbox)
    if patch_shape is not None:
        # correct aspect ratio to patch shape
        target_aspect = float(patch_shape[1]) / patch_shape[0]
        new_width = target_aspect * bbox[3]
        bbox[0] -= (new_width - bbox[2]) / 2
        bbox[2] = new_width

    # convert to top left, bottom right
    bbox[2:] += bbox[:2]
    bbox = bbox.astype(np.int)

    # clip at image boundaries
    bbox[:2] = np.maximum(0, bbox[:2])
    bbox[2:] = np.minimum(np.asarray(image.shape[:2][::-1]) - 1, bbox[2:])
    if np.any(bbox[:2] >= bbox[2:]):
        return None
    sx, sy, ex, ey = bbox
    # remove the background
    contour -= [sx, sy]
    image = image[sy:ey, sx:ex]
    mask = np.zeros(image.shape[0:2])
    cv2.fillPoly(mask, [contour], (1))
    image = image * mask[:, :, None]
    rect = cv2.minAreaRect(contour)
    box = cv2.boxPoints(rect)
    box = np.int0(box)
    image_cropped = crop_minAreaRect(image.astype(np.uint8), rect, box)
    image_cropped = cv2.resize(image_cropped, tuple(patch_shape[::-1]))
    return image_cropped


class ImageEncoder(object):

    def __init__(self, checkpoint_filename, input_name="images",
                 output_name="features"):
        self.session = tf.Session()
        with tf.gfile.GFile(checkpoint_filename, "rb") as file_handle:
            graph_def = tf.GraphDef()
            graph_def.ParseFromString(file_handle.read())
            #text_format.Merge(file_handle.read(),graph_def)
        tf.import_graph_def(graph_def, name="net")
        print(tf.get_default_graph().get_all_collection_keys())
        self.input_var = tf.get_default_graph().get_tensor_by_name(
            "net/%s:0" % input_name)
        self.output_var = tf.get_default_graph().get_tensor_by_name(
            "net/%s:0" % output_name)
        
        #print('----------------', tf.get_default_graph().collections)

        assert len(self.output_var.get_shape()) == 2
        assert len(self.input_var.get_shape()) == 4
        self.feature_dim = self.output_var.get_shape().as_list()[-1]
        self.image_shape = self.input_var.get_shape().as_list()[1:]

    def __call__(self, data_x, batch_size=32):
        out = np.zeros((len(data_x), self.feature_dim), np.float32)
        _run_in_batches(
            lambda x: self.session.run(self.output_var, feed_dict=x),
            {self.input_var: data_x}, out, batch_size)
        return out


def create_box_encoder(model_filename, input_name="images",
                       output_name="features", batch_size=32):
    image_encoder = ImageEncoder(model_filename, input_name, output_name)
    image_shape = image_encoder.image_shape

    def encoder(image, boxes, contours):
        image_patches = []
        for i in range(len(boxes)):
        #for box in boxes:
            box, contour = np.array(boxes[i]), np.array(contours[i]).astype(np.int)
            contour = np.trim_zeros(contour, 'b')
            contour = contour[:-3].reshape(contour[-3:].astype(np.int))
            patch = extract_image_patch(image, box, contour, image_shape[:2])
            if patch is None:
                print("WARNING: Failed to extract image patch: %s." % str(box))
                patch = np.random.uniform(
                    0., 255., image_shape).astype(np.uint8)
            image_patches.append(patch)
        image_patches = np.asarray(image_patches)
        return image_encoder(image_patches, batch_size)

    return encoder


def generate_detections(encoder, mot_dir, output_dir, detection_dir=None):
    """Generate detections with features.

    Parameters
    ----------
    encoder : Callable[image, ndarray] -> ndarray
        The encoder function takes as input a BGR color image and a matrix of
        bounding boxes in format `(x, y, w, h)` and returns a matrix of
        corresponding feature vectors.
    mot_dir : str
        Path to the MOTChallenge directory (can be either train or test).
    output_dir
        Path to the output directory. Will be created if it does not exist.
    detection_dir
        Path to custom detections. The directory structure should be the default
        MOTChallenge structure: `[sequence]/det/det.txt`. If None, uses the
        standard MOTChallenge detections.

    """
    if detection_dir is None:
        detection_dir = mot_dir
    try:
        os.makedirs(output_dir)
    except OSError as exception:
        if exception.errno == errno.EEXIST and os.path.isdir(output_dir):
            pass
        else:
            raise ValueError(
                "Failed to created output directory '%s'" % output_dir)

    for sequence in os.listdir(mot_dir):
        print("Processing %s" % sequence)
        output_filename = os.path.join(output_dir, "%s.npy" % sequence)
        if os.path.isfile(output_filename):
            print(sequence,"already processed.")
            continue
        else:
            sequence_dir = os.path.join(mot_dir, sequence)
    
            image_dir = os.path.join(sequence_dir, "img1")
            #image_filenames = {
            #    int(os.path.splitext(f)[0].split('_')[2]): os.path.join(image_dir, f)
            #    for f in os.listdir(image_dir)}
            image_filenames = {
                int(os.path.splitext(f)[0].split('_')[-1]): os.path.join(image_dir, f)
                for f in os.listdir(image_dir)}
            detection_file = os.path.join(
                detection_dir, sequence, "det", sequence+"_det.txt")
            detections_in = np.loadtxt(detection_file, delimiter=',')
            detections_out = []
    
            frame_indices = detections_in[:,0].astype(np.int)
            min_frame_idx = frame_indices.astype(np.int).min()
            max_frame_idx = frame_indices.astype(np.int).max()
            for frame_idx in range(min_frame_idx, max_frame_idx + 1):
                print("Frame %05d/%05d" % (frame_idx, max_frame_idx))
                mask = frame_indices == frame_idx
                rows = detections_in[mask]
    
                if frame_idx not in image_filenames:
                    print("WARNING could not find image for frame %d" % frame_idx)
                    continue
                bgr_image = cv2.imread(
                    image_filenames[frame_idx], cv2.IMREAD_COLOR)
                features = encoder(bgr_image, rows[:, 2:6].copy(), rows[:, 10:].copy())
                detections_out += [np.r_[(row, feature)] for row, feature
                                   in zip(rows, features)]
    
            
            np.save(
                output_filename, np.asarray(detections_out), allow_pickle=False)


def parse_args():
    """Parse command line arguments.
    """
    parser = argparse.ArgumentParser(description="Re-ID feature extractor")
    parser.add_argument(
        "--model",
        default="../model/cosine/detrac.pb",
        help="Path to freezed inference graph protobuf.")
    parser.add_argument(
        "--mot_dir", help="Path to  detection result directory",
        default="../data/Nvidia")
    parser.add_argument(
        "--detection_dir", help="Path to custom detections. Defaults to "
        "standard detections Directory structure should be the default "
        "Nvidia structure: [sequence]/det/det.txt", default=None)
    parser.add_argument(
        "--output_dir", help="Output directory. Will be created if it does not"
        " exist.", default="../data/track_features")
    return parser.parse_args()


def main():
    args = parse_args()
    encoder = create_box_encoder(args.model, batch_size=8)
    generate_detections(encoder, args.mot_dir, args.output_dir,
                        args.detection_dir)


if __name__ == "__main__":
    main()
