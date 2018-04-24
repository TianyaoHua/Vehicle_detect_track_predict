import os
import sys
import time
import random
import math
import numpy as np
import imageio
import cv2
import tqdm
import pickle
import argparse

from config import Config
import utils
import model as modellib

def mask_to_contours(mask):
    _, contours, hierarchy = cv2.findContours(mask.copy(),
                                            cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE,
                                            offset=(0, 0))
    # return a list of contours
    return contours

def main(model_path, input_video, output_dir, REGENERATE):

    # Directory to save logs and trained model
    MODEL_DIR = "./logs"

    # Local path to trained weights file
    COCO_MODEL_PATH = model_path

    class InferenceConfig(Config):
        NAME = "aic"
        NUM_CLASSES = 80 + 1
        # Set batch size to 1 since we'll be running inference on
        # one image at a time. Batch size = GPU_COUNT * IMAGES_PER_GPU
        GPU_COUNT = 1
        IMAGES_PER_GPU = 8 # 8 is maximum for single P100.

    config = InferenceConfig()
    config.display()   
    
    # Create model object in inference mode.
    model = modellib.MaskRCNN(mode="inference", model_dir=MODEL_DIR, config=config)

    # Load weights trained on MS-COCO
    model.load_weights(COCO_MODEL_PATH, by_name=True)
    print("Successfully load coco pretrained model.")
    
    # COCO Class names
    # Index of the class in the list is its ID. For example, to get ID of
    # the teddy bear class, use: class_names.index('teddy bear')
    class_names = ['BG', 'person', 'bicycle', 'car', 'motorcycle', 'airplane',
                   'bus', 'train', 'truck', 'boat', 'traffic light',
                   'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird',
                   'cat', 'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear',
                   'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie',
                   'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball',
                   'kite', 'baseball bat', 'baseball glove', 'skateboard',
                   'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup',
                   'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
                   'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza',
                   'donut', 'cake', 'chair', 'couch', 'potted plant', 'bed',
                   'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote',
                   'keyboard', 'cell phone', 'microwave', 'oven', 'toaster',
                   'sink', 'refrigerator', 'book', 'clock', 'vase', 'scissors',
                   'teddy bear', 'hair drier', 'toothbrush']
    
    # read video
    video_dir = input_video#"../data/track1_videos/"
    save_dir = output_dir#"../data/detect_output_pkl/"
    if not os.path.isdir(save_dir):
        os.makedirs(save_dir)
    
    videonames = [x for x in os.listdir(video_dir) if x.startswith("Loc")]
    print(videonames)
    batch_size = config.GPU_COUNT*config.IMAGES_PER_GPU
    for videoname in videonames:
        print("Processing video {}...".format(videoname))
        video_file = os.path.join(video_dir, videoname)
        vid = imageio.get_reader(video_file,  'ffmpeg')
        
        save_pkl_dir = os.path.join(save_dir, videoname)
        if not os.path.isdir(save_pkl_dir):
            os.makedirs(save_pkl_dir)
            start_point = 0 # set the start frame for detection
        else:
            if REGENERATE:
                start_point = 0
            else:
                start_point = len(os.listdir(save_pkl_dir))
        
        # tqdm progress bar
        pbar = tqdm.tqdm(total = vid.get_length()-start_point)
        for fnum in range(start_point, vid.get_length(), batch_size):
            batch_data = []
            for i in range(batch_size):
                if fnum+i<vid.get_length():
                    batch_data.append(vid.get_data(fnum+i))
                else:
                    batch_data.append(np.zeros((1080,1920,3)))
            results = model.detect(batch_data, verbose=0)
            for i, r in enumerate(results):
                ## transform mask to contours
                print("frame {} detects {} objects".format(fnum+i, r['rois'].shape[0]))
                contours = []
                for m in r['masks'].transpose(2,0,1):
                    contours.append(mask_to_contours(m.astype('uint8')))
                r['contours'] = contours
                r.pop('masks', None) # This will return my_dict[key] if key exists in the dictionary, and None otherwise.
                ## save a single pkl for each frame
                if fnum + i < vid.get_length():
                    with open(os.path.join(save_pkl_dir, str(fnum+i).zfill(7)+".pkl"), "wb") as f:
                        pickle.dump(r, f, protocol=2)
                
                pbar.update(1)
        pbar.close()
    

def parse_args():
    """ Parse command line arguments.
    """
    parser = argparse.ArgumentParser(description="Mask RCNN Detection")
    parser.add_argument(
        "--model_path", help="Path to the pretrained model",
        default="../model/maskrcnn/mask_rcnn_coco.h5")
    parser.add_argument(
        "--input_video", help="Path to track1 videos.", 
        default="../data/track1_videos/",)
    parser.add_argument(
        "--output_dir", help="Path to the detection output file.",
        default="../data/detect_output_pkl/")
    parser.add_argument("--regenerate", type=bool, default=False)
    return parser.parse_args()            
    

if __name__ == "__main__":
    args = parse_args()
    model_path = args.model_path
    input_video = args.input_video
    output_dir = args.output_dir
    regenerate = args.regenerate
    main(model_path=model_path, input_video=input_video, output_dir=output_dir, REGENERATE=regenerate)
