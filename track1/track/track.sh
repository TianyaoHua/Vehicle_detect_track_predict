#python tools/Video2Frames.py --video_dir /home/ecbm4040/amsterdam120/raw --save_path /home/ecbm4040/amsterdam120/detect_output_txt
#python tools/generate_detections.py --model=../model/cosine/detrac.pb --mot_dir=/home/ecbm4040/amsterdam120/detect_output_txt --detection_dir=/home/ecbm4040/amsterdam120/detect_output_txt --output_dir=/home/ecbm4040/amsterdam120/track_feature
#python deep_sort_app.py --sequence_dir=/home/ecbm4040/amsterdam120/detect_output_txt --detection_file=/home/ecbm4040/amsterdam120/track_feature --output_dir=/home/ecbm4040/amsterdam120/track_output --min_confidence=0.3 --nn_budget=100 --display=0

# Generate Tracking Video
python generate_videos.py --mot_dir=/home/ecbm4040/amsterdam120/detect_output_txt/ --result_dir=/home/ecbm4040/amsterdam120/track_output/ --output_dir=/home/ecbm4040/amsterdam120/track_output/
