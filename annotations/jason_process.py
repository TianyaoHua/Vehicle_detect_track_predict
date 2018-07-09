import json
with open("C:\My_Projects\ProjectsWithZoran\\2018AICITY_Team_Columbia\\annotations\\vehicle-detect-track-predict-2.json") as f:
    annotations = json.load(f)
inserted_regions = annotations['_via_img_metadata']['frame13356.jpg880885']['regions']

with open("C:\My_Projects\ProjectsWithZoran\\2018AICITY_Team_Columbia\\annotations\\vehicle-detect-track-predict_Tony (1).json") as feedsjson:
    annotations = json.load(feedsjson)
for frame in annotations['_via_img_metadata']:
    if int(frame[5:-10]) >= 9752:
        annotations['_via_img_metadata'][frame]['regions'] += inserted_regions

with open("C:\My_Projects\ProjectsWithZoran\\2018AICITY_Team_Columbia\\annotations\\vehicle-detect-track-predict_Tony (1).json", mode='w', encoding='utf-8') as feedsjson:
    json.dump(annotations, feedsjson)
