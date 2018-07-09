import pandas as pd
import numpy as np
data_frame = pd.read_csv("via_region_data.csv")
sample = pd.DataFrame(columns=list(data_frame.keys()))
[X, Y] = data_frame.shape

for i in range(X):
    if data_frame.iloc[i]["filename"] == "frame17.jpg":
        sample = sample.append(data_frame.iloc[i], ignore_index=True)
[sX, sY] = sample.shape
inserted_times = 0
for i in range(X):
    if i > 0 and data_frame.iloc[i]["filename"] != data_frame.iloc[i-1]["filename"] and int(data_frame.iloc[i-1]["filename"][5:-4]) >= 2554:
        last_id = data_frame.iloc[i-1]["region_id"]
        number_region = data_frame.iloc[i-1]["region_count"]
        last_filename = data_frame.iloc[i-1]["filename"]
        inserted_data_frame = sample.copy(deep=True)
        for j in range(sX):
            inserted_data_frame._set_value(j,"filename",last_filename)
            inserted_data_frame._set_value(j,"region_count", number_region)
            inserted_data_frame._set_value(j,"region_id", 1 + last_id + inserted_data_frame.iloc[j]["region_id"])
        data_frame = data_frame.append(inserted_data_frame, ignore_index=True)
        inserted_times += 1
        print(data_frame.shape)
    if int(data_frame.iloc[i]["filename"][5:-4]) >= 2554:
        data_frame._set_value(i, "region_count", sample.iloc[0]["region_count"] + data_frame.iloc[i]["region_count"])
#data_frame["sort_val"] = np.int0(data_frame["filename"][5:-4])
data_frame.sort_values("filename")
data_frame.to_csv("via_region_data_processed.csv")
print(inserted_times)

