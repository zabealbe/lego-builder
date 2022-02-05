#!/bin/python3

import os
from dataset import Dataset, YoloDataset, YofoDataset, yolo_to_yofo

DATASET_PATH = "./dataset/laystable"

if __name__ == "__main__":
    out_folder_yolo = f"{DATASET_PATH}.yolo"
    out_folder_yofo = f"{DATASET_PATH}.yofo"

    YoloDataset(path=os.path.join(DATASET_PATH, "dataset.json")).save_to(out_folder_yolo)

#    yolo_to_yofo(os.path.join(in_folder, 'dataset.json'), out_folder_yofo)

