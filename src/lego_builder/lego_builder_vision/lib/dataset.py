import copy
import json
import os
import random
import time
from pyquaternion import Quaternion

import cv2
import numpy as np
from torch.utils.data import Dataset as TorchDataset
import torch
import utils

"""
    Yolo file tree:
    - dataset.yml
    - train/
        - images/
        - labels/
    - valid/
        - images/
        - labels/
    - test/
        - images/
        - labels/
"""
yolo_dir_tree = [
    {"train": {"images": "images", "labels": "labels", "depths": "depths", "visual": "visual"}},
    {"valid": {"images": "images", "labels": "labels", "depths": "depths", "visual": "visual"}},
    {"test":  {"images": "images", "labels": "labels", "depths": "depths", "visual": "visual"}}
]


class Dataset(TorchDataset):
    def __init__(self, path=None, samples=None, metadata=None):
        """
        Initializes the dataset
        Parameters
        ----------
        data : torch.Tensor
            The data of the dataset
        target : torch.Tensor
            The target of the dataset
        """

        self.samples = None
        self.path = None
        self.metadata = None
        self.images_color = None
        self.images_depth = None
        self.labels = None
        self.visual = None

        if path is not None:
            self.__load_from_path(path)
        elif samples is not None and metadata is not None:
            self.samples = samples
        else:
            raise Exception("You must specify either a path or a samples list and metadata")

    def __len__(self):
        return len(self.samples)

    def __load_from_path(self, path):
        """
        Loads the dataset from a file
        Parameters
        ----------
        path : str
            Path to the dataset.json file
        """
        dataset_json = json.load(open(path, "r"))
        metadata = dataset_json

        dataset_folder = os.path.dirname(path)
        labels_folder = os.path.join(dataset_folder, "labels")

        l = []
        for x in os.listdir(dataset_folder):
            if not os.path.isdir(os.path.join(dataset_folder, x)):
                continue
            _, ext = os.path.splitext(os.listdir(os.path.join(dataset_folder, x))[0])
            l.append((x, ext))

        samples = []
        for f in os.listdir(labels_folder):
            uuid, label_ext = os.path.splitext(f)
            sample = {"uuid": uuid}
            for ff, ext in l:
                sample[ff] = os.path.join(dataset_folder, ff, uuid + ext)
            samples.append(sample)

        self.path = path
        self.samples = samples
        self.metadata = metadata

    def split(self, ratio):
        """
        Create two new datasets with the given ratio
        Parameters
        ----------
        ratio : float
            The ratio of the new datasets
        Returns
        -------
        Dataset
            The first dataset
        Dataset
            The second dataset
        """
        left = self.__class__(
            samples=self.samples[:int(len(self.samples) * ratio)],
            metadata=self.metadata,
        )
        right = self.__class__(
            samples=self.samples[int(len(self.samples) * ratio):],
            metadata=self.metadata,
        )

        return left, right

    def __next__(self):
        return self.__getitem__(self.__iter__().__next__())


class YofoDataset(Dataset):
    def __init__(self, path=None, samples=None, metadata=None):
        super(YofoDataset, self).__init__(path, samples, metadata)
        self._augment = True

    def __getitem__(self, idx):
        sample = self.samples[idx]
        labels = json.load(open(sample["labels"], "r"))["orientation"]

        states = {
            "up":    [1, 0, 0, 0, 0, 0],
            "down":  [0, 1, 0, 0, 0, 0],
            "north": [0, 0, 1, 0, 0, 0],
            "south": [0, 0, 0, 1, 0, 0],
            "east":  [0, 0, 0, 0, 1, 0],
            "west":  [0, 0, 0, 0, 0, 1],
        }

        state = np.array(states[labels["facing"]], dtype=np.float32)

        # load rgb image
        #image_color = cv2.imread(sample["images"])

        # load depth image
        image_depth = np.load(sample["depths"])

        # resize image
        image_depth = cv2.resize(image_depth, (32, 32), interpolation=cv2.INTER_NEAREST)

        if self._augment:
            offset = image_depth.shape[0] - 32, image_depth.shape[1] - 32
            offset = max(offset[0], 0), max(offset[1], 0)

            if self._augment:
                offset = random.randint(0, offset[0]), random.randint(0, offset[1])
            else:
                offset = offset[0] // 2, offset[1] // 2
            # image_depth = np.expand_dims(crop(image_depth, (32, 32), offset).astype(np.float32), axis=2)

            offset = random.randint(0, 10), random.randint(0, 10)
            image_depth = np.pad(image_depth, ((5, 5), (5, 5)), mode="constant", constant_values=image_depth.max())[
                          offset[0]:offset[0] + 32, offset[1]:offset[1] + 32]

            """
            # apply linear transformation
            if random.choice([True, False]):
                image_depth = np.flip(image_depth, axis=0)
            if random.choice([True, False]):
                image_depth = np.flip(image_depth, axis=1)
            if random.choice([True, False]):
                image_depth = np.rot90(image_depth, k=random.randint(0, 3))
            """

            if random.choice([True, False]):
                image_depth = cv2.blur(image_depth, random.choice([(2, 2), (3, 3), (4, 4)]))

            # add random variations
            i_max = np.max(image_depth)
            i_min = np.min(image_depth)
            offset = random.uniform(-(i_max-i_min)*0.6, (i_max-i_min)*0.6)

            image_depth -= random.uniform(0, 0.3) * i_min
            image_depth *= 1 + random.uniform(-0.3, 0.3)

            image_depth = np.where(image_depth == i_max, i_max + offset, image_depth)

        image_depth = np.expand_dims(image_depth.astype(np.float32), axis=2)

        return sample["uuid"], image_depth, state

    def augment(self, augment):
        self._augment = augment

    def view(self, delay=0.1):
        for sample in self:
            uuid   = sample[0]
            clas   = np.array(sample[3])
            image = np.array(sample[2])
            if clas[0] == 1:
                clas = "up"
            elif clas[1] == 1:
                clas = "side"
            elif clas[2] == 1:
                clas = "down"
            else:
                clas = "unknown"
            """
            if not valid(sample):
                cv2.circle(sample, (int(sample.shape[1] / 2), int(sample.shape[0] / 2)), 3, (0, 0, 255), -1)
            cv2.imshow("sample", np.array(sample))
            cv2.waitKey(int(delay*1000))
            """
            image = cv2.resize(image, (256, 256))
            cv2.putText(image, str(clas), (0, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.imwrite(f"dataset_v2/out_visual/visual/{uuid}.png", image)
            print(f"./dataset_v2/out_visual/{uuid}.png")


class YoloDataset(Dataset):
    def __init__(self, path=None, samples=None, metadata=None):
        super(YoloDataset, self).__init__(path, samples, metadata)
        self.metadata["classes"].sort()  # sort classes to have a consistent order

    def __getitem__(self, idx):
        """
        Returns the item at the given index
            lazy loader
        Parameters
        ----------
        idx : int
            The index of the item
        """
        if torch.is_tensor(idx):
            idx = idx.tolist()

        sample = self.samples[idx]

        image_color = cv2.imread(sample["images"], cv2.IMREAD_COLOR)

        image_depth = np.load(sample["depths"])

        labels = json.load(open(sample["labels"], "r"))

        image_visual = cv2.imread(sample["visual"], cv2.IMREAD_COLOR)

        sample = {
            "uuid": sample["uuid"],
            "images": image_color,
            "depths": image_depth,
            "labels": labels,
            "visual": image_visual
        }

        """
        if self.transform:
            sample = self.transform(sample)"""

        return sample

    def save_to(self, path):
        create_tree(path, yolo_dir_tree)

        out_path_config = os.path.join(path, "dataset.yml")
        with open(out_path_config, "w+") as f:
            f.write("path: <path>\n")
            f.write("train: ./train/images\n")
            f.write("val: ./valid/images\n")
            f.write("test: ./test/images\n")
            f.write(f"nc: {len(self.metadata['classes'])}\n")
            f.write("\n")
            f.write(f"classes: {list(self.metadata['classes'])}\n")

        for sample in self:
            split = random.choices(("train", "valid", "test"), (.7, .2, .1), k=1)[0]  # select the destination split
            output_path_image = os.path.join(path, split, "images", f"{sample['uuid']}.png")
            ouput_path_depth  = os.path.join(path, split, "depths", f"{sample['uuid']}.png")
            ouput_path_label  = os.path.join(path, split, "labels", f"{sample['uuid']}.txt")
            ouput_path_visual = os.path.join(path, split, "visual", f"{sample['uuid']}.png")

            image = sample["images"]
            depth = sample["depths"]
            labels_dirty = sample["labels"]
            visual = sample["visual"]

            labels = []  # clean labels
            for bbox, clss in [(x["bbox"], x["class"]) for x in labels_dirty]:
                # Exclude the subsample if empty
                center = np.clip((int(bbox[0]*depth.shape[1]), int(bbox[1]*depth.shape[0])), 0, 640-1)
                if depth[center[1], center[0]] > 0.805:
                    cv2.circle(visual, (center[0], center[1]), 3, (0, 0, 255), thickness=3)
                    continue

                cx, cy, w, h = bbox
                labels.append((bbox, clss))
                #classes.add(clss)

            cv2.imwrite(output_path_image, image)

            # Normalize depth
            depth = -depth + depth.max()
            depth *= 255 / 0.20
            cv2.imwrite(ouput_path_depth, np.expand_dims(depth.astype(np.int32), axis=2))

            cv2.imwrite(ouput_path_visual, visual)

            with open(ouput_path_label, "w+") as f:
                for bbox, clss in labels:
                    f.write(f"{self.metadata['classes'].index(clss)} {bbox[0]} {bbox[1]} {bbox[2]} {bbox[3]}\n")

def yolo_to_yofo(yolo_path, yofo_path, instantiate=False):
    """
    Convert a YOLO dataset to the YOFO format.
    :param yolo_path: Input  path to the YOLO dataset.
    :param yofo_path: Output path to the YOFO dataset.
    """
    yolo = YoloDataset(yolo_path)

    create_tree(yofo_path, [
        "images",
        "depths",
        "labels",
        "visual",
    ])
    for sample in yolo:
        uuid = sample["uuid"]
        labels = sample["labels"]
        image_color = sample["images"]
        image_depth = sample["depths"]

        # Create a subimage for each bounding box
        for label in labels:
            # Get the bounding box (cx, cy, w, h)
            bbox = np.array(label["bbox"], dtype=np.float64).reshape((2, 2))

            # Exclude the subsample if bbox is outside the image
            if np.max(bbox) > 1 or np.min(bbox) < 0:
                continue

            # Convert the bounding box to pixel coordinates
            bbox *= [image_color.shape[1], image_color.shape[0]]
            bbox = bbox.astype(np.int32)

            # Exclude the subsample if empty
            if image_depth[int(bbox[0, 1]), int(bbox[0, 0])] > 0.84:
                continue

            # Convert the bounding box from (cx, cy, w, h) to (x, y, w, h)
            bbox[0, :] -= bbox[1, :] // 2
            bbox[1, :] = bbox[0, :] + bbox[1, :]

            # Clip the bounding box to the image
            bbox = np.clip(bbox, (0, 0), [image_color.shape[1], image_color.shape[0]])

            # Crop the depth image around the bounding box
            sub_image_depth = image_depth[bbox[0][1]: bbox[1][1], bbox[0][0]: bbox[1][0]]
            # Crop the smallest rectangle containing the bounding box
            angle, sub_image_depth = utils.min_area_crop(sub_image_depth)
            # Check if the subimage is empty
            if sub_image_depth is None:
                continue
            if sub_image_depth.shape[0] < 2 or sub_image_depth.shape[1] < 2:
                continue

            # Rotate the quaternion by the angle around z axis

            # Get the model orientation
            q = label["world_pose"]["orientation"]
            q = Quaternion(x=q["x"], y=q["y"], z=q["z"], w=q["w"])
            q = Quaternion(axis=(0, 1, 0), angle=angle) * q

            # Convert the label to the YOFO format
            axis_angle = {
                "axis": list(q.axis),
                "angle": q.angle
            }
            label = {
                "class": label["class"],
                "orientation": {
                    "quaternion": label["world_pose"]["orientation"],
                    "axis_angle": axis_angle,
                    "facing": None  # up, down, north, south, east, west
                }
            }

            sub_uuid = f"{uuid}_{'_'.join([str(x) for x in bbox.flatten('C')])}"
            # Save the color subimage
            sub_image_color = image_color[bbox[0][1]: bbox[1][1], bbox[0][0]: bbox[1][0]]
            cv2.imwrite(os.path.join(yofo_path, "images", f"{sub_uuid}.png"), sub_image_color)

            # Save the depth subimage
            label["orientation"]["facing"] = utils.get_facing_direction(q)
            np.save(os.path.join(yofo_path, "depths", f"{sub_uuid}.npy"), sub_image_depth)

            # Save the data visualisation
            sub_image_depth = cv2.resize(sub_image_depth, (256, 256))
            sub_image_depth = cv2.cvtColor(sub_image_depth, cv2.COLOR_GRAY2RGB, sub_image_depth)

            cv2.putText(sub_image_depth, label["orientation"]["facing"], (0, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (20, 156, 229), 2)
            cv2.imwrite(os.path.join(yofo_path, "visual", f"{sub_uuid}.png"), (sub_image_depth*255).astype(np.uint8))

            # Save the subimage label
            json.dump(label, open(os.path.join(yofo_path, "labels", f"{sub_uuid}.json"), "w+"), indent=4)

        # Save the dataset metadata
        json.dump(yolo.metadata, open(os.path.join(yofo_path, "dataset.json"), "w+"), indent=4)

    return YofoDataset(yofo_path) if instantiate else None


def create_tree(root, file_tree):
    """
    Creates the file tree of the dataset
    Parameters
    ----------
    root : str
        Root of the file tree
    file_tree : list
        The directory tree to create
    """
    for file in file_tree:
        if isinstance(file, str):
            os.makedirs(os.path.join(root, file), exist_ok=True)
        else:
            file, sub_file_tree = list(file.keys())[0], list(file.values())[0]
            create_tree(os.path.join(root, file), sub_file_tree)


def crop(array_2d, size, offset=(0, 0)):
    array_2d_shape = array_2d.shape
    delta = (0, max(size[0] - array_2d_shape[0], 0)), (0, max(size[1] - array_2d_shape[1], 0))

    array_pad = np.pad(array_2d, delta, mode="constant", constant_values=np.max(array_2d))

    return array_pad[offset[0]:offset[0]+size[0], offset[1]:offset[1]+size[1]]


def resize(array_2d, size=(32, 32)):
    array_pad = (array_2d - np.min(array_2d)) / (np.max(array_2d) - np.min(array_2d))
    array_pad = cv2.resize(array_pad, size, interpolation=cv2.INTER_NEAREST)

    return array_pad


def color_distance(rgb1, rgb2):
    '''d = {} distance between two colors(3)'''
    d = np.subtract(rgb1, rgb2) + 1
    d = d ** 6
    d = np.dot(d, d)
    d = np.sqrt(d)
    return d / 6.928203230275509


def is_grey(image, center):
    if len(center) != 2:
        raise ValueError("center must be a 2D point")
    if type(center[0]) == float and type(center[1]) == float:
        center = int(center[0] * image.shape[1]), int(center[1] * image.shape[0])

    # get average color
    avg_color = np.mean(image[center[1] - 4:center[1] + 4, center[0] - 4:center[0] + 4, :], axis=(0, 1))

    d = abs(avg_color[0] - avg_color[1]) + abs(avg_color[1] - avg_color[2]) + abs(avg_color[2] - avg_color[0])
    return d < 1
