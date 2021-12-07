import torch.utils.data as data
import numpy as np
from pathlib import Path
import torch, os
import cv2
from typing import Any, Callable, Dict, IO, List, Optional, Tuple, Union

class DatasetLoader(data.Dataset):
    
    def __init__(self,
                 root: str,
                 train: bool = True,
                 download: bool = False,
                 transform: Optional[Callable] = None,
                 target_transform: Optional[Callable] = None
    ):

        self.root = Path(root)
        self.transform=transform
        self.target_transform=target_transform
        if train:
            self.depth_input_paths = [root+'depth/train/'+d for d in os.listdir(root+'depth/train')]
        else:
            self.depth_input_paths = [root+'depth/test/'+d for d in os.listdir(root+'depth/test/')]
        
        self.length = len(self.depth_input_paths)

            
    def __getitem__(self, index):
        path = self.depth_input_paths[index]
        depth_input = cv2.imread(path,cv2.IMREAD_UNCHANGED).astype(np.float32)
        depth_input = np.moveaxis(depth_input,-1,0)
        depth_input = torch.from_numpy(depth_input).unsqueeze(0)
        # if self.transform is not None:
        #     img = self.transform(depth_input_mod)
        target: Any = []

        return depth_input/torch.max(depth_input), target


    def __len__(self):
        return self.length

if __name__ == '__main__':
    dataset = DatasetLoader()
    print("Initializing dataset from location:")
    print(dataset.root)
    print(len(dataset))
    for item in dataset[0]:
        print(item.size())