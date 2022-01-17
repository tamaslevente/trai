import torch.utils.data as data
import numpy as np
from PIL import Image
from scipy.misc import imread
from path import Path
from constants import *
from torchvision.transforms import Resize, Compose, ToPILImage, ToTensor, RandomHorizontalFlip, CenterCrop, ColorJitter
import torch
import time
import os
import torch.nn.functional as F
import random
import scipy.ndimage as ndimage
from scipy import misc
import cv2

class RandomCrop(object):
    """Crop randomly the image in a sample.

    Args:
        output_size (tuple or int): Desired output size. If int, square crop
            is made.
    """

    def __init__(self, output_size):
        assert isinstance(output_size, (int, tuple))
        if isinstance(output_size, int):
            self.output_size = (output_size, output_size)
        else:
            assert len(output_size) == 2
            self.output_size = output_size

    def __call__(self, sample):
        image, landmarks = sample

        h, w = image.shape[:2]
        new_h, new_w = self.output_size

        top = np.random.randint(0, h - new_h)
        left = np.random.randint(0, w - new_w)

        image = image[top: top + new_h,
                      left: left + new_w, :]

        landmarks = landmarks[top: top + new_h,
                              left: left + new_w]

        return image, landmarks


class CropCenter(object):
    """Crops the given inputs and target arrays at the center to have a region of
    the given size. size can be a tuple (target_height, target_width)
    or an integer, in which case the target will be of a square shape (size, size)
    Careful, img1 and img2 may not be the same size
    """

    def __init__(self, size):
        self.size = size

    def __call__(self, inputs, target):
        h1, w1, _ = inputs.shape
        th, tw = self.size
        x1 = int(round((w1 - tw) / 2.))
        y1 = int(round((h1 - th) / 2.))

        inputs = inputs[y1: y1 + th, x1: x1 + tw]
        target = target[y1: y1 + th, x1: x1 + tw]
        return inputs, target


class RandomCropRotate(object):
    """Random rotation of the image from -angle to angle (in degrees)
    A crop is done to keep same image ratio, and no black pixels
    angle: max angle of the rotation, cannot be more than 180 degrees
    interpolation order: Default: 2 (bilinear)
    """

    def __init__(self, angle, diff_angle=0, order=2):
        self.angle = angle
        self.order = order
        self.diff_angle = diff_angle

    def __call__(self, sample):
        inputs, target = sample
        h, w, _ = inputs.shape

        applied_angle = random.uniform(-self.angle, self.angle)
        diff = random.uniform(-self.diff_angle, self.diff_angle)
        angle1 = applied_angle - diff/2

        angle1_rad = angle1*np.pi/180

        inputs = ndimage.interpolation.rotate(
            inputs, angle1, reshape=True, order=self.order)
        target = ndimage.interpolation.rotate(
            target, angle1, reshape=True, order=self.order)

        # keep angle1 and angle2 within [0,pi/2] with a reflection at pi/2: -1rad is 1rad, 2rad is pi - 2 rad
        angle1_rad = np.pi/2 - np.abs(angle1_rad % np.pi - np.pi/2)

        c1 = np.cos(angle1_rad)
        s1 = np.sin(angle1_rad)
        c_diag = h/np.sqrt(h*h+w*w)
        s_diag = w/np.sqrt(h*h+w*w)

        ratio = 1./(c1+w/float(h)*s1)

        crop = CropCenter((int(h*ratio), int(w*ratio)))
        return crop(inputs, target)


class RandomHorizontalFlip(object):

    def __init__(self, p=0.5):
        self.p = p

    def __call__(self, sample):
        image, landmarks = sample
        if np.random.randn() > 0.5:
            image = image[:, ::-1, :]
            landmarks = landmarks[:, ::-1]

        return image, landmarks


class MyCustomDataset(data.Dataset):
    def __init__(self, root='/home/marian/calibration_ws/monodepth-FPN/MonoDepth-FPN-PyTorch/dataset/training_data/multiP_training_data/main_multiP/', seed=None, train=True):

        np.random.seed(seed)
        self.root = Path(root)
        self.train = train
        if train:
            self.rgb_paths = [root+'input_irdd_data/train/' +
                              d for d in os.listdir(root+'input_irdd_data/train/')]
            # Randomly choose 50k images without replacement
            # self.rgb_paths = np.random.choice(self.rgb_paths, 600, False)
        else:
            self.rgb_paths = [root+'input_irdd_data/test/' +
                              d for d in os.listdir(root+'input_irdd_data/test/')]

        if train != train:  # vis
            self.train = True
            self.rgb_paths = [root+'vis_images/' +
                              d for d in os.listdir(root+'vis_images/')]

        self.augmentation = Compose([RandomHorizontalFlip()])  # , RandomCropRotate(10)
        # ColorJitter(brightness=0.2, contrast=0.2, saturation=0.2, hue=0.1),
        self.rgb_transform = Compose([ToPILImage(mode='RGB'), Resize((480, 640)), ToTensor()])
        self.depth_transform = Compose([ToPILImage(mode='I'), Resize((480, 640)), ToTensor()])

        # if self.train:
        #     self.length = 50000  # len(self.rgb_paths)
        # else:
        self.length = len(self.rgb_paths)

    def __getitem__(self, index):
        path = self.rgb_paths[index]
        rgb = cv2.imread(path,-1) # or rgb = cv2.imread(path,cv2.IMREAD_UNCHANGED)
        
        #### debugging purposes ####
        # rgb_mono = cv2.imread(path,-1) # or rgb = cv2.imread(path,cv2.IMREAD_UNCHANGED)
        
        # rgb = np.zeros((len(rgb_mono),len(rgb_mono[0]),3), 'uint16')  

        # for i in range(len(rgb_mono)):
        #     for j in range(len(rgb_mono[i])):
        #         # rgbArray[i][j][0] = ir[i][j]
        #         rgb[i][j][0] = rgb_mono[i][j]
        #         rgb[i][j][1] = rgb_mono[i][j]
        #         rgb[i][j][2] = rgb_mono[i][j]

        depth = cv2.imread(path.replace('input_irdd_data', 'depth_gt_filled_data'),-1)
        # rgb = np.array(rgb,np.float32)
        # depth = np.array(depth,np.float32)
        rgb = cv2.resize(rgb,(640,360))
        depth = cv2.resize(depth,(640,360))
        
        rgb = np.array(rgb,np.float32,copy=True).transpose((2,0,1)) 
        depth = np.array(depth,np.float32,copy=True)[:, :, None].transpose((2,0,1))

        # a = rgb[0]
        # # a[a!=0.0] = 10000
        # b = depth[0]
        # # b[b!=0.0] = 10000
        # c = np.abs(a-b)
        # save_dir = "/home/marian/calibration_ws/monodepth-FPN/MonoDepth-FPN-PyTorch/dataset/training_data/training_data/training_process_debug/"
        # cv2.imwrite(save_dir+'input_test_'+'.png',np.uint16(a))
        # cv2.imwrite(save_dir+'gt_test_'+'.png',np.uint16(b))
        # cv2.imwrite(save_dir+'diff_in_gt_test_'+'.png',np.uint16(c))
        # import matplotlib.pyplot as plt
        # plt.imshow(a, vmin=0, vmax=7000)
        # plt.colorbar()
        # plt.savefig('real1_'+'.png',bbox_inches='tight')
        # plt.close()

        # max_depth = 6571
        # max_depth = 8000 # depth.max()
        max_depth = depth.max()
        min_depth = 0
        max_ir = 3840
        
        # plt.plot(z.cpu(),'g')
        # plt.savefig("z.png")
        # plt.close()
        
        rgbTensor = torch.from_numpy(rgb).long()/max_depth
        depthTensor = torch.from_numpy(depth).long()/max_depth
        # rgbPIL = Image.fromarray(np.flipud(rgb))
        # depthPIL = Image.fromarray(np.array(depth))
        # cv2.imwrite('rgb.png',rgb)

        # if self.train:
        #     rgb, depth = np.array(rgb,np.uint8,copy=True), np.array(depth,copy=True)[:, :, None]
        #     rgb, depth = self.augmentation((rgb, depth))
        #     return self.rgb_transform(np.array(rgb)), self.depth_transform(np.array(depth)).squeeze(-1) #what's the squeeze job in here?

        # return Compose([Resize((240, 320)), ToTensor()])(rgb), ToTensor()(depth)
        # rgbComp = Compose([Resize((480,640))])(rgbTensor)
        # depthComp = Compose([Resize((480,640))])(depthTensor).unsqueeze(1)
        return  rgbTensor, depthTensor

        return rgbComp, depthComp

    def __len__(self):
        return self.length


class NYUv2Dataset(data.Dataset):

    def __init__(self, root='/disk2/data/nyuv2/', seed=None, train=True):

        np.random.seed(seed)
        self.root = Path(root)
        self.train = train
        if train:
            self.rgb_paths = [root+'train_rgb/' +
                              d for d in os.listdir(root+'train_rgb/')]
            # Randomly choose 50k images without replacement
            self.rgb_paths = np.random.choice(self.rgb_paths, 50000, False)
        else:
            self.rgb_paths = [root+'test_rgb/' +
                              d for d in os.listdir(root+'test_rgb/')]

        if train != train:  # vis
            self.train = True
            self.rgb_paths = [root+'vis_rgb/' +
                              d for d in os.listdir(root+'vis_rgb/')]

        self.augmentation = Compose(
            [RandomHorizontalFlip()])  # , RandomCropRotate(10)
        # ColorJitter(brightness=0.2, contrast=0.2, saturation=0.2, hue=0.1),
        self.rgb_transform = Compose(
            [ToPILImage(), Resize((480, 640)), ToTensor()])
        self.depth_transform = Compose(
            [ToPILImage(), Resize((120, 160)), ToTensor()])

        if self.train:
            self.length = 50000  # len(self.rgb_paths)
        else:
            self.length = len(self.rgb_paths)

    def __getitem__(self, index):
        path = self.rgb_paths[index]
        rgb = Image.open(path)
        depth = Image.open(path.replace('rgb', 'depth'))

        if self.train:
            rgb, depth = np.array(rgb), np.array(depth)[:, :, None]
            rgb, depth = self.augmentation((rgb, depth))
            return self.rgb_transform(rgb), self.depth_transform(depth).squeeze(-1)

        return Compose([Resize((240, 320)), ToTensor()])(rgb), ToTensor()(depth)

        return rgb, depth

    def __len__(self):
        return self.length


if __name__ == '__main__':
    # Testing
    dataset = NYUv2Dataset()
    print(len(dataset))
    for item in dataset[0]:
        print(item.size())
