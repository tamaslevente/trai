# ToFNest: Efficient  normal  estimation  for  ToF Depth  cameras

## Abstract
In this work we propose an efficient normal estimation method for  depth  images  acquired  by  Time-of-Flight  (ToF)  cameras based  on  feature  pyramid  networks.  We  do  the  our  normal estimation training starting from the 2D depth images, projecting  the  measured  data  into  the  3D  space  and  computing  theloss  function  for  the  pointcloud  normal.  Despite  the  simplicity of the methods it proves to be efficient in terms of robustness.Compared with the state of the art methods, our method proved to be faster with similar precision metrics from other methods on  public  datasets.  In order  to  validate  our  proposed  solution we  made  an  extensive  testing  using  both  public  datasets  and custom recorded indoor and ourdoor datasets as well.

## Overview

## Content
- [Prerequisites](#prerequisites)
- [Data Preparation](#data-preparation)
- [Training & Evaluation](#training&evaluation)
- [How to run](#howtorun)
- [Demo](#demo)

## Prerequisites
The code was built using the following libraries ([requirements.txt](requirements.txt)):
- [Python](https://www.python.org/downloads/)  >= 3.6
- [PyTorch](https://github.com/pytorch/pytorch) >= 1.3
- [Scipy](https://github.com/scipy/scipy)
- [OpenCV](https://github.com/opencv/opencv)
- [Imageio](https://imageio.github.io/)
- [Matplotlib](https://matplotlib.org/stable/index.html)
- [Constants](https://pypi.org/project/constants/)

## Data Preparation

## Training & Evaluation
![arch_new](https://user-images.githubusercontent.com/22835687/109430618-f692a780-7a0a-11eb-9270-1421457f8433.png)

## How to run

## Demo


