# MonoDepth-FPN-PyTorch

[![License][license]][license-url]

A simple end-to-end model that achieves state-of-the-art performance in depth prediction implemented in PyTorch. We used a Feature Pyramid Network (FPN) backbone to estimate depth map from a single input RGB image. We tested the performance of our model on the NYU Depth V2 Dataset (Official Split) and the KITTI Dataset (Eigen Split).

## Requirements
* Python 3
* PyTorch v?
* Open3D v?
* And many others 
* CUDA v11? (if using CUDA)


## To Run

Once you are in `.venv_mono_depth`, run this to train the network:
```
python main_fpn_curv_grad.py --cuda --epochs 20
```

```
python3 main_fpn.py --cuda --bs 6
```
To continue training from a saved model, use
```
python3 main_fpn.py --cuda --bs 6 --r True --checkepoch 10
```
To visualize the reconstructed data, run the jupyter notebook in vis.ipynb.


## Data Processing
### My Custom Dataset



### Verify planes curvature
* Be sure you are in the venv_mono_depth (conda or pip) environment.
* Starting from a folder with a bunch of depth images combined of 3 channels, e.g: comb_ddd_images.PNG (name of a folder)

* Create original point clouds and model estimated point clouds, from the above images, by runnning:
```
python save_normalized_raw_pred_pairs --input_image_path path/to/comb_ddd_images.PNG/ --cuda
```
and by changing in the code two replace statements (.replace("comb_ddd_images.PNG","orig.PCD.test") and .replace("comb_ddd_images.PNG","pred.PCD.test")).

* Next is the plane extraction from both original and preditcted point clouds:
```
roscore
```
```
rosrun ddd_plane_extr ddd_plane_extra_save _pcd_folder:=path/to/orig.PCD.test/ 
```
of course after you modified in `ddd_plane_extra_save.cpp` the `saveDirPcd` variable to `path/to/orig_planes.PCD.test/` and 
the a `regex_replace` statements: `regex_replace(pcd_file,regex("orig.PCD.test"),"pred.PCD.test");`

* After you have the planes, all you have to do is to run `point2plane.py` in order to estimate the curvature gradient:
```
python point2plane.py
```
by changing the `directory` variable to `orig_planes.PCD.test` and change the first `.replace` statement to something like this:
```
.replace("orig_planes.PCD.test","pred_planes.PCD.test").replace("_orig.pcd","_pred.pcd")
```
and the name of the `.csv` file if you want, BUT from within the `.venv_measure_curva` environment.
 
## Model

<p align="center"><img src='https://github.com/xanderchf/i2d/blob/master/architecture.png' width=600></p>

* We used Feature Pyramid Network (FPN) with ResNet101 as backbone (shaded yellow), loaded ImageNet pretrained weight as weight initialization.
* We used pixel-shuffle for upsampling and fuse feature maps with add operation; bilinear interpolation is employed after pixel-shuffle to deal with inconsistent feature map size.
* Two consecutive 3x3 convolutions for feature processing.
* No non-linearity in the top-down branch of FPN, and ReLU in other convolution layers, and Sigmoid in the prediction layer for better stability.
* Trained on the weighted sum of the depth loss, the gradient loss, and the normal loss for 20 epochs for the NYU Depth V2 dataset, and 40 epochs for the KITTI dataset; gradient loss added after epoch 1, and normal loss added after epoch 10.
* Outputs prediction of size ¼, and evaluated after bilinear upsampling.

## Loss Function

We employed three parts in the loss function in our model. The loss is a weighted sum of 3 parts: the depth loss, the gradient loss and the surface normal loss.

### Depth Loss
![img](https://latex.codecogs.com/gif.latex?L_%7B%5Ctextup%7Bdepth%7D%7D%20%3D%20%5Cfrac%7B1%7D%7Bn%7D%20%5Csum_%7Bi%3D1%7D%5E%7Bn%7D%20%5Csqrt%5B%5D%7B%5Clog%5E2%28d_i%29%20-%20%5Clog%5E2%28p_i%29%7D)

The depth loss is RMSE in log scale, which we found converges better than L1 and L2 norm. Supervising in log scale makes the classifier focus more on closer objects.

### Gradient Loss
![img](https://latex.codecogs.com/gif.latex?L_%7B%5Ctextup%7Bgrad%7D%7D%20%3D%20%5Cfrac%7B1%7D%7Bn%7D%20%5Csum_%7Bi%3D1%7D%5E%7Bn%7D%20%5Cbig%7C%5Cbig%7C%20%5Cnabla%20%5Ctextbf%7Bd%7D%20-%20%5Cnabla%20%5Ctextbf%7Bp%7D%20%5Cbig%7C%5Cbig%7C_1)

The gradient of depth maps is obtained by a Sobel filter; the gradient loss is the L1 norm of the difference.

### Surface Normal Loss
![img](https://latex.codecogs.com/gif.latex?L_%7B%5Ctextup%7Bnormal%7D%7D%20%3D%20%5Cfrac%7B1%7D%7Bn%7D%20%5Csum_%7Bi%3D1%7D%5E%7Bn%7D%20%5Cbigg%28%201%20-%20%5Cfrac%7B%5Clangle%20n%5Ed_i%2C%20n%5Ep_i%20%5Crangle%7D%7B%7C%7Cn%5Ed_i%7C%7C%5Ctextbf%7B%20%7D%7C%7Cn%5Ep_i%7C%7C%7D%20%5Cbigg%29)
We also employed the normal vector loss proposed by Hu et al., which helps refining details.

The weight ratio between the three loss was set to 1:1:1.

## Qualitative Evaluation
### KITTI
Comparison with state-of-the-art methods:
<p align="center"><img src='https://github.com/xanderchf/i2d/blob/master/kitti_comparison.png' width=900></p>
More comparison:
<p align="center"><img src='https://github.com/xanderchf/i2d/blob/master/kitti.png' width=600></p>


## Quantitative Evaluation

### KITTI
<p align="center"><img src='https://github.com/xanderchf/i2d/blob/master/kitti_performance.png' width=900></p>

We use the following depth evaluation metrics used by Eigen et al.:
<p align="center"><img src='metrics.png' width=400></p>
where T is the number of valid pixels in the test set.

## Discussion

* FPN is an effective backbone for monocular depth estimation because of its ability to extract features and semantics at different scales. It can achieve its potential if guided by proper loss functions.
* Gradient and normal losses help prevent the model getting stuck in local optimum and guide it toward better convergence, as shown in the ablation study.
* Some existing state-of-the-art methods ourperform ours in certain metrics, and we believe that this is due to the adaptive BerHu loss. We will try to incorporate this loss soon.

## Related Work
* Eigen et al. were the first to use CNNs for depth estimation: predicting a coarse global output and then a finer local network.
* Laina et al. explored CNNs pre-trained with AlexNet, VGG16 and ResNet.
* Godard et al. and Kuznietsov et al. introduced unsupervised and semi-supervised methods that relies on 3D geometric assumptions of left-right consistency. They trained the network using binocular image data.
* Fu et al. proposed a framework based on classification of discretized depth ranges and regression. They supervised depth prediction at different resolutions and used a fusion network to produce the final output.
* Hu et al. proposed a novel loss of normal vectors in addition to the conventional depth and gradient losses. They first trained a base network without skip connections, and then train the refine network using the novel loss after freezing the weights of the base network.
* This project: Fully convolutional with no FC layers needed, FPN provides a straightforward and unified backbone to extract features maps that incorporates strong and localized semantic information. We employ an easy to follow curriculum to add in gradient loss and normal loss during training, all losses are calculated on the output feature maps instead of intermediate ones.

## To-dos

- [x] Add ablation study for NYU Depth V2
- [x] Add ablation study for KITTI
- [x] Add visualization ipynb
- [ ] Add support for the ScanNet Dataset
- [ ] Add code for CVPR 2018 ROB Challenge

<!-- Markdown link & img dfn's -->
[KITTI dataset]: http://www.cvlibs.net/datasets/kitti/
[NYU Depth V2 dataset]: https://cs.nyu.edu/~silberman/datasets/nyu_depth_v2.html
[github page]: https://github.com/janivanecky/Depth-Estimation/tree/master/dataset
[license]: https://img.shields.io/github/license/mashape/apistatus.svg
[license-url]: https://github.com/xanderchf/i2d/blob/master/LICENSE
[Robust Vision Challenge]: http://robustvision.net/

## References
* Eigen, D., Puhrsch, C., Fergus, R.: Depth map prediction from a single image using a multiscale
deep network. In: Advances in neural information processing systems (NIPS). (2014)
2366–2374
* Fu, Huan, Mingming Gong, Chaohui Wang and Dacheng Tao. “A Compromise Principle in Deep Monocular Depth Estimation.” CoRR abs/1708.08267 (2017): n. pag.
* R. Garg, V. Kumar, G. Carneiro, and I. Reid. Unsupervised cnn for single view depth estimation: Geometry to the rescue. In Proc. of the European Conf. on Computer Vision (ECCV), 2016.
* Geiger, Andreas, et al. "Vision meets robotics: The KITTI dataset." The International Journal of Robotics Research 32.11 (2013): 1231-1237.
* C. Godard, O. Mac Aodha, and G. J. Brostow. Unsupervised monocular depth estimation with left-right consistency. arXiv:1609.03677v2, 2016.
* Hu, Junjie, et al. "Revisiting Single Image Depth Estimation: Toward Higher Resolution Maps with Accurate Object Boundaries." arXiv preprint arXiv:1803.08673 (2018).
* Kuznietsov, Yevhen, Jörg Stückler, and Bastian Leibe. "Semi-supervised deep learning for monocular depth map prediction." Proc. of the IEEE Conference on Computer Vision and Pattern Recognition. 2017.
* I. Laina, C. Rupprecht, V. Belagiannis, F. Tombari, and N. Navab. Deeper depth prediction with fully convolutional residual networks. In Proc. of the Int. Conf. on 3D Vision (3DV), 2016.
* Levin, Anat, Dani Lischinski, and Yair Weiss. "Colorization using optimization." ACM Transactions on Graphics (ToG). Vol. 23. No. 3. ACM, 2004.
* Silberman, Nathan, et al. "Indoor segmentation and support inference from rgbd images." European Conference on Computer Vision. Springer, Berlin, Heidelberg, 2012.
