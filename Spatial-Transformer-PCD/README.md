# Spatial Transformer for 3D Point Clouds

## Quick Start
For quick addition of the spatial transformer to your network, refer to [network architecture file](point_based/part_seg/part_seg_model_deform.py#L53) of how transformer can be added, and [offset_deform](point_based/utils/tf_util.py#L120-L160) for the transformer implementation.

## Requirements
* [Tensorflow](https://www.tensorflow.org/get_started/os_setup) (for the point-based method, version >= 1.13.1)
* [CAFFE](https://github.com/samaonline/caffe-deform) (for the sampling-based method, please use our version as we rewrite some source codes.)
* [NCCL](https://github.com/NVIDIA/nccl) (for multi-gpu in the sampling-based method)

## Point-based Methods

Please navigate to the specific folder first.

```bash
cd point_based
```

### Install Tensorflow and h5py

Install <a href="https://www.tensorflow.org/get_started/os_setup" target="_blank">TensorFlow</a>. You may also need to install h5py.

To install h5py for Python:
```bash
sudo apt-get install libhdf5-dev
pip install h5py
```

### Data Preparation

Download the data for part segmentation.

```
sh +x download_data.sh
```

### Running Examples

#### Train

Train the deformable spatial transformer. Specify number of gpus used with '--num_gpu'. Specify number of graphs and number of feature dimensions by '--graphnum' and '--featnum', respectively. 

```
cd part_seg
python train_deform.py --graphnum 2 --featnum 64
```

Model parameters are saved every 10 epochs in "train_results/trained_models/".

#### Evaluation

To evaluate the model saved after epoch n, 

```
python test_deform.py --model_path train_results/trained_models/epoch_n.ckpt  --graphnum 2 --featnum 64
```

## Sampling-based Methods

### Install Caffe

Please use [our version of CAFFE](https://github.com/samaonline/caffe-deform), as we provide the implementation of spatial transformers for bilateralNN, as described in the paper. A guide to CAFFE installation can be found [here](https://caffe.berkeleyvision.org/installation.html).

### Data Preparation

Please navigate to the specific folder first.

```bash
cd sampling-based
```

See instructions in [data/README.md](https://github.com/samaonline/spatial-transformer-for-3d-point-clouds/blob/master/sampling-based/data/README.md).

### Running Examples

    * ShapeNet Part segmentation
        * train and evaluate
            ```bash
            cd sampling-based/exp/shapenet3d
            ./train_test.sh
        * test trained model
            ```bash
            cd sampling-based/exp/shapenet3d
            ./test_only.sh
            ```
            Predictions are under `pred/`, with evaluation results in `test.log`.

