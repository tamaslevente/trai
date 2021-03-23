#!/usr/bin/env bash
#/bin/bash
/usr/local/cuda-10.0/bin/nvcc tf_sampling_g.cu -o tf_sampling_g.cu.o -c -O2 -DGOOGLE_CUDA=1 -x cu -Xcompiler -fPIC
#g++ -std=c++11 tf_sampling.cpp tf_sampling_g.cu.o -o tf_sampling_so.so -shared -fPIC -I /data/lirh/anaconda3/envs/tensorflow3/lib/python3.6/site-packages/tensorflow/include  -I /usr/local/cuda-8.0/include -lcudart -L /usr/local/cuda-8.0/lib64/ -O2 -D_GLIBCXX_USE_CXX11_ABI=0

#g++ -std=c++11 tf_sampling.cpp tf_sampling_g.cu.o -o tf_sampling_so.so -shared -fPIC -I  /home/cuda/Alex/.venv_nbv/lib64/python3.7/site-packages/tensorflow_core/include -I /usr/local/cuda-10.0/include -I home/cuda/Alex/.venv_nbv/lib64/python3.7/site-packages/tensorflow_core/include/external/nsync/public -lcudart -L /usr/local/cuda-10.0/lib64/ -L/home/cuda/Alex/.venv_nbv/lib64/python3.7/site-packages/tensorflow_core -ltensorflow_framework -O2 -D_GLIBCXX_USE_CXX11_ABI=0

#g++ -std=c++11 tf_sampling.cpp tf_sampling_g.cu.o -o tf_sampling_so.so -shared  -fPIC -I /home/cuda/Alex/.venv_nbv/lib/python3.7/site-packages/tensorflow_core/include  -I /usr/local/cuda-10.0/include -I /home/cuda/Alex/.venv_nbv/lib64/python3.7/site-packages/tensorflow_core/include/external/nsync/public -lcudart -L /usr/local/cuda-10.0/lib64/ -L/home/cuda/Alex/.venv_nbv/lib/python3.7/site-packages/tensorflow_core/ -ltensorflow_framework -O2 -D_GLIBCXX_USE_CXX11_ABI=0

#g++ -std=c++11 tf_sampling.cpp tf_sampling_g.cu.o -o tf_sampling_so.so -shared -fPIC -I /usr/local/lib/python3.5/dist-packages/tensorflow/include -I /usr/local/cuda-10.0/include -I /usr/local/lib/python3.5/dist-packages/tensorflow/include/external/nsync/public -lcudart -L /usr/local/cuda-10.0/lib64/ -L/usr/local/lib/python3.5/dist-packages/tensorflow -ltensorflow_framework -O2 -D_GLIBCXX_USE_CXX11_ABI=0

g++ -std=c++11 tf_sampling.cpp tf_sampling_g.cu.o -o tf_sampling_so.so -shared -fPIC -I /usr/local/lib/python3.5/dist-packages/tensorflow/include -I /usr/local/cuda-10.0/include -I /usr/local/lib/python3.5/dist-packages/tensorflow/include/external/nsync/public -lcudart -L /usr/local/cuda-10.0/lib64/ -L/usr/local/lib/python3.5/dist-packages/tensorflow -ltensorflow_framework -O2 