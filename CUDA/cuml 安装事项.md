# cuml 安装事项

安装CUML可以通过conda和源码

建议通过conda安装，在官网找到如下命令

```shell
conda create -n rapids-22.06 -c rapidsai -c nvidia -c conda-forge rapids=22.06 python=3.8 cudatoolkit=11.4
```

但是通过这个方式安装其中会有一个包cuda-python会安装版本（11.7.1）过高导致报错

```shell
typeError: C function cuda.ccudart.cudaStreamSynchronize has wrong signature (expected __pyx_t_4cuda_7ccudart_cudaError_t (__pyx_t_4cuda_7ccudart_cudaStream_t), got cudaError_t (cudaStream_t))
```

所以需要降级（11.7.0）

[#ISSUES](https://github.com/rapidsai/cuml/issues/4798)

使用CUML的同时是没法使用torch

[ISSUES](https://github.com/rapidsai/cuml/issues/4444)

是由于cudatoolkit版本过高导致，因此需要升级torch的版本或者降低cudatoolkit的版本，但是降低cudatoolkit的版本会导致cuml的一些其他问题，所以最好就是升级torch



升级torch=1.8.1+cu111 到1.11.0+cu115 即可

torchvision 使用 0.9.1+cu111

cudatoolkit使用11.4.1



