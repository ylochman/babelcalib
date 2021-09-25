# BabelCalib: A Universal Approach to Calibrating Central Cameras

<div align="center">

[![Paper](https://img.shields.io/badge/arXiv-Preprint-brightgreen)](https://arxiv.org/abs/)
[![Conference](https://img.shields.io/badge/ICCV21-Paper-blue)](https://arxiv.org/abs/)
[![Poster](https://img.shields.io/badge/ICCV21-Poster-purple)](https://arxiv.org/abs/)
[![Youtube](https://img.shields.io/badge/ICCV21-Presentation-red)](https://youtu.be/)

This repository contains the MATLAB implementation of the BabelCalib calibration framework.

<img src="./assets/teaser.png" width="500rem">
    
<b>Method overview and result.</b> (left) BabelCalib pipeline: the camera model proposal step ensures a good initialization (right) example result showing residuals of reprojected corners of test images.
    
<br/>

<img src="./assets/ex1.png" width="200rem">
<img src="./assets/ex2.png" width="200rem">
<img src="./assets/ex3.png" width="200rem">
    
<b>Projection of calibration target from estimated calibration.</b> Detected corners are red crosses, target projected using initial calibration are blue squares and using the ﬁnal calibration are cyan circles.

</div>

## Description
BabelCalib is a calibration framework that can estimate camera models for all types of central projection cameras. Calibration is robust and fully automatic. BabelCalib provides models for pinhole cameras with additive distortion as well as omni-directional cameras and catadioptric rigs. The supported camera models are listed under the [solvers](./core/solvers) directory.

BabelCalib supports calibration targets made of a collection of calibration boards, i.e., multiple planar targets. The method is agnostic to the pattern type on the calibration boards. The 3D<->2D correspondence of the calibration board fiducials to the detected corners in the captured images must be porovided. In addition, the board poses must be provided. Any calibration board of the target may be partially or fully occluded in a calibration image. The method is robust to inaccurately localized corners and outlying detections. 

## Table of Contents
- [Installation](#installation)
- [Calibration](#calibration)
    - [Evaluation](#evaluation)
    - [Type Definitions](#defs)
- [Examples and wrappers](#examples)
    - [Calibrating with 3D<->2D correspondences](#calib-csponds)
    - [Calibrating with Deltille](#calib-deltille)
- [Citation](#citation)
- [License](#license)

---
<a name="installation"/>

## Installation
You need to clone the repository. The required library [Visual Geometry Toolkit](https://github.com/prittjam/vgtk) is added as a submodule. Please clone the repository with submodules: 
```bash
git clone --recurse-submodules https://github.com/ylochman/babelcalib
```
If you already cloned the project without submodules, you can run
```bash
git submodule update --init --recursive 
```

<a name="calibration"/>

## Calibration
Calibration is performed by the function [`calibrate.m`](./core/calibrate.m). 
```matlab
function [model, res, corners, boards] = calibrate(corners, boards, imgsize, varargin)
```
Parameters:
* `corners` : type [corners](#corners)
* `boards` : type [boards](#boards)
* `imgsize` : 1x2 array
              specifies the height and width of the images; all the images are assumed to have the same dimensions.
* `varargin` : [optional arguments](#cfg)

Returns
* `model` : type [model](#model)
* `res` : type [res](#res)
* `corners` : type [corners](#corners)
* `boards` : type [boards](#boards)


<a name="evaluation"/>

## Evaluation
BabelCalib adopts the train/test set terminology. The training set contains the images used for calibration, and the test set contains held-out images for evaluation. Comparing on test-set images demonstrated how well the calibration generalizes to new images. During testing, the intriniscs are kept fixed and only the poses of the camera are estimated. The RMS re-projection error is used to assess the calibration quality. The poses are estimated by [`get_poses.m`](./core/get_poses.m):
```matlab
function [model, res, corners, boards] = get_poses(intrinsics, corners, boards, imgsize, varargin)
```    
Parameters:
* `intrinsics` : type [model](#model)
* `corners` : type [corners](#corners)
* `boards` : type [boards](#boards)
* `imgsize` : 1x2 array
              specifies the height and width of the images; all the images are assumed to have the same dimensions
* `varargin` : [optional arguments](#cfg)

Returns
* `model` : type [model](#model)
* `res` : type [res](#res)
* `corners` : type [corners](#corners)
* `boards` : type [boards](#boards)


<a name="defs"/>

## Type Defintions

<a name="corners"/>

#### `corners` : 1xN struct array 
Contains the set of 3D<->2D correspondences of the calibration board fiducials to the detected corners in each image. Let `N` be the number of images; Kn be the number of detected corners in the n-th image, where (n=1,...,N); and `B` be the number of planar calibration boards.

| field         | data type     | description  |
|:-------------:|:---------------------:|:-------------|
| x      | 2xKn array |  2D coordinates specifying the detected corners |
| cspond | 2xKn array | correspondences, where the first row contains the structure point correspondences and the second row contains board correspondences  |

<a name="boards"/>

#### `boards` : 1xB struct array
Contains the set of poses for each of the `B` calibration boards of the target, where (b=1,...,B) indexes the calibration boards.

| field         | data type     | description  |
|:-------------:|:---------------------:|:-------------|
| Rt | 3x4 array | The absolute orientation of each pose is encoded in the 3x4 pose matrix. |
| X  | 2xKb array | 2D coordinates of the fiducials on the board b of the target. The coordinates are specified with respect to a 2D coordinate system attached to each board |

<a name="model"/>

#### `model` : struct
Contains the intrinsics and extrinsics of the regressed camera model. The number of parameters of the back-projection or projection model, denoted `C`, depend on the chosen camera model and model complexity. 

| field         | data type     | description  |
|:-------------:|:---------------------:|:-------------|
| proj_params | 1xC array (C depends on model complexity) | The parameters of projection or back-projection function. |
| K  | 3x3 array | Affine matrix mapping from retinal coordinates to image coordinates (relating to `A` in the paper: `K = inv(A)`) |
| Rt | 3x4xN array | camera poses stacked along the array depth |

<a name="res"/>

#### `res` : struct
Contains extra information about the optimization. 

| field         | data type     | description  |
|:-------------:|:---------------------:|:-------------|

<a name="cfg"/>

#### `cfg`
`cfg` contains the optional configurations. Default values for the optional parameters are loaded from [`parse_cfg.m`](./core/parse_cfg.m). These values can be changed by using the `varargin` parameter. Parameters values passed in by `varargin` take precedence. The varargin format is `'param_1', value_1, 'param_2', value_2, ...`. The parameter descriptions are grouped by which component of BabelCalib they change.

Solver configurations:
* `final_model` - the selected camera model (default: 'kb')
* `final_complexity` - the degree of the polynomial, if the final model is a polynomial; otherwise, ignored (default: 4).

Sampler configurations:
* `min_trial_count` - minimum number of iterations (default: 20)
* `max_trial_count` - maximum number of iterations (default: 50)
* `max_num_retries` - maximum number of sampling tries in the case of a solver failure (default: 50)
* `confidence` - confidence rate (default: 0.995)
* `sample_size` - the number of 3D<->2D correspondences that are sampled for each RANSAC iteration (default: 14)

RANSAC configurations:
* `display` - toggles the display of verbose output of intermediate steps (default: true)
* `display_freq` - frequency of output during the iterations of robust sampling.   (default: 1)
* `irT` - minimum inlier ratio to perform refinement (default: 0)

Refinement configurations:
* `reprojT` - reprojection error threshold (default: 1.5)
* `max_iter` - maximum number of iterations on the refinement (default: 50)


<a name="examples"/>

## Examples and wrappers

<a name="csponds"/>

### 3D<->2D correspondences
See a complete example of using [`calibrate.m`](./core/calibrate.m) and [`get_poses.m`](./core/get_poses.m) in  [`calib_run_opt1.m`](./calib_run_opt1.m).

<a name="delitlle"/>

### Deltille
Wrappers
See a complete example of using `calibrate_OD` and `get_poses_OD` in  [`calib_run_opt2.m`](./calib_run_opt2.m).
- [`calib_run_opt2.m`](./calib_run_opt2.m) (for .orpc+.dsc format) or
The data formats and the prototype functions are explained below.

```
    % varargin : 'param_1', value_1, 'param_2', value_2, ...
    %            where param_i is either in the configurations (see default values in core/parse_cfg.m) or:
    %            model : struct (default: [])
    %                    PLACEHOLDER
    %            board_idxs : array (default: [])
    %                         list of indices of boards to use
    %            img_paths : cell array of character vectors (default: {})
    %                        list of image paths corresponding to orpc_paths
    %
```
See a complete example of using `calibrate` and `get_poses` in  [`calib_run_opt1.m`](./calib_run_opt1.m).


<a name="opt2"/>

### Calibrating from .orpc + .dsc files
1. Function [`calibrate_OD`](./core/calibrate_OD.m):
```matlab
function [model, res, corners_all, boards, imgsize, good_imgs] = calibrate_OD(orpc_paths, dsc_path, varargin)
    % Calibrate camera from .orpc-.dsc files
    %
    % Parameters
    % ----------
    % varargin : 'param_1', value_1, 'param_2', value_2, ...
    %            where param_i is either in the configurations (see default values in core/parse_cfg.m) or:
    %            board_idxs : array (default: [])
    %                         list of indices of boards to use
    %            img_paths : cell array of character vectors (default: {})
    %                        list of image paths corresponding to orpc_paths
    %
    % Returns
    % -------
    % model : struct
    % res : struct
    % corners_all : struct
    % boards : struct
    % imgsize : array
    %           image size as [height, width]
    % good_imgs : cell array of character vectors
```

2. Function [`get_poses_OD`](./core/get_poses_OD.m):
```matlab
function [model, res, corners_all, boards, imgsize, good_imgs] = get_poses_OD(orpc_paths, dsc_path, varargin)

    % Estimate camera poses from .orpc-.dsc files
    %
    % Parameters
    % ----------
    % orpc_paths : cell array of character vectors
    %              list of paths to .orpc files (detected corners)
    % dsc_path : character vector
    %            path to .dsc file (calibration target)
    % varargin : 'param_1', value_1, 'param_2', value_2, ...
    %             param_i is either in the configurations (see default values in core/parse_cfg.m) or:
    %            board_idxs : array (default: [])
    %                         list of indices of boards to use
    %            img_paths : cell array of character vectors (default: {})
    %                        list of image paths corresponding to orpc_paths
    %
    % Returns
    % -------
    % model : struct
    % res : struct
    % corners_all : struct
    % boards : struct
    % imgsize : array
    %           image size as [height, width]
    % good_imgs : cell array of character vectors

```
See a complete example of using `calibrate_OD` and `get_poses_OD` in  [`calib_run_opt2.m`](./calib_run_opt2.m).

<a name="citation"/>

## Citation
If you find this work useful in your research, please consider citing:

```bibtex
@InProceedings{Lochman-ICCV21,
    title     = {BabelCalib: A Universal Approach to Calibrating Central Cameras},
    author    = {Lochman, Yaroslava and Liepieshov, Kostiantyn and Chen, Jianhui and Perdoch, Michal and Zach, Christopher and Pritts, James},
    booktitle = {Proceedings of the IEEE/CVF International Conference on Computer Vision (ICCV)},
    year      = {2021},
}
```

<a name="license"/>

## License
The software is licensed under the MIT license. Please see [`LICENSE`](LICENSE) for details.
 
