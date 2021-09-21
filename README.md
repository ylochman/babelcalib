# BabelCalib: A Universal Approach to Calibrating Central Cameras

<div align="center">

[![Paper](https://img.shields.io/badge/arXiv-Preprint-brightgreen)](https://arxiv.org/abs/)
[![Conference](https://img.shields.io/badge/ICCV21-Paper-blue)](https://arxiv.org/abs/)
[![Poster](https://img.shields.io/badge/ICCV21-Poster-purple)](https://arxiv.org/abs/)
[![Youtube](https://img.shields.io/badge/ICCV21-Presentation-red)](https://youtu.be/)

This repository contains MATLAB-implementation of the BabelCalib calibration framework.

<img src="./assets/teaser.png" width="500rem">
    
<b>Method overview and result.</b> (left) BabelCalib pipeline: the camera model proposal step ensures a good initialization (right) example result showing residuals of reprojected corners of test images.
    
<br/>

<img src="./assets/ex1.png" width="200rem">
<img src="./assets/ex2.png" width="200rem">
<img src="./assets/ex3.png" width="200rem">
    
<b>Projection of calibration target from estimated calibration.</b> Detected corners are red crosses, target projected using initial calibration are blue squares and using the ﬁnal calibration are cyan circles.

</div>

## Content
- [Installation](#installation)
- [Usage](#usage)
    - [Option 1: Calibrating from .orpc + .dsc files](#opt1)
    - [Option 2: Calibrating from 2D-3D correspondences](#opt2)
    - [Configurations](#config)
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

<a name="usage"/>

## Usage
There are two data format options.
To calibrate your camera, you can use either:
- [`calib_run_opt1.m`](./calib_run_opt1.m) (for .orpc+.dsc format) or
- [`calib_run_opt2.m`](./calib_run_opt2.m) (for .mat i.e. 2D(corners)-3D(boards) correspondences format)
The data formats and the prototype functions are explained below.

<a name="opt1"/>

### Option 1: Calibrating from .orpc + .dsc files
1. Function [`calibrate_OD`](./core/calibrate_OD.m):
```matlab
function [model, res, corners_all, boards, imgsize, good_imgs] = calibrate_OD(orpc_paths, dsc_path, varargin)
    % Calibrate camera from .orpc-.dsc files
    %
    % Parameters
    % ----------
    % orpc_paths : cell array of character vectors
    %              list of paths to .orpc files (detected corners)
    % dsc_path : character vector
    %            path to .dsc file (calibration target)
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
See a complete example of using `calibrate_OD` and `get_poses_OD` in  [`calib_run_opt1.m`](./calib_run_opt1.m).

<a name="opt2"/>

### Option 2: Calibrating from 2D(corners)-3D(boards) correspondences
Function [`calibrate`](./core/calibrate.m):
```matlab
function [model, res, corners, boards] = calibrate(corners, boards, imgsize, varargin)
    % Calibrate camera from 2D(corners)-3D(boards) correspondences
    %
    % Parameters
    % ----------
    % corners : struct
    % boards : struct
    % imgsize : array
    %           image size as [height, width]
    % varargin : 'param_1', value_1, 'param_2', value_2, ...
    %            where param_i is either in the configurations (see default values in core/parse_cfg.m) or:
    %            model : struct (default: [])
    %            board_idxs : array (default: [])
    %                         list of indices of boards to use
    %            img_paths : cell array of character vectors (default: {})
    %                        list of image paths corresponding to orpc_paths
    %
    % Returns
    % -------
    % model : struct
    % res : struct
    % corners : struct
    % boards : struct
```
2. Function [`get_poses`](./core/get_poses.m):
```matlab
    function [model, res, corners, boards] = get_poses(intrinsics, corners, boards, imgsize, varargin)
    % Estimate camera poses from 2D(corners)-3D(boards) correspondences
    %
    % Parameters
    % ----------
    % intrinsics : struct
    % corners : struct
    % boards : struct
    % imgsize : array
    %           image size as [height, width]
    % varargin : 'param_1', value_1, 'param_2', value_2, ...
    %            where param_i is either in the configurations (see default values in core/parse_cfg.m) or:
    %            model : struct (default: [])
    %            board_idxs : array (default: [])
    %                         list of indices of boards to use
    %            img_paths : cell array of character vectors (default: {})
    %                        list of image paths corresponding to orpc_paths
    %
    % Returns
    % -------
    % model : struct
    % res : struct
    % corners : struct
    % boards : struct
```
See a complete example of using `calibrate` and `get_poses` in  [`calib_run_opt2.m`](./calib_run_opt2.m).

<a name="config"/>

### Configurations
The default configurations for the calibration functions are defined in [`parse_cfg.m`](./parse_cfg.m).
We provide the descriptions of each parameter below.
```matlab
    % Configurations of the solver
    % ----------
    % sample_size : int (default: 14)
    % solver_model : character vector (default: 'rat')
    % solver_complexity : int (default: 2)
    % final_model : character vector (default: 'kb')
    % final_complexity : int (default: 4)
    %
    % Configurations of the sampler
    % ----------
    % min_trial_count : int (default: 20)
    % max_trial_count : int (default: 50)
    % max_num_retries : int (default: 500)
    % confidence : float (default: 0.995)
    %
    % Configurations of RANSAC
    % ----------
    % display : boolean (default: true)
    % display_freq : int (default: 1)
    % irT : float (default: 0)
    %
    % Configurations of the refinement
    % ----------
    % reprojT : float (default: 1.5)
    % max_iter : int (default: 50)
```

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
The software is licensed under the BSD 3-Clause license. Please see [`LICENSE`](LICENSE) for details.
 
