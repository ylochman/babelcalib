## Supported models
The supported models are shown in this table from the paper:

<div align="center">
<img src="../../assets/supported_models.png" width="800rem">
</div>

### Model-to-model regression functions
All the camera model-to-model regression functions are located under [`m2m_fit`](./m2m_fit).

* Brown-Conrady (BC) - [`fit_bc_to_rat.m`](./m2m_fit/fit_bc_to_rat.m)
* Kannala-Brandt (KB) - [`fit_kb_to_rat.m`](./m2m_fit/fit_kb_to_rat.m)
* Uniﬁed Camera (UCM) - [`fit_ucm_to_rat.m`](./m2m_fit/fit_ucm_to_rat.m)
* Field of View (FOV) - [`fit_fov_to_rat.m`](./m2m_fit/fit_fov_to_rat.m)
* Extended Uniﬁed Camera (EUCM) - [`fit_eucm_to_rat.m`](./m2m_fit/fit_eucm_to_rat.m)
* Double Sphere (DS) - [`fit_ds_to_rat.m`](./m2m_fit/fit_ds_to_rat.m)
* Division (DIV) - [`fit_div_to_rat.m`](./m2m_fit/fit_div_to_rat.m)
* Division-Even - [`fit_rat_to_rat.m`](./m2m_fit/fit_rat_to_rat.m)

Note: the back-projection model used in the solvers (Division-Even) is referred in the code as Rational (`rat`) model, because the Division-Even model is an instance of the Rational model where all the coefficients in the numerator are zero.