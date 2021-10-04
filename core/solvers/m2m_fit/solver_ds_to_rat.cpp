#include <Eigen/Dense>
#include "mex.h"




using namespace Eigen;




MatrixXcd solver_ds_to_rat(const VectorXd& data)
{
	// Compute coefficients
    const double* d = data.data();
    VectorXd coeffs(14);
    coeffs[0] = -1;
    coeffs[1] = -2*d[3];
    coeffs[2] = std::pow(d[2],2) - d[1];
    coeffs[3] = -2*d[2];
    coeffs[4] = 1;
    coeffs[5] = 2*d[0]*d[2];
    coeffs[6] = -2*d[0];
    coeffs[7] = std::pow(d[0],2);
    coeffs[8] = -2*d[7];
    coeffs[9] = std::pow(d[6],2) - d[5];
    coeffs[10] = -2*d[6];
    coeffs[11] = 2*d[4]*d[6];
    coeffs[12] = -2*d[4];
    coeffs[13] = std::pow(d[4],2);



	// Setup elimination template
	static const int coeffs0_ind[] = { 0,0,1,0,0,8,0,0,2,1,8,9,3,1,0,10,0,8,2,9,5,3,2,10,8,11,1,9,6,4,3,4,12,10 };
	static const int coeffs1_ind[] = { 13,7,7,13,11,5,5,11,9,2,7,6,5,12,10,13,3,11,7,12,6,13,6,4,4,12 };
	static const int C0_ind[] = { 0,5,8,9,11,13,18,23,24,25,27,29,32,34,36,37,38,39,41,43,48,49,50,51,52,53,54,55,56,57,58,59,61,63 } ;
	static const int C1_ind[] = { 4,6,9,11,12,14,17,19,20,22,24,25,26,27,28,29,30,31,34,36,38,39,42,44,46,47 };

	Matrix<double,8,8> C0; C0.setZero();
	Matrix<double,8,6> C1; C1.setZero();
	for (int i = 0; i < 34; i++) { C0(C0_ind[i]) = coeffs(coeffs0_ind[i]); }
	for (int i = 0; i < 26; i++) { C1(C1_ind[i]) = coeffs(coeffs1_ind[i]); } 

	Matrix<double,8,6> C12 = C0.partialPivLu().solve(C1);



	// Setup action matrix
	Matrix<double,9, 6> RR;
	RR << -C12.bottomRows(3), Matrix<double,6,6>::Identity(6, 6);

	static const int AM_ind[] = { 4,5,0,1,6,2 };
	Matrix<double, 6, 6> AM;
	for (int i = 0; i < 6; i++) {
		AM.row(i) = RR.row(AM_ind[i]);
	}

	Matrix<std::complex<double>, 2, 6> sols;
	sols.setZero();

	// Solve eigenvalue problem
	EigenSolver<Matrix<double, 6, 6> > es(AM);
	ArrayXcd D = es.eigenvalues();	
	ArrayXXcd V = es.eigenvectors();
V = (V / V.row(0).array().replicate(6, 1)).eval();


    sols.row(0) = D.transpose().array();
    sols.row(1) = V.row(4).array();





	return sols;
}
// Action =  x
// Quotient ring basis (V) = 1,x,x^2,x*y,y,y^2,
// Available monomials (RR*V) = x^3,x^2*y,x*y^2,1,x,x^2,x*y,y,y^2,





void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
	if (nrhs != 1) {
		mexErrMsgIdAndTxt("automatic_generator_cvpr:ds_to_rat:nrhs", "One input required.");
	}
	if (nlhs != 1) {
		mexErrMsgIdAndTxt("automatic_generator_cvpr:ds_to_rat:nlhs", "One output required.");
	}    
	if (!mxIsDouble(prhs[0]) || mxIsComplex(prhs[0])) {
		mexErrMsgIdAndTxt("automatic_generator_cvpr:ds_to_rat:notDouble", "Input data must be type double.");
	}
	if(mxGetNumberOfElements(prhs[0]) % 8 != 0) {
		mexErrMsgIdAndTxt("automatic_generator_cvpr:ds_to_rat:incorrectSize", "Input size must be multiple of 8.");
	}
	int n_instances = mxGetNumberOfElements(prhs[0]) / 8;
	double *input = mxGetPr(prhs[0]);
	plhs[0] = mxCreateDoubleMatrix(2,6*n_instances,mxCOMPLEX);
	double* zr = mxGetPr(plhs[0]);
	double* zi = mxGetPi(plhs[0]);
	for(int k = 0; k < n_instances; k++) {
		const VectorXd data = Map<const VectorXd>(input + k*8, 8);
		MatrixXcd sols = solver_ds_to_rat(data);
		Index offset = k*sols.size();
		for (Index i = 0; i < sols.size(); i++) {
			zr[i+offset] = sols(i).real();
			zi[i+offset] = sols(i).imag();
		}
	}
}


