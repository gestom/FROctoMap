#include "CFrelement.h"

using namespace std;

bool fremenSort(SFrelement i,SFrelement j) 
{ 
return (i.amplitude>j.amplitude); 
}

CFrelement::CFrelement()
{
	signalLength = 0;
	gain = 0;
}

CFrelement::~CFrelement()
{
}

void CFrelement::build(unsigned char* signal,int signalLengthi,int modelOrder)
{
	modelOrder++;
	signalLength = signalLengthi;
	int fftLength = signalLength/2+1;
	fftw_complex *coeffs;
	double *probability,*fftSignal;
	CTimer timer;
	timer.start();
	fftw_plan directFFTPlan;
	fftw_plan inverseFFTPlan;

	probability = (double *)malloc(signalLength*sizeof(double));
	fftSignal = (double*)malloc(signalLength*sizeof(double));
	coeffs = (fftw_complex *)fftw_malloc(fftLength*sizeof(fftw_complex));

	timer.reset();
	for (int i = 0;i<signalLength;i++) fftSignal[i] = signal[i];
	directFFTPlan = fftw_plan_dft_r2c_1d(signalLength,fftSignal,coeffs,FFTW_ESTIMATE);
	cout << "FFT preparation time " << timer.getTime() << endl;

	/*calculation of the spectral model*/
	fftw_execute(directFFTPlan);
	cout << "FFT calculation time " << timer.getTime() << endl;

	SFrelement *tmpFrelements = (SFrelement *)malloc(fftLength*sizeof(SFrelement));
	for(int i=0;i<fftLength;i++)
	{
		complex<double> x (coeffs[i][0],coeffs[i][1]);
		tmpFrelements[i].amplitude = abs(x);
		tmpFrelements[i].frequency = i;
	}
	sort(tmpFrelements,tmpFrelements+(fftLength-1),fremenSort);
	frelements.clear();
	gain = tmpFrelements[0].amplitude/signalLength;
	for(int i=1;i<modelOrder;i++){
		tmpFrelements[i].amplitude /= signalLength;
		tmpFrelements[i].phase = atan2(coeffs[tmpFrelements[i].frequency][1],coeffs[tmpFrelements[i].frequency][0]);
		frelements.push_back(tmpFrelements[i]);
	}
	free(tmpFrelements);
	cout << "Spectral model building time " << timer.getTime() << endl;

	/*signal reconstruction*/
	unsigned char *reconstructed = (unsigned char*)malloc(signalLength*sizeof(unsigned char));
	reconstruct(reconstructed,signalLength);	

	/*calculation of the outlier set*/
	int j=0;
	unsigned char flip = 0;
	for (int i = 0;i<signalLength;i++)
	{
		if (signal[i] != reconstructed[i]^flip){
			 flip = 1-flip;
			 outlierSet.push_back(i);
		}
	}

	fftw_destroy_plan(directFFTPlan); 
	fftw_free(coeffs);
	//free(signal); 
	return;
}

void CFrelement::update(int modelOrder)
{
	int fftLength = signalLength/2+1;
	fftw_complex *coeffs;
	double *probability;
	CTimer timer;
	timer.start();
	fftw_plan direct;
	fftw_plan inverse;
	probability = (double *)malloc(signalLength*sizeof(double));
	signal = (double*)malloc(signalLength*sizeof(double));
	coeffs = (fftw_complex *)fftw_malloc(fftLength*sizeof(fftw_complex));
	cout << "Time " << timer.getTime() << endl;

	direct = fftw_plan_dft_r2c_1d(signalLength,signal,coeffs,FFTW_ESTIMATE);
	cout << "Time construction direct" << timer.getTime() << endl;
	inverse = fftw_plan_dft_c2r_1d(signalLength,coeffs,probability,FFTW_ESTIMATE);

	cout << "Time construction inverse" << timer.getTime() << endl;

	/*reconstructing the frequency spectrum*/
	memset(coeffs,1,fftLength*sizeof(fftw_complex));
	coeffs[0][0] = gain;
	for (int i=0;i<frelements.size();i++){
		coeffs[frelements[i].frequency][0] = frelements[i].amplitude*cos(frelements[i].phase);
		coeffs[frelements[i].frequency][1] = frelements[i].amplitude*sin(frelements[i].phase);
	}
	fftw_execute(inverse);
	cout << "Time inverse " << timer.getTime() << endl;

	/*application of the outlier set*/
	int j=0;
	unsigned char flip = 0;
	for (int i = 0;i<signalLength;i++)
	{
		if (outlierSet[j] == i){
			 flip = 1-flip;
			 j++;
		}
		signal[i] = ((probability[i]>0.5)^flip);
	}

	/*coefficient recalculation*/
	fftw_execute(direct);
	cout << "Time direct " << timer.getTime() << endl;

	double *absCoeffs;
	SFrelement *tmpFrelements;
	tmpFrelements = (SFrelement *)malloc(fftLength*sizeof(SFrelement));
	for(int i=0;i<fftLength;i++)
	{
		complex<double> x (coeffs[i][0],coeffs[i][1]);
		tmpFrelements[i].amplitude = abs(x);
		tmpFrelements[i].frequency = i;
	}
	sort(tmpFrelements,tmpFrelements+(fftLength-1),fremenSort);
	frelements.clear();
	for(int i=0;i<modelOrder;i++){
		tmpFrelements[i].amplitude /= signalLength;
		tmpFrelements[i].phase = atan2(coeffs[tmpFrelements[i].frequency][1],coeffs[tmpFrelements[i].frequency][0]);
		frelements.push_back(tmpFrelements[i]);
	}

	fftw_destroy_plan(direct); 
	fftw_destroy_plan(inverse);
	fftw_free(coeffs);
	//free(signal); 
	return;
}

void CFrelement::reconstruct(unsigned char* signal,int signalLength)
{
	CTimer timer;
	timer.start();

	int fftLength = signalLength/2+1;
	fftw_complex *coeffs;
	double *probability;
	fftw_plan inverseFFTPlan;

	probability = (double *)malloc(signalLength*sizeof(double));
	coeffs = (fftw_complex *)fftw_malloc(fftLength*sizeof(fftw_complex));

	inverseFFTPlan = fftw_plan_dft_c2r_1d(signalLength,coeffs,probability,FFTW_ESTIMATE);

	/*reconstructing the frequency spectrum*/
	memset(coeffs,1,fftLength*sizeof(fftw_complex));
	coeffs[0][0] = gain;
	for (int i=0;i<frelements.size();i++){
		coeffs[frelements[i].frequency][0] = frelements[i].amplitude*cos(frelements[i].phase);
		coeffs[frelements[i].frequency][1] = frelements[i].amplitude*sin(frelements[i].phase);
	}
	cout << "IFFT preparation " << timer.getTime() << endl;

	fftw_execute(inverseFFTPlan);
	cout << "IFFT calculation " << timer.getTime() << endl;

	/*application of the outlier set*/
	int j=0;
	unsigned char flip = 0;
	//for (int i = 0;i<signalLength;i++) cout << probability[i] << " " << estimate(i) << endl;
	if (outlierSet.size() > 0){
		for (int i = 0;i<signalLength;i++)
		{
			if (outlierSet[j] == i){
				flip = 1-flip;
				j++;
			}
			signal[i] = ((probability[i]>0.5)^flip);
		}
	}else{
		for (int i = 0;i<signalLength;i++) signal[i] = probability[i]>0.5;
	}
	cout << "Signal reconstruction time " << timer.getTime() << endl;

	fftw_destroy_plan(inverseFFTPlan);
	fftw_free(coeffs);
	free(probability); 
	return;
}

/*fills with values*/
void CFrelement::fill(unsigned char values[],int number)
{
}

/*gets length in terms of values measured*/
int CFrelement::getLength()
{
	return signalLength;
}

  /*gets length in terms of values measured*/
void CFrelement::add(unsigned char value)
{
	if (((estimate(signalLength) > 0.5)^((outlierSet.size()%2)==1))!=value)outlierSet.push_back(signalLength);
	signalLength++;
	return; 
}

/*text representation of the FREMEN model*/
void CFrelement::print()
{
	int errs = 0;
	for (int i=0;i<outlierSet.size()/2;i++) errs+=(outlierSet[2*i+1]-outlierSet[2*i]);
	if (outlierSet.size()%2 == 1) errs+=signalLength-outlierSet[outlierSet.size()-1];
	std::cout << "Model order " << frelements.size() << " prior: " << gain << " error: " << ((float)errs/signalLength) << " size: " << sizeof(this)<< endl;
	for (int i = 0;i<frelements.size();i++){
		std::cout << "Frelement " << i << " " << frelements[i].amplitude << " " << frelements[i].phase << " " << frelements[i].frequency << " " << endl;
	}
	std::cout << "Outlier set size " << outlierSet.size() << ":";
	copy(outlierSet.begin(), outlierSet.end(), ostream_iterator<int>(cout, " "));	
	std::cout << endl; 
}

/*clears all*/
void CFrelement::clear()
{
}

/*retrieves a boolean*/
unsigned char CFrelement::retrieve(int timeStamp)
{
	int i = 0;
	for (i= 0;i<outlierSet.size();i++){
		if (timeStamp < outlierSet[i]) break;
	}
	return (estimate(signalLength) > 0.5)^(i%2);
}
 
float CFrelement::estimate(int timeStamp)
{
	float time = (float)timeStamp/signalLength;
	float estimate = gain;
	for (int i = 0;i<frelements.size();i++){
		estimate+=2*frelements[i].amplitude*cos(frelements[i].phase+time*frelements[i].frequency*2*M_PI);
	}
	return estimate;
}
