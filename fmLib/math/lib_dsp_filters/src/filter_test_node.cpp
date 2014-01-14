/*
 * filter_test_node.cpp
 *
 *  Created on: Jun 24, 2013
 *      Author: morten
 */

#include <iostream>


#include <DspFilters/Butterworth.h>


int main(int argc, char** argv)
{
	Dsp::SimpleFilter<Dsp::Butterworth::LowPass<8>, 1>  filter;
	filter.setup(7,100.0,5);

	double* impulse;
	impulse = new double[1024];
	impulse[0] = 1.0;
	for (int i = 1; i<1024;i++)
	{
		impulse[i] = 0.0;
	}

	/*
	 * Print filter response at different frequencies
	 * */
	for(int i = 0; i<100;i++)
	{
		
		Dsp::complex_t val = filter.response(i/100.0);
		std::cout << i << "\t" << std::norm(val) << "\t" << val.imag() <<  std::endl;
	}

	// filter signal in impulse array
	filter.process<double>(1024,&impulse);

	// print filtered signal (impulse array is modified)
	for(int i = 0; i<1024; i++)
	{
	
		std::cout << impulse[i] << std::endl;
	}



	return 0;
}
