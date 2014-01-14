#include "roboteq_sdc2130/regulator.hpp"

Regulator::Regulator()
{
	p = i = d = i_max = out_max = 0;
	integrator = previous = 0;
	first = true;

	ff_term = p_term = i_term = d_term  = 0.0;
}

/* PID regulator. Input and output must be in equal units */
double Regulator::output_from_input( double setpoint , double input , double period)
{
	// Implement max period
	if(period > 0.5)
		period = 0.5;

	// Calculate errors
	double error = setpoint - input;

	// Calculate integrator
	integrator += error * period;

	// Implement anti wind up
	if(integrator > i_max)
		integrator = i_max;
	else if(integrator < -i_max)
		integrator = -i_max;

	// Calculate differentiator

	double differentiator = ( previous - input ) / period;

	if(first)
	{
		first = false;
		differentiator = 0.0;
	}

	ff_term = setpoint * feed_forward;
	p_term = (error * p);
	i_term = (i*integrator);
	d_term = (differentiator * d);

	// Calculate output
	double output = ff_term + p_term + i_term + d_term;

	// Implement output max
	if(output > out_max)
		output = out_max;
	else if(output < -out_max)
		output = -out_max;

	// Upkeep
	previous = input;

	return output;
}
