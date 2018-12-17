# Statistics_RT
A statistic library for Arduino.
Last Modified:	March 07, 2015, at 03:00 PM
By:	robtillaart
Platforms:	All
remarks & comments

Latest version on - Github

Intro
One of the main applications for the Arduino board is reading and logging of sensor data. For instance one monitors the temperature and air pressure every minute of the day. As that implies a lot of records, we often want the average and standard deviation to get an image of the variations of the temperature of that day.

Background reading - tutorial statistic formulas

Statistic library
The Statistic library just calculates the average and stdev of a set of data(floats). Furthermore it holds the minimum and maximum values entered. The interface consists of nine functions: (version 0.3.3 on Github)

	Statistic();		// constructor
	void clear();		// reset all counters
	void add(double);	// add a new value
	long count();		// # values added
	double sum();		// total
	double minimum();	// minimum
	double maximum();	// maximum
	double average();	// average
	double pop_stdev();	// population std deviation
	double unbiased_stdev();	// unbiased std deviation 

Internally the library does not record the individual values, only the count, sum and the squared sum (sum*sum), minimum and maximum. These five are enough to calculate the average and stdev. The nice part is that it does not matter if one adds 10, 100 or 1000 values.
