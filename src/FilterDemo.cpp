//============================================================================
// Name        : FilterDemo.cpp
// Author      : Samuel Charreyron
// Version     :
// Copyright   : Your copyright notice
// Description :
//============================================================================

#include <iostream>
using namespace std;

#include <cassert>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>

#include <boost/assign/list_of.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>

#include "pointmatcher/PointMatcher.h"

using namespace std;
using namespace PointMatcherSupport;
using namespace boost::assign;


typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;

enum FilterOption {
	BOUNDING_BOX,
	FIX_STEP_SAMPLING,
	MAX_DENSITY,
	MAX_DIST,
	MAX_POINT_COUNT,
	MAX_QUANTILE_ON_AXIS,
	MIN_DIST,
	OBSERVATION_DIRECTION,
	ORIENT_NORMALS,
	RANDOM_SAMPLING,
	REMOVE_NAN,
	SAMPLING_SURFACE_NORMAL,
	SHADOW,
	SIMPLE_SENSOR_NOISE,
	SURFACE_NORMAL
};

typedef map<FilterOption,string> Opt_2_Str;
Opt_2_Str filter_opt_m = map_list_of
		(BOUNDING_BOX,"bounding box")
		(FIX_STEP_SAMPLING,"fixed step sampling")
		(MAX_DENSITY,"maximum density")
		(MAX_DIST,"maximum distance")
		(MAX_POINT_COUNT,"maximum point count")
		(MAX_QUANTILE_ON_AXIS,"maximum quantile on axis")
		(MIN_DIST,"minimum distance")
		(OBSERVATION_DIRECTION,"observation direction")
		(ORIENT_NORMALS,"orient normals")
		(RANDOM_SAMPLING,"random sampling")
		(REMOVE_NAN,"remove NaN")
		(SAMPLING_SURFACE_NORMAL,"sampling surface normal")
		(SHADOW,"shadow filter")
		(SIMPLE_SENSOR_NOISE,"simple sensor noise")
		(SURFACE_NORMAL,"surface normal");

void usage(char *argv[])
{
	cerr << "Usage " << argv[0] << " <filterType (0-6)> INPUT.csv/.vtk OUTPUT.csv/.vtk" << endl;
	cerr << endl << "Example:" << endl;
	cerr << argv[0] << " 0 ../examples/data/cloud.00000.vtk /tmp/output.vtk" << endl << endl;
}

void print_filter_opts() {
	BOOST_FOREACH(Opt_2_Str::value_type & i, filter_opt_m) {
		cout << setw(30) << left << i.second << "\t" << i.first << endl;
	}
}

int main(int argc, char *argv[])
{

	if (argc < 3)
	{
		usage(argv);
		return 1;
	}

	FilterOption filter_opt;
	if (argc == 3) {
		print_filter_opts();
		cout << "\nChoose a filter type and press return" << endl;
		char C;
		cin >> C;
		while (1) {
			try {
				int filter_i = boost::lexical_cast<int>(C);
				filter_opt = static_cast<FilterOption>(filter_i);
				break;
			} catch (boost::bad_lexical_cast& e) {
				cerr << "invalid filter type entered. Enter something else..." << endl;
			}
		}
	} else {
		try {
			int filter_i = boost::lexical_cast<int>(argv[1]);
			filter_opt = static_cast<FilterOption>(filter_i);
		} catch (boost::bad_lexical_cast & e) {
			cerr << "invalid filter type, allowed types:" << endl;
			print_filter_opts();
			return -1;
		}
	}

	setLogger(PM::get().LoggerRegistrar.create("FileLogger"));

	DP in(DP::load(argv[argc-2]));
	DP out;

	PM::DataPointsFilter* f;
	switch(filter_opt) {
	case BOUNDING_BOX: {
		PM::DataPointsFilter* boundBox(
				PM::get().DataPointsFilterRegistrar.create(
						"BoundingBoxDataPointsFilter",
						map_list_of
						("xMin","-1.0")
						("xMax","1.0")
						("yMin","-1.0")
						("yMax","1.0")
						("zMin","-1.0")
						("zMax","1.0")
						)
					);
		out = boundBox->filter(in);
		break;
	}
	case FIX_STEP_SAMPLING: {
		PM::DataPointsFilter* fixStep(
						PM::get().DataPointsFilterRegistrar.create(
								"FixStepSamplingDataPointsFilter",
								map_list_of
								("startStep","10.0")
								("endStep","10.0")
								("stepMult","1.0")
							)
				);
		out = fixStep->filter(in);
		break;
	}
	case MAX_DIST: {
		PM::DataPointsFilter* maxDist(
						PM::get().DataPointsFilterRegistrar.create(
								"MinDistDataPointsFilter",
								map_list_of
								("maxDist", "1.0")
						)
				);
		out = maxDist->filter(in);
		break;
	}
	case MIN_DIST:
	{
		PM::DataPointsFilter* minDist(
				PM::get().DataPointsFilterRegistrar.create(
						"MinDistDataPointsFilter",
						map_list_of
						("minDist", "1.0")
				)
		);
		out = minDist->filter(in);
		break;
	}
	case MAX_POINT_COUNT: {
		PM::DataPointsFilter* maxCount(
				PM::get().DataPointsFilterRegistrar.create(
						"MaxPointCountDataPointsFilter",
						map_list_of
						("maxCount","1000")
						)
				);
		out = maxCount->filter(in);
		break;

	}
	case MAX_QUANTILE_ON_AXIS: {
		PM::DataPointsFilter* maxQuant(
				PM::get().DataPointsFilterRegistrar.create(
						"MaxQuantileOnAxisDataPointsFilter",
						map_list_of
						("ratio","0.5")
						)
					);
		out = maxQuant->filter(in);
		break;
	}
	case OBSERVATION_DIRECTION: {
		PM::DataPointsFilter* obsDir(
				PM::get().DataPointsFilterRegistrar.create(
						"ObservationDirectionDataPointsFilter",
						map_list_of
						("x","0.0")
						("y","0.0")
						("z","0.0")
						)
				);
		out = obsDir->filter(in);
		break;
	}

	case REMOVE_NAN: {
		PM::DataPointsFilter* removeNaN(
						PM::get().DataPointsFilterRegistrar.create(
								"RemoveNaNDataPointsFilter")
						);
		out = removeNaN->filter(in);
		break;
	}
	case RANDOM_SAMPLING:
	{
		PM::DataPointsFilter* randomSample(
				PM::get().DataPointsFilterRegistrar.create(
						"RandomSamplingDataPointsFilter",
						map_list_of
						("prob", toParam(0.65))
				)
		);
		out = randomSample->filter(in);
		break;
	}
	case SURFACE_NORMAL:
	{
		PM::DataPointsFilter* surfaceNormal(
				PM::get().DataPointsFilterRegistrar.create(
						"SurfaceNormalDataPointsFilter",
						map_list_of
						("binSize", "10")
						("epsilon", "5")
						("keepNormals","1")
						("keepDensities","0")
				)
		);
		out = surfaceNormal->filter(in);
		break;
	}
	case ORIENT_NORMALS:
	{
		PM::DataPointsFilter* orientNormals(
				PM::get().DataPointsFilterRegistrar.create(
						"OrientNormalsDataPointsFilter",
						map_list_of
						("towardCenter", "1")
				)
		);
		out = orientNormals->filter(in);
		break;
	}
	case MAX_DENSITY:
	{
		PM::DataPointsFilter* maxDensity(
				PM::get().DataPointsFilterRegistrar.create(
						"MaxDensityDataPointsFilter",
						map_list_of
						("maxDensity", toParam(30))
				)
		);
		out = maxDensity->filter(in);
		break;
	}
	case SHADOW:
	{
		PM::DataPointsFilter* shadowFilter(
				PM::get().DataPointsFilterRegistrar.create(
						"ShadowDataPointsFilter"
				)
		);
		out = shadowFilter->filter(in);
		break;
	}
	default:
		cerr << "Invalid filter type.  Must be 0-6" << endl;
		return -1;
	}


	out.save(argv[argc-1]);

	return 0;
}
