/*
 * Output.h
 *
 *  Created on: 29 Mar 2016
 *      Author: Enrico Piazza
 */

#ifndef EXAMPLES_MONOCULAR_OUTPUT_H_
#define EXAMPLES_MONOCULAR_OUTPUT_H_

#include "KeyFrame.h"
#include "Map.h"
#include <string>
#include <vector>

namespace ORB_SLAM2{

class Output {
public:
	Output(std::vector<KeyFrame*> KFs);
	virtual ~Output();
	std::string getJSON();
	std::string getYamlSfM();
	std::vector<KeyFrame*> vpKFs;

};

#endif /* EXAMPLES_MONOCULAR_OUTPUT_H_ */

}
