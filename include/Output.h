/*
 * Output.h
 *
 *  Created on: 29 Mar 2016
 *      Author: Enrico Piazza
 */

#include <Thirdparty/rapidjson/document.h>

#ifndef EXAMPLES_MONOCULAR_OUTPUT_H_
#define EXAMPLES_MONOCULAR_OUTPUT_H_

#include "KeyFrame.h"
#include "Map.h"
#include <string>
#include <vector>

namespace ORB_SLAM2{

class Output {
public:
	Output();
//	Output(std::vector<KeyFrame*> KFs);
	virtual ~Output();
	std::string getJSON(std::vector<KeyFrame*> KFs);

	void add(KeyFrame* pKF);
	std::string getIncJSON();

	set<long unsigned int> sKFIds_;

	std::vector<KeyFrame*> vpKFs;

	rapidjson::Document jsonDoc_;
	//rapidjson::Document::AllocatorType& allocator_;
	rapidjson::Value viewsArray_;

};

#endif /* EXAMPLES_MONOCULAR_OUTPUT_H_ */

}
