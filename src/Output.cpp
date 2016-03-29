/*
 * Output.cpp
 *
 *  Created on: 29 Mar 2016
 *      Author: Enrico Piazza
 */

#include <Output.h>
#include <Thirdparty/rapidjson/document.h>
#include <Thirdparty/rapidjson/prettywriter.h>
#include <Thirdparty/rapidjson/rapidjson.h>
#include <Thirdparty/rapidjson/stringbuffer.h>

namespace ORB_SLAM2{

Output::Output(std::vector<KeyFrame*> KFs) {
	vpKFs = KFs;
}

Output::~Output() {
}


std::string Output::getJSON(){
	sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

	rapidjson::Document jsonDoc;
	jsonDoc.SetObject();
	rapidjson::Document::AllocatorType& allocator = jsonDoc.GetAllocator();

	// create view objects
	rapidjson::Value viewsArray(rapidjson::kArrayType);
	for(size_t i=0; i<vpKFs.size(); i++){
		KeyFrame* pKF = vpKFs[i]; //TODO right?

		if(pKF->isBad()){
			std::cout << "bad KF not inserted" << std::endl;
			continue;
		}

		rapidjson::Value viewObject(rapidjson::kObjectType);

		// create value/ptr_wrapper/data
		rapidjson::Value valueObject(rapidjson::kObjectType);
		rapidjson::Value ptr_wrapperObject(rapidjson::kObjectType);
		rapidjson::Value dataObject(rapidjson::kObjectType);

		viewObject.AddMember("key", pKF->mnId, allocator);
		//TODO? ptr_wrapper/id

		viewObject.AddMember("num_points", pKF->GetMapPoints().size(), allocator);

		dataObject.AddMember("local_path", "/", allocator);
		dataObject.AddMember("filename", "TODO", allocator);
		dataObject.AddMember("width", 640, allocator);//TODO
		dataObject.AddMember("height", 480, allocator);//TODO
		dataObject.AddMember("id_view", pKF->mnId, allocator);
		dataObject.AddMember("id_intrinsic", 0, allocator);
		dataObject.AddMember("id_pose", pKF->mnId, allocator);


		// insert data in ptr_wrapper in value in view
		ptr_wrapperObject.AddMember("data", dataObject, allocator);
		valueObject.AddMember("ptr_wrapper", ptr_wrapperObject, allocator);
		viewObject.AddMember("value", valueObject, allocator);


		// insert view in array
		viewsArray.PushBack(viewObject, allocator);
	}

	// create intrinsics
	rapidjson::Value intrinsicsArray(rapidjson::kArrayType);
	rapidjson::Value intrinsicObject(rapidjson::kObjectType);

	intrinsicObject.AddMember("key", 0, allocator);

	rapidjson::Value valueObject(rapidjson::kObjectType);
	valueObject.AddMember("polymorphic_id", 0, allocator);
	valueObject.AddMember("polymorphic_name", "pinhole", allocator);

	rapidjson::Value ptr_wrapperObject(rapidjson::kObjectType);
	//TODO? ptr_wrapper/id

	rapidjson::Value dataObject(rapidjson::kObjectType);
	dataObject.AddMember("width", 640, allocator);
	dataObject.AddMember("height", 480, allocator);
	dataObject.AddMember("focal_length", 0.0, allocator);

	rapidjson::Value principal_pointArray(rapidjson::kArrayType);
	principal_pointArray.PushBack(1.0f, allocator).PushBack(2.0f, allocator);

	dataObject.AddMember("principal_point", principal_pointArray, allocator);

	ptr_wrapperObject.AddMember("data", dataObject, allocator);
	valueObject.AddMember("ptr_wrapper", ptr_wrapperObject, allocator);

	intrinsicObject.AddMember("value", valueObject, allocator);
	intrinsicsArray.PushBack(intrinsicObject, allocator);

	// create extrinsic objects
	rapidjson::Value extrinsicsArray(rapidjson::kArrayType);
	for(size_t i=0; i<vpKFs.size(); i++){
		KeyFrame* pKF = vpKFs[i]; //TODO right?

		if(pKF->isBad()){
			std::cout << "bad KF not inserted" << std::endl;
			continue;
		}
		rapidjson::Value extrinsicObject(rapidjson::kObjectType);

		extrinsicObject.AddMember("key", pKF->mnId, allocator);

		// create value/ptr_wrapper/data
		rapidjson::Value valueObject(rapidjson::kObjectType);
		rapidjson::Value rotationRowsArray(rapidjson::kArrayType);
		rapidjson::Value rotationRow1Array(rapidjson::kArrayType);
		rapidjson::Value rotationRow2Array(rapidjson::kArrayType);
		rapidjson::Value rotationRow3Array(rapidjson::kArrayType);
		rapidjson::Value centerArray(rapidjson::kArrayType);

		cv::Mat R = pKF->GetRotation().t(); //TODO check type. float?
		rotationRow1Array.PushBack(R.at<float>(0,0), allocator).PushBack(R.at<float>(0,1), allocator).PushBack(R.at<float>(0,2), allocator);
		rotationRow2Array.PushBack(R.at<float>(1,0), allocator).PushBack(R.at<float>(1,1), allocator).PushBack(R.at<float>(1,2), allocator);
		rotationRow3Array.PushBack(R.at<float>(2,0), allocator).PushBack(R.at<float>(2,1), allocator).PushBack(R.at<float>(2,2), allocator);

		rotationRowsArray.PushBack(rotationRow1Array, allocator);
		rotationRowsArray.PushBack(rotationRow2Array, allocator);
		rotationRowsArray.PushBack(rotationRow3Array, allocator);

		cv::Mat t = pKF->GetCameraCenter();
		centerArray.PushBack(t.at<float>(0), allocator).PushBack(t.at<float>(1), allocator).PushBack(t.at<float>(2), allocator);

		// insert data in ptr_wrapper in value in view
		valueObject.AddMember("rotation", rotationRowsArray, allocator);
		valueObject.AddMember("center", centerArray, allocator);
		extrinsicObject.AddMember("value", valueObject, allocator);


		// insert view in array
		extrinsicsArray.PushBack(extrinsicObject, allocator);
	}

	// create structure objects
	rapidjson::Value structureArray(rapidjson::kArrayType);

	// for each KF, add the observed MPs to the MPs set.
	std::set<MapPoint*> vpAllMPs;

	for(auto pKF : vpKFs){
		if(pKF->isBad()) continue;

		std::set<MapPoint*> vpMPs = pKF->GetMapPoints();
		for(auto pMP : vpMPs){
			vpAllMPs.insert(pMP);
		}
	}

	for(auto pMP : vpAllMPs){

		rapidjson::Value structureObject(rapidjson::kObjectType);
		rapidjson::Value valueObject(rapidjson::kObjectType);
		rapidjson::Value XArray(rapidjson::kArrayType);

		structureObject.AddMember("key", pMP->mnId, allocator);

		cv::Mat p = pMP->GetWorldPos();
		XArray.PushBack(p.at<float>(0), allocator).PushBack(p.at<float>(0), allocator).PushBack(p.at<float>(0), allocator);

		structureObject.AddMember("num_obs", pMP->Observations(), allocator);
		rapidjson::Value observationsArray(rapidjson::kArrayType);
		for(auto obsKF: pMP->GetObservations()){
			rapidjson::Value observationObject(rapidjson::kObjectType);
			rapidjson::Value obsValueObject(rapidjson::kObjectType);
			rapidjson::Value xArray(rapidjson::kArrayType);

			observationObject.AddMember("key", obsKF.first->mnId, allocator);

			//TODO? id_feat

			xArray.PushBack(1.0f, allocator).PushBack(2.0f, allocator);

			obsValueObject.AddMember("x", xArray, allocator);
			observationObject.AddMember("value", obsValueObject, allocator);

			observationsArray.PushBack(observationObject, allocator);
		}

		// insert data in ptr_wrapper in value in view
		valueObject.AddMember("X", XArray, allocator);
		valueObject.AddMember("observations", observationsArray, allocator);
		structureObject.AddMember("value", valueObject, allocator);


		// insert view in array
		structureArray.PushBack(structureObject, allocator);
	}

	rapidjson::Value control_pointsArray(rapidjson::kArrayType);

	jsonDoc.AddMember("sfm_data_version", "0.3", allocator);
	jsonDoc.AddMember("root_path", "/home/andrea/Scrivania/Datasets/Middelbury/dinoRing", allocator);
	jsonDoc.AddMember("views", viewsArray, allocator);
	jsonDoc.AddMember("intrinsics", intrinsicsArray, allocator);
	jsonDoc.AddMember("extrinsics", extrinsicsArray, allocator);
	jsonDoc.AddMember("structure", structureArray, allocator);
	jsonDoc.AddMember("control_points", control_pointsArray, allocator);

	rapidjson::StringBuffer pretty;
	rapidjson::PrettyWriter<rapidjson::StringBuffer> prettyWriter(pretty);
	jsonDoc.Accept(prettyWriter);

	return pretty.GetString();
}

}
