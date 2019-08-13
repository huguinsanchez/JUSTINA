#include "ObjRecognizer.hpp"

ObjRecognizer::ObjRecognizer(int binNo)
{
	this->binNo = binNo; 
	this->TrainingDir = "TrainingDir";

	this->heightErrorThres = 0.01; 
	this->shapeErrorThres = 0.2;
	this->colorErrorThres = 0.6; 

	// Getting params from config file. 
	std::string configDir = ros::package::getPath("obj_reco") + "/ConfigDir";
	if( !boost::filesystem::exists(configDir ) )
		boost::filesystem::create_directory(configDir); 

	std::string configFile = configDir + "/ObjRecognizerConfig.xml"; 
	cv::FileStorage fs; 
	if( fs.open( configFile, fs.READ) ) 
	{
		this->heightErrorThres = (float)fs["heightErrorThres"]; 
		this->shapeErrorThres = (float)fs["shapeErrorThres"]; 
		this->colorErrorThres = (float)fs["colorErrorsVec"];  

		std::cout << "Readed configFile " << configFile << std::endl;  
		std::cout << "	- heightErrorThres: " << this->heightErrorThres << std::endl;
		std::cout << "	- shapeErrorThres: " << this->shapeErrorThres << std::endl;
		std::cout << "	- colorErrorsVec: " << this->colorErrorThres << std::endl;

		fs.release(); 
	}
	else
	{
		if(fs.open( configFile, fs.WRITE ) )
		{
			fs << "heightErrorThres" << this->heightErrorThres; 
			fs << "shapeErrorThres" << this->shapeErrorThres; 
			fs << "colorErrorsVec" << this->colorErrorThres; 

			fs.release(); 
		}
	}
	std::ifstream file;
	std::string knowledgeDir=ros::package::getPath("knowledge")+ "/vision";
	std::string knowledgeFile = knowledgeDir+"/GCM.txt";
	std::string line;
	std::vector<std::string> result;
	if( !boost::filesystem::exists(configDir)){
		boost::filesystem::create_directory(configDir);
		std::cout<<"GMC doesn\'t have base knowledge"<<std::endl;
		this->no_base_knowledge=true; 
	}
	else{
		this->no_base_knowledge=false;
		file.open(knowledgeFile.c_str());
		if (file.is_open()){
			while (!file.eof()){
				getline(file, line);
				boost::split(result, line, boost::is_any_of(" ,\t"), boost::token_compress_on);
				std::cout<<line<<std::endl;
				line=result[0];
				result.erase(result.begin());
				this->object_locs.insert(std::pair<std::string, std::vector<std::string> >(line, result));
			}
			file.close();
			/*std::map<std::string, std::vector<std::string> >::iterator it;
			for(it=this->object_locs.begin();it!=this->object_locs.end(); it++){
				std::cout<<"<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<"<<std::endl;
				std::cout<<"<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<"<<std::endl;
				std::cout<<it->first<<" :"<<std::endl;
				for(size_t i=0; i<it->second.size();i++){
					std::cout<<"\t"<<it->second[i]<<std::endl;

				}
			}*/
		}else{
			this->no_base_knowledge=true;
			std::cout<<"problem openning the file with GCM knoledge base"<<std::endl;
		}
	}
}

ObjRecognizer::ObjRecognizer()
{
	this->binNo = 18; 
	this->TrainingDir = "TrainingDir";

	this->heightErrorThres = 0.01; 
	this->shapeErrorThres = 0.2;
	this->colorErrorThres = 0.6; 

	// Getting params from config filer 
	std::string configDir = ros::package::getPath("obj_reco") + "/ConfigDir";
	if( !boost::filesystem::exists(configDir ) )
		boost::filesystem::create_directory(configDir); 

	std::string configFile = configDir + "/ObjRecognizerConfig.xml"; 
	cv::FileStorage fs; 
	if( fs.open( configFile, fs.READ) ) 
	{
		this->heightErrorThres = (float)fs["heightErrorThres"]; 
		this->shapeErrorThres = (float)fs["shapeErrorThres"]; 
		this->colorErrorThres = (float)fs["colorErrorsVec"];  

		fs.release(); 
	}
	else
	{
		if(fs.open( configFile, fs.WRITE ) )
		{
			fs << "heightErrorThres" << this->heightErrorThres; 
			fs << "shapeErrorThres" << this->shapeErrorThres; 
			fs << "colorErrorsVec" << this->colorErrorThres; 

			std::cout << "Readed " << configFile << std::endl;  

			fs.release(); 
		}
	}

	std::ifstream file;
	std::string knowledgeDir=ros::package::getPath("obj_reco")+ "/vision";
	std::string knowledgeFile = knowledgeDir+"/GCM.txt";
	std::string line;
	std::vector<std::string> result;
	if( !boost::filesystem::exists(configDir)){
		boost::filesystem::create_directory(configDir);
		std::cout<<"GMC doesn\'t have base knowledge"<<std::endl;
		this->no_base_knowledge=true; 
	}
	else{
		this->no_base_knowledge=false;
		file.open(knowledgeFile.c_str());
		if (file.is_open()){
			while (!file.eof()){
				getline(file, line);
				boost::split(result, line, boost::is_any_of(" ,\t"), boost::token_compress_on);
				std::cout<<line<<std::endl;
				line=result[0];
				result.erase(result.begin());
				this->object_locs.insert(std::pair<std::string, std::vector<std::string> >(line, result));
			}
			file.close();
			/*std::map<std::string, std::vector<std::string> >::iterator it;
			for(it=this->object_locs.begin();it!=this->object_locs.end(); it++){
				std::cout<<"<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<"<<std::endl;
				std::cout<<"<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<"<<std::endl;
				std::cout<<it->first<<" :"<<std::endl;
				for(size_t i=0; i<it->second.size();i++){
					std::cout<<"\t"<<it->second[i]<<std::endl;

				}
			}*/
		}else{
			this->no_base_knowledge=true;
			std::cout<<"problem openning the file with GCM knoledge base"<<std::endl;
		}
	}

}
//original function
/*std::string ObjRecognizer::RecognizeObject(DetectedObject detObj, cv::Mat bgrImage)
{
	std::vector<double> heightErrorsVec; 
	std::vector<double> shapeErrorsVec; 
	std::vector<double> colorErrorsVec;

	
	// Getting ERRORS
	cv::Mat detObjHisto = CalculateHistogram( bgrImage, detObj.oriMask ); 
	for( int i=0; i< this->trainingNames.size(); i++)
	{
		// Getting Height Errors 
		float heightError = std::abs( detObj.height - this->trainingHeights[i] ); 
		heightErrorsVec.push_back( heightError ); 
		
		// Getting Shape Errors
		double shapeError = cv::matchShapes( detObj.shadowContour2D, this->trainingCont2D[i], CV_CONTOURS_MATCH_I1, 0.0); 
		shapeErrorsVec.push_back( shapeError ); 

		// Getting Color Errors
		double colorError = cv::compareHist( detObjHisto, this->trainingHistos[i], CV_COMP_INTERSECT);
		colorErrorsVec.push_back( colorError ); 
	}

    // recognizing Object 
	std::string recoName = "";
	double bestColorErrorSoFar = 0.0; 
	for( int i=0; i<this->trainingNames.size(); i++)
	{
		if( heightErrorsVec[i] < this->heightErrorThres && shapeErrorsVec[i] < this->shapeErrorThres && colorErrorsVec[i] > this->colorErrorThres  )
		{
			if( colorErrorsVec[i] > bestColorErrorSoFar )
			{
				bestColorErrorSoFar = colorErrorsVec[i]; 
				recoName = this->trainingNames[i]; 
			}
		}

	}

	return recoName;
}*/

//function modified to calculate an euclidian distance insted of cascade method
std::string ObjRecognizer::RecognizeObject(DetectedObject detObj, cv::Mat bgrImage)
{
	std::cout<<"RecognizeObject"<<std::endl;
	std::vector<double> heightErrorsVec; 
	std::vector<double> shapeErrorsVec; 
	std::vector<double> colorErrorsVec;
	std::vector<double> global_error;

	// Getting ERRORS
	cv::Mat detObjHisto = CalculateHistogram( bgrImage, detObj.oriMask ); 
	for( int i=0; i< this->trainingNames.size(); i++)
	{
		// Getting Height Errors 
		float heightError = std::abs( detObj.height - this->trainingHeights[i] ); 
		heightErrorsVec.push_back( heightError ); 
		
		// Getting Shape Errors
		double shapeError = cv::matchShapes( detObj.shadowContour2D, this->trainingCont2D[i], CV_CONTOURS_MATCH_I1, 0.0); 
		shapeErrorsVec.push_back( shapeError ); 

		// Getting Color Errors
		double colorError = exp(-cv::compareHist( detObjHisto, this->trainingHistos[i], CV_COMP_INTERSECT));
		colorErrorsVec.push_back( colorError ); 
	}

	for (size_t i=0; i <this->trainingNames.size();i++){
		global_error.push_back(sqrt((heightErrorsVec[i]*heightErrorsVec[i])+(shapeErrorsVec[i]*shapeErrorsVec[i])+(colorErrorsVec[i]*colorErrorsVec[i])));
	}

    // recognizing Object 
	std::string recoName = "";
	double minErrorSoFar = 0.0;
	if(this->trainingNames.size()>0) minErrorSoFar=global_error[0];
	
	for(size_t i=0;i<trainingNames.size();i++){
		if(minErrorSoFar>global_error[i]){
			minErrorSoFar=global_error[i];
			recoName=this->trainingNames[i];
			/*std::cout<<"heightError: "<<colorErrorsVec[i]<<std::endl;
			std::cout<<"shapeError: "<<shapeErrorsVec[i]<<std::endl;
			std::cout<<"colorError: "<<colorErrorsVec[i]<<std::endl;
			std::cout<<"minErrorSoFar: "<<minErrorSoFar<<std::endl;*/
		}
	}

	return recoName;
}

std::string ObjRecognizer::RecognizeObjectGCM(DetectedObject detObj, cv::Mat bgrImage, std::string location, std::map<std::string,double> locs)
{
	std::cout<<"RecognizeObjectGCM"<<std::endl;
	std::cout<<location<<std::endl;

	std::vector<double> heightErrorsVec; 
	std::vector<double> shapeErrorsVec; 
	std::vector<double> colorErrorsVec;
	std::vector<double> global_error;

	std::map<std::string, double >::iterator it_locs;
	std::map<std::string, std::vector<std::string> >::iterator it_object_loc;
	std::vector<std::string>::iterator it_strings;
	/*std::cout<<"<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<"<<std::endl;
	std::cout<<"<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<"<<std::endl;
	for(it_locs=locs.begin();it_locs!=locs.end(); ++it_locs){
		std::cout<<it_locs->first<<" :"<<std::endl;
		std::cout<<"\t"<<it_locs->second<<std::endl;
	}
	std::cout<<"<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<"<<std::endl;
	std::cout<<"<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<"<<std::endl;*/

	// Getting ERRORS diferences
	cv::Mat detObjHisto = CalculateHistogram( bgrImage, detObj.oriMask ); 
	for( int i=0; i< this->trainingNames.size(); i++)
	{
		// Getting Height Errors 
		float heightError = std::abs( detObj.height - this->trainingHeights[i] ); 
		heightErrorsVec.push_back( heightError ); 
		
		// Getting Shape Errors
		double shapeError = cv::matchShapes( detObj.shadowContour2D, this->trainingCont2D[i], CV_CONTOURS_MATCH_I1, 0.0); 
		shapeErrorsVec.push_back( shapeError ); 

		// Getting Color Errors
		double colorError = exp(-cv::compareHist( detObjHisto, this->trainingHistos[i], CV_COMP_INTERSECT));
		colorErrorsVec.push_back( colorError ); 
	}


	//euclidian distance
	for (size_t i=0; i <this->trainingNames.size();i++){
		
		global_error.push_back(sqrt((heightErrorsVec[i]*heightErrorsVec[i])+(shapeErrorsVec[i]*shapeErrorsVec[i])+(colorErrorsVec[i]*colorErrorsVec[i])));
	}
	
	//calculating similities
	/*std::cout<<"before similities"<<std::endl;
	for(size_t i=0;i<global_error.size();i++){

		std::cout<<global_error[i]<<std::endl;
	}*/

	for(size_t i=0;i<global_error.size();i++){

		global_error[i]=exp(-.05*global_error[i]);
	}
	//debbugin function
	/*std::cout<<"after similities"<<std::endl;
	for(size_t i=0;i<global_error.size();i++){

		std::cout<<global_error[i]<<std::endl;
	}*/

	//applying GCM a=m*s

	/*for (size_t i=0; i <this->trainingNames.size();i++){
		
		if(location=="kitchen"){
			if(this->trainingNames[i]=="book"||this->trainingNames[i]=="green_mug"||this->trainingNames[i]=="cutting_board"){
				//std::cout<<"m = kitchen"<<std::endl;
				global_error[i]*=1.0/5;
			}
		}
		else if(location=="office"){
			if(this->trainingNames[i]=="green_pen_holder"||this->trainingNames[i]=="organ_attack"||this->trainingNames[i]=="white_board"){
				//std::cout<<"m = office"<<std::endl;
				global_error[i]*=1.0/5;
			}
		}
	}*/
	//real function
	for (size_t i=0; i <this->trainingNames.size();++i){
		std::cout<<"valur of iterator : "<<i<<" , "<<trainingNames[i]<<std::endl;
		it_object_loc=this->object_locs.find(trainingNames[i]); //looking for object locations knowledge
		if(it_object_loc!=object_locs.end()){
			std::cout<<"I found knowledge about  "<<trainingNames[i]<<std::endl;
			it_strings=find(it_object_loc->second.begin(),it_object_loc->second.end(), location);//looking if current location is in the list
			if(it_strings!=it_object_loc->second.end()){//if its in the list
				std::cout<<"this object should be in this room: "<<location<<std::endl;
				it_locs=locs.find(location);//getting prob
				if(it_locs!=locs.end()){
					std::cout<<"apply this m: "<<it_locs->second<<std::endl;
					global_error[i]*=it_locs->second;//applying prob
				}else{
					std::cout<<"It shouldn't be in here or there is no probability for my location"<<std::endl;
				}
			}else{//current location is not in the list of this object
				std::cout<<"this object SHOUDEN'T be in "<<location<<std::endl;
				it_locs=locs.find(it_object_loc->second[0]);
				if(it_locs!=locs.end()){
					global_error[i]*=it_locs->second;//applying prob of one of their room
					std::cout<<"M APPLY OF "<<it_locs->first<<" :"<<it_locs->second<<std::endl;
				}else{
					global_error[i]*=.5;
					std::cout<<"m applied of 0.5"<<std::endl;
				}
			}

		}
		else{
			std::cout<<"I DIDN'T found knowledge about "<<trainingNames[i]<<"M applied 0.5"<<std::endl;
			global_error[i]*=.5;
		}
	}

    // recognizing Object 
	std::string recoName = "";
	double minErrorSoFar = 0.0;
	if(this->trainingNames.size()>0) minErrorSoFar=global_error[0];
	
	for(size_t i=0;i<trainingNames.size();i++){
		if(minErrorSoFar<global_error[i]){
			minErrorSoFar=global_error[i];
			recoName=this->trainingNames[i];
			/*std::cout<<"heightError: "<<colorErrorsVec[i]<<std::endl;
			std::cout<<"shapeError: "<<shapeErrorsVec[i]<<std::endl;
			std::cout<<"colorError: "<<colorErrorsVec[i]<<std::endl;
			std::cout<<"minErrorSoFar: "<<minErrorSoFar<<std::endl;*/
		}
	}
	return recoName;
}

bool ObjRecognizer::LoadTrainingDir(std::string trainingFolder)
{
    try
    {
        std::cout << "\nObjRecognizer.LoadTrainingDir ->Trying to load training dir: " << trainingFolder << std::endl;

        if( !boost::filesystem::exists(trainingFolder) )
        {
            std::cout << "\nObjRecognizer.LoadTrainingDir-> Training dir doesnt exist." << trainingFolder << std::endl;
            return false; 
        } 
		else
			std::cout << "\nObjRecognizer.LoadTrainingDir-> Loading successfully :) path: " << trainingFolder << std::endl;       

        cv::FileStorage fs; 
        std::string nodeName = "obj"; 

        std::vector< std::string > trainingNames; 
        std::vector< int > trainingIds; 
        std::vector< float > trainingHeights; 
        std::vector< cv::Mat > trainingHistos; 
        std::vector< std::vector< cv::Point2f > > trainingCont2D; 

        boost::filesystem::path pathTrainDir( trainingFolder ); 
        boost::filesystem::directory_iterator endIt; 
        for( boost::filesystem::directory_iterator dirIt( pathTrainDir ) ; dirIt != endIt ; ++dirIt )
        {
            if( boost::filesystem::is_directory( dirIt->status() ) )
            {
                boost::filesystem::path p = dirIt->path(); 
                std::string trainingFilePath  = p.string() +"/" + p.filename().string() + ".xml"; 
                std::string objName = p.filename().string(); 

                // Loading for create new 
                int idCnt = 0;
                if( fs.open( trainingFilePath, fs.READ) ) 
                {				
                    cv::FileNode contoursNode = fs[ nodeName ]; 
                    cv::FileNodeIterator it = contoursNode.begin(); 
                    cv::FileNodeIterator it_end = contoursNode.end(); 

                    for( ; it != it_end ; ++it )
                    {
                        int oId = (int)(*it)["id"]; 

                        float oHeight = (float)(*it)["height"]; 

                        std::vector < cv::Point2f > oCont; 
                        (*it)["contour2d"] >> oCont; 

                        cv::Mat oHist; 
                        (*it)["histogram"] >> oHist; 

                        trainingNames.push_back( objName ); 
                        trainingIds.push_back( oId ); 
                        trainingHeights.push_back( oHeight  ); 
                        trainingHistos.push_back( oHist ); 
                        trainingCont2D.push_back( oCont ) ; 
                    }	
                    fs.release(); 
                }
            }

        }
        this->trainingNames   =   trainingNames   ;   
        this->trainingIds     =   trainingIds     ;
        this->trainingHeights =   trainingHeights ;
        this->trainingHistos  =   trainingHistos  ;
        this->trainingCont2D  =   trainingCont2D  ;

        for(int i=0; i< trainingNames.size(); i++)
        {
            std::cout << "ObjReco Trained: [" << i << "] "  << this->trainingNames[i] << " Hei:" <<  this->trainingHeights[i] << std::endl; 
        }
    }

    catch(std::exception& e)
    {
       std::cout << "Exception at LoadTrainingDir: " << e.what() << std::endl; 
        return false; 
    }
}

bool ObjRecognizer::TrainObject(DetectedObject detObj, cv::Mat bgrImage, std::string name)
{
	try
	{
        std::cout << "\nTrainingDir:" <<  this->TrainingDir << std::endl;   
		std::string trainingDirPath = this->TrainingDir;

        // Checking if directory of training exists.
		if( !boost::filesystem::exists(trainingDirPath) )
			boost::filesystem::create_directory(trainingDirPath); 

		std::string objDirPath = trainingDirPath + std::string("/") +  name;  
		// Checking if directory of object exist
		if( !boost::filesystem::exists(objDirPath) )
			boost::filesystem::create_directory(objDirPath); 

		std::string objFilePath = objDirPath + std::string("/") + name + std::string(".xml"); 

		std::vector< int > objIdVec; 
		std::vector< float > objHeightVec; 
		std::vector< std::vector< cv::Point2f > > objCont2DVec; 
		std::vector< cv::Mat > objHistoVec; 

		cv::FileStorage fs; 
		std::string nodeName = "obj"; 

		// Loading for create new 
		int idCnt = 0; 
		if( fs.open( objFilePath, fs.READ) ) 
		{
			cv::FileNode contoursNode = fs[ nodeName ]; 
			cv::FileNodeIterator it = contoursNode.begin(); 
			cv::FileNodeIterator it_end = contoursNode.end(); 

			for( ; it != it_end ; ++it )
			{
				int oId = (int)(*it)["id"]; 

				float oHeight = (float)(*it)["height"]; 

				std::vector < cv::Point2f > oCont; 
				(*it)["contour2d"] >> oCont; 

				cv::Mat oHist; 
				(*it)["histogram"] >> oHist; 

				objIdVec.push_back( oId ); 
				objHeightVec.push_back( oHeight ); 
				objCont2DVec.push_back( oCont ); 
				objHistoVec.push_back( oHist ); 
		
				if( idCnt <= oId )
					idCnt = oId; 
			}
			fs.release(); 
		}

		idCnt ++ ; 

		objIdVec.push_back( idCnt ); 
		objHeightVec.push_back( detObj.height ); 
		objCont2DVec.push_back( detObj.shadowContour2D ); 
		objHistoVec.push_back( this->CalculateHistogram( bgrImage, detObj.oriMask) ); 

		if( fs.open( objFilePath, fs.WRITE) )
		{
			fs << nodeName << "["; 
			for( int i=0; i< objIdVec.size(); i++)
			{
				fs << "{:"; 
				fs << "id" << objIdVec[i]; 
				fs << "height" << objHeightVec[i]; 
				fs << "contour2d" << objCont2DVec[i]; 
				fs << "histogram" << objHistoVec[i]; 
				fs << "}"; 	
			}
			fs << "]"; 
			fs.release(); 

			// Saving image to file; 
			cv::Mat masked; 
			bgrImage.copyTo( masked, detObj.oriMask );
			cv::Mat objIma = masked( detObj.boundBox );
			std::stringstream ss; 	
			ss << objDirPath << std::string("/") <<  name << "_" << idCnt <<".jpg"; 
			cv::imwrite(ss.str(), objIma); 

			std::cout << "Trained obj [" << objFilePath << "]" << std::endl; 
		}
		else
		{
			std::cout << "Cant write trining file: " << objFilePath << std::endl; 
			return false; 
		}
	}
	catch(std::exception& e) 
    {
		std::cout << "Exception at TrainObject: " << e.what() << std::endl; 
		return false; 
	}

	return true; 
}

cv::Mat ObjRecognizer::CalculateHistogram( cv::Mat bgrImage, cv::Mat mask )
{
	cv::Mat hsvImage; 
	cv::cvtColor( bgrImage, hsvImage, CV_BGR2HSV_FULL ); 

	cv::Mat histogram;
	int chan[] = { 0 }; 
	int histSize[] = { this->binNo };
	float hueRange[] = { 0, 255 };
	const float* ranges[] = { hueRange };
	
	//cv::Mat bgrMasked; 
	//bgrImage.copyTo( bgrMasked, mask ); 
	//cv::imshow("bgrMasked", bgrMasked); 
	
	//Hues
	cv::Mat satValMask; 
	cv::inRange( hsvImage, cv::Scalar( 0, 50, 50), cv::Scalar(255, 255, 205), satValMask);
	cv::Mat maskAnd = mask & satValMask; 

	// Blacks
	cv::Mat satM; 
	cv::inRange( hsvImage, cv::Scalar( 0, 0, 0), cv::Scalar(255, 50, 50), satM);
	cv::Mat maskAnd_1 = mask & satM; 
	int blackPx = cv::countNonZero( maskAnd_1 ); 

	// Whites
	cv::Mat valM; 
	cv::inRange( hsvImage, cv::Scalar( 0, 0, 205), cv::Scalar(255, 50, 255), valM);
	cv::Mat maskAnd_2 = mask & valM; 
	int whitePx = cv::countNonZero( maskAnd_2 ); 

	/*cv::imshow( "mask", mask ); 
	cv::imshow( "satValMask", satValMask); 
	cv::imshow( "bitAnd", maskAnd);

	cv::imshow( "maskAnd_1", maskAnd_1);
	cv::imshow( "maskAnd_2", maskAnd_2);
	cv::waitKey(30); */

	calcHist(&hsvImage, 1, chan, maskAnd, histogram, 1, histSize, ranges, true, false);
	//std::cout << "histogram" << histogram << std::endl; 

	cv::Mat newHisto = cv::Mat( histogram.rows+2, histogram.cols, histogram.type() ); 
	for( int i=0; i< histogram.rows; i++)
		newHisto.at<float>(i,0) = histogram.at<float>( i, 0); 
	
	newHisto.at<float>( histogram.rows , 0 ) = blackPx; 
	newHisto.at<float>( histogram.rows+1 , 0 ) = whitePx; 
	/*std::cout << "newHisto" << newHisto <<std::endl; */

	cv::normalize(newHisto, newHisto, 1.0, 0.0, cv::NORM_L1); 
	/*std::cout << "newHistoNorm" << newHisto <<std::endl; */

	//cv::normalize(histogram, histogram,  0, 1, CV_MINMAX); 
	return newHisto; 
}

