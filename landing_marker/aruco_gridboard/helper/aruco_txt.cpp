
        ROS_INFO_STREAM("folder");
        std::string folderPath = "/home/analys/Documents/full";
        std::string outputFolderPath = "/home/analys/Documents/Grid2";
        std::ofstream outputFile("/home/analys/Documents/grid.txt", std::ios::app); 
        for (const auto& entry : fs::directory_iterator(folderPath))
        {
            if (fs::is_regular_file(entry) && entry.path().extension() == ".jpg")
            {   
                // ROS_INFO_STREAM(entry.path().string());
                cv::Mat imageCopy = cv::imread(entry.path().string());

                if (imageCopy.empty())
                {
                    std::cerr << "Failed to read image: " << entry.path().string() << std::endl;
                    continue;
                }
                std::string imageName = entry.path().filename().string();
                std::string outputPath = outputFolderPath + "/" + imageName; // Construct the output path
                cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();
                bool readOk = readDetectorParameters(detector_param_path_, detectorParams);
                if(!readOk) {
                    std::cerr << "Invalid detector parameters file" << std::endl;
                    return ;
                }
                detectorParams->cornerRefinementMethod = 1; // do corner refinement in markers
                // Now detect the markers
                std::vector< int > ids;
                std::vector< std::vector< cv::Point2f > > corners, rejected;
                cv::aruco::detectMarkers(imageCopy, board->dictionary, corners, ids, detectorParams, rejected);

                // Now estimate the pose of the board
                int markersOfBoardDetected = 0;
                if(ids.size() > 1)
                    markersOfBoardDetected = cv::aruco::estimatePoseBoard(corners, ids, board, camMatrix_, distCoeffs_, rvec, tvec);
                if (markersOfBoardDetected && cv::norm(tvec) > 0.00001)
                {
                    // Publish pose
                    cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
                    cv::Mat rmat;
                    cv::Rodrigues(rvec, rmat);
                    cv::Point center(imageCopy.cols / 2, imageCopy.rows / 2);  // Calculate the center of the image

                    int crossSize = 50;  // Size of the cross

                    // Draw the vertical line of the cross
                    cv::line(imageCopy, cv::Point(center.x, center.y - crossSize), cv::Point(center.x, center.y + crossSize), cv::Scalar(0, 255, 0), 2);

                    // Draw the horizontal line of the cross
                    cv::line(imageCopy, cv::Point(center.x - crossSize, center.y), cv::Point(center.x + crossSize, center.y), cv::Scalar(0, 255, 0), 2);

                    // Invert the rotation matrix and translation vector
                    cv::Mat rmatInverse = rmat.t();
                    cv::Mat tvecInverse = -rmatInverse * tvec;
                    cv::Point2f textPosition(imageCopy.cols - 500, imageCopy.rows - 20);  // Position to write the text
                    std::stringstream ss;
                    ss << "CP: (" << tvecInverse.at<double>(0) << ", " << tvecInverse.at<double>(1) << ", " << tvecInverse.at<double>(2) << ")";
                    cv::putText(imageCopy, ss.str(), textPosition, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
                    cv::aruco::drawAxis(imageCopy, camMatrix_, distCoeffs_, rvec, tvec, 4.0);
                    cv::imwrite(outputPath, imageCopy);
                    if (outputFile.is_open())
                    {
                        // Write the image name and tvecInverse to the file as a new line
                        ROS_INFO_STREAM(imageName << " " << tvecInverse.at<double>(0) << " " << tvecInverse.at<double>(1) << " " << tvecInverse.at<double>(2) << "\n");
                        outputFile << imageName << " " << tvecInverse.at<double>(0) << " " << tvecInverse.at<double>(1) << " " << tvecInverse.at<double>(2)<< " " << rvec[0] << " " << rvec[1]<< " " << rvec[2] << "\n";
                    }
                }
                cont++;
               }
            }
            // outputFile.close();
