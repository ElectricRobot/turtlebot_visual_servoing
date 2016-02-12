
/* Computes mean displacement of the kp of a matche */
void computeDisplacement(const vector<DMatch>& matches,const vector<KeyPoint>& kp1,
                                         const vector<KeyPoint>& kp2,Mat& disp,Mat& mean, Mat& dev ){

    // Convert keypoints into Point2f and select good kp
    float x1,x2,y1,y2, dx, dy;
    Mat row;
    for (vector<DMatch>::const_iterator it= matches.begin();it!= matches.end(); ++it) {
        // Get the position of query keypoints
        x1 = kp1[it->queryIdx].pt.x;
        y1 = kp1[it->queryIdx].pt.y;
        // Get the position of train keypoints
        x2= kp2[it->trainIdx].pt.x;
        y2= kp2[it->trainIdx].pt.y;
        // Displacement
        dx = x2 - x1;
        dy = y2 - y1;
        row = (Mat_<float>(1,2)<<dx,dy);
        disp.push_back(row);
    }
    // Compute mean and Std deviation of the displacements (in both axis X and Y)
    mean = Mat(1,2,CV_64FC1);
    dev = Mat(1,2,CV_64FC1);
    meanStdDev(disp.col(0),mean.col(0),dev.col(0));
    meanStdDev(disp.col(1),mean.col(1),dev.col(1));
}

int ratioTest(vector< vector< DMatch> > & matches, double ratio)
{
    int removed=0;
    // for all matches
    for (vector< vector <DMatch> >::iterator matchIterator= matches.begin();matchIterator!= matches.end();++matchIterator){
        // if 2 NN has been identified
        if (matchIterator->size() > 1){
            // check distance ratio
            if ((*matchIterator)[0].distance/
                    (*matchIterator)[1].distance > ratio) {
                matchIterator->clear(); // remove match
                removed++;
            }
        } else { // does not have 2 neighbours
            matchIterator->clear(); // remove match
            removed++;
        }
    }
    return removed;
}

void symmetryTest(const vector<vector<DMatch> >& matches1,
                                  const vector<vector<DMatch> >& matches2,vector<DMatch>& symMatches) {

    // for all matches image 1 -> image 2
    for (vector<vector<DMatch> >::const_iterator matchIterator1= matches1.begin();
         matchIterator1!= matches1.end(); ++matchIterator1) {
        // ignore deleted matches
        if (matchIterator1->size() < 2)continue;

        // for all matches image 2 -> image 1
        for (vector<vector<DMatch> >::const_iterator matchIterator2= matches2.begin();
             matchIterator2!= matches2.end();++matchIterator2) {
            // ignore deleted matches
            if (matchIterator2->size() < 2)continue;
            // Match symmetry test
            if ((*matchIterator1)[0].queryIdx == (*matchIterator2)[0].trainIdx &&
                    (*matchIterator2)[0].queryIdx == (*matchIterator1)[0].trainIdx) {
                // add symmetrical match
                symMatches.push_back(DMatch((*matchIterator1)[0].queryIdx,(*matchIterator1)[0].trainIdx,
                                       (*matchIterator1)[0].distance));
                break; // next match in image 1 -> image 2
            }
        }
    }
}

void ransacTest(const vector<DMatch>& matches,const vector<KeyPoint>& kp1,
                               const vector<KeyPoint>& kp2,vector<DMatch>& outMatches) {

    // Convert keypoints into Point2f and select good kp
    vector<Point2f> points1, points2;
    float x1,x2,y1,y2;
    for (vector<DMatch>::const_iterator it= matches.begin();it!= matches.end(); ++it) {
        // Get the position of query keypoints
        x1 = kp1[it->queryIdx].pt.x;
        y1 = kp1[it->queryIdx].pt.y;
        points1.push_back(Point2f(x1,y1));
        // Get the position of train keypoints
        x2= kp2[it->trainIdx].pt.x;
        y2= kp2[it->trainIdx].pt.y;
        points2.push_back(Point2f(x2,y2));
    }

    // Compute F matrix using RANSAC
    vector<uchar> inliers(points1.size(),0);
    double confidence = 0.99;
    double distance = 5.0;
    Mat fundemental= findFundamentalMat(Mat(points1),Mat(points2),inliers,CV_FM_RANSAC,
                                        distance,confidence);

    // extract the surviving (inliers) matches
    vector<uchar>::const_iterator itIn= inliers.begin();
    vector<DMatch>::const_iterator itM= matches.begin();
    // for all matches
    for ( ;itIn!= inliers.end(); ++itIn, ++itM) {
        if (*itIn) outMatches.push_back(*itM);}

    bool refineF = false;
    if (refineF) {
        // The F matrix will be recomputed with all accepted matches
        points1.clear();
        points2.clear();
        for (vector<DMatch>::const_iterator it= outMatches.begin(); it!= outMatches.end(); ++it){
            // Get the position of query keypoints
            float x= kp1[it->queryIdx].pt.x;
            float y= kp1[it->queryIdx].pt.y;
            points1.push_back(Point2f(x,y));
            // Get the position of train keypoints
            x= kp2[it->trainIdx].pt.x;
            y= kp2[it->trainIdx].pt.y;
            points2.push_back(Point2f(x,y));
        }
        // Compute 8-point F from all accepted matches
        fundemental= findFundamentalMat(Mat(points1),Mat(points2),CV_FM_8POINT); // 8-point method
    }
}

void devTest(const vector<DMatch>& ransacMatches,const vector<KeyPoint>& kp1,
                             const vector<KeyPoint>& kp2,vector<DMatch>& finalMatches){

    // Compute displacements, mean and Std deviation
    Mat disp, mean, dev;
    computeDisplacement(ransacMatches,kp1,kp2,disp,mean,dev);

    // extract the surviving (inliers) matches
    vector<DMatch>::const_iterator itR= ransacMatches.begin();
    int i = 0;
    // for all matches if displacement is inside 2 Stddev from mean...
    for (; itR!= ransacMatches.end(); ++itR) {
        if ( disp.at<float>(i,0) < (mean.at<double>(0,0) + dev.at<double>(0,0)) &&
             disp.at<float>(i,0) > (mean.at<double>(0,0) - dev.at<double>(0,0)) &&
             disp.at<float>(i,1) < (mean.at<double>(0,1) + dev.at<double>(0,1)) &&
             disp.at<float>(i,1) > (mean.at<double>(0,1) - dev.at<double>(0,1)) ){
            // ... compute as a valide matche!
            finalMatches.push_back((*itR));
        }
        i++;
    }
}

void features2d(Mat img1, Mat img2)
{
    /* features2d calculates the key-points and descriptors for two images and matches them*/
    cv::Mat des1, des2, img1_cp, img2_cp, mask1, mask2;
    std::vector<cv::DMatch> symMatches, ransacMatches;

    //-- Step 0: Pre-processing
    img1_cp = img1.clone();
    img2_cp = img2.clone();
    cv::equalizeHist(img1_cp,img1_cp);
    cv::equalizeHist(img2_cp,img2_cp);
    //mask for kp detection
    cv::blur(img1,mask1,Size(101,101));
    cv::threshold(mask1,mask1,1,255,THRESH_BINARY);
    cv::blur(img2,mask2,Size(101,101));
    cv::threshold(mask2,mask2,1,255,THRESH_BINARY);

    //qDebug()<<"equalizeHist";

    /// USING SURF DETECTOR AND EXTRACTOR
    //-- Step 1: Detect the keypoints using SURF Detector
    cv::SurfFeatureDetector surfDetector = SURF(300);
    cv::surfDetector.detect(img1_cp, kp1,mask1);
    cv::surfDetector.detect(img2_cp, kp2,mask2);

    //qDebug()<<"detect";

    //-- Step 2: Calculate descriptors (feature vectors)
    cv::SurfDescriptorExtractor surfExtractor;
    cv::surfExtractor.compute(img1_cp,kp1,des1);
    scv::urfExtractor.compute(img2_cp,kp2,des2);

    //-- Step 3: Matching descriptor vectors using Brute Force matcher
    cv::BFMatcher bfmatcher;
    std::vector<std::vector< cv::DMatch > > matches12, matches21;
    cv::bfmatcher.knnMatch(des1, des2, matches12,2); //from img1 to img2
    cv::bfmatcher.knnMatch(des2, des1, matches21,2); //from img2 to img1

    //qDebug()<<"knnMatch";

    //-- Step 3.1: Remove matches for which NN ratio is > than threshold
    double ratio = 0.85;
    // clean image 1 -> image 2 matches
    int removed= ratioTest(matches12,ratio);
    // clean image 2 -> image 1 matches
    removed = ratioTest(matches21,ratio);

    //qDebug()<<"ratioTest";

    //-- Step 3.2: Symetric test
    symmetryTest(matches12,matches21,symMatches);

    //qDebug()<<"symmetryTest";

    //-- Step 3.3: Validate with Ransac test
    // Only apply ransac and dev tests is we have more than 8 matches
    if ( symMatches.size() > 8 ){
        ransacTest(symMatches,kp1,kp2,ransacMatches);
        //qDebug()<<"ransacTest";

        devTest(ransacMatches,kp1,kp2,finalMatches);
        //qDebug()<<"devTest";

    }
    else finalMatches = symMatches;

    //qDebug()<<"finalMatches";

	// drawing the results
	namedWindow("matches", 1);
	Mat img_matches;
	drawMatches(img1, kp1, img2, kp2, finalMatches, img_matches);
	imshow("matches", img_matches);
}
