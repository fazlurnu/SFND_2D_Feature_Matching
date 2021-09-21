for detector in SHITOMASI HARRIS FAST BRISK ORB AKAZE SIFT
    do
        for descriptor in BRISK BRIEF ORB FREAK AKAZE SIFT
            do
                ./2D_feature_tracking $detector $descriptor
            done
    done
