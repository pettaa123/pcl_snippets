// segment pcl pointcloud by 2D polygon using convex hull and extract polygonal prism data

    std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> > testpoints;
    for(int i=0;i<5;i++){
        float x=i+0.5;
        for(int j=0;j<5;j++){
            float y=j+0.5;
            testpoints.push_back(pcl::PointXYZ(x,1.0,y));
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud (new pcl::PointCloud<pcl::PointXYZ> ());

    pointCloud->points=testpoints;
    pointCloud->width=testpoints.size();
    pointCloud->height=1;
    //hull points
    std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> > pcl_polygon;
    pcl_polygon.push_back(pcl::PointXYZ(0 , 0 , 0));
    pcl_polygon.push_back(pcl::PointXYZ(0 , 0 , 2));
    pcl_polygon.push_back(pcl::PointXYZ(2 , 0 , 2));
    pcl_polygon.push_back(pcl::PointXYZ(2 , 0 , 0));

    pcl::PointCloud<pcl::PointXYZ>::Ptr hull_vertices (new pcl::PointCloud<pcl::PointXYZ> ());
    hull_vertices->points=pcl_polygon;
    hull_vertices->width=4;
    hull_vertices->height=1;

    pcl::ConvexHull<pcl::PointXYZ> hull;
    hull.setInputCloud (hull_vertices);

    pcl::PointCloud<pcl::PointXYZ>::Ptr hull_points (new pcl::PointCloud<pcl::PointXYZ> ());
    hull.reconstruct (*hull_points);
    //hull_points->push_back(hull_points->at(0));


    if (hull.getDimension () != 2){
        PCL_ERROR ("The input cloud does not represent a planar surface.\n");
        return;
    }

    pcl::PointIndices::Ptr inPrism (new pcl::PointIndices);

    pcl::ExtractPolygonalPrismData<pcl::PointXYZ> searchPrism;
    searchPrism.setInputCloud (pointCloud);
    searchPrism.setInputPlanarHull (hull_points);
    searchPrism.setHeightLimits (-10, 10);
    searchPrism.segment (*inPrism);

    qDebug() << QString::number(inPrism->indices.size());
