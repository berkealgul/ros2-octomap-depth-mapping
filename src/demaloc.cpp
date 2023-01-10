#include "demaloc.hpp"


Demaloc::Demaloc()
{
    myfile.open(p1);
    //ocmap = std::make_shared<octomap::OcTree>(0.1);
}

void Demaloc::read_dataset_once()
{
    std::stringstream path_stream;
    path_stream << p << std::setw(5) << std::setfill('0') << data_counter << "-depth.png";
    std::string path = path_stream.str();
    
    cv::Mat mat = cv::imread(path, cv::IMREAD_ANYDEPTH);
    data_counter++;

    std::string line;
    std::getline(myfile, line);
    double x, y, z, a, b, c ,d;
    std::istringstream iss(line);
    iss >> a >> b >> c >> d >> x >> y >> z;  

    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation.x = a;
    pose.orientation.y = b;
    pose.orientation.z = c;
    pose.orientation.w = d;

    //process map
    update_map(mat, pose);
}

double Demaloc::rawDepthToMeters(ushort raw_depth) 
{
    if(raw_depth > 6408)
    {
        return (double)(((2.5-0.9)/(15800.0-6408.0))*raw_depth);
    }        

    return 0;
}

void Demaloc::update_map(cv::Mat& img, geometry_msgs::msg::Pose& pose)
{
    pc.clear();

    tf2::Transform t, t_i;
    tf2::fromMsg(pose, t);
    t_i = t.inverse();
    
    tf2::Matrix3x3 m(524, 0, 316.8, 0, 524, 238.5, 0, 0, 1);
    auto m_i = m.inverse();
    auto r_i = t_i.getBasis();
    auto v_i = t_i.getOrigin();

    octomap::point3d origin(pose.position.x, pose.position.y, pose.position.z);

    for(int i = 0; i < img.rows; i+=1)
    {
        for(int j = 0; j < img.cols; j+=1)
        {
            cv::Point minLoc, maxLoc;
            double min, raw = 0;
            //cv::minMaxLoc(img, &min, &raw, &minLoc, &maxLoc);
            //raw = (double)img.at<uchar>(i, j);
            ushort r = img.at<ushort>(i, j);
            double d = rawDepthToMeters(r);
            //std::cout << r << " ";

            tf2::Vector3 p(i*d, j*d, d);
            p = tf2::Vector3(m_i[0].dot(p), m_i[1].dot(p), m_i[2].dot(p));
            p+=v_i;
            p = tf2::Vector3(r_i[0].dot(p), r_i[1].dot(p), r_i[2].dot(p));
            octomap::point3d target(p.getX(), p.getY(), p.getZ());

            //std::cout << target << " " << origin << std::endl;
            pc.push_back(target);
            //ocmap.updateNode(target, true);
            ocmap.insertRay(origin, target);
        }
    }
}