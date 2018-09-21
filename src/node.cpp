#define BACKWARD_HAS_DW 1
#include "../include/rviz_hybrid_imshow/backward.hpp"
namespace backward
{
backward::SignalHandling sh;
}

#include <camera_model/camera_models/CameraFactory.h>
#include <code_utils/eigen_utils.h>
#include <iomanip>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

template< typename Derived >
static Eigen::Matrix< typename Derived::Scalar, 3, 3 >
ypr2R( const Eigen::MatrixBase< Derived >& ypr )
{
    typedef typename Derived::Scalar Scalar_t;

    Scalar_t y = ypr( 0 ) / 180.0 * M_PI;
    Scalar_t p = ypr( 1 ) / 180.0 * M_PI;
    Scalar_t r = ypr( 2 ) / 180.0 * M_PI;

    Eigen::Matrix< Scalar_t, 3, 3 > Rz;
    Rz << cos( y ), -sin( y ), 0, sin( y ), cos( y ), 0, 0, 0, 1;

    Eigen::Matrix< Scalar_t, 3, 3 > Ry;
    Ry << cos( p ), 0., sin( p ), 0., 1., 0., -sin( p ), 0., cos( p );

    Eigen::Matrix< Scalar_t, 3, 3 > Rx;
    Rx << 1., 0., 0., 0., cos( r ), -sin( r ), 0., sin( r ), cos( r );

    return Rz * Ry * Rx;
}

int
main( int argc, char** argv )
{
    ros::init( argc, argv, "reproject_test2" );
    ros::NodeHandle n( "~" );

    ros::Publisher pub_point = n.advertise< sensor_msgs::PointCloud2 >( "Pont", 100 );

    double s;
    double y, p, r;
    double tx, ty, tz;
    std::string camera_model_file;
    std::string image_name;

    n.getParam( "camera_model", camera_model_file );
    n.getParam( "image_name", image_name );
    n.getParam( "scale", s );
    n.getParam( "yaw", y );
    n.getParam( "pitch", p );
    n.getParam( "roll", r );
    n.getParam( "tx", tx );
    n.getParam( "ty", ty );
    n.getParam( "tz", tz );

    Eigen::Vector3d euler, T_wc;
    euler << y, p, r;

    T_wc << tx, ty, tz;
    Eigen::Matrix3d R_wc = ypr2R( euler );

    camera_model::CameraPtr cam;

    std::cout << "#INFO: camera config is " << camera_model_file << std::endl;
    cam = camera_model::CameraFactory::instance( )->generateCameraFromYamlFile( camera_model_file );

    vector< double > params;
    cam->writeParameters( params );
    std::cout << cam->parametersToString( ) << std::endl;

    std::cout << "#INFO: LOADing camera config is DONE." << camera_model_file << std::endl;

    std::cout << "#INFO: image is " << image_name << std::endl;
    cv::Mat image_in = cv::imread( image_name );

    cv::Mat Image_color;

    if ( image_in.channels( ) == 1 )
        cv::cvtColor( image_in, Image_color, CV_GRAY2RGB );
    else
        Image_color = image_in;

    int w, h, level;
    level = 0;
    w     = ( int )( image_in.cols );
    h     = ( int )( image_in.rows );

    sensor_msgs::PointCloud2 imagePoint;
    imagePoint.header.stamp    = ros::Time::now( );
    imagePoint.header.frame_id = "world";
    imagePoint.height          = h;
    imagePoint.width           = w;
    imagePoint.fields.resize( 4 );
    imagePoint.fields[0].name     = "x";
    imagePoint.fields[0].offset   = 0;
    imagePoint.fields[0].count    = 1;
    imagePoint.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    imagePoint.fields[1].name     = "y";
    imagePoint.fields[1].offset   = 4;
    imagePoint.fields[1].count    = 1;
    imagePoint.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    imagePoint.fields[2].name     = "z";
    imagePoint.fields[2].offset   = 8;
    imagePoint.fields[2].count    = 1;
    imagePoint.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    imagePoint.fields[3].name     = "rgb";
    imagePoint.fields[3].offset   = 12;
    imagePoint.fields[3].count    = 1;
    imagePoint.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
    imagePoint.is_bigendian       = false;
    imagePoint.point_step         = sizeof( float ) * 4;
    imagePoint.row_step           = imagePoint.point_step * imagePoint.width;
    imagePoint.data.resize( imagePoint.row_step * imagePoint.height );
    imagePoint.is_dense = true;

    int i = 0;
    for ( int row_index = 0; row_index < image_in.rows; ++row_index )
    {
        for ( int col_index = 0; col_index < image_in.cols; ++col_index, ++i )
        {
            Eigen::Vector2d p_u( col_index, row_index );
            Eigen::Vector3d P;

            Eigen::Vector2d p_u2;
            cam->liftSphere( p_u, P );
            cam->spaceToPlane( P, p_u2 );

            P.normalize( );
            P = P * s;
            P = R_wc * P + T_wc;

            float x = P( 0 );
            float y = P( 1 );
            float z = P( 2 );

            int32_t rgb;

            uint g = ( uchar )image_in.at< uchar >( row_index, col_index );
            rgb    = ( g << 16 ) | ( g << 8 ) | g;

            uint gr = ( uchar )image_in.at< cv::Vec3b >( row_index, col_index )[0];
            uint gg = ( uchar )image_in.at< cv::Vec3b >( row_index, col_index )[1];
            uint gb = ( uchar )image_in.at< cv::Vec3b >( row_index, col_index )[2];
            rgb     = ( gb << 16 ) | ( gg << 8 ) | gr;

            memcpy( &imagePoint.data[i * imagePoint.point_step + 0], &x, sizeof( float ) );
            memcpy( &imagePoint.data[i * imagePoint.point_step + 4], &y, sizeof( float ) );
            memcpy( &imagePoint.data[i * imagePoint.point_step + 8], &z, sizeof( float ) );
            memcpy( &imagePoint.data[i * imagePoint.point_step + 12], &rgb, sizeof( int32_t ) );
        }
    }

    ros::Duration delay( 0.1 );
    for ( int i = 0; i < 10; ++i )
    {
        pub_point.publish( imagePoint );
        delay.sleep( );
    }

    cv::namedWindow( "input_image", cv::WINDOW_NORMAL );
    cv::imshow( "input_image", Image_color );
    cv::waitKey( 0 );

    return 0;
}
