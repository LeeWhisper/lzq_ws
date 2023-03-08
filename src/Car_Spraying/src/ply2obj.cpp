#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/point_types.h>

int main(int argc, char const *argv[])
{
    pcl::TextureMeshPtr mesh;
    pcl::PolygonMeshPtr ply;
    pcl::io::loadOBJFile("", *mesh);
    pcl::io::loadPLYFile("", *ply);

    
    return 0;
}
