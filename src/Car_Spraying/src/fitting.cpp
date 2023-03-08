// 源文件fitting.cpp
#include "Car_Spraying/fitting.h"
fitting::fitting() // 构造函数
{
}
fitting::~fitting() // 析构函数
{
    cloud->clear();
}
// 定义点云输入函数
void fitting::setinputcloud(PointCloud<PointT>::Ptr input_cloud)
{
    cloud = input_cloud;
    getMinMax3D(*input_cloud, point_min, point_max); // getMinMax3D该函数输入点云数据，将所有xyz中最小的值和xyz中最大的值输出到pcl::PointXYZ中，即pcl::PointXYZ min_p, max_p;
}
// 将点云数据投影至XOY，并划分为规则格网，求每个格网内点云坐标均值
void fitting::grid_mean_xyz(double x_resolution, double y_resolution, vector<double> &x_mean, vector<double> &y_mean, vector<double> &z_mean, PointCloud<PointT>::Ptr &new_cloud)
{
    if (y_resolution <= 0) // 表示如果输入的分辨率小于0，则不进行Y方向的分段处理
    {
        y_resolution = point_max.y - point_min.y; // 输入的分辨率小于0则Y方向的分辨率重新调整为Y值的最大值与最小值之差
    }
    int raster_rows, raster_cols;                                   // 定义行数row和列数col
    raster_rows = ceil((point_max.x - point_min.x) / x_resolution); // ceil() 函数向上舍入为最接近的整数。如需向下舍入为最接近的整数，请查看 floor(),函数floor(123.5)返回123。如需对浮点数进行四舍五入，请查看 round() 函数。
    raster_cols = ceil((point_max.y - point_min.y) / y_resolution); // ceil(123.5)返回大于或者等于指定表达式的最小整数，返回124
    // 容器都是类模板，是一种可变长数组。它们实例化后就成为容器类。用容器类定义的对象称为容器对象。
    // vector 是顺序容器的一种。vector 是可变长的动态数组
    vector<int> idx_point;                 // idx_point就是容器对象//点的索引
    vector<vector<vector<float>>> row_col; // row_col是一个三重数组，第一重是实际行号（就是有点的）（因为上述划分的raster_rows是总行数有可能有的行列里边没有投影进去点），第二重是实际列号第三重是该行列中的点坐标及索引即（行，列，XYZ索引）
    vector<vector<float>> col_;            // col_是一个二重数组，表示上述三重数组中第二重存放的是列
    vector<float> vector_4;                // vector_4是个一重数组，存放的是点坐标及索引
    vector_4.resize(4);                    // 将vector_4的现有元素个数调整至4个，多则删，少则补，其值随机
    col_.resize(raster_cols, vector_4);    // 将col_的现有元素个数调整至raster_cols个，多则删，少则补，其值为col_（因为raster_cols是划分定义的行数）
    row_col.resize(raster_rows, col_);     // 将row_col的现有元素个数调整至raster_rows个，多则删，少则补，其值为col_
    int point_num = cloud->size();
    // 填充三重数组
    for (int i_point = 0; i_point < point_num; i_point++)
    {
        // 找到当前点所在的(行)(列)
        int row_idx = ceil((cloud->points[i_point].x - point_min.x) / x_resolution) - 1; // 该点当前所在的行（有可能很多个点算出来都在这个行内比如都是3.3就取第三行）
        int col_idx = ceil((cloud->points[i_point].y - point_min.y) / y_resolution) - 1; // 计算当前Y值被划分到了哪个列
        if (row_idx < 0)
            row_idx = 0; // 如果行列都小于0,则将row_idx赋值为0；（此处不存在因为减去的就是最小值不可能为负）
        if (col_idx < 0)
            col_idx = 0;
        // 找到了行列后把(xyz索引)塞进去
        row_col[row_idx][col_idx][0] += cloud->points[i_point].x; // 把落尽该格子内的所有点的所有x值相加，为了下边求格网的平均值
        row_col[row_idx][col_idx][1] += cloud->points[i_point].y; // 把点云坐标的所有y值相加
        row_col[row_idx][col_idx][2] += cloud->points[i_point].z; // 把点云坐标的所有z值相加
        row_col[row_idx][col_idx][3] += 1;                        // 落到该网格的总的点数就是一共有多少个点落到目前这个网格子
    }
    PointT point_mean_tem;
    // 求每个网格的平均值
    for (int i_row = 0; i_row < row_col.size(); i_row++)
    {
        for (int i_col = 0; i_col < row_col[i_row].size(); i_col++)
        {
            if (row_col[i_row][i_col][3] != 0) // 不等于0就表明
            {
                double x_mean_tem = row_col[i_row][i_col][0] / row_col[i_row][i_col][3]; // 求x的平均（该网格内点的x总和除以点的总个数）
                double y_mean_tem = row_col[i_row][i_col][1] / row_col[i_row][i_col][3]; // 求y的平均
                double z_mean_tem = row_col[i_row][i_col][2] / row_col[i_row][i_col][3]; // 求z的平均
                x_mean.push_back(x_mean_tem);
                y_mean.push_back(y_mean_tem);
                z_mean.push_back(z_mean_tem);
                point_mean_tem.x = x_mean_tem;
                point_mean_tem.y = y_mean_tem;
                point_mean_tem.z = z_mean_tem;
                new_cloud->push_back(point_mean_tem); // 将他们的平均值保存在新的点云中
            }
        }
    }
}
// 每个格网内点云坐标均值结果三维展示
void fitting::grid_mean_xyz_display(PointCloud<PointT>::Ptr new_cloud)
{
    visualization::PCLVisualizer::Ptr view(new visualization::PCLVisualizer("Piecewise centroid fitting")); // 可视化界面标题
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sources_cloud_color(source,250,0,0); //这句话的意思是：对输入为pcl::PointXYZ类型的点云，着色为红色。其中，source表示真正处理的点云，sources_cloud_color表示处理结果.
    visualization::PointCloudColorHandlerCustom<PointT> color_1(new_cloud, 255, 0, 0); // 创建一个自定义颜色处理器PointCloudColorHandlerCustom对象，给点云着色并设置颜色为红色
    // view->addPointCloud(source,sources_cloud_color,"sources_cloud_v1",v1); //将点云source,处理结果sources_cloud_color,添加到视图中，其中,双引号中的sources_cloud_v1,表示该点云的”标签“，我们依然可以称之为”名字“，之所以设置各个处理点云的名字，是为了在后续处理中易于区分; v1表是添加到哪个视图窗口（pcl中可设置多窗口模式）
    view->addPointCloud(new_cloud, color_1, "11"); // 加载点云
    // view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3,"sources_cloud_v1"); //设置点云属性. 其中PCL_VISUALIZER_POINT_SIZE表示设置点的大小为3,双引号中”sources_cloud_v1“,就是步骤2中所说的标签。
    // view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,1,"sources_cloud_v1"); //主要用来设置标签点云的不透明度，表示对标签名字为"sources_cloud_v1"的标签点云设置不透明度为1,也就是说透明度为0. 默认情况下完全不透明。
    view->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 3, "11");
    PointCloud<PointT>::Ptr new_cloud_final(new PointCloud<PointT>); // 创建一个新的点云用于存储结果
    for (int i_point = 0; i_point < cloud->size(); i_point++)
    {
        PointT tem_point;
        tem_point.x = cloud->points[i_point].x;
        tem_point.y = cloud->points[i_point].y;
        tem_point.z = cloud->points[i_point].z;
        new_cloud_final->push_back(tem_point);
    }
    view->addPointCloud(new_cloud_final, "22");
    view->spin();
}
// 拟合平面直线y=kx+b
// 引用的本质：在定义或声明函数时，我们可以将函数的形参指定为引用的形式，这样在调用函数时就会将实参和形参绑定在一起，让它们都指代同一份数据。如此一来，如果在函数体中修改了形参的数据，那么实参的数据也会被修改，从而拥有“在函数内部影响函数外部数据”的效果。
void fitting::line_fitting(vector<double> x, vector<double> y, double &k, double &b) // 此函数输入是x,y；输出是k,b；是C++中的引用知识（“在函数内部影响函数外部数据”）
{
    MatrixXd A_(2, 2), B_(2, 1), A12(2, 1);        // A_是个矩阵两行两列，其他同理
    int num_point = x.size();                      // num_point等于输入的x的总个数
    double A01(0.0), A02(0.0), B00(0.0), B10(0.0); // A01表示第0行第1列//A01(0.0), A02(0.0)表明A矩阵的第一行都初始化为0
    for (int i_point = 0; i_point < num_point; i_point++)
    {
        A01 += x[i_point] * x[i_point];
        A02 += x[i_point];
        B00 += x[i_point] * y[i_point];
        B10 += y[i_point];
    }
    A_ << A01, A02,
        A02, num_point; // 把这四个数塞给A矩阵；A矩阵就是推导中的x转置乘x的逆
    B_ << B00,
        B10;                 // B矩阵就是X转置乘Y
    A12 = A_.inverse() * B_; // A_.inverse()是求A的逆矩阵；
    k = A12(0, 0);           // A12是两行一列，那么k就是第0行第0列
    b = A12(1, 0);           // b就是第1行第0列
}
// 拟合y=a*x^2+b*x+c;//平面曲线
void fitting::polynomial2D_fitting(vector<double> x, vector<double> y, double &a, double &b, double &c)
{
    MatrixXd A_(3, 3), B_(3, 1), A123(3, 1);
    int num_point = x.size();
    double A01(0.0), A02(0.0), A12(0.0), A22(0.0), B00(0.0), B10(0.0), B12(0.0);
    for (int i_point = 0; i_point < num_point; i_point++)
    {
        A01 += x[i_point];
        A02 += x[i_point] * x[i_point];
        A12 += x[i_point] * x[i_point] * x[i_point];
        A22 += x[i_point] * x[i_point] * x[i_point] * x[i_point];
        B00 += y[i_point];
        B10 += x[i_point] * y[i_point];
        B12 += x[i_point] * x[i_point] * y[i_point];
    }
    A_ << num_point, A01, A02,
        A01, A02, A12,
        A02, A12, A22;
    B_ << B00,
        B10,
        B12;
    A123 = A_.inverse() * B_;
    a = A123(2, 0);
    b = A123(1, 0);
    c = A123(0, 0);
}
// 拟合空间曲线
void fitting::polynomial3D_fitting(vector<double> x, vector<double> y, vector<double> z, double &a, double &b, double &c)
{
    int num_point = x.size();
    MatrixXd A_(3, 3), B_(3, 1), A123(3, 1);
    double A01(0.0), A02(0.0), A12(0.0), A22(0.0), B00(0.0), B10(0.0), B12(0.0);
    for (int i_point = 0; i_point < num_point; i_point++)
    {
        double x_y = sqrt(pow(x[i_point], 2) + pow(y[i_point], 2));
        A01 += x_y;
        A02 += pow(x_y, 2);
        A12 += pow(x_y, 3);
        A22 += pow(x_y, 4);
        B00 += z[i_point];
        B10 += x_y * z[i_point];
        B12 += pow(x_y, 2) * z[i_point];
    }
    A_ << num_point, A01, A02,
        A01, A02, A12,
        A02, A12, A22;
    B_ << B00,
        B10,
        B12;
    A123 = A_.inverse() * B_;
    line_fitting(x, y, k_line, b_line);
    a = A123(2, 0);
    b = A123(1, 0);
    c = A123(0, 0);
    c_3d = c;
    b_3d = b;
    a_3d = a;
}
// 空间曲线展示
void fitting::polynomial3D_fitting_display(double step_) // 传入的参数相当于步长
{
    PointT point_min_, point_max_;
    getMinMax3D(*cloud, point_min_, point_max_);
    // 利用最小外包框的x值，向拟合的直线做垂足，垂足的交点即为三维曲线的端点值***********
    int idx_minx, idx_maxy; // x取到最大值和最小值的点号索引
    for (int i_point = 0; i_point < cloud->size(); i_point++)
    {
        if (cloud->points[i_point].x == point_min_.x)
            idx_minx = i_point;
        if (cloud->points[i_point].x == point_max_.x)
            idx_maxy = i_point;
    }
    float m_min = cloud->points[idx_minx].x + k_line * cloud->points[idx_minx].y;
    float m_max = cloud->points[idx_maxy].x + k_line * cloud->points[idx_maxy].y;

    float x_min = (m_min - b_line * k_line) / (1 + k_line * k_line);
    float x_max = (m_max - b_line * k_line) / (1 + k_line * k_line);
    //---------------------------------------------------------------------------------------
    vector<double> xx, yy, zz;
    int step_num = ceil((x_max - x_min) / step_);
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    for (int i_ = 0; i_ < step_num + 1; i_++)
    {
        double tem_value = x_min + i_ * step_;
        if (tem_value > x_max)
        {
            tem_value = x_max;
        }
        xx.push_back(tem_value);
        yy.push_back(k_line * xx[i_] + b_line);
        double xxyy = sqrt(pow(xx[i_], 2) + pow(yy[i_], 2));
        zz.push_back(c_3d + b_3d * xxyy + a_3d * pow(xxyy, 2));
        points->InsertNextPoint(xx[i_], yy[i_], zz[i_]);
    }
    vtkSmartPointer<vtkPolyLine> polyLine = vtkSmartPointer<vtkPolyLine>::New();
    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();
    polyData->SetPoints(points);
    polyLine->GetPointIds()->SetNumberOfIds(points->GetNumberOfPoints());
    for (unsigned int i = 0; i < points->GetNumberOfPoints(); i++)
        polyLine->GetPointIds()->SetId(i, i);
    cells->InsertNextCell(polyLine);
    polyData->SetLines(cells);
    visualization::PCLVisualizer::Ptr viewer(new visualization::PCLVisualizer("Output curve"));
    viewer->addModelFromPolyData(polyData, "1");
    //*******************************************
    PointCloud<PointT>::Ptr tem_point(new PointCloud<PointT>);
    for (int i = 0; i < xx.size(); i++)
    {
        PointT point_;
        point_.x = xx[i];
        point_.y = yy[i];
        point_.z = zz[i];
        tem_point->push_back(point_);
    }
    visualization::PointCloudColorHandlerCustom<PointT> color1(tem_point, 255, 0, 0);
    viewer->addPointCloud(tem_point, color1, "point1");
    viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 3, "point1");

    PointCloud<PointT>::Ptr tem_point1(new PointCloud<PointT>);
    for (int i = 0; i < cloud->size(); i++)
    {
        PointT point_1;
        point_1.x = cloud->points[i].x;
        point_1.y = cloud->points[i].y;
        point_1.z = cloud->points[i].z;
        tem_point1->push_back(point_1);
    }
    viewer->addPointCloud(tem_point1, "orginal");
    viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 2, "orginal");
    // 显示端点
    PointCloud<PointT>::Ptr duandian_point(new PointCloud<PointT>);
    duandian_point->push_back(tem_point->points[0]);
    duandian_point->push_back(tem_point->points[tem_point->size() - 1]);
    visualization::PointCloudColorHandlerCustom<PointT> color2(duandian_point, 0, 255, 255);
    viewer->addPointCloud(duandian_point, color2, "duandian");
    viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 5, "duandian");
    cout << "端点值1为："
         << "X1= " << duandian_point->points[0].x << ", "
         << "Y1= " << duandian_point->points[0].y << ", "
         << "Z1= " << duandian_point->points[0].z << endl;
    cout << "端点值2为："
         << "X2= " << duandian_point->points[1].x << ", "
         << "Y2= " << duandian_point->points[1].y << ", "
         << "Z2= " << duandian_point->points[1].z << endl;
    cout << "空间多项式曲线方程为： "
         << "z=" << a_3d << "*(x^2+y^2)+" << b_3d << "*sqrt(x^2+y^2)+" << c_3d << endl;
    // viewer->spin();
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        // boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    // 拟合曲线+端点值+散点图二维平面展示，有需要可以取消注释----------------------------------------------------------
    /*vector<double>vector_1, vector_2, vector_3, vector_4;
    vector_1.push_back(duandian_point->points[0].x);
    vector_1.push_back(duandian_point->points[1].x);
    vector_2.push_back(duandian_point->points[0].y);
    vector_2.push_back(duandian_point->points[1].y);
    for (int i = 0; i < cloud->size();i++)
    {
    vector_3.push_back(cloud->points[i].x);
    vector_4.push_back(cloud->points[i].y);
    }
    std::vector<double> func1(2, 0);
    func1[0] = b_line;
    func1[1] = k_line;
    visualization::PCLPlotter *plot_line1(new visualization::PCLPlotter);
    plot_line1->addPlotData(func1, vector_1[0], vector_1[1]);
    plot_line1->addPlotData(vector_3, vector_4, "display", vtkChart::POINTS);//X,Y均为double型的向量
    plot_line1->addPlotData(vector_1, vector_2, "display", vtkChart::POINTS);//X,Y均为double型的向量
    plot_line1->setShowLegend(false);
    plot_line1->plot();*/
}
void fitting::display_point(vector<double> vector_1, vector<double> vector_2)
{
    visualization::PCLPlotter *plot_line1(new visualization::PCLPlotter);
    plot_line1->addPlotData(vector_1, vector_2, "display", vtkChart::POINTS); // X,Y均为double型的向量
    plot_line1->setShowLegend(false);
    plot_line1->plot();
}
void fitting::display_line(vector<double> vector_1, vector<double> vector_2, double c, double b, double a)
{
    visualization::PCLPlotter *plot_line1(new visualization::PCLPlotter);
    std::vector<double> func1(3, 0);
    func1[0] = c;
    func1[1] = b;
    func1[2] = a;
    plot_line1->addPlotData(func1, point_min.x, point_max.x);
    plot_line1->addPlotData(vector_1, vector_2, "display", vtkChart::POINTS); // X,Y均为double型的向量
    plot_line1->setShowLegend(false);
    plot_line1->plot();
}