#include<iostream>
using namespace std;
#include<Eigen/Core>
#include<Eigen/Geometry>
#include"sophus/so3.h"

int main()
{
    double data[4]={-0.0004327,-0.113131,-0.0326832,0.993042};
    Eigen::Quaterniond q(data[3],data[0],data[1],data[2]);
    Eigen::Matrix3d rotation_matrix=q.matrix();

    cout<<"选取四元数q:"<<q.coeffs().transpose()<<endl;
    cout<<"其对应的旋转矩阵为："<<endl<<rotation_matrix<<endl;
    Eigen::Vector3d w(0.01,0.02,0.03);
    cout<<"选取小量w="<<w.transpose()<<endl;

    //方式一…………………………………………………………………………………………………………//
    Sophus::SO3 R(rotation_matrix);
    Sophus::SO3 result_R=R*Sophus::SO3::exp(w);
    cout<<"方式一结果：R*exp(w^)="<<endl;
    cout<<result_R.matrix()<<endl;
    cout<<"转换得到四元数为："<<endl;
    Eigen::Quaterniond qforR(result_R.matrix());
    cout<<qforR.coeffs().transpose()<<endl;

    //方式二…………………………………………………………………………………………………………//
    Eigen::Quaterniond update_w(1,1.0/2*w[0],1.0/2*w[1],1.0/2*w[2]);
    Eigen::Quaterniond result_q=q*update_w;
    result_q.normalize();
    cout<<"方式二结果：q*[1,1/2w]="<<endl;
    cout<<result_q.coeffs().transpose()<<endl;
    cout<<"转换得到矩阵为："<<endl;
    cout<<result_q.matrix()<<endl;


    return 0;
}