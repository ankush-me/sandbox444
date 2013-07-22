#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <math.h>
#include <limits>
#include <vector>

using namespace Eigen;
using namespace std;

int main(int argc, char** argv) {
  MatrixXf m(3,3);
  m << 1,2,3,4,5,6,7,8,9;
  Vector3f v =  m.col(0);
  Vector3f v2 =  m.col(1);
  Vector3f v3 =  m.col(2);

  cout<<"Mat = "<<m<<endl;
  cout<<"vector = " << v<<endl;
  
  MatrixXf n = (m.rowwise() -= v);
  cout<<"Mat2 = "<<n<<endl;
  
  Matrix3f r;
  r.col(0)  = v;
  r.col(1)  = v2;
  r.col(2)  = v3;
  cout<<"Mat3 = "<<r<<endl;
  r *= 2;
  cout<<"Mat3 = "<<r<<endl;
  cout<<"Mat3 = "<<m<<endl;


  float arr[] = {1,2,3};
  vector<float> w(arr, arr + sizeof(arr) / sizeof(float));
  for(int i = 0; i < w.size(); i+=1)
    cout<<w[i]<<"\t";
  cout<<endl<<"---------------"<<endl;

  /** Inverting a singular matrix. **/
  m << 1,0,0,1,0,0,1,0,0;
  cout<<" Should segfault/ get garbage: "<<m.determinant()<<endl;

  /** Affine matrix in Eigen. */
  MatrixXf ml(4,4);
  ml << 1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16;
  Affine3f t;
  Vector3f d(10,20,30);
  t.linear() = ml.transpose().block(0,0,3,3);
  t.translation() = ml.block(0,3,3,1);
  cout<<"Affine: "<<endl<<t.affine()<<endl;
  Vector3f tt(1,2,3);
  t.translation() += tt;
  cout<<"Affine after translation: "<<endl<<t.affine()<<endl;


  /** Blocks. */
  MatrixXf matr(3,2);
  matr << 1,2,3,4,5,6;
  MatrixXf combo(4,2);
  combo <<matr, MatrixXf::Ones(1,matr.cols());
  cout<<"Matr = "<<matr<<endl;
  cout<<"Comb = "<<combo<<endl;
    
  MatrixXf rrr = combo.block(0,0,3,2);
  cout<<"rrr = "<<rrr<<endl;


  /** Filling up a matrix*/
  std::cout<<"---------------------------------------"<<std::endl;
  MatrixXf matF(5,3);
  Vector3f vF(1,.3,3);
  Vector3f vF1(.123,.2,.3);
  Vector3f vF2(33.4,23.3,12.07);
  Vector3f vF3(0.54,8.96,14.3);
  Vector3f vF4(8.9,0.34,32.2);

  matF.row(0) = vF;
  matF.row(1) = vF1;
  matF.row(2) = vF2;
  matF.row(3) = vF3;
  matF.row(4) = vF4;

  RowVector3f means = matF.colwise().sum()/5;
  MatrixXf centered = (matF.rowwise() - means);

  cout<<"Matrix Fill = \n"<<matF<<std::endl;
  cout<<"Means = \n"<<means<<std::endl;
  cout<<"Centered = \n"<<centered<<std::endl;

  Eigen::JacobiSVD<MatrixXf> svd(centered / sqrt(5), ComputeFullV);
  cout << "Its singular values are:" << endl << svd.singularValues() << endl;
  cout << "Its right singular vectors are the columns of the thin V matrix:" << endl << svd.matrixV() << endl;
  MatrixXf V   = svd.matrixV();
  Vector3f f1  = V.col(0);
  Vector3f  f2 = V.col(1);
  Vector3f  f3 = V.col(2);
  cout << "f1(0)" << f1(0) << endl;

  VectorXf faa(V.rows());
  faa.setZero();
  //for (int i= 0; i < V.rows(); i++)
  V.col(2) = faa;
  cout << "V " << V<< endl;


  cout << "<v1,v2>" << f1.dot(f2) << endl;
  cout << "<v1,v3>" << f1.dot(f3) << endl;
  cout << "<v3,v2>" << f3.dot(f2) << endl;


  std::cout<<"---------------------------------------"<<std::endl;

  Vector3f o3(1,2,3), o2(4,5,6);
  cout << "outer? : "<< o3.cwiseProduct(o2) <<endl;

  MatrixXd mxx(3,3);
  mxx << 1,0,0,0,2,0,0,0,3;
  cout << "mxx : "<< endl<<mxx <<endl;

  vector<double> v31(3);
  VectorXd::Map(&v31[0], 3) = mxx.row(0);

  cout << "v: "<<endl;
  for(int i=0; i < 3; i++) 
    cout<< v31[i]<< " " <<endl;
  cout<<endl;

  v31[1] = 100;
  cout << "v: "<<endl;
  for(int i=0; i < 3; i++) 
    cout << v31[i]<< " " <<endl;
  cout<<endl;

  cout << " mxx : "<<endl <<mxx<<endl;

  Matrix3d tmat = Matrix3d::Identity();
  tmat(0,0) = 0; tmat(1,1) = 10; tmat(1,2) = 20;
  cout << "tmat: " << tmat<<endl;
  cout << "------------\n";
  for (unsigned i=0; i < tmat.rows(); i++) {
    double sum = tmat.row(i).sum() + numeric_limits<double>::min();
    tmat.row(i) /= sum;
  }

  cout << tmat << endl;

  MatrixXd src(4,3);
  src<< 0,0,0,1,1,1,2,2,2,3,3,3;
  MatrixXd target(1,3);
  target<< 1,1,1;

  cout <<"cloud err: "<<  (src.rowwise() -target.row(0)).rowwise().squaredNorm().transpose()<<endl;

  
  return 0;
}
