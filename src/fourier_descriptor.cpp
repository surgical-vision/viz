#define _USE_MATH_DEFINES
#include <cmath>

#include "fourier_descriptor.hpp"

#include <fstream>
#include <opencv2/imgproc/imgproc_c.h>

int str::EllipticalFourierDescriptorBuilder::number_of_harmonics_ = 50;

std::vector<str::EllipticalFourierDescriptor> str::EllipticalFourierDescriptorBuilder::BuildFromImage(const std::string &filepath) const{

  cv::Mat im = cv::imread(filepath, 0);

  return BuildFromImage(im);

}

std::vector<str::EllipticalFourierDescriptor> str::EllipticalFourierDescriptorBuilder::BuildFromImage(const cv::Mat &im) const {

  std::vector<cv::Point> contour;

  /*
  std::ifstream ifs(filepath);
  while( !ifs.eof() ){
  int x,y;
  ifs >> x; ifs >> y;
  contour.push_back( cv::Point(x,y) );
  }
  return BuildFromContour(contour);
  */


  std::vector<std::vector<cv::Point> >contours;
  cv::findContours(im, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

  if (contours.size() != 1) throw(std::runtime_error("Error, testing - should only be one contour!\n"));

  return BuildFromContour(contours[0]);

}




std::vector<str::EllipticalFourierDescriptor> str::EllipticalFourierDescriptorBuilder::BuildFromContour(const std::vector<cv::Point> &contour) const{

  std::vector<int> deltaX, deltaY;
  std::vector<double> deltaT, time;
  GetIncrementXYT(deltaX, deltaY, deltaT, time, contour);

  std::vector< str::EllipticalFourierDescriptor > FSDs;

  double A0 = GetA0Coeff(deltaX, deltaT, time, contour[0].x);
  double C0 = GetC0Coeff(deltaY, deltaT, time, contour[0].y);

  FSDs.push_back(str::EllipticalFourierDescriptor(A0, 0, C0, 0));

  auto ANCoeff = GetANCoeffs(deltaX, deltaT, time);
  auto BNCoeff = GetBNCoeffs(deltaX, deltaT, time);
  auto CNCoeff = GetCNCoeffs(deltaY, deltaT, time);
  auto DNCoeff = GetDNCoeffs(deltaY, deltaT, time);

  if (ANCoeff.size() != BNCoeff.size() && BNCoeff.size() != CNCoeff.size() && CNCoeff.size() != DNCoeff.size()) throw std::runtime_error("Error, vectors are not the same size!\n");

  for (size_t i = 0; i<ANCoeff.size(); ++i){

    FSDs.push_back(str::EllipticalFourierDescriptor(ANCoeff[i], BNCoeff[i], CNCoeff[i], DNCoeff[i]));

  }


  double size, orientation;
  NormalizeAndGetSizeAndOrientation(FSDs, size, orientation);

  const cv::Point center = GetCenterOfContour(contour);

  //FSDs.insert(FSDs.begin(), str::EllipticalFourierDescriptor(center.x, center.y, size, orientation));

  return FSDs;

}

std::vector<double> str::EllipticalFourierDescriptorBuilder::ConvertDescriptorsToFeatureVector(const std::vector<EllipticalFourierDescriptor> &descriptor) const {

  std::vector<double> ret;

  for (auto desc = descriptor.begin(); desc != descriptor.end(); ++desc){
    for (size_t i = 0; i<desc->vals_.size(); ++i) ret.push_back(desc->vals_[i]);
  }

  return ret;

}



cv::Point str::EllipticalFourierDescriptorBuilder::GetCenterOfContour(const std::vector<cv::Point> &contour) const {

  cv::Point ret(0, 0);

  for (auto pt = contour.begin(); pt != contour.end(); ++pt){
    ret += *pt;
  }
  ret.x = (int)((double)ret.x / contour.size());
  ret.y = (int)((double)ret.y / contour.size());

  return ret;

}


void str::EllipticalFourierDescriptorBuilder::NormalizeAndGetSizeAndOrientation(std::vector<str::EllipticalFourierDescriptor> &fds, double &size, double &orientation) const {

  double theta1 = 0.5 * atan(2 * (fds[1][0] * fds[1][1] + fds[1][2] * fds[1][3]) / ((fds[1][0] * fds[1][0]) + (fds[1][2] * fds[1][2]) - (fds[1][1] * fds[1][1]) - (fds[1][3] * fds[1][3])));

  std::vector<str::EllipticalFourierDescriptor> star_fds;

  for (int i = 0; i<number_of_harmonics_; ++i){
    double A = cos(i * theta1) * fds[i][0] + sin(i * theta1) * fds[i][1];
    double B = -sin(i * theta1) * fds[i][0] + cos(i * theta1) * fds[i][1];
    double C = cos(i * theta1) * fds[i][2] + sin(i * theta1) * fds[i][3];
    double D = -sin(i * theta1) * fds[i][2] + cos(i * theta1) * fds[i][3];
    star_fds.push_back(str::EllipticalFourierDescriptor(A, B, C, D));
  }


  double psi1 = atan(star_fds[1][2] / star_fds[1][0]);
  double semi_major = sqrt((star_fds[1][0] * star_fds[1][0]) + (star_fds[1][2] * star_fds[1][2]));

  //for(auto fd = fds.begin(); fd != fds.end() ; ++fd) *fd = str::EllipticalFourierDescriptor( (*fd)[0]/semi_major, (*fd)[1]/semi_major, (*fd)[2]/semi_major, (*fd)[3]/semi_major ) ;
  for (size_t i = 0; i<fds.size(); ++i)
    fds[i] = str::EllipticalFourierDescriptor(star_fds[i][0] / semi_major, star_fds[i][1] / semi_major, star_fds[i][2] / semi_major, star_fds[i][3] / semi_major);

  for (int i = 0; i<number_of_harmonics_; ++i){

    fds[i][0] = (cos(psi1) * star_fds[i][0] + sin(psi1) * star_fds[i][2]) / semi_major;
    fds[i][1] = (cos(psi1) * star_fds[i][1] + sin(psi1) * star_fds[i][3]) / semi_major;
    fds[i][2] = (-sin(psi1) * star_fds[i][0] + cos(psi1) * star_fds[i][2]) / semi_major;
    fds[i][3] = (-sin(psi1) * star_fds[i][1] + cos(psi1) * star_fds[i][3]) / semi_major;

  }

  size = semi_major;
  orientation = psi1;

}



double str::EllipticalFourierDescriptorBuilder::GetA0Coeff(std::vector<int> &deltaX, std::vector<double> &deltaT, std::vector<double> &time, const double contour_startx) const{

  //get A0 coefficient
  double sum1 = 0.0;
  for (size_t i = 1; i<deltaT.size(); ++i){
    double sum2 = 0.0;
    double sum3 = 0.0;
    double inner_diff = 0.0;

    for (size_t j = 1; j<i; ++j){
      sum2 += deltaX[j - 1];
      sum3 += deltaT[j - 1];
    }

    inner_diff = sum2 - ((double)deltaX[i - 1] / deltaT[i - 1])*sum3;
    sum1 += (((double)deltaX[i - 1] / (2 * deltaT[i - 1]))*((time[i] * time[i]) - (time[i - 1] * time[i - 1])) + inner_diff*(time[i] - time[i - 1]));
  }

  return ((1.0 / time.back()) * sum1) + contour_startx;

}

std::vector<double> str::EllipticalFourierDescriptorBuilder::GetANCoeffs(std::vector<int> &deltaX, std::vector<double> &deltaT, std::vector<double> &time) const{

  std::vector<double> ret;
  const double pi2n_sqrd = M_PI * M_PI * 2.0;
  const double pi2n = M_PI * 2.0;

  for (int i = 1; i<number_of_harmonics_; ++i){
    double sum1 = 0.0;
    for (size_t j = 0; j<deltaT.size() - 1; ++j){
      sum1 += ((double)deltaX[j] / deltaT[j])*((cos(pi2n*i*time[j + 1] / time.back()) - cos(pi2n*i*time[j] / time.back())));
    }


    ret.push_back((time.back() / (pi2n_sqrd*i*i)) * sum1);

  }

  return ret;
}
std::vector<double> str::EllipticalFourierDescriptorBuilder::GetBNCoeffs(std::vector<int> &deltaX, std::vector<double> &deltaT, std::vector<double> &time) const{

  std::vector<double> ret;
  const double pi2n_sqrd = M_PI * M_PI * 2.0;
  const double pi2n = M_PI * 2.0;

  for (int i = 1; i<number_of_harmonics_; ++i){
    double sum1 = 0.0;
    for (size_t j = 0; j<deltaT.size() - 1; ++j)
      sum1 += ((double)deltaX[j] / deltaT[j])*((sin(pi2n*i*time[j + 1] / time.back()) - sin(pi2n*i*time[j] / time.back())));

    ret.push_back((time.back() / (pi2n_sqrd*i*i)) * sum1);
  }

  return ret;
}

double str::EllipticalFourierDescriptorBuilder::GetC0Coeff(std::vector<int> &deltaY, std::vector<double> &deltaT, std::vector<double> &time, const double contour_starty) const{

  //get A0 coefficient
  double sum1 = 0.0;
  for (size_t i = 1; i<deltaT.size(); ++i){
    double sum2 = 0.0;
    double sum3 = 0.0;
    double inner_diff = 0.0;

    for (size_t j = 1; j<i; ++j){
      sum2 += deltaY[j - 1];
      sum3 += deltaT[j - 1];
    }

    inner_diff = sum2 - ((double)deltaY[i - 1] / deltaT[i - 1])*sum3;
    sum1 += (((double)deltaY[i - 1] / (2 * deltaT[i - 1]))*((time[i] * time[i]) - (time[i - 1] * time[i - 1])) + inner_diff*(time[i] - time[i - 1]));
  }

  return ((1.0 / time.back()) * sum1) + contour_starty;

}


std::vector<double> str::EllipticalFourierDescriptorBuilder::GetCNCoeffs(std::vector<int> &deltaY, std::vector<double> &deltaT, std::vector<double> &time) const{

  std::vector<double> ret;
  const double pi2n_sqrd = M_PI * M_PI * 2.0;
  const double pi2n = M_PI * 2.0;

  for (int i = 1; i<number_of_harmonics_; ++i){
    double sum1 = 0.0;
    for (size_t j = 0; j<deltaT.size() - 1; ++j)
      sum1 += ((double)deltaY[j] / deltaT[j])*((cos(pi2n*i*time[j + 1] / time.back()) - cos(pi2n*i*time[j] / time.back())));

    ret.push_back((time.back() / (pi2n_sqrd*i*i)) * sum1);
  }

  return ret;

}

std::vector<double> str::EllipticalFourierDescriptorBuilder::GetDNCoeffs(std::vector<int> &deltaY, std::vector<double> &deltaT, std::vector<double> &time) const{

  std::vector<double> ret;
  const double pi2n_sqrd = M_PI * M_PI * 2.0;
  const double pi2n = M_PI * 2.0;

  for (int i = 1; i<number_of_harmonics_; ++i){
    double sum1 = 0.0;
    for (size_t j = 0; j<deltaT.size() - 1; ++j)
      sum1 += ((double)deltaY[j] / deltaT[j])*((sin(pi2n*i*time[j + 1] / time.back()) - sin(pi2n*i*time[j] / time.back())));

    ret.push_back((time.back() / (pi2n_sqrd*i*i)) * sum1);
  }

  return ret;

}

void str::EllipticalFourierDescriptorBuilder::GetIncrementXYT(std::vector<int> &deltaX, std::vector<int> &deltaY, std::vector<double> &deltaT, std::vector<double> &time, const std::vector<cv::Point> &contour) const{

  for (size_t i = 1; i < contour.size(); ++i){
    deltaX.push_back(contour[i].x - contour[i - 1].x);
    deltaY.push_back(contour[i].y - contour[i - 1].y);
  }


  for (size_t i = 0; i< deltaX.size(); ++i)
    deltaT.push_back(std::sqrt(double(deltaX[i] * deltaX[i] + deltaY[i] * deltaY[i])));

  RemoveZeros(deltaX, deltaY, deltaT);

  time.push_back(0.0);
  for (size_t i = 1; i<deltaT.size(); ++i)
    time.push_back(time[i - 1] + deltaT[i - 1]);

}


void str::EllipticalFourierDescriptorBuilder::RemoveZeros(std::vector<int> &deltaX, std::vector<int> &deltaY, std::vector<double> &deltaT) const{

  std::vector<int> new_deltaX, new_deltaY;
  std::vector<double> new_deltaT;

  for (size_t i = 0; i<deltaX.size(); ++i){

    if (deltaT[i] != 0.0){

      new_deltaX.push_back(deltaX[i]);
      new_deltaY.push_back(deltaY[i]);
      new_deltaT.push_back(deltaT[i]);

    }
  }

  deltaX = new_deltaX;
  deltaY = new_deltaY;
  deltaT = new_deltaT;

}