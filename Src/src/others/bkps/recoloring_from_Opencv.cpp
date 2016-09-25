////////////////////////////////// the .h file :

#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/ml.hpp>

#include <vector>

using namespace cv;
using namespace cv::ml;
using namespace std;

class Recoloring
{
private:
	void TrainGMM(Ptr<EM> source_model, Mat& source, Mat& source_mask);
	vector<int> MatchGaussians(Ptr<EM> source_model, Ptr<EM> target_model);

public:
	void Recolor(Mat& source, Mat& source_mask, Mat& target, Mat& target_mask);
	Recoloring() {};
	~Recoloring(void);
};


////////////////////////////////// the .cpp file :

// Originally from http://www.morethantechnical.com/2010/06/24/image-recoloring-using-gaussian-mixture-model-and-expectation-maximization-opencv-wcode/
// By "Roy"

// Which is based on :
//  * Shapira09 : https://www.cs.tau.ac.il/~liors/research/papers/image_appearance_exploration.pdf
//  * http://www.sciweavers.org/files/docs/2358/icassp_cvd_poster_pdf_4a383d1fb0.pdf

// Reworked for OpenCV 3
// Adrien "Adriweb" Bertrand

#include "Recoloring_GMM.h"

#include <iostream>
#include <limits>


#define MAX_MATCH_ITERS 10

Recoloring::~Recoloring(void)
{
}

vector<int> Recoloring::MatchGaussians(Ptr<EM> source_model, Ptr<EM> target_model)
{
    unsigned long num_g = (unsigned long) source_model->getClustersNumber(); // 3

    Mat src_mu = source_model->getMeans(); // mu -> means of the Gaussian mixture (cluster centers)
    Mat tar_mu = target_model->getMeans();

    std::vector<Mat> source_covs, target_covs;
    target_model->getCovs(target_covs);
    source_model->getCovs(source_covs);

    double best_dist = std::numeric_limits<double>::max();

    vector<int> best_res(num_g);
    vector<int> permuts(num_g);

    for (int _ = 0; _ < MAX_MATCH_ITERS; _++)
    {
        // Make one permutation then shuffle
        for (int i = 0; i < num_g; i++)
            permuts[i] = i;
        randShuffle(Mat(permuts));

        // Greedy selection
        vector<int> res(num_g);
        vector<bool> taken(num_g);
        for (int sg = 0; sg < num_g; sg++)
        {
            double min_dist = std::numeric_limits<double>::max();
            int minv = -1;
            for (int tg = 0; tg < num_g; tg++)
            {
                if (taken[tg])
                    continue;

                // TODO: can save on re-calculation of pairs - calculate affinity matrix ahead
                // double d = norm(sMu(Range(permuts[sg], permuts[sg] + 1), Range(0, 3)), tMu(Range(tg, tg + 1), Range(0, 3)));

                // symmetric Kullback-Leibler
                auto sMu_tmp = src_mu(Range(permuts[sg], permuts[sg] + 1), Range(0, 3));
                auto tMu_tmp = tar_mu(Range(tg, tg + 1), Range(0, 3));
                Mat diff = Mat(sMu_tmp - tMu_tmp);
                Mat d = diff * Mat(Mat(source_covs[permuts[sg]]).inv() + Mat(target_covs[tg]).inv()) * diff.t();
                Scalar tr = trace(Mat(
                        Mat(Mat(source_covs[permuts[sg]]) * Mat(target_covs[tg])) +
                        Mat(Mat(target_covs[tg]) * Mat(source_covs[permuts[sg]]).inv()) +
                        Mat(Mat::eye(3, 3, CV_64FC1) * 2)
                ));
                double kl_dist = ((double*) d.data)[0] + tr[0];

                if (kl_dist < min_dist)
                {
                    min_dist = kl_dist;
                    minv = tg;
                }
            }
            res[permuts[sg]] = minv;
            taken[minv] = true;
        }

        //total distance for the permutation
        double dist = 0;
        for (int i = 0; i < num_g; i++)
        {
            dist += norm(src_mu(Range(permuts[i], permuts[i] + 1), Range(0, 3)),
                         tar_mu(Range(res[permuts[i]], res[permuts[i]] + 1), Range(0, 3)));
        }
        if (dist < best_dist)
        {
            best_dist = dist;
            best_res = res;
        }
    }

    return best_res;
}

void Recoloring::TrainGMM(Ptr<EM> source_model, Mat& source, Mat& source_mask)
{
    int src_samples_size = countNonZero(source_mask);
    Mat source_samples(src_samples_size, 3, CV_32FC1);

    int sample_count = 0;
    for (int y = 0; y < source.rows; y++)
    {
        Vec3f* row = source.ptr<Vec3f>(y);
        uchar* mask_row = source_mask.ptr<uchar>(y);
        for (int x = 0; x < source.cols; x++)
        {
            if (mask_row[x] > 0)
            {
                source_samples.at<Vec3f>(sample_count++, 0) = row[x];
            }
        }
    }

    //source_model->clear();
    cout << "Training..." << endl;
    source_model->trainEM(source_samples);
}

void Recoloring::Recolor(Mat& _source, Mat& source_mask, Mat& _target, Mat& target_mask)
{
    Mat source, target;
    _source.convertTo(source, CV_32F, 1.0 / 255.0);
    _target.convertTo(target, CV_32F, 1.0 / 255.0);

    const int numberOfGaussians = 3; // 3 Components (R, G, B)

    Ptr<EM> source_model = EM::create();
    source_model->setClustersNumber(numberOfGaussians);
    TrainGMM(source_model, source, source_mask);

    Ptr<EM> target_model = EM::create();
    target_model->setClustersNumber(numberOfGaussians);
    TrainGMM(target_model, target, target_mask);

    vector<int> match = MatchGaussians(source_model, target_model);

    Mat target_32f;
    target.copyTo(target_32f);

    std::vector<Mat> source_covs, target_covs;
    source_model->getCovs(source_covs);
    target_model->getCovs(target_covs);

    Mat sMu, tMu;
    source_model->getMeans().convertTo(sMu, CV_64F);
    target_model->getMeans().convertTo(tMu, CV_64F);

    int num_g = target_model->getClustersNumber();

    Mat post_probs; // posterior probabilities of each component given the sample (cf. the predict2 method below)
    Mat samp(1, 3, CV_32FC1);
    for (int y = 0; y < target.rows; y++)
    {
        Vec3f* row = target_32f.ptr<Vec3f>(y);
        uchar* mask_row = target_mask.ptr<uchar>(y);
        for (int x = 0; x < target.cols; x++)
        {
            if (mask_row[x] > 0)
            {
                memcpy(samp.data, &(row[x][0]), 3 * sizeof(float));
                Vec2d retVal = target_model->predict2(samp, post_probs);

/*
                cout << "likelihood logarithm value : " << retVal[0] << endl;
                cout << "index of the most probable mixture component : " << retVal[1] << endl;
                cout << "pr :" << ((float*) post_probs.data)[0] << "," << ((float*) post_probs.data)[1] << "," << ((float*) post_probs.data)[2] << endl;
                cout << "----------" << endl;
*/

                Mat samp_64f;
                samp.convertTo(samp_64f, CV_64F);

                //From Shapira09: Xnew = Sum_i { pr(i) * Sigma_source_i * (Sigma_target_i)^-1 * (x - mu_target) + mu_source }
                Mat Xnew(1, 3, CV_64FC1, Scalar(0));
                for (int i = 0; i < num_g; i++)
                {
                    if (((float*) post_probs.data)[i] <= 0)
                        continue;
                    Xnew += Mat((
                                        //Mat(source_covs[match[i]]) *
                                        //Mat(target_covs[i]).inv() *
                                        Mat(samp_64f - tMu(Range(i, i + 1), Range(0, 3))).t() +
                                        sMu(Range(match[i], match[i] + 1), Range(0, 3)).t()
                                ) * (double) (((float*) post_probs.data)[i])).t();
                }

                Mat _tmp;
                Xnew.convertTo(_tmp, CV_32F);
                memcpy(&(row[x][0]), _tmp.data, sizeof(float) * 3);
            }
        }
    }

    namedWindow("orig target");
    imshow("orig target", target);
    namedWindow("source orig");
    imshow("source orig", source);
    namedWindow("source masked");
    Mat source_masked;
    source.copyTo(source_masked, source_mask);
    imshow("source masked", source_masked);
    namedWindow("dest target");
    imshow("dest target", target_32f);

    waitKey(0);

    target_32f.convertTo(_target, CV_8UC3, 255.0);
}


int main(int argc, char** argv)
{

    Recoloring r;

    Mat dst, dst_mask, src, src_mask;

    double scale_factor = 0.75;
    resize(imread("images/img1.jpg"), dst, Size(), scale_factor, scale_factor);
    resize(imread("images/img1_mask.jpg", IMREAD_GRAYSCALE), dst_mask, Size(), scale_factor, scale_factor, INTER_NEAREST);
    resize(imread("images/img2.jpg"), src, Size(), scale_factor, scale_factor);
    resize(imread("images/img2_mask.jpg", IMREAD_GRAYSCALE), src_mask, Size(), scale_factor, scale_factor, INTER_NEAREST);

    r.Recolor(src, src_mask, dst, dst_mask);

    return 0;
}