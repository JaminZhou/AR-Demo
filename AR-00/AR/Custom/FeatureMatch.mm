//
//  FeatureMatch.m
//  AR
//
//  Created by JaminZhou on 2018/6/4.
//  Copyright © 2018 JaminZhou. All rights reserved.
//

#import "FeatureMatch.h"
#include <iostream>
#include <vector>
#include "opencv2/core/core.hpp"
#include <opencv2/imgcodecs/ios.h>
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace std;
using namespace cv;

@implementation FeatureMatch

+ (instancetype)matchImage:(UIImage *)src dstFrame:(ARFrame *)frame {
    FeatureMatch *featureMatch = [FeatureMatch new];
    
    Mat img_1,img_2;
    UIImageToMat(src, img_1);
    UIImageToMat([self imageFromPixelBuffer:frame.capturedImage], img_2);
    
    Ptr<ORB> orb = ORB::create(2000);
    vector<KeyPoint> keyPoints_1, keyPoints_2;
    Mat descriptors_1, descriptors_2;
    
    orb->detectAndCompute(img_1, Mat(), keyPoints_1, descriptors_1);
    orb->detectAndCompute(img_2, Mat(), keyPoints_2, descriptors_2);
    
    Ptr<DescriptorMatcher> matcher =DescriptorMatcher::create("BruteForce");
    vector<DMatch> matches;
    matcher->match(descriptors_1, descriptors_2, matches);
    
    double max_dist = 0; double min_dist = 100;
    for (int i = 0; i < descriptors_1.rows; i++) {
        double dist = matches[i].distance;
        if (dist < min_dist) min_dist = dist;
        if (dist > max_dist) max_dist = dist;
    }
    
    vector<DMatch> good_matches;
    vector<Point2f> srcPoints, dstPoints;
    for( int i = 0; i < descriptors_1.rows; i++ ) {
        if( matches[i].distance < 0.6 * max_dist ) {
            DMatch match = matches[i];
            good_matches.push_back(match);
            srcPoints.push_back(keyPoints_1[match.queryIdx].pt);
            dstPoints.push_back(keyPoints_2[match.trainIdx].pt);
        }
    }
    
    Matx33f homography = findHomography(srcPoints, dstPoints, CV_RANSAC, 8.0);
    matrix_float3x3 intr = frame.camera.intrinsics;
    Matx33f intrinsics = Matx33f(intr.columns[0].x, 0, 0,
                                 0, intr.columns[1].y, 0,
                                 intr.columns[2].x, intr.columns[2].y, 1);
    Matx33f r = intrinsics * homography;
    Vec3f r1, r2;
    float s1 = 0, s2 = 0;
    for(int i= 0; i < 3; i++){
        r1(i) = r(i, 0);
        r2(i) = r(i, 1);
        s1 += r1(i) * r1(i);
        s2 += r2(i) * r2(i);
    }
    float s = sqrt((s1 + s2) / 2);
    r1 = r1 / s;
    r2 = r2 / s;
    
    Matx33f optR = [self optimizedRSVD:r1 r2:r2];
    
    Point2f c1 = Point2f(img_1.cols / 2, img_1.rows / 2);
    Point3f homoC = homography * c1;
    Point2f c = Point2f(homoC.x / homoC.z, homoC.y / homoC.z);
    
    Vec3f pre_r1 = Vec3f(-1.0 * CV_PI / 2, 0 , 0);
    Vec3f pre_r2 = Vec3f(0, -1.0 * CV_PI / 2, 0);
    Matx33f pre_R1, pre_R2;
    Rodrigues(pre_r1, pre_R1);
    Rodrigues(pre_r2, pre_R2);
    Matx33f pre_R = pre_R2*pre_R1;
    
    Matx33f post_R = optR.inv();
    Vec3f post_r;
    Rodrigues(post_R, post_r);
    
    Vec3f src_r;
    src_r(0) = post_r(0);
    src_r(1) = -post_r(2);
    src_r(2) = post_r(1);
    Matx33f src_R;
    Rodrigues(src_r, src_R);
    Matx33f rotationMat = src_R * pre_R;
    
    CGPoint hit = CGPointMake(c.x, c.y);
    CGFloat width = frame.camera.imageResolution.width;
    CGFloat height = frame.camera.imageResolution.height;
    CGPoint hitP = CGPointMake(hit.y / width, 1.0 - hit.x / height);
    
    NSArray<ARHitTestResult *>* results = [frame hitTest:hitP types:ARHitTestResultTypeEstimatedHorizontalPlane];
    if(results.count == 0) return featureMatch;
    ARHitTestResult *result = [results firstObject];
    
    Matx33f rotation;
    for(int i = 0; i < 3; i++){
        rotation(0, i)=frame.camera.transform.columns[i].x;
        rotation(1, i)=frame.camera.transform.columns[i].y;
        rotation(2, i)=frame.camera.transform.columns[i].z;
    }
    
    rotation = rotation * rotationMat.inv();
    Vec3f y1 = Vec3f(0.0,1.0,0.0);
    Vec3f y2 = rotation * y1;
    Vec3f vec1 = y2;
    Vec3f vec2 = y1;
    Vec3f location = Vec3f(vec1(1) * vec2(2) - vec1(2) * vec2(1),
                           0.0,
                           vec1(0) * vec2(1) - vec1(1) * vec2(0));
    
    float normalize = sqrt(location(0) * location(0) + location(1) * location(1) + location(2) * location(2));
    location = Vec3f(location(0) / normalize,
                     location(1) / normalize,
                     location(2) / normalize);
    
    Vec3f adjust_r = location*acosf(y2(1));
    Matx33f adjust_R;
    Rodrigues(adjust_r, adjust_R);
    rotation = adjust_R * rotation;
    
    Vec3f x = rotation * Vec3f(1.0, 0.0, 0.0);
    Vec3f y = rotation * Vec3f(0.0, 1.0, 0.0);
    Vec3f z = rotation * Vec3f(0.0, 0.0, 1.0);
    
    matrix_float4x4 worldMat;
    worldMat.columns[0] = simd_make_float4(x(0), x(1), x(2),0.0);
    worldMat.columns[1] = simd_make_float4(y(0), y(1), y(2),0.0);
    worldMat.columns[2] = simd_make_float4(z(0), z(1), z(2),0.0);
    worldMat.columns[3] = result.worldTransform.columns[3];
    
    featureMatch.success = YES;
    featureMatch.worldTransform = SCNMatrix4FromMat4(worldMat);
    return featureMatch;
}

+ (Matx33f)optimizedRSVD:(Vec3f)r1 r2:(Vec3f)r2 {
    Vec3f r3 = r1.cross(r2);
    Matx33f tmpR;
    for(int i = 0; i < 3; i++){
        tmpR(i, 0) = r1(i);
        tmpR(i, 1) = r2(i);
        tmpR(i, 3) = r3(i);
    }
    Mat tmpR_mat = Mat(tmpR);
    Mat u, vt, w;
    SVDecomp(tmpR_mat, w, u, vt);
    Mat outR_mat = u * vt;
    float* data = reinterpret_cast<float*>(outR_mat.data);
    Matx33f outR(data);
    return outR;
}

+ (UIImage *)imageFromPixelBuffer:(CVPixelBufferRef)pixelBuffer {
    CGImageRef imageRef = [[CIContext contextWithOptions:nil]
                           createCGImage:[CIImage imageWithCVPixelBuffer:pixelBuffer]
                           fromRect:CGRectMake(0,
                                               0,
                                               CVPixelBufferGetWidth(pixelBuffer),
                                               CVPixelBufferGetHeight(pixelBuffer))];
    UIImage *image = [[UIImage alloc] initWithCGImage:imageRef
                                                scale:1
                                          orientation:UIImageOrientationRight];
    CGImageRelease(imageRef);
    image = [self fixOrientation:image];
    return image;
}

+ (UIImage *)fixOrientation:(UIImage *)image {
    if (image.imageOrientation == UIImageOrientationUp) return image;
    CGAffineTransform transform = CGAffineTransformIdentity;
    switch (image.imageOrientation) {
        case UIImageOrientationDown:
        case UIImageOrientationDownMirrored:
            transform = CGAffineTransformTranslate(transform, image.size.width, image.size.height);
            transform = CGAffineTransformRotate(transform, M_PI);
            break;
        case UIImageOrientationLeft:
        case UIImageOrientationLeftMirrored:
            transform = CGAffineTransformTranslate(transform, image.size.width, 0);
            transform = CGAffineTransformRotate(transform, M_PI_2);
            break;
        case UIImageOrientationRight:
        case UIImageOrientationRightMirrored:
            transform = CGAffineTransformTranslate(transform, 0, image.size.height);
            transform = CGAffineTransformRotate(transform, -M_PI_2);
            break;
        case UIImageOrientationUp:
        case UIImageOrientationUpMirrored:
            break;
    }
    
    switch (image.imageOrientation) {
        case UIImageOrientationUpMirrored:
        case UIImageOrientationDownMirrored:
            transform = CGAffineTransformTranslate(transform, image.size.width, 0);
            transform = CGAffineTransformScale(transform, -1, 1);
            break;
        case UIImageOrientationLeftMirrored:
        case UIImageOrientationRightMirrored:
            transform = CGAffineTransformTranslate(transform, image.size.height, 0);
            transform = CGAffineTransformScale(transform, -1, 1);
            break;
        case UIImageOrientationUp:
        case UIImageOrientationDown:
        case UIImageOrientationLeft:
        case UIImageOrientationRight:
            break;
    }
    
    CGContextRef ctx = CGBitmapContextCreate(NULL, image.size.width, image.size.height,
                                             CGImageGetBitsPerComponent(image.CGImage), 0,
                                             CGImageGetColorSpace(image.CGImage),
                                             CGImageGetBitmapInfo(image.CGImage));
    CGContextConcatCTM(ctx, transform);
    switch (image.imageOrientation) {
        case UIImageOrientationLeft:
        case UIImageOrientationLeftMirrored:
        case UIImageOrientationRight:
        case UIImageOrientationRightMirrored:
            CGContextDrawImage(ctx, CGRectMake(0, 0, image.size.height, image.size.width), image.CGImage);
            break;
        default:
            CGContextDrawImage(ctx, CGRectMake(0, 0, image.size.width, image.size.height), image.CGImage);
            break;
    }
    
    CGImageRef cgimg = CGBitmapContextCreateImage(ctx);
    UIImage *img = [UIImage imageWithCGImage:cgimg];
    CGContextRelease(ctx);
    CGImageRelease(cgimg);
    return img;
}

@end
