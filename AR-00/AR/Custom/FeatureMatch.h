//
//  FeatureMatch.h
//  AR
//
//  Created by JaminZhou on 2018/6/4.
//  Copyright © 2018 JaminZhou. All rights reserved.
//

#import <UIKit/UIKit.h>
#import <ARKit/ARKit.h>

@interface FeatureMatch : NSObject
@property (nonatomic) BOOL success;
@property (nonatomic) SCNMatrix4 worldTransform;
+ (instancetype)matchImage:(UIImage *)src dstFrame:(ARFrame *)frame;
@end
