rgb1_1 = imread('image/point1-1.bmp');
rgb2_1 = imread('image/point2-1.bmp');
gray_image1_1 = rgb2gray(rgb1_1);
gray_image2_1 = rgb2gray(rgb2_1);
% figure;
% imshow(gray_image1);
[centers1_1,radii1_1] = imfindcircles(gray_image1_1,[1 20]);
[centers2_1,radii2_1] = imfindcircles(gray_image2_1,[1 20]);

% viscircles(centers1, radii1,'Color','b');

IntrinsicMatrix1 = [1199.7, 0.0, 0.0; 0.0, 1201.5, 0.0; 673.2, 511.1, 1.0];
IntrinsicMatrix2 = [1197.1, 0.0, 0.0; 0.0, 1198.1, 0.0; 629.3, 497.2, 1.0];
cameraParameters1 = cameraParameters('IntrinsicMatrix',IntrinsicMatrix1);
cameraParameters2 = cameraParameters('IntrinsicMatrix',IntrinsicMatrix2);
rotationOfCamera2 = [0.228476 -0.968786 0.0961932;
                     0.973418   0.225705  -0.0389042;
                     0.0159785    0.102525    0.994602];
translationOfCamera2 = [-276.2; -1395.4; 11.35];
stereoParams = stereoParameters(cameraParameters1,cameraParameters2,rotationOfCamera2,translationOfCamera2);

point3d_1 = triangulate(centers1_1, centers2_1, stereoParams);


rgb1_2 = imread('image/point1-2.bmp');
rgb2_2 = imread('image/point2-2.bmp');
gray_image1_2 = rgb2gray(rgb1_2);
gray_image2_2 = rgb2gray(rgb2_2);
% [centers1_2,radii1_2] = imfindcircles(gray_image1_2,[1 20]);
% [centers2_2,radii2_2] = imfindcircles(gray_image2_2,[1 20]);


point3d_2 = triangulate(centers1_2, centers2_2, stereoParams);
distanceInMeters = norm(point3d_2 - point3d_1)/1000
